from artiq.language.core import kernel, delay_mu
from artiq.coredevice import spi
from artiq.language.types import TList, TBytes

from ..host.protocol import PDQBase, PDQ_CMD


_PDQ_SPI_CONFIG = (
        0*spi.SPI_OFFLINE | 0*spi.SPI_CS_POLARITY |
        0*spi.SPI_CLK_POLARITY | 0*spi.SPI_CLK_PHASE |
        0*spi.SPI_LSB_FIRST | 0*spi.SPI_HALF_DUPLEX
        )


class PDQ(PDQBase):
    """PDQ smart arbitrary waveform generator stack.

    Provides access to a stack of PDQ boards connected via SPI using PDQ
    gateware version 3 or later.

    The SPI bus is wired with ``CS_N`` from the core device connected to
    ``F2 IN`` on the master PDQ, ``CLK`` connected to ``F3 IN``, ``MOSI``
    connected to ``F4 IN`` and ``MISO`` (optionally) connected to ``F5 OUT``.
    ``F1 TTL Input Trigger`` remains as waveform trigger input.
    Due to hardware constraints, there can only be one board connected to the
    core device's MISO line and therefore there can only be SPI readback
    from one board at any time.

    :param spi_device: Name of the SPI bus this device is on.
    :param chip_select: Value to drive on the chip select lines of the SPI bus
        during transactions.
    """
    kernel_invariants = {"core", "chip_select", "bus"}

    def __init__(self, dmgr, spi_device, chip_select=1, **kwargs):
        self.core = dmgr.get("core")
        self.bus = dmgr.get(spi_device)
        self.chip_select = chip_select
        PDQBase.__init__(self, **kwargs)

        # lists to store the output of :meth:`program_host`, for writing to the
        # hardware using :meth:`program_kernel`.
        self.channel_list = []
        self.channel_data_list = []

    @kernel
    def setup_bus(self, write_div=24, read_div=64):
        """Configure the SPI bus and the SPI transaction parameters
        for this device. This method has to be called before any other method
        if the bus has been used to access a different device in the meantime.

        This method advances the timeline by the duration of two
        RTIO-to-Wishbone bus transactions.

        :param write_div: Write clock divider.
        :param read_div: Read clock divider.
        """
        # write: 4*8ns >= 20ns = 2*clk (clock de-glitching 50MHz)
        # read: 15*8*ns >= ~100ns = 5*clk (clk de-glitching latency + miso
        #   latency)
        self.bus.set_config_mu(_PDQ_SPI_CONFIG, write_div, read_div)
        self.bus.set_xfer(self.chip_select, 16, 0)

    @kernel
    def set_reg(self, adr, data, board):
        """Set a PDQ register.

        :param adr: Address of the register (``_PDQ_ADR_CONFIG``,
            ``_PDQ_ADR_FRAME``, ``_PDQ_ADR_CRC``).
        :param data: Register data (8 bit).
        :param board: Board to access, ``0xf`` to write to all boards.
        """
        self.bus.set_xfer(self.chip_select, 16, 0)
        self.bus.write((PDQ_CMD(board, 0, adr, 1) << 24) | (data << 16))
        delay_mu(self.bus.ref_period_mu)  # get to 20ns min cs high

    @kernel
    def get_reg(self, adr, board):
        """Get a PDQ register.

        :param adr: Address of the register (``_PDQ_ADR_CONFIG``,
          ``_PDQ_ADR_FRAME``, ``_PDQ_ADR_CRC``).
        :param board: Board to access, ``0xf`` to write to all boards.

        :return: Register data (8 bit).
        """
        self.bus.set_xfer(self.chip_select, 16, 8)
        self.bus.write(PDQ_CMD(board, 0, adr, 0) << 24)
        delay_mu(self.bus.ref_period_mu)  # get to 20ns min cs high
        self.bus.read_async()
        self.bus.set_xfer(self.chip_select, 16, 0)
        return int(self.bus.input_async() & 0xff)  # FIXME: m-labs/artiq#713

    @kernel
    def write_mem(self, mem, adr, data, board=0xf):
        """Write to DAC channel waveform data memory.

        :param mem: DAC channel memory to access (0 to 2).
        :param adr: Start address.
        :param data: (bytes) Memory data.
        :param board: Board to access (0-15) with ``0xf = 15`` being broadcast
            to all boards.
        """
        self.bus.set_xfer(self.chip_select, 24, 0)
        self.bus.write((PDQ_CMD(board, 1, mem, 1) << 24) |
                       ((adr & 0x00ff) << 16) | (adr & 0xff00))
        delay_mu(-self.bus.write_period_mu-3*self.bus.ref_period_mu)
        self.bus.set_xfer(self.chip_select, 8, 0)
        for i in range(len(data)):
            self.bus.write(data[i] << 24)
            delay_mu(-self.bus.write_period_mu)
        delay_mu(self.bus.write_period_mu + self.bus.ref_period_mu)
        # get to 20ns min cs high

    @kernel
    def read_mem(self, mem, adr, data, board=0xf, buffer=8):
        """Read from DAC channel waveform data memory.

        :param mem: DAC channel memory to access (0 to 2).
        :param adr: Start address.
        :param data: (bytearray) Memory data.
        :param board: Board to access (0-15) with ``0xf = 15`` being broadcast
            to all boards.
        """
        n = len(data)
        if not n:
            return
        self.bus.set_xfer(self.chip_select, 24, 8)
        self.bus.write((PDQ_CMD(board, 1, mem, 0) << 24) |
                       ((adr & 0x00ff) << 16) | (adr & 0xff00))
        delay_mu(-self.bus.write_period_mu-3*self.bus.ref_period_mu)
        self.bus.set_xfer(self.chip_select, 0, 8)
        for i in range(n):
            self.bus.write(0)
            delay_mu(-self.bus.read_period_mu)
            if i > 0:
                delay_mu(-3*self.bus.ref_period_mu)
                self.bus.read_async()
            if i > buffer:
                data[i - 1 - buffer] = self.bus.input_async() & 0xff
        delay_mu(self.bus.read_period_mu)
        self.bus.read_async()
        for i in range(max(0, n - 1 - buffer), n):
            data[i] = self.bus.input_async() & 0xff

    def program_host(self, program, channels=None):
        """Serialize a wavesynth program. The result is stored in member
        variables and returned for manual handling. Use :meth:`program_kernel`
        to write it to memory.

        The :class:`Channel` targeted are cleared and each frame in the
        wavesynth program is appended to a fresh set of :class:`Segment`
        of the channels. All segments are allocated, the frame address table
        is generated and the channels are serialized.

        Short single-cycle lines are prepended and appended to each frame to
        allow proper write interlocking and to assure that the memory reader
        can be reliably parked in the frame address table.
        The first line of each frame is mandatorily triggered.

        :param program: (list) Wavesynth program.
        :param channels: (list[int]) Channel indices to use. If unspecified, all
                channels are used.

        :return (list[int]), (list[bytes]): List of channels and list of channel
        data to be written to the hardware using :meth:`program_kernel`
        """
        if channels is None:
            channels = range(self.num_channels)
        chs = [self.channels[i] for i in channels]
        for channel in chs:
            channel.clear()
        for frame in program:
            segments = [c.new_segment() for c in chs]
            self.program_segments(segments, frame)
            # append an empty line to stall the memory reader before jumping
            # through the frame table (`wait` does not prevent reading
            # the next line)
            for segment in segments:
                segment.line(typ=3, data=b"", trigger=True, duration=1, aux=1,
                             jump=True)
        self.channel_list = []
        self.channel_data_list = []
        for channel, ch in zip(channels, chs):
            self.channel_list.append(channel)
            self.channel_data_list.append(ch.serialize())
        return self.channel_list, self.channel_data_list

    def program_rpc(self, program, channels=None) -> TList(TBytes):
        """
        Wrapper for :meth:`program_host` with only one return value for use in RPCs
        :param program: (list) Wavesynth program.
        :param channels: (list[int]) Channel indices to use. If unspecified, all
                         channels are used.
        """
        _, channel_data_list = self.program_host(program, channels)
        return channel_data_list

    @kernel
    def program_kernel(self, channel_list=[-1], channel_data_list=[bytes([0])]):
        """
        Write the output of :meth:`program_host` via SPI.

        :param channel_list: (list[int]) (optional) List of channels as returned by
                             :meth:`program_host`. Set to [-1] to use values stored
                             from the last invocation of :meth:`program_host`.
        :param channel_data_list: (list[bytes]) (optional) List of corresponding
                                  data as returned by :meth:`program_host`
        """
        if channel_list == [-1]:
            channel_list = list(self.channel_list)
            channel_data_list = list(self.channel_data_list)
        assert len(channel_data_list) >= len(channel_list)
        for i in range(len(channel_list)):
            board = channel_list[i] // self.num_dacs
            mem = channel_list[i] % self.num_dacs
            self.write_mem(mem=mem, adr=0, data=channel_data_list[i],
                           board=board)

    @kernel
    def program(self, program, channels=[-1]):
        """
        Replaces the inherited :meth:`program` method from :class:`PDQBase`,
        which doesn't work via SPI due to the mixture of host and kernel
        functions. Because of the RPC, this is not the most efficient solution.
        :param program (list): Wavesynth program.
        :param channels (list[int]): Channel indices to use. By default, all
                                     channels are used.
        """
        if channels == [-1]:
            channels = list(range(self.num_channels))

        channel_data_list = self.program_rpc(program, channels)
        self.program_kernel(channels, channel_data_list)
