from artiq.experiment import *

from pdq.test.test_dac import _test_program
from pdq.host.usb import PDQ


class PDQ2SPI(EnvExperiment):
    """ PDQ SPI write and read tests.
    Example experiment controling a PDQ board stack from ARTIQ over SPI.

    This assumes a working ARTIQ installation (see the ARTIQ manual), working
    and configured core device (e.g. KC705), and a hardware adapter with
    an RTIO SPI master connected to the PDQ's SPI bus (see the PDQ or ARTIQ
    manual).
    After building the desired PDQ bitstream flash that bitstream to the
    boards (see the PDQ manual).

    Example device_db entries are provided in device_db.py. Adapt them to
    your specific situation.
    
    Note that test_prog is expected to fail for stacks of boards (i.e.
    devices with num_boards >1), as readback via SPI is only supported
    for one board at a time.
    """
    def build(self):
        self.setattr_device("core")
        self.setattr_device("led")
        # SPI access
        self.setattr_device("pdq")
        # USB access
        self.pdq_usb = PDQ("hwgrep://PULSER01")

    def run(self):
        prog = _test_program
        # generate binary program for later tests and writing over SPI
        self.pdq_program(self.pdq, prog)
        # test register access, readback, memory write, memory readback
        self.test()
        if True:
            # program waveform data over SPI and run
            self.prog_spi()
        else:
            # program waveform data over USB and run
            self.prog_usb(prog)

    def prog_usb(self, prog):
        self.pdq_usb.set_config(reset=1)
        self.pdq_usb.set_config(reset=0, clk2x=1, enable=0, trigger=0, aux_miso=1,
                board=0xf)
        self.pdq_usb.program(prog)
        self.pdq_usb.set_frame(0)
        self.pdq_usb.set_config(reset=0, clk2x=1, enable=1, trigger=1, aux_miso=1,
                board=0xf)

    @kernel
    def prog_spi(self):
        self.core.reset()
        self.core.break_realtime()
        self.pdq.setup_bus(write_div=50, read_div=50)
        self.pdq.set_config(reset=1)
        delay(1*ms)
        self.pdq.set_config(reset=0, clk2x=1, enable=0, trigger=0, aux_miso=1,
                board=0xf)
        self.test_prog()
        self.pdq.set_frame(0)
        self.pdq.set_config(reset=0, clk2x=1, enable=1, trigger=1, aux_miso=1,
                board=0xf)

    @kernel
    def test(self):
        self.core.reset()
        self.core.break_realtime()
        self.pdq.setup_bus(write_div=50, read_div=50)
        self.pdq.set_config(reset=1)

        for i in range(100):
            delay(80*us)
            self.led.on()
            self.pdq.set_config(clk2x=1, trigger=0, enable=0, aux_miso=1)
            self.pdq.set_crc(0)
            self.pdq.set_frame(0)
            self.led.off()
        for i in range(100):
            self.test_reg()
        for i in range(100):
            self.test_mem()
        for i in range(100):
            self.test_prog()

    @kernel
    def trigger(self):
        """Example showing how to trigger a PDQ stack over SPI: set and clear
        the trigger flag in the configuration register"""
        self.pdq.set_config(clk2x=1, trigger=1, enable=0, aux_miso=1)
        delay(2*us)
        self.pdq.set_config(clk2x=1, trigger=0, enable=0, aux_miso=1)

    @kernel
    def test_reg(self):
        self.pdq.set_config(reset=1)
        delay(100*us)
        self.led.on()
        self.pdq.set_config(clk2x=1, trigger=0, enable=0, aux_miso=1)
        delay(100*us)
        if self.pdq.get_config() != 242:
            raise ValueError("wrong config")
        delay(100*us)
        if self.pdq.get_frame() != 0:
            raise ValueError("wrong frame")
        delay(100*us)
        self.pdq.set_crc(0)
        if self.pdq.get_crc() != 104:
            raise ValueError("wrong crc")
        delay(100*us)
        self.pdq.set_frame(25)
        if self.pdq.get_frame() != 25:
            raise ValueError("wrong frame")
        delay(100*us)
        if self.pdq.get_crc() == 104:
            raise ValueError("wrong crc")
        delay(100*us)
        self.led.off()

    @kernel
    def test_mem(self):
        self.pdq.set_config(reset=1)
        delay(100*us)
        self.led.on()
        self.pdq.set_config(clk2x=1, trigger=0, enable=0, aux_miso=1)
        delay(100*us)
        data_write = bytes([1, 2, 3, 4, 5, 6, 7, 8])
        self.pdq.write_mem(0, 0, data_write)
        data_read = bytearray([0]*(len(data_write)))
        self.pdq.read_mem(0, 0, data_read)
        for i in range(len(data_write)):
            if data_read[i] != data_write[i]:
                raise ValueError("wrong memory")
        delay(100*us)
        self.led.off()

    @kernel
    def test_prog(self):
        delay(1*ms)
        for i in range(self.pdq.num_channels):
            board = i // self.pdq.num_dacs
            mem = i % self.pdq.num_dacs
            data_write = self.pdq_data[i]
            self.pdq.write_mem(mem, 0, data_write, board)
            data_read = bytearray([0]*(len(data_write)))
            self.pdq.read_mem(mem, 0, data_read, board)
            delay(100*us)
            for j in range(len(data_write)):
                if data_read[j] != data_write[j]:
                    raise ValueError("bad readback")

    def pdq_program(self, pdq, program):
        chs = [pdq.channels[i] for i in range(pdq.num_channels)]
        for channel in chs:
            channel.clear()
        for frame in program:
            segments = [c.new_segment() for c in chs]
            pdq.program_segments(segments, frame)
            for segment in segments:
                segment.line(typ=3, data=b"", trigger=True, duration=1, aux=1,
                             jump=True)
        self.pdq_data = []
        for ch in chs:
            data = ch.serialize()
            self.pdq_data.append(data)
