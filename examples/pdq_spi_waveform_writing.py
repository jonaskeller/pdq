from artiq.experiment import *

class PDQSPITEST(EnvExperiment):
    """PDQ SPI waveform writing examples"""
    
    def build(self):
        self.setattr_device("core")
        self.setattr_device("pdq_spi0")
        self.setattr_device("ttl15")  # scope trigger
        
        self.setattr_argument("write_method",NumberValue(0, ndecimals=0, step=1, min=0, max=2))
                
        tstep=5000
        scale=5
        self.frames=32
        self.wavesynth_example = [
                [
                    {
                        "trigger": True,
                        "duration": tstep,
                        "channel_data": [
                            {"bias":     {"amplitude": [0, i/tstep/scale]}},
                        ],
                    },
                    {
                        "duration": tstep,
                        "channel_data": [
                            {"bias": {"amplitude": [i/scale]}},
                        ],
                    },
                    {
                        "duration": tstep,
                        "channel_data": [
                            {"bias": {"amplitude": [i/scale, -i/tstep/scale]}},
                        ],
                    },
                ]
                for i in range(self.frames)
        ]
        self.wavesynth_example2 = [
                [
                    {
                        "trigger": True,
                        "duration": tstep,
                        "channel_data": [
                            {"dds":     {"amplitude": [scale, 0,0,0],
                                         "phase": [0,i*1e-5]}},
                        ],
                    },
                ]
                for i in range(self.frames)
        ]

   
    @kernel
    def init(self):
        self.core.reset()
        self.pdq_spi0.setup_bus(write_div=50, read_div=50)
        self.pdq_spi0.set_config(reset=1)
        delay(10*us)

    @kernel
    def trigger(self):
        """Switches frames via SPI and triggers each time."""
        self.core.break_realtime()
        for frame in range(self.frames):
            self.pdq_spi0.set_frame(frame)
            self.ttl15.pulse(10*us)  # scope trigger
            self.pdq_spi0.set_config(clk2x=0, trigger=1, enable=1, aux_miso=1)
            self.pdq_spi0.set_config(clk2x=0, trigger=0, enable=1, aux_miso=1)
            delay(0.25*s)
            
 
    def prepare(self):
        self.chan_list, self.chan_data_list = self.pdq_spi0.program_host(self.wavesynth_example)
        self.pdq_spi0.program_host(self.wavesynth_example2)
    
    @kernel
    def write_waveform(self, method=0):
        """Writes the wavesynth data to the PDQs via SPI using one of three different methods"""
        if method == 0:  # use latest output of meth:`program_host`
            self.pdq_spi0.program_kernel()
        if method == 1:  # use manually stored output of meth:`program_host`
            self.pdq_spi0.program_kernel(self.chan_list, self.chan_data_list)
        if method == 2:  # convert wavesynth data via an RPC (not particularly efficient)
            delay(0.5*s)  # some slack for the RPC
            self.pdq_spi0.program(self.wavesynth_example)
        
    @kernel
    def run(self):
        self.init()
        self.write_waveform(self.write_method)  
        self.trigger()
