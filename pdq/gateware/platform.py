# Copyright 2013-2017 Robert Jordens <jordens@gmail.com>
#
# This file is part of pdq.
#
# pdq is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# pdq is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with pdq.  If not, see <http://www.gnu.org/licenses/>.

from migen.build.generic_platform import *
from migen.build.xilinx import XilinxPlatform


_io = [
        ("clk50", 0, Pins("P80"), IOStandard("LVCMOS25")),

        ("comm", 0,
            Subsignal("data", Pins("P101 P91 P76 P72 P71 P60 P58 P57")),
            Subsignal("rdl", Pins("P97")),
            Subsignal("rxfl", Pins("P54")),

            Subsignal("g1_in", Pins("P159")), #GO_1
            Subsignal("g1_out", Pins("P102")), #G1
            IOStandard("LVCMOS25"),
        ),

        ("ctrl", 0,
            Subsignal("reset", Pins("P96")), # dac_reset

            Subsignal("board", Pins("P32 P6 P14 P43")), # active low
            Subsignal("trigger", Pins("P110")), #F1
            Subsignal("frame", Pins("P118 P124 P98")), #F2 F3 F4
            Subsignal("aux", Pins("P99")), #F5 out

            Subsignal("g2_in", Pins("P169")), #GO_2
            Subsignal("g2_out", Pins("P100")), #G2
            IOStandard("LVCMOS25"),
        ),

        ("dac", 0,
            Subsignal("clk_p", Pins("P89"), IOStandard("LVCMOS25")),
            Subsignal("clk_n", Pins("P90"), IOStandard("LVCMOS25")),
            Subsignal("data_p", Pins(
                "P106 P108 P112 P115 P119 P122 P126 P128 "
                "P132 P134 P137 P139 P144 P146 P150 P152"),
                IOStandard("LVDS_25")),
            Subsignal("data_n", Pins(
                "P107 P109 P113 P116 P120 P123 P127 P129 "
                "P133 P135 P138 P140 P145 P147 P151 P153"),
                IOStandard("LVDS_25")),
            Subsignal("data_clk_p", Pins("P93"), IOStandard("LVDS_25")),
            Subsignal("data_clk_n", Pins("P94"), IOStandard("LVDS_25")),
        ),

        ("dac", 1,
            Subsignal("clk_p", Pins("P68"), IOStandard("LVCMOS25")),
            Subsignal("clk_n", Pins("P69"), IOStandard("LVCMOS25")),
            Subsignal("data_p", Pins(
                "P160 P162 P164 P167 P171 P82 P177 P180 "
                "P74 P185 P189 P192 P196 P199 P202 P205"),
                IOStandard("LVDS_25")),
            Subsignal("data_n", Pins(
                "P161 P163 P165 P168 P172 P83 P178 P181 "
                "P75 P186 P190 P193 P197 P200 P203 P206"),
                IOStandard("LVDS_25")),
            Subsignal("data_clk_p", Pins("P77"), IOStandard("LVDS_25")),
            Subsignal("data_clk_n", Pins("P78"), IOStandard("LVDS_25")),
        ),

        ("dac", 2,
            Subsignal("clk_p", Pins("P62"), IOStandard("LVCMOS25")),
            Subsignal("clk_n", Pins("P63"), IOStandard("LVCMOS25")),
            Subsignal("data_p", Pins(
                "P2 P4 P8 P11 P15 P18 P22 P24 "
                "P28 P30 P33 P35 P39 P41 P47 P49"),
                IOStandard("LVDS_25")),
            Subsignal("data_n", Pins(
                "P3 P5 P9 P12 P16 P19 P23 P25 "
                "P29 P31 P34 P36 P40 P42 P48 P50"),
                IOStandard("LVDS_25")),
            Subsignal("data_clk_p", Pins("P64"), IOStandard("LVDS_25")),
            Subsignal("data_clk_n", Pins("P65"), IOStandard("LVDS_25")),
        ),
]


class Platform(XilinxPlatform):
    """PDQ Platform.

        * Xilinx Spartan 3A 500E in a PQ208 package.
        * 50 MHz single ended input clock.
        * Single FT245R USB parallel FIFO.
        * Three 16 bit LVDS DACs.
        * Several TTL control lines.
    """
    default_clk_name = "clk50"
    default_clk_period = 20

    def __init__(self):
        XilinxPlatform.__init__(self, "xc3s500e-4pq208", _io)
        self.toolchain.xst_opt = """-ifmt MIXED
-bram_utilization_ratio -1
-opt_level 2
-opt_mode SPEED
-register_balancing yes"""
        self.toolchain.bitgen_opt += (" -g GTS_cycle:3 -g LCK_cycle:4 "
                                      "-g GWE_cycle:5 -g DONE_cycle:6")
        self.toolchain.ise_commands += """
trce -v 12 -fastpaths -o {build_name} {build_name}.ncd {build_name}.pcf
promgen -w -spi -c FF -p mcs -o {build_name}.mcs -u 0 {build_name}.bit
"""
