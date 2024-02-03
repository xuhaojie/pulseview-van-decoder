##
## This file is part of the libsigrokdecode project.
##
## Copyright (C) 2012-2013 Uwe Hermann <uwe@hermann-uwe.de>
## Copyright (C) 2019 Stephan Thiele <stephan.thiele@mailbox.org>
##
## This program is free software; you van redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation; either version 2 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program; if not, see <http://www.gnu.org/licenses/>.
##

import sigrokdecode as srd

class SamplerateError(Exception):
    pass

def dlc2len(dlc):
    return [0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64][dlc]

class Decoder(srd.Decoder):
    api_version = 3
    id = 'van'
    name = 'VAN'
    longname = 'Vehicle Area Network'
    desc = 'Field bus protocol for distributed realtime control.'
    license = 'gplv2+'
    inputs = ['logic']
    outputs = []
    tags = ['Automotive']
    channels = (
        {'id': 'van_rx', 'name': 'VAN RX', 'desc': 'VAN bus line'},
    )
    options = (
        {'id': 'nominal_bitrate', 'desc': 'Nominal bitrate (bits/s)', 'default': 125000},
        {'id': 'sample_point', 'desc': 'Sample point (%)', 'default': 70.0},
    )
    annotations = (
        ('data', 'VAN payload data'),
        ('sof', 'Start of frame'),
        ('eof', 'End of frame'),
        ('id', 'Identifier'),
        ('ext', 'Ext'),
        ('rak', 'RAK'),
        ('rw', 'R/W'),
        ('rtr', 'RTR'),
        ('crc', 'CRC'),
        ('eod', 'End of datat'),
        ('ack', 'ACK'),
        ('crc-sequence', 'CRC sequence'),
        ('crc-delimiter', 'CRC delimiter'),
        ('ack-slot', 'ACK slot'),
        ('ack-delimiter', 'ACK delimiter'),
        ('stuff-bit', 'Stuff bit'),
        ('warnings', 'Human-readable warnings'),
        ('bit', 'Bit'),
    )

    annotation_rows = (
        ('bits', 'Bits', (15, 17)),
        ('fields', 'Fields', tuple(range(15))),
        ('warnings', 'Warnings', (16,)),
    )

    def __init__(self):
        self.reset()

    def reset(self):
        self.samplerate = None
        self.reset_variables()

    def start(self):
        self.out_ann = self.register(srd.OUTPUT_ANN)

    def set_bit_rate(self, bitrate):
        self.bit_width = float(self.samplerate) / float(bitrate)
        self.sample_point = (self.bit_width / 100.0) * self.options['sample_point']

    def set_nominal_bitrate(self):
        self.set_bit_rate(self.options['nominal_bitrate'])

    def metadata(self, key, value):
        if key == srd.SRD_CONF_SAMPLERATE:
            self.samplerate = value
            self.bit_width = float(self.samplerate) / float(self.options['nominal_bitrate'])
            self.sample_point = (self.bit_width / 100.0) * self.options['sample_point']

    # Generic helper for VAN bit annotations.
    def putg(self, ss, es, data):
        left, right = int(self.sample_point), int(self.bit_width - self.sample_point)
        self.put(ss - left, es + right, self.out_ann, data)

    # Single-VAN-bit annotation using the current samplenum.
    def putx(self, data):
        self.putg(self.samplenum, self.samplenum, data)
    
    # Multi-VAN-bit annotation from self.ss_block to current samplenum.
    def putb(self, data):
        self.putg(self.ss_block, self.samplenum, data)

    # Multi-VAN-bit annotation from self.ss_block to current samplenum.
    def putb_pre(self, data):
        self.putg(self.ss_block, self.samplenum - int(self.bit_width), data)

    def reset_variables(self):
        self.state = 'IDLE'
        self.rawbits = [] # All bits, including stuff bits
        self.bits = [] # Only actual VAN frame bits (no stuff bits)
        self.data = []
        self.crc = None
        self.ss_crc_block = None
        self.curbit = 0 # Current bit of VAN frame (bit 0 == SOF)
        self.last_databit = 30 # Positive value that bitnum+x will never match
        self.ack_bit= 9999
        self.ss_block = None
        #self.ss_data_strid = None
        #self.ss_data_block = None
        self.data_blocks = []

    # Poor man's clock synchronization. Use signal edges which change to
    # dominant state in rather simple ways. This naive approach is neither
    # aware of the SYNC phase's width nor the specific location of the edge,
    # but improves the decoder's reliability when the input signal's bitrate
    # does not exactly match the nominal rate.
    def dom_edge_seen(self, force = False):
        self.dom_edge_snum = self.samplenum
        self.dom_edge_bcount = self.curbit

    def bit_sampled(self):
        # EMPTY
        pass

    # Determine the position of the next desired bit's sample point.
    def get_sample_point(self, bitnum):
        samplenum = self.dom_edge_snum
        samplenum += self.bit_width * (bitnum - self.dom_edge_bcount)
        samplenum += self.sample_point
        return int(samplenum)

    def is_stuff_bit(self):
        bit = len(self.rawbits) -1
        if bit <= 10 :
            return False
        
        elif (bit % 5) == 4:
            return True
        
        return False

    def is_valid_crc(self, crc_bits):
        return True # TODO

    def handle_bit(self, van_rx):
        self.rawbits.append(van_rx)
        bitnum = len(self.rawbits) - 1

        if self.is_stuff_bit():
            self.putx([15, [str(van_rx)]])
        else:
            self.putx([17, [str(van_rx)]])
            self.bits.append(van_rx)

        if bitnum == 0:
            self.ss_block = self.samplenum
              
        elif bitnum == 9:
            self.putb([1, ['Start of frame', 'SOF', 'S']])
            sof = int(''.join(str(d) for d in self.bits[0:10]), 2)
            self.bits.clear()
            if sof != 0x3D:            
                s = "Error! SOF = 0x%X, should be 0x3D" % sof
                self.putb([16, [s]])

        elif bitnum == 10:
            self.ss_block = self.samplenum

        elif bitnum == 24:
            self.id = int(''.join(str(d) for d in self.bits[0:]), 2)
            s = '%d (0x%X)' % (self.id, self.id),
            self.putb([3, ['Identifier: %s' % s, 'ID: %s' % s, 'ID']])
            self.bits.clear()

        elif bitnum == 25:
            self.putx([4, ['EXT']])

        elif bitnum == 26:
            self.putx([5, ['RAK']])

        elif bitnum == 27:
            self.putx([6, ['R/W']])

        elif bitnum == 28:
            self.putx([7, ['RTR']])
        
        
        elif bitnum == self.last_databit:
            self.ss_block = self.samplenum
            
        elif bitnum == self.last_databit + 9:
            
            self.data_blocks.append((self.ss_block, self.samplenum))
            byte = int(''.join(str(d) for d in self.bits), 2)
            self.bits.clear()
            raw = int(''.join(str(d) for d in self.rawbits[-8:]), 2)
            if (raw & 0x03) > 0:
                self.ss_crc_block = self.ss_block
                #self.putb([0, ['Data[%d]=0x%02X' % (i,byte), 'D[%d]=0x%02x' % (i,byte), 'D']])
                self.last_databit += 10
                self.data.append(byte)
            else: # 遇到EOD
                b = self.data.pop()
                crc = (b << 7) + (byte >> 1 )
                self.crc = crc
                self.ss_block = self.ss_crc_block
                for i in range(len(self.data)):
                    byte = self.data[i]
                    (ss,es) = self.data_blocks[i]
                    self.putg(ss, es, [0, ['Data[%d]=0x%02X' % (i,byte), 'D[%d]=0x%02x' % (i,byte), 'D']])
                    ss = es

                ss = self.ss_crc_block
                es = self.samplenum - 2*int(self.bit_width)
                self.putg(ss, es, [8, ['CRC=0x%04X' % crc, 'C=0x%04x' % crc, 'C']])
                
                
                ss = self.samplenum - int(self.bit_width)
                es = self.samplenum
                self.putg(ss,es,[9, ['EOD']])
                self.ack_bit = self.last_databit + 10
            
        elif bitnum == self.ack_bit:
            self.ss_block = self.samplenum
        elif bitnum == self.ack_bit + 1:
            self.putb([10, ['ACK']])
        elif bitnum == self.ack_bit + 2:
            self.putx([2, ['EOF']])
            self.reset_variables()
            return True

        self.curbit += 1

    def decode(self):
        if not self.samplerate:
            raise SamplerateError('Cannot decode without samplerate.')

        while True:
            # State machine.
            if self.state == 'IDLE':
                # Wait for a dominant state (logic 0) on the bus.
                (van_rx,) = self.wait({0: 'l'})
                #self.sof = self.samplenum
                self.dom_edge_seen(force = True)
                self.state = 'GET BITS'
            elif self.state == 'GET BITS':
                # Wait until we're in the correct bit/sampling position.
                pos = self.get_sample_point(self.curbit)
                (van_rx,) = self.wait([{'skip': pos - self.samplenum}, {0: 'f'}])
                if self.matched[1]:
                    self.dom_edge_seen()
                if self.matched[0]:
                    self.handle_bit(van_rx)
                    self.bit_sampled()
