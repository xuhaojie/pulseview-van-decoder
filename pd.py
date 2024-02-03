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
        ('rtr0', 'Remote transmission request'),
        ('srr', 'Substitute remote request'),
        ('dlc', 'Data length count'),
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

    # Single-VAN-bit annotation using the samplenum of VAN bit 12.
    def put12(self, data):
        self.putg(self.ss_bit12, self.ss_bit12, data)

    # Single-VAN-bit annotation using the samplenum of VAN bit 32.
    def put32(self, data):
        self.putg(self.ss_bit32, self.ss_bit32, data)

    # Multi-VAN-bit annotation from self.ss_block to current samplenum.
    def putb(self, data):
        self.putg(self.ss_block, self.samplenum, data)

    def reset_variables(self):
        self.decode_state = "VAN_SOF1"
        self.bit_count = 0
        self.state = 'IDLE'
        self.sof = None
        self.rawbits = [] # All bits, including stuff bits
        self.bits = [] # Only actual VAN frame bits (no stuff bits)
        self.data_count = 0
        self.curbit = 0 # Current bit of VAN frame (bit 0 == SOF)
        self.last_databit = 30 # Positive value that bitnum+x will never match
        self.ss_block = None
        self.ss_bit12 = None
        self.ss_bit32 = None
        self.ss_databytebits = []
        self.fd = False
        self.rtr = None

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

    def decode_error_frame(self, bits):
        pass # TODO

    def decode_overload_frame(self, bits):
        pass # TODO

    # Both standard and extended frames end with CRC, CRC delimiter, ACK,
    # ACK delimiter, and EOF fields. Handle them in a common function.
    # Returns True if the frame ended (EOF), False otherwise.
    def decode_frame_end(self, van_rx, bitnum):

        # Remember start of CRC sequence (see below).
        if bitnum == (self.last_databit + 1):
            self.ss_block = self.samplenum
            if self.fd:
                if dlc2len(self.dlc) < 16:
                    self.crc_len = 27 # 17 + SBC + stuff bits
                else:
                    self.crc_len = 32 # 21 + SBC + stuff bits
            else:
                self.crc_len = 15

        # CRC sequence (15 bits, 17 bits or 21 bits)
        elif bitnum == (self.last_databit + self.crc_len):
            if self.fd:
                if dlc2len(self.dlc) < 16:
                    crc_type = "CRC-17"
                else:
                    crc_type = "CRC-21"
            else:
                crc_type = "CRC-15"

            x = self.last_databit + 1
            crc_bits = self.bits[x:x + self.crc_len + 1]
            self.crc = int(''.join(str(d) for d in crc_bits), 2)
            self.putb([11, ['%s sequence: 0x%04x' % (crc_type, self.crc),
                            '%s: 0x%04x' % (crc_type, self.crc), '%s' % crc_type]])
            if not self.is_valid_crc(crc_bits):
                self.putb([16, ['CRC is invalid']])

        # CRC delimiter bit (recessive)
        elif bitnum == (self.last_databit + self.crc_len + 1):
            self.putx([12, ['CRC delimiter: %d' % van_rx,
                            'CRC d: %d' % van_rx, 'CRC d']])
            if van_rx != 1:
                self.putx([16, ['CRC delimiter must be a recessive bit']])

            if self.fd:
                self.set_nominal_bitrate()

        # ACK slot bit (dominant: ACK, recessive: NACK)
        elif bitnum == (self.last_databit + self.crc_len + 2):
            ack = 'ACK' if van_rx == 0 else 'NACK'
            self.putx([13, ['ACK slot: %s' % ack, 'ACK s: %s' % ack, 'ACK s']])

        # ACK delimiter bit (recessive)
        elif bitnum == (self.last_databit + self.crc_len + 3):
            self.putx([14, ['ACK delimiter: %d' % van_rx,
                            'ACK d: %d' % van_rx, 'ACK d']])
            if van_rx != 1:
                self.putx([16, ['ACK delimiter must be a recessive bit']])

        # Remember start of EOF (see below).
        elif bitnum == (self.last_databit + self.crc_len + 4):
            self.ss_block = self.samplenum

        # End of frame (EOF), 7 recessive bits
        elif bitnum == (self.last_databit + self.crc_len + 10):
            self.putb([2, ['End of frame', 'EOF', 'E']])
            if self.rawbits[-7:] != [1, 1, 1, 1, 1, 1, 1]:
                self.putb([16, ['End of frame (EOF) must be 7 recessive bits']])
            self.reset_variables()
            return True

        return False

    # Returns True if the frame ended (EOF), False otherwise.
    def decode_standard_frame(self, van_rx, bitnum):

        # Bit 14: FDF (Flexible data format)
        # Has to be sent dominant when FD frame, has to be sent recessive
        # when classic VAN frame.
        if bitnum == 24:
            self.fd = True if van_rx else False
            if self.fd:
                self.putx([7, ['Flexible data format: %d' % van_rx,
                               'FDF: %d' % van_rx, 'FDF']])
            else:
                self.putx([7, ['Reserved bit 0: %d' % van_rx,
                               'RB0: %d' % van_rx, 'RB0']])

            if self.fd:
                # Bit 12: Substitute remote request (SRR) bit
                self.put12([8, ['Substitute remote request', 'SRR']])
                self.dlc_start = 18
            else:
                # Bit 12: Remote transmission request (RTR) bit
                # Data frame: dominant, remote frame: recessive
                # Remote frames do not contain a data field.
                rtr = 'remote' if self.bits[12] == 1 else 'data'
                self.put12([8, ['Remote transmission request: %s frame' % rtr,
                                'RTR: %s frame' % rtr, 'RTR']])
                self.dlc_start = 15

        if bitnum == 25 and self.fd:
            self.putx([7, ['Reserved: %d' % van_rx, 'R0: %d' % van_rx, 'R0']])

        if bitnum == 26 and self.fd:
            self.putx([7, ['Bit rate switch: %d' % van_rx, 'BRS: %d' % van_rx, 'BRS']])

        if bitnum == 27 and self.fd:
            self.putx([7, ['Error state indicator: %d' % van_rx, 'ESI: %d' % van_rx, 'ESI']])

        # Remember start of DLC (see below).
        elif bitnum == self.dlc_start:
            self.ss_block = self.samplenum

        # Bits 15-18: Data length code (DLC), in number of bytes (0-8).
        elif bitnum == self.dlc_start + 3:
            self.dlc = int(''.join(str(d) for d in self.bits[self.dlc_start:self.dlc_start + 4]), 2)
            self.putb([10, ['Data length code: %d' % self.dlc,
                            'DLC: %d' % self.dlc, 'DLC']])
            self.last_databit = self.dlc_start + 3 + (dlc2len(self.dlc) * 8)
            if self.dlc > 8 and not self.fd:
                self.putb([16, ['Data length code (DLC) > 8 is not allowed']])

        # Remember all databyte bits, except the very last one.
        elif bitnum in range(self.dlc_start + 4, self.last_databit):
            self.ss_databytebits.append(self.samplenum)

        # Bits 19-X: Data field (0-8 bytes, depending on DLC)
        # The bits within a data byte are transferred MSB-first.
        elif bitnum == self.last_databit:
            self.ss_databytebits.append(self.samplenum) # Last databyte bit.
            for i in range(dlc2len(self.dlc)):
                x = self.dlc_start + 4 + (8 * i)
                b = int(''.join(str(d) for d in self.bits[x:x + 8]), 2)
                ss = self.ss_databytebits[i * 8]
                es = self.ss_databytebits[((i + 1) * 8) - 1]
                self.putg(ss, es, [0, ['Data byte %d: 0x%02x' % (i, b),
                                       'DB %d: 0x%02x' % (i, b), 'DB']])
            self.ss_databytebits = []

        elif bitnum > self.last_databit:
            return self.decode_frame_end(van_rx, bitnum)

        return False

    def handle_bit(self, van_rx):
        self.rawbits.append(van_rx)


        # Get the index of the current VAN frame bit (without stuff bits).
        bitnum = len(self.rawbits) - 1

        # If this is a stuff bit, remove it from self.bits and ignore it.
        if self.is_stuff_bit():
            self.putx([15, [str(van_rx)]])
        else:
            self.putx([17, [str(van_rx)]])
            self.bits.append(van_rx)


        # Bit 0: Start of frame (SOF) bit

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

        # Bits 1-11: Identifier (ID[10..0])
        # The bits ID[10..4] must NOT be all recessive.
        elif bitnum == 24:
            self.id = int(''.join(str(d) for d in self.bits[0:]), 2)
            s = '%d (0x%x)' % (self.id, self.id),
            self.putb([3, ['Identifier: %s' % s, 'ID: %s' % s, 'ID']])
            self.bits.clear()
            if (self.id & 0x7f0) == 0x7f0:
                self.putb([16, ['Identifier bits 10..4 must not be all recessive']])

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
            i = self.data_count
            b = int(''.join(str(d) for d in self.bits), 2)
            self.putb([0, ['Data[%d]=0x%02x' % (i,b), 'D[%d]=0x%02x' % (i,b), 'DB']])
            self.data_count += 1
            self.last_databit += 10
            self.bits.clear()


        # Bits 14-X: Frame-type dependent, passed to the resp. handlers.




        # After a frame there are 3 intermission bits (recessive).
        # After these bits, the bus is considered free.

        self.curbit += 1

    def decode(self):
        if not self.samplerate:
            raise SamplerateError('Cannot decode without samplerate.')

        while True:
            # State machine.
            if self.state == 'IDLE':
                # Wait for a dominant state (logic 0) on the bus.
                (van_rx,) = self.wait({0: 'l'})
                self.sof = self.samplenum
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
