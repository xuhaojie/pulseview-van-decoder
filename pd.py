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
		('sof', 'Start of frame'), # 0
		('id', 'Identifier'), # 1
		('ext', 'Ext'), # 2
		('rak', 'RAK'), # 3 
		('rw', 'R/W'), # 4
		('rtr', 'RTR'), # 5
		('data', 'VAN payload data'),  # 6
		('crc', 'CRC'), # 7
		('eod', 'End of datat'), # 8
		('ack', 'ACK'), # 9
		('eof', 'End of frame'), # 10
		('bit', 'Bit'), # 11
		('stuff-bit', 'Stuff bit'), # 12
		('ack-bit', 'Ack bit'), # 13
		('eof-bit', 'EOF bit'), # 14
		('warnings', 'Human-readable warnings'), # 15
	)

	annotation_rows = (
		('bits', 'Bits', (11, 12, 13,14)),
		('fields', 'Fields', tuple(range(11))),
		('warnings', 'Warnings', (15,)),
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

	def reset_variables(self):
		self.state = 'IDLE'
		self.rawbits = [] # All bits, including stuff bits
		self.bits = [] # Only actual VAN frame bits (no stuff bits)
		self.bit_groups = []
		self.curbit = 0 # Current bit of VAN frame (bit 0 == SOF)
		self.ss_block = None
		self.data_blocks = []
		self.done = False

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
		
	def decode_frame(self):
		count = 0
		begin = 0
		end = 0
		byte = 0
		groups = len(self.bit_groups)
		for i in range(groups):
			(ss,es,bits) = self.bit_groups[i]
			if count == 0:
				begin = ss
				byte =  int(''.join(str(d) for d in bits), 2)
				count = 1
			elif count == 1:
				count = 0
				byte <<= 4
				byte +=  int(''.join(str(d) for d in bits), 2)
				end = es
			if i == 1:
				self.putg(begin, end, [0, ['Start of frame', 'SOF', 'S']])	
				if byte != 0x0E:            
					s = "Error! SOF = 0x%X, should be 0x0E" % byte
					self.putg(begin, end,[15, [s]])
					
			elif i == 4:
				id = int(''.join(str(d) for d in self.bits[8:20]), 2)
				s = '%d (0x%X)' % (id, id)
				(begin,_,_) = self.bit_groups[2]
				(_,end,_) = self.bit_groups[4]
				self.putg(begin, end, [1,  ['Identifier: %s' % s, 'ID: %s' % s, 'ID']])	
			elif i == 5:
				begin=ss
				self.putg(begin,begin,[2, ['EXT']])

				begin +=  int(self.bit_width)
				self.putg(begin,begin,[3, ['RAK']])

				begin +=  int(self.bit_width)
				self.putg(begin,begin,[4, ['R/W']])

				begin +=  int (self.bit_width)
				self.putg(begin,begin,[5, ['RTR']])

			elif i >= 7 and i < (groups - 4) and i %2 == 1:
				index = int((i - 7 ) / 2)
				self.putg(begin, end, [6, ['Data[%d]=0x%02X' % (index,byte), 'D[%d]=0x%02x' % (index,byte), 'D']])
			
			elif i == groups - 3:
				(begin,_,_) = self.bit_groups[-4]
				(_,end,_) = self.bit_groups[-1]
				end -= 2*int(self.bit_width)
				crc = int(''.join(str(d) for d in self.bits[-16:-1]), 2)
				self.putg(begin, end, [7, ['CRC=0x%04X' % crc, 'C=0x%04x' % crc, 'C']])

			elif i == groups - 1:
				end = es
				begin = end - int(self.bit_width)
				self.putg(begin,end,[8, ['EOD']])
				self.done = True

		return True				

	def handle_bit(self, van_rx):
		self.rawbits.append(van_rx)

		bitnum = len(self.rawbits) -1
		


		if self.done:
					
			groups = len(self.bit_groups) 
			if bitnum == groups * 5:
				self.putx([11, [str(van_rx)]])
				if van_rx != 1:
					self.putx([15, ['ACK delimitermust be a recessive bit']])
			elif bitnum == groups*5 + 1:
				self.putx([13, [str(van_rx)]])
				(_,es,_) = self.bit_groups[-1]
				begin = es + int(self.bit_width)
				end = begin + int(self.bit_width)
				self.putg(begin,end,[9, ['ACK']])
			elif bitnum == groups*5 + 2:
				self.putx([14, [str(van_rx)]])
				self.ss_block = self.samplenum
			elif bitnum == groups*5 + 3:
				self.putx([14, [str(van_rx)]])
			elif bitnum == groups*5 + 4:
				self.putx([14, [str(van_rx)]])
				self.putg(self.ss_block, self.samplenum,[10, ['EOF']])
				if van_rx != 1:
					self.putg(self.ss_block, self.samplenum,[15, ['End of frame (EOF) must be a recessive bit']])						
				self.reset_variables()
				return True			
		else:
			stuff_bit = bitnum % 5 == 4

			if stuff_bit:
				self.putx([12, [str(van_rx)]])
			else:
				self.putx([11, [str(van_rx)]])
				self.bits.append(van_rx
					 )			
			if bitnum % 5 == 0:
				self.ss_block = self.samplenum
			elif stuff_bit:
				bits = self.bits[-4:]
				self.bit_groups.append((self.ss_block, self.samplenum, bits))
				raw = int(''.join(str(d) for d in self.rawbits[-5:]), 2)
				if raw & 3 == 0:
					self.decode_frame()

		self.curbit += 1

	def decode(self):
		if not self.samplerate:
			raise SamplerateError('Cannot decode without samplerate.')

		while True:
			# State machine.
			if self.state == 'IDLE':
				# Wait for a dominant state (logic 0) on the bus.
				(van_rx,) = self.wait({0: 'l'})
				self.dom_edge_seen(force = True)
				self.state = 'GET_BITS'
			elif self.state == 'GET_BITS':
				# Wait until we're in the correct bit/sampling position.
				pos = self.get_sample_point(self.curbit)
				(van_rx,) = self.wait([{'skip': pos - self.samplenum}, {0: 'f'}])
				if self.matched[1]:
					self.dom_edge_seen()
				if self.matched[0]:
					self.handle_bit(van_rx)
					self.bit_sampled()
