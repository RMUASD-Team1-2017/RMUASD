#!/usr/bin/env python
#/****************************************************************************
# UBLOX SARA G3 Interface
# Copyright (c) 2017, Martin Skriver <maskr@mmmi.sdu.dk & maskr09@gmail.com
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#	* Redistributions of source code must retain the above copyright
#	  notice, this list of conditions and the following disclaimer.
#	* Redistributions in binary form must reproduce the above copyright
#	  notice, this list of conditions and the following disclaimer in the
#	  documentation and/or other materials provided with the distribution.
#   * Neither the name of the copyright holder nor the names of its
#	 contributors may be used to endorse or promote products derived from
#	 this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#****************************************************************************/
"""
Source for UBLOX SARA G340 01S-00 GSM modem
Connecting ans sending packages via UDP

Easy command line to send test UDP messages: rostopic pub /fmLib/udp_tx std_msgs/String "a2"

2017-06-10 MS First version
"""

import serial
import rospy
from std_msgs.msg import String

DEBUG = True

class modemDriver:
	def __init__(self):

		# read topic names
		self.udp_pub_topic = rospy.get_param("~udp_rx_pub",'/fmLib/udp_rx')
		self.udp_sub_topic = rospy.get_param("~udp_tx_sub",'/fmLib/udp_tx')

		# read launch file parameters
		self.device = rospy.get_param("~serial_device", '/dev/ttyUSB0')
		self.baud_rate = rospy.get_param("~serial_baudrate", 115200)
		self.apn= rospy.get_param("~gsm_apn", 'internet')
		self.server_name = rospy.get_param("~server_name", 'drone.stefanrvo.dk')
		self.server_port = rospy.get_param("~server_port", 1235)

		# Serial device status variables
		self.rx_buf = ''
		self.tx_buf = []

		# gsm modem status variables
		self.gsm_inited = False

		# Open serial device
		if self.open_serial_connnection() == False:

			# Server ports and info
			self.server_ip = ''

			# SARA G340 flags and variables
			self.CPIN_CHECK = False
			self.CGATT = 0
			self.PSD_APN_CONFIGURED = False
			self.PSD_IP_CONFIGURED = False
			self.PSA_PDP_ACTIVATED = False
			self.SOCKET_CREATED = False
			self.socket_no = '9'
			self.SERVER_IP_RESEIVED = False
			self.connection_established = False
			self.ready_to_send_data = True
			self.recieve_bytes_pending = '0'
			self.OK_RESPONSE = False
			self.server_ip = ""

			# Signal quality
			self.rssi = 99
			self.rssi_updated = False

			# Publisher
			self.pub = rospy.Publisher(self.udp_pub_topic, String, queue_size=1)


			# setup subscription topic callbacks
			rospy.Subscriber(self.udp_sub_topic, String, self.on_tx_topic)

			# Error flag
 			r = rospy.Rate(10)
			while (True):
				if self.gsm_inited == False:
					rospy.loginfo(rospy.get_name() + ": Resetting serial device")

					self.gsm_inited = self.gsm_modem_init()
				else:
					# if new messages ready for tx, send the first one
					self.update_par(20)
					rospy.sleep(0.01)
					self.retrieve_data_from_modem()
					self.get_gsm_signal_q()
					rospy.sleep(0.02)
					if len(self.tx_buf) > 0:
						self.send_msg_to_server(self.tx_buf[0])
						self.tx_buf.pop(0)

					# keep tx buffer length below threshold
						while len(self.tx_buf) > 10:
							self.tx_buf.pop(0)
				r.sleep()
		else:
			rospy.logerr(rospy.get_name() + ": Unable to open serial device: %s" % self.device)

	def on_tx_topic(self, msg):
		self.tx_buf.append(msg.data)

 	def gsm_modem_init(self):
		success = False
		self.gsm_error = False

	  	self.reset_modem()
		self.sync_gsm_modem()

		# Open socket to server
		if self.udp_socket_connect():
			success = True
		return success

	def publish_server_string(self, msg):
		self.pub.publish(msg)

	def get_gsm_signal_q(self):
		self.rssi = 99
		self.rssi_updated = False

		self.update_par(5)
		self.ser.write('AT+CSQ\r')
		self.update_par(100)

		if self.rssi_updated == False:
			if DEBUG:
				print('rssi not updated')
			#todo make warning

	def retrieve_data_from_modem(self):
		self.ser.write('AT+USORF='+self.socket_no+',0\r')
		self.update_par(100)

		if self.recieve_bytes_pending != '0':
			rospy.sleep(0.02)
			self.ser.write('AT+USORF=' + self.socket_no + ',' + self.recieve_bytes_pending + '\r')
			self.update_par(100)

	def send_msg_to_server(self, msg):
		self.ready_to_send_data = False
		if self.socket_no != '' and self.server_ip != '':
			self.ser.write('AT+USOST='+self.socket_no+','+self.server_ip+','+str(self.server_port)+','+str(len(msg))+'\r')

			self.update_par(500)

			if self.ready_to_send_data != False:
				rospy.sleep(0.02)
				self.ser.write(msg)
			else:
				self.gsm_inited = False
				rospy.loginfo(rospy.get_name() + ": GSM modem is restarting after failing opening msg!")

			self.update_par(500)
			if self.msg_send:
				self.msg_send = False


	def sync_gsm_modem(self):
		while self.OK_RESPONSE != True:
			if DEBUG:
				print ("Sync with gsm modem")
			self.ser.write("AT\r")
			rospy.sleep(0.1)
			while self.ser.inWaiting():
				print(self.ser.inWaiting())
				self.update_par(100)
			rospy.sleep(0.2)

	def udp_socket_connect(self):
		timeout_ms = 0
		max_retry = 0
		count = 0
		string_to_send = ''
		string_to_send_old = ''
		dont_send = False
		success = False
		stop_flag = False

		while self.connection_established != True and not stop_flag:
			rospy.sleep(0.1)

			self.update_par(timeout_ms)
			rospy.sleep(0.02)

			if self.CPIN_CHECK != True:
				if DEBUG:
					print ("Check pin code")
				string_to_send = 'AT+CPIN?\r'
				timeout_ms = 100
				max_retry = 5

			elif self.CGATT != 1:
				if DEBUG:
					print ("Check GPRS attach")
				string_to_send = 'AT+CGATT?\r'
				timeout_ms = 100
				max_retry = 150


			elif self.PSD_APN_CONFIGURED != True:
				string_to_send = 'AT+UPSD=0,1,\"'+self.apn+'\"\r'
				if string_to_send_old == string_to_send and self.OK_RESPONSE == True :
					self.OK_RESPONSE == False
					self.PSD_APN_CONFIGURED = True
					dont_send = True
				if DEBUG:
					print ("Check PSD APN configured")
				timeout_ms = 1000
				max_retry = 10

			elif self.PSD_IP_CONFIGURED != True:
				if DEBUG:
					print ("Check PSD IP configured")
				string_to_send = 'AT+UPSD=0,7,\"0.0.0.0\"\r' # dynamich IP assignment
				if string_to_send_old == string_to_send and self.OK_RESPONSE == True :
					self.OK_RESPONSE == False
					self.PSD_IP_CONFIGURED = True
					dont_send = True
				timeout_ms = 1000
				max_retry = 10

			elif self.PSA_PDP_ACTIVATED != True:
				if DEBUG:
					print ("Check PSD PDP activated")
				string_to_send = 'AT+UPSDA=0,3\r' # activate GPRS signal
				if string_to_send_old == string_to_send and self.OK_RESPONSE == True :
					self.OK_RESPONSE == False
					self.PSA_PDP_ACTIVATED = True
					dont_send = True
				timeout_ms = 3000
				max_retry = 10

			elif self.SOCKET_CREATED != True:
				if DEBUG:
					string_to_send = 'AT+USOCR=17\r' # use UDP protocol on port 5000
					print ("Check if socket is created")
				timeout_ms = 1000
				max_retry = 10

			elif self.SERVER_IP_RESEIVED != True:
				if DEBUG:
					print("Receive server IP from DNS")
				string_to_send = 'AT+UDNSRN=0,\"'+self.server_name+'\"\r'
				timeout_ms = 5000
				max_retry = 10

			else:
				self.connection_established = True
				dont_send = True
				success = True
				rospy.loginfo(rospy.get_name() + ": Server connected established")
				if DEBUG:
					print("Connected to server")

			if string_to_send_old == string_to_send:
				count = count+1
			else:
				count = 0
			if count > max_retry:
				if DEBUG:
					print ("time and retry error in connect")
				stop_flag = True
				rospy.logwarn(rospy.get_name() + ": GSM modem is restarting after failing: %s!", string_to_send)
				count = 0

			elif not dont_send:
				self.update_par(10)
				rospy.sleep(0.02)
				self.ser.write(string_to_send)
			string_to_send_old = string_to_send
			dont_send = False
		return success

	def get_line(self):
			self.get_serial_data()
			first_line = ''

			if self.rx_buf:
				while self.rx_buf and (self.rx_buf[0] == '\n' or self.rx_buf[0] == '\r') :
					self.rx_buf = self.rx_buf[1:]
				if self.rx_buf:
					if self.rx_buf[0] == '@':
						first_line = self.rx_buf[0]
						self.rx_buf = self.rx_buf[1:]
					elif self.rx_buf[:len('+USORF: '+self.socket_no+',' + self.server_ip + ',' + str(self.server_port) + ',' + self.recieve_bytes_pending)] == '+USORF: '+self.socket_no+',' + self.server_ip + ',' + str(self.server_port) + ',' + self.recieve_bytes_pending:
						first_line = self.rx_buf[:len('+USORF: '+self.socket_no+',' + self.server_ip + ',' + str(self.server_port) + ',' + self.recieve_bytes_pending + ',\"\"')+int(self.recieve_bytes_pending)]
						self.rx_buf = self.rx_buf[len('+USORF: '+self.socket_no+',' + self.server_ip + ',' + str(self.server_port) + ',' + self.recieve_bytes_pending + ',\"\"')+int(self.recieve_bytes_pending):]
					else:
						first_line = self.rx_buf[:self.rx_buf.find('\r')+1]
						self.rx_buf = self.rx_buf[self.rx_buf.find('\r')+1:]
			if first_line:
				if DEBUG:
					print(first_line)
			return first_line

	def update_par(self, time_out_ms):
			get_sec_line = False
			first_line = ''
			sec_line = ''
			msg_found = False
			timer = 0

			while not msg_found and time_out_ms > timer:
				if first_line == '' or get_sec_line == False:
					first_line = self.get_line()
				if get_sec_line and sec_line == '':
					sec_line = self.get_line()

				if first_line[:len('+CPIN: READY\r')] == '+CPIN: READY\r':
					get_sec_line = True
					if sec_line[:3] == 'OK\r':
						self.CPIN_CHECK = True
						msg_found = True

				elif first_line[:len('+CGATT: 1\r')] == '+CGATT: 1\r':
					get_sec_line = True
					if sec_line[:len('OK\r')] == 'OK\r':
						self.CGATT = 1
						msg_found = True

				elif first_line[:len('+USOCR: ')] == '+USOCR: ':
					self.socket_no = first_line[len('+USOCR: ')]
					get_sec_line = True
					if sec_line[:len('OK\r')] == 'OK\r':
						self.SOCKET_CREATED = True
						msg_found = True

				elif first_line[:len('+UDNSRN: ')] == '+UDNSRN: ':
					self.server_ip = first_line[len('+UDNSRN: '):len(first_line)-1]
					get_sec_line = True
					if sec_line[:len('OK\r')] == 'OK\r':
						self.SERVER_IP_RESEIVED = True
						msg_found = True

				elif first_line[:len('+USOST: ')] == '+USOST: ':
					get_sec_line = True
					if sec_line[:len('OK\r')] == 'OK\r':
						self.msg_send = True
						msg_found = True

				elif first_line[:1] == '@':
					self.ready_to_send_data = True
					msg_found = True

				elif first_line[:len('+CSQ: ')] == '+CSQ: ':
					get_sec_line = True
					if sec_line[:len('OK\r')] == 'OK\r':
						self.rssi = first_line[len('+CSQ: '):first_line.find(',')]
						self.rssi_updated = True
						msg_found = True

				elif first_line[:len('+USORF: '+self.socket_no+',')] == '+USORF: '+self.socket_no+',':
					get_sec_line = True
					if first_line[len('+USORF: '+self.socket_no+',') : len('+USORF: '+self.socket_no+','+self.server_ip)] == self.server_ip:
						if sec_line[:len('OK\r')] == 'OK\r':
							self.publish_server_string(first_line[len('+USORF: '+self.socket_no+','+self.server_ip+',' + str(self.server_port) +','+ self.recieve_bytes_pending + ',')+1:])
						self.recieve_bytes_pending = '0'
					else:
						self.recieve_bytes_pending = first_line[len('+USORF:'+self.socket_no+',')+1:-1]
					if sec_line[:len('OK\r')] == 'OK\r':
						if DEBUG:
							print (first_line[len('+USORF: '+self.socket_no+','+self.server_ip+',' + str(self.server_port) +','+ self.recieve_bytes_pending + ',')+1:])
						msg_found = True

				elif first_line[:len('OK\r')] == 'OK\r':
					self.OK_RESPONSE = True
					msg_found = True

				if not msg_found:
					timer = timer + 10
					rospy.sleep(0.010)

			if time_out_ms > timer and self.rx_buf:
				self.update_par(time_out_ms-timer)


	def get_serial_data(self):

		while self.ser.inWaiting():
			self.rx_buf = self.rx_buf + self.ser.read()

	def open_serial_connnection(self):
		ser_error = False
		try :
			self.ser = serial.Serial(self.device, self.baud_rate, timeout=0)
		except Exception as e:
			ser_error = True
		return ser_error

	def reset_modem(self):
		if DEBUG:
			print ('Reset GSM modem')
		self.rx_buf = ''
		self.OK_RESPONSE = False
		self.sync_gsm_modem()
		self.ser.write("AT+CFUN=1,1\r")
		rospy.sleep(4)
		self.ser.write("ATE0\r")

		self.server_ip = ''
		self. CPIN_CHECK = False
		self.CGATT = 0
		self.PSD_APN_CONFIGURED = False
		self.PSD_IP_CONFIGURED = False
		self.PSA_PDP_ACTIVATED = False
		self.SOCKET_CREATED = False
		self.socket_no = ''
		self.SERVER_IP_RESEIVED = False
		self.connection_established = False
		self.OK_RESPONSE = False

		rospy.sleep(1)

	def gsm_modem_pwr_off(self):
		self.ser.write("AT+CPWROFF\r")
		# Don't do it for v1 print!

# Main function.
if __name__ == '__main__':
	# Initialize the node and name it.
	rospy.init_node('gsm_modem')

	# Go to class functions that do all the heavy lifting. Do error checking.
	try:
		node_class = modemDriver()
	except rospy.ROSInterruptException:
		pass
