#! /usr/bin/env python
import rospy, time
# Sockets
import socket, struct

class LabviewServer:
	def __init__(self):	
		# Set up socket
		self.master_ip = '138.100.76.211'
		self.master_port = 5555
		self.master_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.master_sock.bind((self.master_ip, self.master_port))
		rospy.loginfo('Socked listening on port [udp://%s:%d]' % (self.master_ip, self.master_port))
		
	def recv_timeout(self, timeout=0.001):
		self.master_sock.setblocking(0)
		total_data=[]
		data=''
		begin=time.time()
		while 1:
			#if you got some data, then break after wait sec
			if total_data and time.time()-begin>timeout:
				break
			#if you got no data at all, wait a little longer
			elif time.time()-begin>timeout*2:
				break
			try:
				data=self.master_sock.recv(8192)
				if data:
					total_data.append(data)
					begin=time.time()
			except:
				pass
		return ''.join(total_data)
		
		
if __name__ == '__main__':
	rospy.init_node('labview_server')
	server = LabviewServer()
	while not rospy.is_shutdown():
		data = server.recv_timeout()
		if data:
			fmt = '<IiiI5sIIssIssIssIssIssIssIssssIfffffff'
			msg = struct.unpack(fmt, data[:struct.calcsize(fmt)])
			print msg[-7:]
