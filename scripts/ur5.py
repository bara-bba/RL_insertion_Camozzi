import socket
import sys

counter = 0

HOST = '' 				# Symbolic name, meaning all available interfaces
PORT = 30000			# Arbitrary non-privileged port

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

print('Socket created')

try:
	s.bind((HOST, PORT))
except socket.error as msg:
	print('Bind failed. Error Code : ' + str(msg[0]) + ' Message ' + msg[1])
	sys.exit()

print('Socket bind complete')

# LISTENING
s.listen(5)
print('Socket now listening')

conn, addr = s.accept()
print('Connected with ' + addr[0] + ':' + str(addr[1]))

print('')
print('-----------------------------------------------')
print('')

mode = 0
t = 0

while True:

	mode = int.from_bytes(conn.recv(1), "big")
	# print("MODE: " + str(mode))

	# WAITING MODE
	if mode==0 and t==0:
		str = conn.send(str(0.01,0.0,0.03,0.0,0.0,0.0).encode("ascii")) # Meters
		t=1
		print(str)
		print("Action sent...")

	# ACTION SENT
	elif mode==1:
		print("Waiting...")
		try:
			mode = conn.recv(1)
		except:
			print("Error 1")

	# ACTION DONE
	elif mode==2:
		print("Action done.")
		t=0
		#conn.recvfrom() # Recieve Observations
		mode = 0

	# ERROR
	# else:
	# 	s.close()
	# 	print("ERROR... Closing the connection")
	# 	break

s.close()

# def readSensor():
	# get_actual_tcp_pose() + get_target_tcp_pose?	"(X, Y, Z, Rx, Ry, Rz)" WRT the base frame
	# get_actual_tcp_speed()						"(X, Y, Z, Rx, Ry, Rz)" WRT the base frame
	# get_tcp_force()								"(Fx. Fy, Fz, TRx, TRy, TRz)" WRT the base frame MAX: 300N