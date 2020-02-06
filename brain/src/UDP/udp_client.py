import socket
import sys

def main():
	stand_page_number = 132
	command_sent = 1
	#posture_file = open("../cat_robot_posture.txt", "r")
	robot_info = {}
	command_list = {}
	# Open jimmy info file and create dictionary
	# Currently this project only uses one robot
	with open("robot_info.txt","r") as fin:
		for line in fin:
			line = line.strip('\n')
			parts = line.split(',')
			temp = {}
			temp['host'] = parts[1]
			temp['port'] = int(parts[2])
			robot_info[parts[0]] = temp
	
	#print(robot_info)
	#print("info above")
	previous_data = 99
	
	response = "a"
	#print(robot_info)
	#print(temp['host'])
	#print(temp['port'])
	#print("test")
	# result.txt is the output file of the OpenCV C++ program, it will
	# contain the command number to be sent over to the robot
	while response != "q":
		command_file = open("../result.txt", "r")
		data = command_file.readline()
		command_file.close()
		#print(data)
		#print(previous_data)

		try:
			if data != previous_data:
				command_sent = 0
				print(data)
				print(previous_data)
				previous_data = data

				#print(data)
		except:
			pass
		
		host = temp['host']
		HOST = '{}'.format(host)
		PORT = temp['port'] 

		#print(command_sent)		
		try:
			# Try to open a UDP connection, and send the command to the robot
			if int(command_sent) == 0:
				#print("t1")
				sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
				#print("t2")
				# 3 is the command used by Motion script to play a page number from RME,
				# this could be extended to send other kinds of commands as well, such as walk
				# or turn. Assign stand_page_number above for what page number you want to send over
				#command = "%d" % (data)
				print(HOST)
				print(PORT)
				sock.sendto(bytes(data, "utf-8"), (HOST, PORT))
				#print("t3")
				print(HOST)
				print(PORT)
				print("Sent:     %s" % (data))
				command_sent = 1
		except:
			pass


if __name__ == "__main__":
	main()
