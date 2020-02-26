
from json_parsing import read_json



# TODO: this function is a member of the inmoov object

def execute_animation(animation_id):
	
	#Open up the JSON and get the high level dictionary containing a list of poses to execute
	animation_data = {}
	animation_data = read_json(animation_id)
	
	print("Executing animation ", str(animation_id))
	
	#Top level dict keys: pose ID
	#Top level value: Another dictionary of servo IDS and PWMs
	for key, servo_sequence_dictionary in animation_data.items():
		print("Executing pose ", str(key))
		
		#Loop through the individual gester, setting each servo to its associated PWM for the current gesture
		for pose_id, gesture_time_list in servo_sequence_dictionary.items():
			
			#This is a string that identifies the pose to do
			pose_name = gesture_time_list[0]
			
			#Int value defining time to hold the pose
			sleep_time = gesture_time_list[1]
			
			#Open up the file containing for the pose data
			pose_file_data = read_json("gestures.json")
			
			pose_data = {}
			pose_data = pose_file_data[pose_name]
			
			for servo_id, servo_angle in pose_data.items():
				
	
				#Obtain a handle to the actual servo object 
				servo = self.find_servo_by_name(str(servo_id))
				servo.rotate(servo_angle)
		
		
			time.sleep(sleep_time)
		
	
	
	return