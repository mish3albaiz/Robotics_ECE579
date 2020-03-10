import json

def read_json(filename):
    # types of what it returns are json standard
    # lists, dicts, strings, bools, ints, floats are unclear
    with open(filename) as f:
      data = json.load(f)
      return data

def add_object_to_json(filename, gesture_name, servo):
    # assumes json top-level is dict structure
    # adds an entry to the dict with key=gesture_name and value=servo
    # because it assigns to the dict, this works as both add and as overwrite
    data = read_json(filename)
    if gesture_name in data:
        print("Warning: item '%s' already exists in the json, overwriting it" % gesture_name)
    data[gesture_name] = servo
    with open(filename, 'w') as json_file:
        json.dump(data, json_file, indent=4)

def remove_object_from_json(filename, gesture_name):
    # assumes json top-level is dict structure
    # deletes item with key=gesture_name
    data = read_json(filename)
    del data[gesture_name]
    with open(filename, 'w') as json_file:
        json.dump(data, json_file, indent=4)

def pretty_print(filename):
    # just print
    print(json.dumps(read_json(filename), indent=4, sort_keys=True))

def change_object_from_json(filename, gesture_name, servo_name, servo_value):
    # assumes json top-level is dict structure
    # within one gesture, change the value of one servo
    data = read_json(filename)
    data[gesture_name][servo_name] = servo_value
    with open(filename, 'w') as json_file:
        json.dump(data, json_file, indent=4)
    
    
    
