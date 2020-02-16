import json

my_file = 'simple.json'

def read_json(filename):
    with open(filename) as f:
      data = json.load(f)
      return data

def add_object_to_json(filename, gesture_name, servo):
    data = read_json(filename)
    data[gesture_name] = servo
    with open(filename, 'w') as json_file:
        json.dump(data, json_file)

def remove_object_from_json(filename, gesture_name):
    data = read_json(filename)
    del data[gesture_name]
    with open(filename, 'w') as json_file:
        json.dump(data, json_file)

def pretty_print(filename):
    print(json.dumps(read_json(filename), indent=4, sort_keys=True))
