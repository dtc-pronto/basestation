import requests

import os
from dotenv import load_dotenv
import ast
import rospy

#TODO: add casulty_id, team, system, time_ago to submit_scorecard and submit_image functions
def submit_image(image_path, time, id):
    # Load .env file
    load_dotenv()

    TOKEN = os.getenv("TOKEN")
    BASE_URL = os.getenv("BASE_URL")

    headers = {
        "accept": "application/json",
        "Authorization": TOKEN,
    }

    files = {
        'file': open(image_path, 'rb')
    }

    data = {
        "casualty_id": id,
        "team": "Penn",
        "system": "PennPRONTO",
        "time_ago": rospy.Time.now().secs - time,
    }

    r = requests.post(f"{BASE_URL}/api/casualty_image", headers=headers, files=files, data=data)
    print(r.status_code, r.json())
    return r.json().get("image_id", None)


def convert_to_enum(value):
    if (value == "absence"):
        return 0
    elif (value == "presence"):
        return 1
    elif (value == "wound"):
        return 1
    elif (value == "amputation"):
        return 2
    elif (value == "open"):
        return 0
    elif (value == "closed"):
        return 1
    elif (value == "untestable"):
        return 3

def start_run():
    load_dotenv()
    print("Creating new run...")
    TOKEN = os.getenv("TOKEN")
    BASE_URL = os.getenv("BASE_URL")

    url = f"{BASE_URL}/api/run/new"
    headers = {"accept": "application/json"}

    r = requests.get(url, headers=headers)
    print(r.status_code, r.json())

    print("Starting run...")
    url = f"{BASE_URL}/api/run/start"
    headers = {"accept": "application/json"}
    
    response = requests.get(url, headers=headers)
    print(response.status_code, r.json())

def update_casualty(payload):
    load_dotenv()
    print("Updating scorecard...")
    TOKEN = os.getenv("TOKEN")
    BASE_URL = os.getenv("BASE_URL")
    
    headers = {
        "accept": "application/json",
        "Authorization": TOKEN,
        "Content-Type": "application/json"
    }

    #Go through all time_ago fields and update them to be the difference between now and the time in the payload
    current_time = rospy.Time.now().secs
    for key in payload:
        for subkey in payload[key]:
            if subkey == "time_ago":
                if isinstance(payload[key], dict) and "time_ago" in payload[key]:
                    payload[key]["time_ago"] = current_time - payload[key]["time_ago"]
    
    r = requests.post(f"{BASE_URL}/api/update_report", headers=headers, json=payload)
    print(r.status_code, r.json())

def report_new_casualty(id, lat, long, time):
    load_dotenv()
    print("new casualty...")
    TOKEN = os.getenv("TOKEN")
    BASE_URL = os.getenv("BASE_URL")
    
    headers = {
        "accept": "application/json",
        "Authorization": TOKEN,
        "Content-Type": "application/json"
    }
    
    payload = {
      "hr": {
        "value": 0,
        "time_ago": 0,
      },
      "rr": {
        "value": 0,
        "time_ago": 0,
      },
      "alertness_ocular": {
        "value": 0,
        "time_ago": 0,
      },
      "alertness_verbal": {
        "value": 0,
        "time_ago": 0,
      },
      "alertness_motor": {
        "value": 0,
        "time_ago": 0,
      },
      "severe_hemorrhage": {
        "value": 0,
        "time_ago": 0,
      },
      "respiratory_distress": {
        "value": 0,
        "time_ago": 0,
      },
      "trauma_head": 0,
      "trauma_torso": 0,
      "trauma_lower_ext": 0,
      "trauma_upper_ext": 0,
      "temp": {
        "value": 98,
        "time_ago": rospy.Time.now().secs - time,
      },
      "casualty_id": id,
      "team": "Penn",
      "system": "PennPRONTO",
      "location": {
        "latitude": lat,
        "longitude": long,
        "time_ago": rospy.Time.now().secs - time,
      }
    }

    r = requests.post(f"{BASE_URL}/api/initial_report", headers=headers, json=payload)
    print(r.status_code, r.json())

def parse_report_string(report_str):
  #time_now = rospy.Time.now().secs

    payload = {
      "hr": {
        "value": 0,
        "time_ago": 0,
      },
      "rr": {
        "value": 0,
        "time_ago": 0,
      },
      "alertness_ocular": {
        "value": 0,
        "time_ago": 0,
      },
      "alertness_verbal": {
        "value": 0,
        "time_ago": 0,
      },
      "alertness_motor": {
        "value": 0,
        "time_ago": 0,
      },
      "severe_hemorrhage": {
        "value": 0,
        "time_ago": 0,
      },
      "respiratory_distress": {
        "value": 0,
        "time_ago": 0,
      },
      "trauma_head": 0,
      "trauma_torso": 0,
      "trauma_lower_ext": 0,
      "trauma_upper_ext": 0,
      "temp": {
        "value": 98,
        "time_ago": 0,
      },
      "casualty_id": 0,
      "team": "test_team",
      "system": "test_system",
      "location": {
        "latitude": 0,
        "longitude": 0,
        "time_ago": 0,
      }
    }

    if report_str:
    # Parse the raw report string into a dictionary
        try:
            report_dict = ast.literal_eval(report_str)
            if isinstance(report_dict, dict):
                for key, value in report_dict.items():
                    if key in payload:
                        if key in ["trauma_head", "trauma_torso", "trauma_lower_ext", "trauma_upper_ext"]:
                            payload[key] = convert_to_enum(value)
                        elif key in ["alertness_ocular", "alertness_verbal", "alertness_motor"]:
                            payload[key]["value"] = convert_to_enum(value)
                            payload[key]["time_ago"] = 0
                        elif key in ["hr", "rr", "temp"]:
                            payload[key]["value"] = int(value)
                            payload[key]["time_ago"] = rospy.Time.now().secs
                        elif key == "severe_hemorrhage":
                            payload[key]["value"] = convert_to_enum(value)
                            payload[key]["time_ago"] = rospy.Time.now().secs
                        elif key == "respiratory_distress":
                            payload[key]["value"] = convert_to_enum(value)
                            payload[key]["time_ago"] = rospy.Time.now().secs
                        elif key == "location" and isinstance(value, dict):
                            if "latitude" in value and "longitude" in value:
                                payload[key]["latitude"] = float(value["latitude"])
                                payload[key]["longitude"] = float(value["longitude"])
                                payload[key]["time_ago"] = rospy.Time.now().secs
                        elif key == "casualty_id":
                            payload[key] = int(value)
                        elif key in ["team", "system"]:
                            payload[key] = str(value)
                    else:
                        print(f"Key '{key}' not in payload, skipping.")      
            else:
              print("Parsed report is not a dictionary.")
        except Exception as e:
            print(f"Error parsing report string: {e}")
    #pretty print the payload
    return payload

    
    
