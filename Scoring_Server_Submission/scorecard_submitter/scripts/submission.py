import requests

import os
from dotenv import load_dotenv
import ast
import rospy

#TODO: add casulty_id, team, system, time_ago to submit_scorecard and submit_image functions
def submit_image(image_path):
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
        "casualty_id": 0,
        "team": "test_team",
        "system": "test_system",
        "time_ago": 0
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

def update_scorecard(run_id, scorecard_id):
    load_dotenv()
    print("Updating scorecard...")
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
        "value": 0,
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

    r = requests.post(f"{BASE_URL}/api/update_report", headers=headers, json=payload)
    print(r.status_code, r.json())

def submit_scorecard(raw_report_str=None):

    # Load .env file
    load_dotenv()

    TOKEN = os.getenv("TOKEN")
    BASE_URL = os.getenv("BASE_URL")

    headers = {
        "accept": "application/json",
        "Authorization": TOKEN,
        "Content-Type": "application/json"
    }

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
        "value": 0,
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

    if raw_report_str:
    # Parse the raw report string into a dictionary
        try:
            report_dict = ast.literal_eval(raw_report_str)
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
    import json
    print("Submitting payload:")
    print(json.dumps(payload, indent=2))

    r = requests.post(f"{BASE_URL}/api/initial_report", headers=headers, json=payload)
    print(r.status_code, r.json())

if __name__ == "__main__":
    # code that only runs when script is executed directly
    start_run()
    submit_scorecard()
    submit_scorecard()
    submit_scorecard()
    submit_scorecard()
    submit_scorecard()
    submit_scorecard()
    update_scorecard(1, 1)
    submit_image("1755288246triage_image.jpg")


    
    
