import requests

import os
from dotenv import load_dotenv
import ast
import rospy


from enum import IntEnum

class TraumaHead(IntEnum):
    ABSENT = 0
    PRESENT = 1

class TraumaTorso(IntEnum):
    ABSENT = 0
    PRESENT = 1

class TraumaLowerExt(IntEnum):
    ABSENCE = 0
    WOUND = 1
    AMPUTATION = 2

class TraumaUpperExt(IntEnum):
    ABSENCE = 0
    WOUND = 1
    AMPUTATION = 2

class AlertnessOcular(IntEnum):
    OPEN = 0
    CLOSED = 1
    UNTESTABLE = 2

class AlertnessVerbal(IntEnum):
    NORMAL = 0
    ABNORMAL = 1
    ABSENT = 2
    UNTESTABLE = 3

class AlertnessMotor(IntEnum):
    NORMAL = 0
    ABNORMAL = 1
    ABSENT = 2
    UNTESTABLE = 3


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

    payload = {
      "hr": {
        "value": 0,
        "time_ago": rospy.Time.now().secs
      },
      "rr": {
        "value": 0,
        "time_ago": rospy.Time.now().secs
      },
      "alertness_ocular": {
        "value": 2,
        "time_ago": rospy.Time.now().secs
      },
      "alertness_verbal": {
        "value": 0,
        "time_ago": rospy.Time.now().secs
      },
      "alertness_motor": {
        "value": 0,
        "time_ago": rospy.Time.now().secs
      },
      "severe_hemorrhage": {
        "value": 1,
        "time_ago": rospy.Time.now().secs
      },
      "respiratory_distress": {
        "value": 1,
        "time_ago": rospy.Time.now().secs
      },
      "trauma_head": 0,
      "trauma_torso": 0,
      "trauma_lower_ext": 0,
      "trauma_upper_ext": 0,
      "temp": {
        "value": 0,
        "time_ago": rospy.Time.now().secs
      },
      "casualty_id": 5,
      "team": "test_team",
      "system": "test_system",
      "location": {
        "latitude": 0,
        "longitude": 0,
        "time_ago": rospy.Time.now().secs
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
                            payload[key]["time_ago"] = rospy.Time.now().secs
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
    submit_scorecard()