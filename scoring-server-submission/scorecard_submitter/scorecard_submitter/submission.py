import requests

import os
import json
from dotenv import load_dotenv
import ast

total_init_pos = 0
total_init_supp = 0
total_update = 0

def submit_image(image_path, time, time_now, id):
    # Load .env file
    load_dotenv()
    print("Submitting Image")
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
        "team": "PennPRONTO",
        "system": "JackalNVILA",
        "time_ago": max(time - time_now, 0),
    }

    r = requests.post(f"{BASE_URL}/api/casualty_image", headers=headers, files=files, data=data)
    print(r.status_code, r.json())
    return r


def convert_to_enum(value, key=None):
    # value = value.lower()

    binary_keys = {
        "severe_hemorrhage": {"absence": 0, "presence": 1},
        "respiratory_distress": {"absence": 0, "presence": 1},
        "trauma_head": {"absence": 0, "presence": 1},
        "trauma_torso": {"absence": 0, "presence": 1},
    }

    trauma_ext_keys = {
        "trauma_upper_ext": {"absence": 0, "wound": 1, "amputation": 2},
        "trauma_lower_ext": {"absence": 0, "wound": 1, "amputation": 2},
    }

    alertness_keys = {
        "alertness_verbal": {"presence": 0, "abnormal": 1, "absence": 2},
        "alertness_ocular": {"open": 0, "closed": 1},
        "alertness_motor": {"presence": 0, "absence": 2}  
    }

    if key in binary_keys:
        return binary_keys[key].get(value)
    elif key in trauma_ext_keys:
        return trauma_ext_keys[key].get(value)
    elif key in alertness_keys:
        return alertness_keys[key].get(value)
    
    return None  # or raise an error

def start_run(content: dict):
    load_dotenv()
    print("Creating new run...")
    TOKEN = os.getenv("TOKEN")
    BASE_URL = os.getenv("BASE_URL")

    url = f"{BASE_URL}/api/run/new"
    headers = {
        "accept": "application/json",
        "Authorization": TOKEN,
        "Content-Type": "application/json"
    }

    r = requests.post(url, headers=headers, json=content)
    print(r.status_code, r.json())

    print("Starting run...")
    url = f"{BASE_URL}/api/run/start"
    headers = {"accept": "application/json"}

    response = requests.post(url, headers=headers)
    print(response.status_code, response.json())
    return response

def post_system_location(lat, lon, alt, system, type):
    load_dotenv()
    print("Posting system location...")
    TOKEN = os.getenv("TOKEN")
    BASE_URL = os.getenv("BASE_URL")

    url = f"{BASE_URL}/api/system_location"
    headers = {
        "accept": "application/json",
        "Authorization": TOKEN,
        "Content-Type": "application/json"
    }

    content = {
      "team": "PennPRONTO",
      "system": system,
      "type": type,
      "latitude": lat,
      "longitude": lon,
      "altitude": alt
    }

    r = requests.post(url, headers=headers, json=content)
    print(r.status_code, r.json())
    return r

def location_report(lat, lon, level, id, system):
    load_dotenv()
    print("Posting location report...")
    TOKEN = os.getenv("TOKEN")
    BASE_URL = os.getenv("BASE_URL")

    url = f"{BASE_URL}/api/location_report"
    headers = {
        "accept": "application/json",
        "Authorization": TOKEN,
        "Content-Type": "application/json"
    }

    content = {
      "latitude": lat,
      "longitude": lon,
      "level": level,
      "casualty_id": id,
      "team": "PennPRONTO",
      "system": system
    }

    r = requests.post(url, headers=headers, json=content)
    print(r.status_code, r.json())
    return r

def triage_report(category, id, system):
    load_dotenv()
    print("Posting triage report...")
    TOKEN = os.getenv("TOKEN")
    BASE_URL = os.getenv("BASE_URL")

    url = f"{BASE_URL}/api/triage"
    headers = {
        "accept": "application/json",
        "Authorization": TOKEN,
        "Content-Type": "application/json"
    }

    content = {
      "category": category,
      "casualty_id": id,
      "team": "PennPRONTO",
      "system": system
    }

    r = requests.post(url, headers=headers, json=content)
    print(r.status_code, r.json())
    return r

def trauma_report(type, value, time_ago, id, system):
    load_dotenv()
    print("Posting trauma report...")
    TOKEN = os.getenv("TOKEN")
    BASE_URL = os.getenv("BASE_URL")

    url = f"{BASE_URL}/api/trauma"
    headers = {
        "accept": "application/json",
        "Authorization": TOKEN,
        "Content-Type": "application/json"
    }

    content = {
      "type": type,
      "value": value,
      "time_ago": time_ago,
      "casualty_id": id,
      "team": "PennPRONTO",
      "system": system
    }

    r = requests.post(url, headers=headers, json=content)
    print(r.status_code, r.json())
    return r

def vitals_report(type, value, time_ago, id, system):
    load_dotenv()
    print("Posting vitals report...")
    TOKEN = os.getenv("TOKEN")
    BASE_URL = os.getenv("BASE_URL")

    url = f"{BASE_URL}/api/vitals"
    headers = {
        "accept": "application/json",
        "Authorization": TOKEN,
        "Content-Type": "application/json"
    }

    content = {
      "type": type,
      "value": value,
      "time_ago": time_ago,
      "casualty_id": id,
      "team": "PennPRONTO",
      "system": system
    }

    r = requests.post(url, headers=headers, json=content)
    print(r.status_code, r.json())
    return r

def hmt_location_report(lat, lon, category, time_ago, id, system):
    load_dotenv()
    print("Posting HMT location report...")
    TOKEN = os.getenv("TOKEN")
    BASE_URL = os.getenv("BASE_URL")

    url = f"{BASE_URL}/api/hmt/casualty"
    headers = {
        "accept": "application/json",
        "Authorization": TOKEN,
        "Content-Type": "application/json"
    }

    content = {
      "latitude": lat,
      "longitude": lon,
      "category": category,
      "time_ago": time_ago,
      "casualty_id": id,
      "team": "PennPRONTO",
      "system": system
    }

    r = requests.post(url, headers=headers, json=content)
    print(r.status_code, r.json())
    return r

def hmt_assessment_report(type, value, time_ago, id, system):
    load_dotenv()
    print("Posting HMT assessment report...")
    TOKEN = os.getenv("TOKEN")
    BASE_URL = os.getenv("BASE_URL")

    url = f"{BASE_URL}/api/hmt/assessment"
    headers = {
        "accept": "application/json",
        "Authorization": TOKEN,
        "Content-Type": "application/json"
    }

    content = {
      "type": type,
      "value": value,
      "time_ago": time_ago,
      "casualty_id": id,
      "team": "PennPRONTO",
      "system": system
    }

    r = requests.post(url, headers=headers, json=content)
    print(r.status_code, r.json())
    return r

def update_casualty(id, payload, time_now):
    global total_update
    total_update += 1
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
    for key in payload:
      if isinstance(payload[key], dict) and "time_ago" in payload[key].keys():
        payload[key]["time_ago"] = max(time_now - payload[key]["time_ago"], 0)

    payload["casualty_id"] = id
                 
    r = requests.post(f"{BASE_URL}/api/update_report", headers=headers, json=payload)
    print(r.status_code, r.json())
    return r

def init_position(id, lat, lon, time, time_now):
    global total_init_pos
    total_init_pos += 1
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
      "casualty_id": id,
      "team": "PennPRONTO",
      "system": "JackalNVILA",
      "location": {
        "latitude": lat,
        "longitude": lon,
        "time_ago": max(time - time_now, 0),
      }
    }
    
    print(payload)

    r = requests.post(f"{BASE_URL}/api/initial_report", headers=headers, json=payload)
    print(r.status_code, r.json())
    return r

def init_supplement(id, payload, time_now):
    global total_init_supp
    total_init_supp += 1
    load_dotenv()
    print("Initial Supplement...")
    TOKEN = os.getenv("TOKEN")
    BASE_URL = os.getenv("BASE_URL")
    
    headers = {
        "accept": "application/json",
        "Authorization": TOKEN,
        "Content-Type": "application/json"
    }

    #Go through all time_ago fields and update them to be the difference between now and the time in the payload
    for key in payload:
      if isinstance(payload[key], dict) and "time_ago" in payload[key].keys():
        payload[key]["time_ago"] = max(time_now - payload[key]["time_ago"], 0)

    payload["casualty_id"] = id

    print(payload)
                 
    r = requests.post(f"{BASE_URL}/api/initial_report", headers=headers, json=payload)
    print(r.status_code, r.json())
    return r

def total_posts():
    global total_init_pos, total_init_supp, total_update
    print(f"Total Initial Positions: {total_init_pos}")
    print(f"Total Initial Supplements: {total_init_supp}")
    print(f"Total Updates: {total_update}")

def report_new_casualty(id, lat, long, time, time_now):
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
        "time_ago":  max(time - time_now, 0),
      },
      "rr": {
        "value": 0,
        "time_ago":  max(time - time_now, 0),
      },
      "alertness_ocular": {
        "value": 0,
        "time_ago":  max(time - time_now, 0),
      },
      "alertness_verbal": {
        "value": 0,
        "time_ago":  max(time - time_now, 0),
      },
      "alertness_motor": {
        "value": 0,
        "time_ago":  max(time - time_now, 0),
      },
      "severe_hemorrhage": {
        "value": 0,
        "time_ago":  max(time - time_now, 0),
      },
      "respiratory_distress": {
        "value": 0,
        "time_ago":  max(time - time_now, 0),
      },
      "trauma_head": 0,
      "trauma_torso": 0,
      "trauma_lower_ext": 0,
      "trauma_upper_ext": 0,
      "temp": {
        "value": 98,
        "time_ago":  max(time - time_now, 0),
      },
      "casualty_id": id,
      "team": payload["system"],
      "system": payload["system"],
      "location": {
        "latitude": lat,
        "longitude": long,
        "time_ago":  max(time - time_now, 0),
      }
    }

    r = requests.post(f"{BASE_URL}/api/initial_report", headers=headers, json=payload)
    print(r.status_code, r.json())
    return r

def parse_report_string_as_json(report_str, time_now:str):
  #time_now = rospy.Time.now().secs
    data = json.loads(report_str)
    print(json.dumps(data, indent=2))
    

    payload = {
      "hr": {
        "value": data["hr"]["value"],
        "time_ago":  data["hr"]["time_ago"] if data["hr"]["time_ago"] > 0 else time_now,
      },
      "rr": {
        "value": data["rr"]["value"],
        "time_ago":  data["rr"]["time_ago"] if data["rr"]["time_ago"] > 0 else time_now,
      },
      "alertness_ocular": {
        # "value": convert_to_enum(data["alertness_ocular"]["value"], "alertness_ocular"),
        "value": data["alertness_ocular"]["value"],
        "time_ago":  data["alertness_ocular"]["time_ago"] if data["alertness_ocular"]["time_ago"] > 0 else time_now,
      },
      "alertness_verbal": {
        # "value": convert_to_enum(data["alertness_verbal"]["value"], "alertness_verbal"),
        "value": data["alertness_verbal"]["value"],
        "time_ago":  data["alertness_verbal"]["time_ago"] if data["alertness_verbal"]["time_ago"] > 0 else time_now,
      },
      "alertness_motor": {
        # "value": convert_to_enum(data["alertness_motor"]["value"], "alertness_motor"),
        "value": data["alertness_motor"]["value"],
        "time_ago":  data["alertness_motor"]["time_ago"] if data["alertness_motor"]["time_ago"] > 0 else time_now,
      },
      "severe_hemorrhage": {
        # "value": convert_to_enum(data["severe_hemorrhage"]["value"], "severe_hemorrhage"),
        "value": data["severe_hemorrhage"]["value"],
        "time_ago":  data["severe_hemorrhage"]["time_ago"] if data["severe_hemorrhage"]["time_ago"] > 0 else time_now,
      },
      "respiratory_distress": {
        # "value": convert_to_enum(data["respiratory_distress"]["value"], "respiratory_distress"),
        "value": data["respiratory_distress"]["value"],
        "time_ago":  data["respiratory_distress"]["time_ago"] if data["respiratory_distress"]["time_ago"] > 0 else time_now,
      },
      # "trauma_head": convert_to_enum(data["trauma_head"], "trauma_head"),
      # "trauma_torso": convert_to_enum(data["trauma_torso"], "trauma_torso"),
      # "trauma_lower_ext": convert_to_enum(data["trauma_lower_ext"], "trauma_lower_ext"),
      # "trauma_upper_ext": convert_to_enum(data["trauma_upper_ext"], "trauma_upper_ext"),
      'trauma_head': data["trauma_head"],
      'trauma_torso': data["trauma_torso"],
      'trauma_lower_ext': data["trauma_lower_ext"],
      'trauma_upper_ext': data["trauma_upper_ext"],
      "temp": {
        "value": 98,
        "time_ago":  data["temp"]["time_ago"] if data["temp"]["time_ago"] > 0 else time_now,
      },
      "casualty_id": data["casualty_id"],
      "team": data["team"],
      "system": data["system"],
      "location": {
        "latitude": data["location"]["latitude"],
        "longitude": data["location"]["longitude"],
        "time_ago":  data["location"]["time_ago"] if data["location"]["time_ago"] > 0 else time_now,
      }
    }
    return payload

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
      "team": "PennPRONTO",
      "system": "JackalNVILA",
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
                            payload[key] = convert_to_enum(value, key)
                        elif key in ["alertness_ocular", "alertness_verbal", "alertness_motor", "severe_hemorrhage", "respiratory_distress"] and isinstance(value, dict):
                            payload[key]["value"] = convert_to_enum(value["value"], key)
                            payload[key]["time_ago"] = float(value[key]["timestamp"])
                        elif key in ["hr", "rr", "temp"] and isinstance(value, dict):
                            payload[key]["value"] = float(value["value"])
                            payload[key]["time_ago"] = float(value[key]["timestamp"])
                        elif key == "location" and isinstance(value, dict):
                            if "latitude" in value and "longitude" in value:
                                payload[key]["latitude"] = float(value["latitude"])
                                payload[key]["longitude"] = float(value["longitude"])
                                payload[key]["time_ago"] = float(value[key]["timestamp"])
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

    
    
