import requests
import os
from dotenv import load_dotenv

def api_config():
    """Load environment variables"""
    load_dotenv()
    token = os.getenv("TOKEN")
    base_url = os.getenv("BASE_URL")
    
    assert token, "TOKEN not set"
    assert base_url, "BASE_URL not set"
    
    return {"token": token, "base_url": base_url}


def test_initial_report(api_config):
    TOKEN = api_config["token"]
    BASE_URL = api_config["base_url"]
    headers = {
        "accept": "application/json",
        "Authorization": TOKEN,
        "Content-Type": "application/json"
    }

    payload = {
      "hr": {
        "value": 0,
        "time_ago":  0,
      },
      "rr": {
        "value": 0,
        "time_ago":  0,
      },
      "alertness_ocular": {
        "value": 0,
        "time_ago":  0,
      },
      "alertness_verbal": {
        "value": 0,
        "time_ago":  0,
      },
      "alertness_motor": {
        "value": 0,
        "time_ago":  0,
      },
      "severe_hemorrhage": {
        "value": 0,
        "time_ago":  0,
      },
      "respiratory_distress": {
        "value": 0,
        "time_ago":  0,
      },
      "trauma_head": 0,
      "trauma_torso": 0,
      "trauma_lower_ext": 0,
      "trauma_upper_ext": 0,
      "temp": {
        "value": 98,
        "time_ago":  0,
      },
      "casualty_id": 0,
      "team": "PRONTO",
      "system": "PHOBOS",
      "location": {
        "latitude": 32.2,
        "longitude": -75.1,
        "time_ago":  0,
      }
    }

    r = requests.post(f"{BASE_URL}/api/initial_report", headers=headers, json=payload)
    assert r.status_code == 200
    print("passed")

if __name__ == "__main__":
    api = api_config()
    test_initial_report(api)
