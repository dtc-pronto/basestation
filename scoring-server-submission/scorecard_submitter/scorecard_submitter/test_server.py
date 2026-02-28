import pytest
import requests
import os
from dotenv import load_dotenv

@pytest.fixture(scope="module")
def api_config():
    """Load environment variables"""
    load_dotenv()
    token = os.getenv("TOKEN")
    base_url = os.getenv("BASE_URL")
    
    assert token, "TOKEN not set"
    assert base_url, "BASE_URL not set"
    
    return {"token": token, "base_url": base_url}

def test_create_new_run(api_config):
    """Test creating a new run"""
    url = f"{api_config['base_url']}/api/run/new"
    headers = {
        "accept": "application/json",
        "Authorization": api_config["token"],
        "Content-Type": "application/json"
    }

    content = {
      "casualty_count": 10,
      "lat": 30.5738,
      "lon": -96.3499,
      "duration": 60
    }

    response = requests.post(url, headers=headers, json=content)
    print(f"Create run - Status: {response.status_code}, Response: {response.text}")
    
    assert response.status_code == 200

def test_start_run(api_config):
    """Test starting a run"""
    url = f"{api_config['base_url']}/api/run/start"
    headers = {
        "accept": "application/json",
        "Authorization": api_config["token"],
        "Content-Type": "application/json"
    }

    response = requests.post(url, headers=headers)
    print(f"Start run - Status: {response.status_code}, Response: {response.text}")
    
    assert response.status_code == 200

def test_system_location(api_config):
    """Test system location endpoint"""
    url = f"{api_config['base_url']}/api/system_location"
    headers = {
        "accept": "application/json",
        "Authorization": api_config["token"],
        "Content-Type": "application/json"
    }

    content = {
      "team": "string",
      "system": "string",
      "type": 0,
      "latitude": 0,
      "longitude": 0,
      "altitude": 0
    }

    response = requests.post(url, headers=headers, json=content)
    print(f"System location - Status: {response.status_code}, Response: {response.text}")

    assert response.status_code == 200


def test_rapid_report(api_config):
    TOKEN = api_config["token"]
    BASE_URL = api_config["base_url"]
    headers = {
        "accept": "application/json",
        "Authorization": TOKEN,
        "Content-Type": "application/json"
    }

    # payload = {
    #   "hr": {
    #     "value": 0,
    #     "time_ago":  0,
    #   },
    #   "rr": {
    #     "value": 0,
    #     "time_ago":  0,
    #   },
    #   "alertness_ocular": {
    #     "value": 0,
    #     "time_ago":  0,
    #   },
    #   "alertness_verbal": {
    #     "value": 0,
    #     "time_ago":  0,
    #   },
    #   "alertness_motor": {
    #     "value": 0,
    #     "time_ago":  0,
    #   },
    #   "severe_hemorrhage": {
    #     "value": 0,
    #     "time_ago":  0,
    #   },
    #   "respiratory_distress": {
    #     "value": 0,
    #     "time_ago":  0,
    #   },
    #   "trauma_head": 0,
    #   "trauma_torso": 0,
    #   "trauma_lower_ext": 0,
    #   "trauma_upper_ext": 0,
    #   "temp": {
    #     "value": 98,
    #     "time_ago":  0,
    #   },
    #   "casualty_id": 0,
    #   "team": "PRONTO",
    #   "system": "PHOBOS",
    #   "location": {
    #     "latitude": 32.2,
    #     "longitude": -75.1,
    #     "time_ago":  0,
    #   }
    # }

    payload = {
      "category": 0,
      "casualty_id": 0,
      "team": "string",
      "system": "string"
    }

    r = requests.post(f"{BASE_URL}/api/triage", headers=headers, json=payload)
    assert r.status_code == 200
