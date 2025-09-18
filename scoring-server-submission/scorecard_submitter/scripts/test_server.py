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
    headers = {"accept": "application/json"}
    
    response = requests.get(url, headers=headers)
    print(f"Create run - Status: {response.status_code}, Response: {response.text}")
    
    assert response.status_code == 200

def test_start_run(api_config):
    """Test starting a run"""
    url = f"{api_config['base_url']}/api/run/start"
    headers = {"accept": "application/json"}
    
    response = requests.get(url, headers=headers)
    print(f"Start run - Status: {response.status_code}, Response: {response.text}")
    
    assert response.status_code == 200
