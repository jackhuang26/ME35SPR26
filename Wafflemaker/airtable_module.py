# airtable_module.py

import requests
import json 
import time

# Airtable config

BASE_ID = "base_id"
TABLE_ID = "table_id"
RECORD_ID = "record_id"

TOKEN = "token"

URL = f"https://api.airtable.com/v0/{BASE_ID}/{TABLE_ID}/{RECORD_ID}"

Headers = {
        "Authorization": f"Bearer {TOKEN}",
        "Content-Type": "application/json"
}

# internal helper functions - keep private!! 

def _get_row(): 
        r = requests.get(url = URL, headers = Headers, params = {})

        if r.status_code != 200:
                raise Exception(f"HTTP Error: {r.status_code} - {r.text}")

        data = r.json() 

        if not data["fields"]:
                raise Exception("no fields in Airtable response")
        
        return data["fields"]
        
def _update_fields(fields):
        r = requests.patch(url = URL, headers = Headers, json = {"fields": fields})

        if r.status_code != 200:
                raise Exception(f"HTTP Error: {r.status_code} - {r.text}")

# main public functions - use these in your code!! 

# get current status of a station 
def get_status(station):
        record = _get_row()
        return record.get(station)

# set a new status for a station
def update_status(station, updated_status):
        _update_fields({station: updated_status})
        time.sleep(0.2)

# wait until station reaches a target status
def wait_until_status(station, target, poll=0.5):
        while True:
                status = get_status(station)

                if status == target: 
                        return True
                
                time.sleep(poll)

# convenience function to wait for station to be ready
def wait_until_ready(station):
        wait_until_status(station, "ready")
