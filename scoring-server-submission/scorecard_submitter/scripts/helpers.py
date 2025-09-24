import os
import json
import math
import utm
from submission import parse_report_string
import rospy
import copy

#make a struct for matching table entry
matching_entry = {
    "report_id": None,
    "casualty_id": None,
    "uav": {
        "id": None,
        "lat": None,
        "lon": None
    },
    "ugv": {
        "id": None,
        "lat": None,
        "lon": None
    },
    "matched": False,
    "action": "",
    "report": {},
    "image_path": None,
    "timestamp": None,
    "pos_sent": False,
    "init_supp_sent": False,
    "update_sent": False

}



def gps_distance(lat1, lon1, lat2, lon2):
    """
    Compute Euclidean distance (in meters) between two GPS coordinates
    using UTM projection.
    """
    x1, y1, zone1, letter1 = utm.from_latlon(lat1, lon1)
    x2, y2, zone2, letter2 = utm.from_latlon(lat2, lon2)
    
    # Ensure both points are in the same UTM zone
    if (zone1, letter1) != (zone2, letter2):
        raise ValueError("Points are in different UTM zones, can't use simple Euclidean distance")
    
    return math.hypot(x2 - x1, y2 - y1)

def closest_casualty(report, casualty_list):
    """
    Find the closest casualty to the given GPS coordinates.
    """

    payload = parse_report_string(report)
    latitude = payload["location"]["latitude"]
    longitude = payload["location"]["longitude"]

    print(f"Parsed report location: lat={latitude}, lon={longitude}")

    min_distance = float('inf')
    closest_casualty = None
    idx = -1
    for i in range(len(casualty_list)):
        casualty = casualty_list[i]
        if casualty["position"] is not None:
            lat2, lon2 = casualty["position"]
            distance = gps_distance(latitude, longitude, lat2, lon2)
            print(distance)
            if distance < min_distance:
                min_distance = distance
                closest_casualty = casualty
                idx = i
                
    return closest_casualty, min_distance, idx, payload

def update_drone_casualty_db(ugv_uav_threshold=7):
    print("UPDATING DRONE CASUALTY DB")
    with open("/home/dtc/ws/data/casualty_db/uav_casualty_list.json", "r") as f:
        uav_db = json.load(f)

    with open("/home/dtc/ws/data/casualty_db/matching_table.json", "r") as f:
        matching_table = json.load(f)

    #first see this is a casualty the drone has seen before, but not the jackal
    for drone_det in uav_db:
        for entry in matching_table:
            if entry["uav"]["id"] == drone_det["id"] and entry["ugv"]["id"] == None:
                print(f"{entry['uav']['id']} + {drone_det['id']}")
                entry["uav"]["lat"], entry["uav"]["lon"] = drone_det["lat"], drone_det["lon"]
                print(f"Updating drone position for casualty {entry['casualty_id']}")
                entry["action"] = ""
                entry["timestamp"] = rospy.Time.now().to_sec()
    
    #For each drone 
    for drone_det in uav_db:
        min_dist = float('inf')
        closest_idx = -1
        #Compare to each entry in the matching table
        for entry in matching_table:
            if entry["ugv"]["id"] != None:
                lat = entry["ugv"]["lat"]
                lon = entry["ugv"]["lon"]
                distance = gps_distance(lat, lon, drone_det["lat"], drone_det["lon"])
                if distance < min_dist:
                    min_dist = distance
                    closest_idx = entry["casualty_id"]
        #then check if the jackal has seen this casualty before
        print(f"Min dist: {min_dist}, Closest idx: {closest_idx}")
        if min_dist < ugv_uav_threshold: #this means another jackal found this first
            matching_table[closest_idx]["uav"]["id"] = drone_det["id"]
            matching_table[closest_idx]["uav"]["lat"], matching_table[closest_idx]["uav"]["lon"] = drone_det["lat"], drone_det["lon"]
            matching_table[closest_idx]["action"] = "" #we don't do anything since the jackal found it first
            matching_table[closest_idx]["timestamp"] = rospy.Time.now().to_sec()
        #or else this is a new casualty for the drone
        elif not any(entry["uav"]["id"] == drone_det["id"] for entry in matching_table):
            new_entry = copy.deepcopy(matching_entry)
            new_entry["uav"]["id"] = drone_det["id"]
            new_entry["uav"]["lat"], new_entry["uav"]["lon"] = drone_det["lat"], drone_det["lon"]
            new_entry["casualty_id"] = len(matching_table)
            new_entry["action"] = "init"
            new_entry["timestamp"] = rospy.Time.now().to_sec()
            print(new_entry)
            matching_table.append(new_entry)

    with open("/home/dtc/ws/data/casualty_db/matching_table.json", "w") as f:
        json.dump(matching_table, f, indent=2)


def update_jackal_casualty_db(db_path, ugv_uav_threshold=7,ugv_ugv_threshold=2):
    print("UPDATING JACKAL CASUALTY DB")
    with open(os.path.join(db_path,"ugv_casualty_list.json"), "r") as f:
        ugv_db = json.load(f)
    with open(os.path.join(db_path, "matching_table.json"), "r") as f:
        matching_table = json.load(f)

    latest_casualty = ugv_db[-1]
    min_dist = float('inf')
    closest_idx = -1
    #First check if another Jackal has found this casualty
    for i in range(len(matching_table)):
        entry = matching_table[i]
        if entry["ugv"]["id"] != None:
            lat = entry["ugv"]["lat"]
            lon = entry["ugv"]["lon"]
            distance = gps_distance(lat, lon, latest_casualty["lat"], latest_casualty["lon"])
            if distance < min_dist:
                min_dist = distance
                closest_idx = i
    if min_dist < ugv_ugv_threshold: #this means another jackal found this first
        print("UPDATE request ran")
        matching_table[closest_idx]["report"] = latest_casualty["report"]
        matching_table[closest_idx]["image_path"] = latest_casualty["image_path"]
        matching_table[closest_idx]["action"] = "update"
        matching_table[closest_idx]["timestamp"] = rospy.Time.now().to_sec()

        with open(os.path.join(db_path, "matching_table.json"), "w") as f:
            json.dump(matching_table, f, indent=2)
        return
    
    #Now check if the drone has found this casualty
    min_dist = float('inf')
    closest_idx = -1
    #First check if drone has found this casualty
    for i in range(len(matching_table)):
        entry = matching_table[i]
        if entry["uav"]["id"] != None:
            lon = entry["uav"]["lon"]
            lat = entry["uav"]["lat"]
            distance = gps_distance(lat, lon, latest_casualty["lat"], latest_casualty["lon"])
            if distance < min_dist:
                min_dist = distance
                closest_idx = i
    if min_dist < ugv_uav_threshold: #this means drone found this first
        matching_table[closest_idx]["ugv"]["id"] = latest_casualty["id"]
        matching_table[closest_idx]["ugv"]["lat"], matching_table[closest_idx]["ugv"]["lon"] = latest_casualty["lat"], latest_casualty["lon"]
        matching_table[closest_idx]["report"] = latest_casualty["report"]
        matching_table[closest_idx]["image_path"] = latest_casualty["image_path"]
        matching_table[closest_idx]["action"] = "init_update"
        matching_table[closest_idx]["timestamp"] = rospy.Time.now().to_sec()
    else: #this means jackal found this before a drone did
        new_entry = copy.deepcopy(matching_entry)
        new_entry["ugv"]["id"] = latest_casualty["id"]
        new_entry["ugv"]["lat"], new_entry["ugv"]["lon"] = latest_casualty["lat"], latest_casualty["lon"]
        new_entry["report"] = latest_casualty["report"]
        new_entry["image_path"] = latest_casualty["image_path"]
        new_entry["action"] = "init_update"
        new_entry["casualty_id"] = len(matching_table)
        new_entry["timestamp"] = rospy.Time.now().to_sec()
        matching_table.append(new_entry)

    with open(os.path.join(db_path, "matching_table.json"), "w") as f:
        json.dump(matching_table, f, indent=2)

    
