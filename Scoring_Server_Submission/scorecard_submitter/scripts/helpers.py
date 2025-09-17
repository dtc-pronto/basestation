import math
import utm
from submission import parse_report_string

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



    min_distance = float('inf')
    closest_casualty = None
    idx = -1
    for i in len(casualty_list):
        casualty = casualty_list[i]
        if casualty["position"] is not None:
            lat2, lon2 = casualty["position"]
            distance = gps_distance(latitude, longitude, lat2, lon2)
            if distance < min_distance:
                min_distance = distance
                closest_casualty = casualty
                idx = i
                
    return closest_casualty, min_distance, idx, payload