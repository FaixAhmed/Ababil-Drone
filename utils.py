# utils.py
import math

def clamp(val, min_val, max_val):
    return max(min_val, min(val, max_val))

def haversine_distance(lat1, lon1, lat2, lon2, earth_radius_km):
    if None in (lat1, lon1, lat2, lon2):
        return float('inf')
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dlat / 2) ** 2 + math.cos(math.radians(lat1)) * \
        math.cos(math.radians(lat2)) * math.sin(dlon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return earth_radius_km * c * 1000 # meters