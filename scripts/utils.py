import math
import numpy as np
import pandas as pd

class CacheROSMessage:
    def __init__(self, max_size=100):
        self.max_size = max_size
        self.cache = []
        self.time = []

    #add element in the cache
    def add_element(self, data):
        if len(self.cache) >= self.max_size:
            self.time.pop(0)
            self.cache.pop(0)

        self.cache.append(data)
        self.time.append(data.header.stamp.secs)

    #get all the elements in the cache    
    def get_all(self):
        return self.cache
    
    #clear the cache
    def clear_cache(self):
        self.cache = []
        self.time = []
    
    #get element at a particular index
    def get_index(self, index):
        return self.cache[index]

    #get element from a particular time or return None
    def get_element_from_time(self, time):
        if time not in self.time:
            return None

        index = self.time.index(time)
        return self.cache[index]

    #get the oldest element in the cache       
    def get_oldest_element(self):
        return self.cache[0]
   
    #get the latest n elements in the cache, if n > cache size, return None
    def get_last_n_elements(self, n):
        if n > len(self.cache):
            return self.get_all()

        return self.cache[-n:]

class CacheHeaderlessROSMessage:
    def __init__(self, max_size=100):
        self.max_size = max_size
        self.cache = []

    #add element in the cache
    def add_element(self, data):
        if len(self.cache) >= self.max_size:
            self.cache.pop(0)

        self.cache.append(data)

    #get all the elements in the cache    
    def get_all(self):
        return self.cache
    
    #clear the cache
    def clear_cache(self):
        self.cache = []
        self.time = []
    
    #get element at a particular index
    def get_index(self, index):
        return self.cache[index]

    #get the oldest element in the cache       
    def get_oldest_element(self):
        return self.cache[0]

    def get_last_element(self):
        if len(self.cache) == 0:
            return None
        
        return self.cache[-1]

    #get the latest n elements in the cache, if n > cache size, return None
    def get_last_n_elements(self, n):
        if n > len(self.cache):
            return self.get_all()

        return self.cache[-n:]


def read_csv(filename):
    df = pd.read_csv(filename)
    return df.values.tolist()

def substract_two_angles(angle1, angle2):
    #calculate the difference between two angles
    diff = angle1 - angle2
    if diff > 180:
        diff = 360 - diff  
    elif diff < -180:
        diff = diff + 360
    return diff

def round_up_angles(angle):
    if angle < 0:
        angle = angle + 360
    elif angle > 360:
        angle = angle - 360
    return angle 

def euclidean_distance(self, x1, y1, x2, y2):
    return math.sqrt((x1-x2)**2 + (y1-y2)**2)

def check_matching_pose(x1, y1, x2, y2):
    if abs(x1 - x2) < 0.5 and abs(y1 - y2) < 0.5:
        return True
    else:
        return False

def find_next_heading_to_waypoint(current_position, current_heading, waypoint):
    '''This function calculates the next heading to the waypoint,
    it takes the current_position, current_heading and waypoint as input,
    outputs the next heading in radians, if the heading is not in the range -45 to +45 degree then
    just go to +45 or -45 for now'''
    #first find the bearing using calculate heading function
    #then find the difference between current heading and bearing
    difference_in_heading = difference_heading(current_heading, math.degrees(calculate_heading(current_position, waypoint)))

    #implement the logic to find the next heading
    if difference_in_heading >= -28 and difference_in_heading <= 28:
        if abs(difference_in_heading) < 1:
            return math.radians(0)
        return math.radians(difference_in_heading)
    elif difference_in_heading < -28:
        return math.radians(-28)
    else:
        return math.radians(28)

#heading is in degrees
def difference_heading(heading1, heading2):
    '''This function calculates the difference between two headings,
    rounds up the heading if the difference is greater than 180 degrees,
    needs input in degrees, returns the difference in degrees'''
    diff = heading1 - heading2
    if diff > 180:
        diff = diff - 360
    elif diff < -180:
        diff = diff + 360
    return diff

#Calculate heading between two gps coordinates
def calculate_heading(CoordinateA,CoordinateB):
    '''Calculates the heading between two gps coordinates,
        returns the heading in radians'''
    X = math.cos(CoordinateB[0]) * math.sin(CoordinateB[1] - CoordinateA[1])
    Y = math.cos(CoordinateA[0]) * math.sin(CoordinateB[0]) - math.sin(CoordinateA[0]) * math.cos(CoordinateB[0]) * math.cos(CoordinateB[1] - CoordinateA[1])

    heading = math.atan2(X,Y)
    return heading

def distance_between_two_points(self, waypoint1, waypoint2):
    '''Calculate the distance between two gps coordinates
        using haversine formula, returns the distance in meters'''        
    lat1, lon1 = math.radians(waypoint1[0]), math.radians(waypoint1[1])
    lat2, lon2 = math.radians(waypoint2[0]), math.radians(waypoint2[1])
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
    c = 2 * math.asin(math.sqrt(a))
    return self.radius_at_equator * c * 1000 #in meters
