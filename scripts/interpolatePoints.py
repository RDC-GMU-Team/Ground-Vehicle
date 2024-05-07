import csv
import math

def read_csv_file(filename):
    waypoints = []
    with open(filename, mode='r') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            x, y = map(float, row)
            waypoints.append((x, y))
    return waypoints

def calculate_distance(point1, point2):
    return math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)

def insert_points(waypoints):
    new_waypoints = [waypoints[0]]
    
    for i in range(1, len(waypoints)):
        point1 = waypoints[i-1]
        point2 = waypoints[i]
        
        distance = calculate_distance(point1, point2)
        
        if distance > 0.00003:#3 meters
            num_intervals = int(distance / 0.00001)#1 meter
            
            for j in range(1, num_intervals):
                x = round(point1[0] + (point2[0] - point1[0]) * j / num_intervals, 8)
                y = round(point1[1] + (point2[1] - point1[1]) * j / num_intervals, 8)
                new_waypoints.append((x, y))
        
        new_waypoints.append(point2)
    
    return new_waypoints

def write_csv_file(filename, waypoints):
    with open(filename, mode='w', newline='') as file:
        csv_writer = csv.writer(file)
        #csv_writer.writerow(["x", "y"])
        for x, y in waypoints:
            csv_writer.writerow([x, y])

if __name__ == "__main__":
    waypoints = read_csv_file("raw_waypoints.csv")
    new_waypoints = insert_points(waypoints)
    write_csv_file("waypoints.csv", new_waypoints)