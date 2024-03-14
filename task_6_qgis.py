''' 
* Team Id : <1810>
* Author List : <Harsh Chandgotia, Shashwat Anand, Arunil Jain, Sanarth Agarwal>
* Filename: <task_6_qgis.py>
* Theme: <Geoguide>
* Functions: <read_csv, wite_csv, tracker, tracker1, key_with_minimum_value, distance_cal, centre_coord>
* Global Variables: <lat_lon, aruco_dict, parameters, dictionary, cap1, ret1, img, cap, coord1, coord2, coord3, coord4, coord5, ip, list1>
'''


import cv2
import numpy as np
from cv2 import aruco
import csv
import socket


'''
* Function Name: read_csv
* Input: csv file name
* Output: dictionary of aruco id and its coordinates
* Logic: -> open csv file (lat_lon.csv)
         -> read "lat_lon.csv" file 
         -> store csv data in lat_lon dictionary as {id:[lat, lon].....}
         -> return lat_lon
* Example Call: read_csv(csv_filename)

'''
def read_csv(csv_name):
    lat_lon = {}

    with open(csv_name, 'r', encoding='UTF-8') as csv_file:
      csv_reader = csv.reader(csv_file)

    
      for row_number,row in enumerate(csv_reader):
        
        id = row[0]  
        lat = row[1]
        lon = row[2]  

        lat_lon[id] = [lat,lon]
                     
    return lat_lon 


'''
* Function Name: write_csv
* Input: list of coordinate [x,y], csv file name
* Logic: -> open csv file (lat_lon.csv)
         -> write column names "lat", "lon"
         -> write loc ([lat, lon]) in respective columns
* Example Call: write_csv(coordinates, csv_filename)

'''

def write_csv(loc, csv_name):

    with open(csv_name, 'w', newline='') as csv_file:
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(['lat', 'lon'])
        csv_writer.writerow([loc[0],loc[1]])


'''
* Function Name: tracker
* Input: aruco_id, dictionary
* Output: coordintes of the aruco id
* Logic: -> find the lat, lon associated with ar_id (aruco id)
         -> write these lat, lon to "live_data.csv"
         -> also return coordinate ([lat, lon]) associated with respective ar_id.
* Example Call: tracker(aruco_id, dictionary_name)

'''
def tracker(ar_id, lat_lon):

    csv_filename = 'live_data.csv'

    ar_id1 = str(ar_id)
    coordinate = lat_lon[ar_id1]

    write_csv(coordinate, csv_filename)    

    return coordinate


'''
* Function Name: tracker1
* Input: coordinates(lat,lon)
* Output: coordintes of the aruco id
* Logic: -> write these lat, lon to "live_data.csv"
         -> also return coordinate ([lat, lon])
* Example Call: tracker1(coordinates)

'''
def tracker1(coordinate):
    csv_filename = 'live_data.csv'

    write_csv(coordinate, csv_filename)    

    return coordinate    


'''
* Function Name: key_minimum_value
* Input: input_dictionary
* Output: key with minimum value
* Logic: -> uses the min function of python to find the minimum value
* Example Call: key_with_minimum_value(input_dict)

'''
def key_with_minimum_value(input_dict):
    if not input_dict:
        return None  # Return None for an empty dictionary

    temp = min(input_dict.values())
    min_key = [key for key in input_dict if input_dict[key] == temp]
    return min_key[0]


'''
* Function Name: distance_cal
* Input: coordinates, aruco_id
* Output: distance between two coordinates
* Logic: -> uses the np.linalg.norm function between two coordinates
* Example Call: distance_cal(coordinates, aruco_id)

'''
def distance_cal(coord2, id):
    coord1 = dictionary[str(id)]
    dist = coord2 - coord1
    distance = np.linalg.norm(dist)
    return distance


'''
* Function Name: centre_coord
* Input: list index of an aruco id in a list, list of corner coordinates of an aruco marker
* Output: returns the centre coordinates of an aruco marker
* Logic: -> uses the sum function of the math library and finds the average
* Example Call: centre_coord(index, aruco_corners)

'''
def centre_coord(index, corners):
    fixed_marker_corners = corners[int(index)][0]
    fixed_corner_coordinates = []

    for corner in fixed_marker_corners:
        x, y = corner
        fixed_corner_coordinates.append([x, y])


    fixed_center_x = sum(x for x, y in fixed_corner_coordinates) / len(fixed_corner_coordinates)
    fixed_center_y = sum(y for x, y in fixed_corner_coordinates) / len(fixed_corner_coordinates)
    fixed_center_coordinates = [int(fixed_center_x), int(fixed_center_y)]
    fixed_center_nparray = np.array(fixed_center_coordinates)
    return fixed_center_nparray


#reads the lat_long.csv file and creates a dictionary with the aruco markers and their coordinates as key value pair 
lat_lon = read_csv('lat_long.csv')

# Create an ArUco dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)

# Create parameters for ArUco detection
parameters = aruco.DetectorParameters()


print("---------------------------------------")
print("First storing all the aruco markers on the arena")
print("---------------------------------------")

#this dictionary is used to store all the aruco markers present on the arena which are also there in the lat_long.csv file
dictionary = {}

# Open the webcam
cap1 = cv2.VideoCapture(0, cv2.CAP_DSHOW)

ret1, img = cap1.read()
ret1, img = cap1.read()

#this loop stores all the aruco markers that it reads in the dictionary till the length of the dictionary is same as the length of the lat_long.csv file

while (len(dictionary) != len(lat_lon) - 1):
    ret1, img = cap1.read()
    img = cv2.resize(img, (1920, 1080), fx=1, fy=1, interpolation=cv2.INTER_LINEAR)
    corners, ids, rejectedImgPoints = aruco.detectMarkers(img, aruco_dict, parameters=parameters)
    
    for i in range(len(ids)):
        if str(ids[i][0]) in lat_lon: 

            center_coordinates1_np = centre_coord(i, corners)

            if str(ids[i][0]) not in dictionary:
                dictionary[str(ids[i][0])] = center_coordinates1_np
                

    print("--------------------------")
    print(f"len of dict: {len(dictionary)}")
    print("---------------------------")
cap1.release()


print("--------------------------------------------------")
print("Dictionary with the centre coordinates:")
print(dictionary)
print("------------------------------------------")
print("All arena aruco IDs stored inside dictionary")
print("------------------------------------------")

cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)


#these are some of the coordinates near the event boxes that are not present in the lat_long.csv file 
coord1 = [39.613478, -74.362548]
coord2 = [39.613461, -74.36245]
coord3 = [39.613214, -74.361318]
coord4 = [39.612942, -74.36149]
coord5 = [39.612847, -74.362702]

ip = "192.168.133.241"    #Enter IP address of laptop after connecting it to WIFI hotspot

#this code snippet is used to establish a connection between esp32 and this python code
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((ip, 8002))
    s.listen()
    conn, addr = s.accept()
    with conn:
        print(f"Connected by {addr}")
        conn.sendall(str.encode('b'))
        conn.close()
    print("Connection Closed")

#this list is created so as to store certain aruco markers when the bot comes closest to that marker
list1 = []

'''
the loop below implements the logic of finding the shortest distance between the 
fixed aruco on the bot and the aruco markers on the arena and updates the live location 
in the csvfile - "live_data.csv
'''
while True:
    # Read a frame from the webcam
    ret, frame = cap.read()
    image = cv2.resize(frame, (1920,1080), fx =1, fy = 1, interpolation=cv2.INTER_LINEAR)

    image2 = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # Detect ArUco markers and estimate poses
    corners, ids, rejectedImgPoints = aruco.detectMarkers(image2, aruco_dict, parameters=parameters)
    aruco.drawDetectedMarkers(image, corners, ids)


    # Draw markers and estimate poses
    if ids is not None:
        distances = {}
        fixed_marker_id = 100  #ID of the fixed marker

        # Find the index of the fixed marker in the detected markers
        fixed_marker_index = np.where(ids == fixed_marker_id)[0]

        # Check if the fixed marker is detected
        if fixed_marker_index.size > 0:
            fixed_center_nparray = centre_coord(fixed_marker_index, corners)

            for id, coordinates in dictionary.items():
                center_coordinates_nparray = dictionary[str(id)]
                distances[int(id)] = distance_cal(fixed_center_nparray, id)
                if len(distances) == len(dictionary):
                    print("------------------------------------------")

                    id1 = key_with_minimum_value(distances)
                    print(f'Minimum distance of {id1}: {distances[id1]}')
                    if (id1 == 50 and id1 not in list1):
                        list1.append(50)
                    if (id1 == 51 and id1 not in list1):
                        list1.append(51)
                    if (id1 == 31 and id1 not in list1):
                        list1.append(31)
                    if (id1 == 28 and id1 not in list1):
                        list1.append(28)
                    if (id1 == 26 and id1 not in list1):
                        list1.append(26)
                    if (id1 == 27 and id1 not in list1):
                        list1.append(27)
                        
                    if (id1 == 42 and 51 not in list1):
                        tracker1(coord1)
                    elif(id1 == 41 and 51 not in list1):
                        tracker(str(34), lat_lon)
                    elif(id1 == 38 or id1 == 37):
                        tracker(str(31), lat_lon)
                    elif(id1 == 30 and 31 in list1):
                        tracker1(coord3)
                    elif(id1 == 29 and 28 in list1):
                        tracker1(coord4)
                    elif(id1 == 25 and (26 not in list1 and 27 not in list1)):
                        tracker1(coord5)
                    else:
                        tracker(str(id1),lat_lon)

                    
                    print(list1)
                    print("------------------------------------------")                              
     
        else:
            print(f"Fixed marker with ID {fixed_marker_id} not found.")


    else:
        print("Any Aruco Marker not Found")

    # Display the frame
    cv2.namedWindow("Aruco Distance Measurement", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Aruco Distance Measurement", 800, 800)
    cv2.imshow('Aruco Distance Measurement', image)

    # Break the loop when 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):

        break

# Release the webcam and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()
