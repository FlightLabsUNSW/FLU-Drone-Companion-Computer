#!/usr/local/bin/python3

from __future__ import print_function
import time
import json
import math
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, mavutil

# Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None

'''
def load_params():
    with open("mission_config.dat", "r") as f:
        for line in f:
            if line != "":
                process_param(line)


def process_param(line):
    global zip, ssid, profile, mission, zip_area, image_src, image_dest, camera_ssid, camera_profile
    line = line.replace(" ", "")
    s = line.split("=")
    if(len(s) < 2): return
    type = s[0]
    data = s[1]

    if type == "zip":
        zip = int(data)
    elif type == "wifi_ssid":
        wifi_ssid = data
    elif type == "wifi_profile":
        wifi_profile = data
    elif type == "mission_name":
        mission = "missions/" + data
    elif type == "zip_area":
        zip_area = data
        #print("Zip area: ", zip_area)
    elif type == "image_src":
        image_src = data
    elif type == "image_dest":
        image_dest = data
    elif type == "camera_ssid":
        camera_ssid = data
    elif type == "camera_profile":
        camera_profile = data



load_params()
'''


# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()


import pyttsx3

engine = pyttsx3.init()
engine.say("Connecting to vehicle... please wait...")
engine.runAndWait()

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True, baud=57600, heartbeat_timeout=120)

import pyttsx3

engine = pyttsx3.init()
engine.say("Vehicle Connected!")
engine.runAndWait()

'''
while True:
    print("Loop: %s" % str(vehicle.last_heartbeat))
    time.sleep(1)
'''
#time.sleep(10)


def upload_mission(aFileName):
    """
    Upload a mission from a file.
    """
    print(aFileName)
    # Read mission from file
    missionlist = readmission(aFileName)

    print("\nUpload mission from a file: %s" % aFileName)
    # Clear existing mission from vehicle
    print(' Clear mission')
    cmds = vehicle.commands
    cmds.clear()

    # Add new mission to vehicle
    for command in missionlist:
        cmds.add(command)
    print(' Upload mission')
    vehicle.commands.upload()

def readmission(aFileName):
    """
    Load a mission from a file into a list.
    This function is used by upload_mission().
    """
    print("Reading mission from file: %s\n" % aFileName)
    cmds = vehicle.commands
    missionlist=[]
    with open(aFileName) as f:
        for i, line in enumerate(f):
            if i==0:
                if not line.startswith('QGC WPL 110'):
                    raise Exception('File is not supported WP version')
            else:
                linearray=line.split('\t')
                ln_index=int(linearray[0])
                ln_currentwp=int(linearray[1])
                ln_frame=int(linearray[2])
                ln_command=int(linearray[3])
                ln_param1=float(linearray[4])
                ln_param2=float(linearray[5])
                ln_param3=float(linearray[6])
                ln_param4=float(linearray[7])
                ln_param5=float(linearray[8])
                ln_param6=float(linearray[9])
                ln_param7=float(linearray[10])
                ln_autocontinue=int(linearray[11].strip())
                cmd = Command( 0, 0, 0, ln_frame, ln_command, ln_currentwp, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_param5, ln_param6, ln_param7)
                missionlist.append(cmd)
    return missionlist

def do_arm():
    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    import pyttsx3

    engine = pyttsx3.init()
    engine.say("Arming motors... please stand clear...")
    engine.runAndWait()

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    vehicle.flush()

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("We are armed!")
    engine = pyttsx3.init()
    engine.say("Motors Armed... preparing for takeoff...")
    engine.runAndWait()

do_arm()
    '''
    print("Taking off!")
    vehicle.simple_takeoff(5)  # Take off to target altitude
    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= 5 * 0.95:
            print("Reached target altitude")
            break
        time.sleep(0.25)
    '''


upload_mission(mission)

#time.sleep(10)
# #eee

vehicle.channels.overrides['3'] = 1500

vehicle.commands.next=0

# Set mode to AUTO to start mission
vehicle.mode = VehicleMode("AUTO")
last = 0

while True:
    nextwaypoint = vehicle.commands.next
    print("Next waypoint = ", nextwaypoint)
    if nextwaypoint == len(vehicle.commands):
        break


#print("Returning to Launch")
#vehicle.mode = VehicleMode("RTL")
'''
while True:
    print(" Altitude: ", vehicle.location.global_relative_frame.alt)
    # Break and return from function just below target altitude.
    if vehicle.location.global_relative_frame.alt <= 0 + 1:
        print("Landed")
        break
    time.sleep(0.25)
'''

time.sleep(5)

#vehicle.mode = VehicleMode("GUIDED")

#print(math.degrees(vehicle.attitude.yaw))

#copy_control.copytree(image_dest, image_src)

#while not False: time.sleep(0.1)

#time.sleep(30)
