#!/usr/local/bin/python3

#####
# WELCOME TO FLU
#####
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

# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

# Speech
import pyttsx3
engine = pyttsx3.init()
engine.say("Connecting to wren... please wait...")
engine.runAndWait()

#####
# Connect to the wren
#####
print('Connecting to wren on: %s' % connection_string)
wren = connect(connection_string, wait_ready=True, baud=57600, heartbeat_timeout=120)

# Speech
import pyttsx3
engine = pyttsx3.init()
engine.say("wren Connected!")
engine.runAndWait()

'''
while True:
    print("Loop: %s" % str(wren.last_heartbeat))
    time.sleep(1)
'''
#time.sleep(10)

#####
# Upload Mission Functions
#####
def upload_mission(aFileName):
    """
    Upload a mission from a file.
    """
    print(aFileName)
    # Read mission from file
    missionlist = readmission(aFileName)

    print("\nUpload mission from a file: %s" % aFileName)
    # Clear existing mission from wren
    print(' Clear mission')
    cmds = wren.commands
    cmds.clear()

    # Add new mission to wren
    for command in missionlist:
        cmds.add(command)
    print(' Upload mission')
    wren.commands.upload()

def readmission(aFileName):
    """
    Load a mission from a file into a list.
    This function is used by upload_mission().
    """
    print("Reading mission from file: %s\n" % aFileName)
    cmds = wren.commands
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

#####
# Arm Wren
#####
def do_arm():
    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not wren.is_armable:
        print(" Waiting for wren to initialise...")
        time.sleep(1)

    # Speech
    import pyttsx3
    engine = pyttsx3.init()
    engine.say("Arming motors... please stand clear...")
    engine.runAndWait()

    print("Arming motors")
    # Copter should arm in GUIDED mode
    wren.mode = VehicleMode("GUIDED")
    wren.armed = True
    wren.flush()

    # Confirm wren armed before attempting to take off
    while not wren.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    # Speech
    print("We are armed!")
    engine = pyttsx3.init()
    engine.say("Motors Armed... preparing for takeoff...")
    engine.runAndWait()

do_arm()
    '''
    print("Taking off!")
    wren.simple_takeoff(5)  # Take off to target altitude
    # Wait until the wren reaches a safe height before processing the goto
    #  (otherwise the command after wren.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", wren.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if wren.location.global_relative_frame.alt >= 5 * 0.95:
            print("Reached target altitude")
            break
        time.sleep(0.25)
    '''

#####
# Upload Mission
#####
upload_mission(mission)

#time.sleep(10)
# #eee

wren.channels.overrides['3'] = 1500

wren.commands.next=0

#####
# Set mode to AUTO to start drop mission
#####
wren.mode = VehicleMode("AUTO")
last = 0

while True:
    nextwaypoint = wren.commands.next
    print("Next waypoint = ", nextwaypoint)
    if nextwaypoint == len(wren.commands):
        break

#####
# Set Wren mode to loiter and wait for next mission
#####


#####
# Set mode to loiter and wait for next mission
#####


#####
# Set mode to AUTO to start drop mission
#####

#####
# Set mode to loiter and wait for next mission
#####

#####
# Set mode to AUTO to start drop mission
#####


#print("Returning to Launch")
#wren.mode = VehicleMode("RTL")
'''
while True:
    print(" Altitude: ", wren.location.global_relative_frame.alt)
    # Break and return from function just below target altitude.
    if wren.location.global_relative_frame.alt <= 0 + 1:
        print("Landed")
        break
    time.sleep(0.25)
'''

time.sleep(5)

## TODO:
# add second vechile
# add loiter

#Close wren object before exiting script
print("Close wren object")
wren.close()

# Shut down simulator if it was started.
if sitl is not None:
    sitl.stop()

#wren.mode = VehicleMode("GUIDED")

#print(math.degrees(wren.attitude.yaw))

#copy_control.copytree(image_dest, image_src)

#while not False: time.sleep(0.1)

#time.sleep(30)
