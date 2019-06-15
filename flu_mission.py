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

connection_string_wren = args.connect
connection_string_cygnet = '10.0.2.15:14551'
## Change for winch
winchChannel = 4
winchTime = 30
sitl = None

# Start SITL if no connection string specified
if not connection_string_wren:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

# Speech
import pyttsx3
engine = pyttsx3.init()
engine.say("Connecting to wren and cygnet... please wait...")
engine.runAndWait()

#####
# Connect to the wren
#####
print('Connecting to Wren on: %s' % connection_string_wren)
wren = connect(connection_string_wren, wait_ready=True, baud=57600, heartbeat_timeout=120)

#####
# Connect to the cygnet
#####

print('Connecting to Cygnet on: %s' % connection_string_cygnet)
cygnet = connect(connection_string_cygnet, wait_ready=True, baud=57600, heartbeat_timeout=120)

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
# Upload WREN Mission Functions
#####
def upload_mission_wren(aFileName):
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

def readmission_wren(aFileName):
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
def do_arm_wren():

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

#####
# Upload cygnet Mission Functions
#####
def upload_mission_cygnet(aFileName):
    """
    Upload a mission from a file.
    """
    print(aFileName)
    # Read mission from file
    missionlist = readmission(aFileName)

    print("\nUpload mission from a file: %s" % aFileName)
    # Clear existing mission from cygnet
    print(' Clear mission')
    cmds = cygnet.commands
    cmds.clear()

    # Add new mission to cygnet
    for command in missionlist:
        cmds.add(command)
    print(' Upload mission')
    cygnet.commands.upload()

def readmission_cygnet(aFileName):
    """
    Load a mission from a file into a list.
    This function is used by upload_mission().
    """
    print("Reading mission from file: %s\n" % aFileName)
    cmds = cygnet.commands
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
# Arm cygnet
#####
def do_arm_cygnet():

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not cygnet.is_armable:
        print(" Waiting for cygnet to initialise...")
        time.sleep(1)

    # Speech
    import pyttsx3
    engine = pyttsx3.init()
    engine.say("Arming cygnet... please stand clear...")
    engine.runAndWait()

    print("Arming motors")
    # Copter should arm in GUIDED mode
    cygnet.mode = VehicleMode("GUIDED")
    cygnet.armed = True
    cygnet.flush()

    # Confirm cygnet armed before attempting to take off
    while not cygnet.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    # Speech
    print("We are armed!")
    engine = pyttsx3.init()
    engine.say("Motors Armed... preparing for drive...")
    engine.runAndWait()

print("Ready for mission? Press y:")
# Speech
import pyttsx3
engine = pyttsx3.init()
engine.say("Ready for mission? Press y")
engine.runAndWait()

while not input().lower().endswith("y"):
    continue

#####
# Upload Mission
#####
upload_mission_wren("mission/waypoints/uav_mission.waypoints")
upload_mission_cygnet("mission/waypoints/ugv_mission_update.waypoints")

#####
# Arm WREN
#####
do_arm_wren()

# wren.location.global_relative_frame.alt



#time.sleep(10)

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

time.sleep(1)
#####
# Set Wren mode to loiter and wait for next mission
#####
wren.mode = VehicleMode("LOITER")
time.sleep(1)

print("If ready for drop press y:")
# Speech
import pyttsx3
engine = pyttsx3.init()
engine.say("Ready to drop")
engine.runAndWait()

while not input().lower().endswith("y"):
    continue

#####
# Send winch
#####
#Sendit !!!!! lol
print("Dropping")
# Speech
import pyttsx3
engine = pyttsx3.init()
engine.say("Dropping the Package... Dropping the Package...")
engine.runAndWait()

wren.channels.overrides[winchChannel] = 1500
time.sleep(1)
wren.channels.overrides[winchChannel] = 0


#####
# Arm
#####
print("Waiting for drop")
# Speech
import pyttsx3
engine = pyttsx3.init()
engine.say("Waiting for drop to complete")
engine.runAndWait()

time.sleep(winchTime)

print("Drop Complete")
# Speech
import pyttsx3
engine = pyttsx3.init()
engine.say("Drop Complete")
engine.runAndWait()

print("Arming Cygnet")
# Speech
import pyttsx3
engine = pyttsx3.init()
engine.say("Arming Cygnet... Please stand clear...")
engine.runAndWait()

do_arm_cygnet()

# disconnect winch
print("Disconnecting winch")
# Speech
import pyttsx3
engine = pyttsx3.init()
engine.say("Disconnecting winch")
engine.runAndWait()
wren.channels.overrides[winchChannel] = 1500

cygnet.commands.next=0

#####
# Set mode to AUTO to start drop mission
#####
cygnet.mode = VehicleMode("AUTO")
last = 0

while True:
    nextwaypoint = cygnet.commands.next
    print("Next waypoint = ", nextwaypoint)
    if nextwaypoint == len(wren.commands):
        break

cygnet.armed = False
time.sleep(1)

wren.mode = VehicleMode("RTL")

wren.armed = False
time.sleep(1)

time.sleep(5)


#Close wren object before exiting script
print("Say bye to Wren")
wren.close()

print("Say bye to Cygnet")
cygnet.close()

# Shut down simulator if it was started.
if sitl is not None:
    sitl.stop()

#time.sleep(30)

#wren.mode = VehicleMode("GUIDED")

#print(math.degrees(wren.attitude.yaw))

#copy_control.copytree(image_dest, image_src)

#while not False: time.sleep(0.1)
