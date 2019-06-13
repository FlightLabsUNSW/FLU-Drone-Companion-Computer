# Importing Required Libraries
import numpy as np
import math
import json
import argparse

# === FUNCTIONS ===

# Import Waypoint and Obstacle Data from Mission Data
def extractMissionData(mission_data_input):
	### INPUTS ###
	# mission_data - dictionary - Received Mission Data
	### OUTPUTS ###
	# waypoints - list - List of Coordinate Vectors for Waypoints
	# exclusionPoints - list - List of Coordinate Vectors for Obstacles
	# exclusionRadius - list - List of Exclusion Radii around Obstacles

    # Importing the Data from the .json file
    with open(mission_data_input,'r') as data_f:
    	mission_data = json.load(data_f)

    waypoints = []
	# Extract the Latitude and Longitude from each Waypoint
    for waypoint in mission_data["waypoints"]:
            waypoints.append(np.array([waypoint["latitude"], waypoint["longitude"]]))

    exclusionPoints = []
    exclusionRadius = []
	# Extract the Latitude, Longitude and Radius from each Obstacle
    for obstacle in mission_data["stationaryObstacles"]:
        exclusionPoints.append(np.array([ obstacle["latitude"], obstacle["longitude"] ]))
        exclusionRadius.append(obstacle["radius"])

    return (waypoints, exclusionPoints, exclusionRadius)

### NOTE ###
# Coordinates of All Points should be given as:
# Latitude - Considered x Coordinates
# Longitude - Considered y Coordinates

# Determine Distance between Two Points (Also the Haversine Function!)
def distanceTwoPoints(firstPoint, secondPoint):
	### INPUTS ###
    # firstPoint - 2x1 numpy.array - coordinate Vector to First Point
    # secondPoint - 2x1 numpy.array - Coordinate Vector to Second Point
	### OUTPUT ###
	# distance - Float - Distance between Two Points

    R = 6372.795477598 # Radius of Earth in km

    # Haversine Function
    dLat = math.radians(secondPoint[0] - firstPoint[0])
    dLon = math.radians(secondPoint[1] - firstPoint[1])
    lat1 = math.radians(firstPoint[0])
    lat2 = math.radians(secondPoint[0])

    a = math.sin(dLat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dLon / 2)**2
    c = 2 * math.asin(math.sqrt(a))
    distance = R * c * 3280.84

    return distance

# Obtains Position of Third Point given Two Points and Distance between First and Third
def extendVecByDist(firstPoint, secondPoint, distance):
    ### INPUTS ###
    # firstPoint - 2x1 numpy.array - Coordinate Vector to First Point
    # secondPoint - 2x1 numpy.array - Coordinate Vector to Second Point
    # distance - float - Distance between First Point and Third Point
    ### OUTPUT ===
    # thirdPoint - 2x1 numpy.array - Coordinate Vector to Third Point

	# Mathematically Define Position of Third Point
    thirdPoint = firstPoint + distance/distanceTwoPoints(firstPoint, secondPoint)*(secondPoint - firstPoint)

    return thirdPoint

# Calculates the Angular Bearing between Two Points
def calculateBearing(firstPoint, secondPoint):
    ### INPUTS ###
    # firstPoint - 2x1 numpy.array - Coordinate Vector to First Point
    # secondPoint - 2x1 numpy.array - Coordinate Vector to Second Point
    ### OUTPUT ###
    # theta - float - Bearing Angle in Radians

	# # Determine change in latitude and longitude
    # delta_Lat = math.log( math.tan( secondPoint[0]/2 + math.pi/4 )/math.tan( firstPoint[0]/2 + math.pi/4) )
    # delta_Long = abs(firstPoint[1] - secondPoint[1])
    #
	# # Keep the change in longitude within the domain of -180 < delta_Long < 180
    # if delta_Long > 180:
    #     delta_Long = delta_Long % 180
    #
	# # Calculate bearing angle
    # bearing = math.atan2(delta_Long, delta_Lat)

    lat1 = math.radians(firstPoint[0])
    lat2 = math.radians(secondPoint[0])

    diffLong = math.radians(secondPoint[1] - firstPoint[1])

    x = math.sin(diffLong) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1) * math.cos(lat2) * math.cos(diffLong))

    initial_bearing = math.atan2(x, y)

    # Now we have the initial bearing but math.atan2 return values
    # from -180° to + 180° which is not what we want for a compass bearing
    # The solution is to normalize the initial bearing as shown below
    initial_bearing = math.degrees(initial_bearing)
    bearing = (initial_bearing + 360) % 360
    bearing = math.radians(bearing)

    return bearing

# Obtains Second Point given Distance and Bearing from First Point
def pointFromDistBear(firstPoint, thirdPoint, distance):
    ### INPUTS ###
    # firstPoint - 2x1 numpy.array - coordinate Vector to First Point
    # secondPoint - 2x1 numpy.array - Coordinate Vector to Second Point
    # distance - Float - Distance from First to Second Point
    ### OUTPUT ###
    # secondPoint - 2x1 numpy.array - Coordinate Vector to Second Point

    # Calculations: https://www.sunearthtools.com/tools/distance.php

    R = 6372.795477598 # Radius of Earth in km
    distance = distance / 3028.84
    bearing = calculateBearing(firstPoint, thirdPoint)

    lat1 = math.radians(firstPoint[0]);
    lon1 = math.radians(firstPoint[1]);

    lat2 = math.asin( math.sin(lat1)*math.cos(distance/R) +
         math.cos(lat1)*math.sin(distance/R)*math.cos(bearing))

    lon2 = lon1 + math.atan2(math.sin(bearing)*math.sin(distance/R)*math.cos(lat1),
                 math.cos(distance/R)-math.sin(lat1)*math.sin(lat2))

    lat2 = math.degrees(lat2)
    lon2 = math.degrees(lon2)

    secondPoint = [lat2, lon2]

    return secondPoint

# ===== Main Function =====
#Determine Flight Path with Given Waypoints and Target Points
def calculateFlightPath(waypoints, exclusionPoints, stepDistance, exclusionRadius, exclusionRadiusTolerance):
	### INPUTS ###
	# waypoints - nx1 numpy.array - List of Waypoint Coordinate Vectors
	# exclusionPoints - nx1 numpy.array - List of Centres for Exclusion Zones
	# stepDistance - float - Incremental Distance between Consecutive Target Points
	# exclusionRadius - nx1 list - List of Radius of Exclusion Zones to be Considered
	### Output ###
	# flightPath - nx1 numpy.array - List of Target Point Coordinate Vectors Tracing Flight Path

    if len(waypoints) < 2:
        print("There are not enough waypoints provided!")
        return None

    flightPath = []
	# For each pair of waypoints...
    for i in range(len(waypoints) - 1):
        flightPath.append(waypoints[i])
		# 1. Determine list of target points between two waypoints at set incremental distance
		# 1a. Find distance between two waypoints
        totalDistance = distanceTwoPoints(waypoints[i], waypoints[i+1])
		# 1b. Calculate total number of target points between two waypoints
        numPoints = math.floor(totalDistance/ stepDistance)
		# 1c. Locate target points at incremental distances
        targetPoints = []
        for j in range(numPoints):
            targetPoint = pointFromDistBear( waypoints[i], waypoints[i+1], stepDistance * (j+1))
			# 2. Shift target points if within exclusion radius
            exclusionDistance = []
			# 2a. For each target point, evaluate distance to all exclusion points
            for k in range(len(exclusionPoints)):
                exclusionDistance.append(distanceTwoPoints(targetPoint, exclusionPoints[k]))
			# 2b. Identify the closest distance
			# 2c. If distance is less than exclusion radius, shift target point to outside of it
            exclusionRadiusTotal = exclusionRadius[k] + exclusionRadiusTolerance
            if (min(exclusionDistance) < exclusionRadiusTotal):
                minIndex = exclusionDistance.index(min(exclusionDistance))
                targetPoint = extendVecByDist(exclusionPoints[minIndex], targetPoint, exclusionRadiusTotal)
                flightPath.append(targetPoint)
			# 2d. Iterate for the other target points
        flightPath.append(waypoints[i + 1])

    # 3. If three consecutive waypoints are collinear, remove the middle waypoint.
    removeIndices = []
    flightPathTemp = flightPath
    flightPath = [flightPathTemp[0]]

    for index, point in enumerate(flightPathTemp[1:-1]):
        bearing1 = calculateBearing(flightPathTemp[index-1], point)
        bearing2 = calculateBearing(point, flightPathTemp[index+1])
        if bearing1 != bearing2 and not np.array_equal(point, flightPath[-1]):
            flightPath.append(point)

    return flightPath

def exportMission(waypointType, flightPath, default_alt, header_name):
    ### INPUTS ###
    # waypointType - integer - Number defining Waypoint Type
    # flightPath - nx1 list - List storing Coordinate Vectors for Travel Points for Flight
    # default_alt - float - Default Altitude
    ### OUTPUT ###
    # mpmission.waypoint - waypoint file - Waypoint File for Mission Planner (Exported)

    file = open("mpmission.waypoints","w")

    file.write(header_name + "\n")
    for idx, point in enumerate(flightPath):
        file.write( str(idx) + "\t0\t0\t" + str(waypointType) + "\t0.000000\t0.000000\t0.000000\t0.000000\t" +
            "{:.6f}".format(point[0]) + "\t" + "{:.6f}".format(point[1]) + "\t" + "{:.6f}".format(default_alt) +
            "\t" + str(1) + "\n")

def main(mission_data, stepDistance, exclusionRadiusTolerance, waypointType, default_alt, header_name):
	### INPUTS ###
	# mission_data - dictionary - Received Mission Data
    ### OUTPUT ###
    # mpmission.waypoint - waypoint file - Waypoint File for Mission Planner

    # 1. Import Mission Data
    waypoints, exclusionPoints, exclusionRadius = extractMissionData(mission_data)

    # 2. Calculate the Flight Path
    flightPath = calculateFlightPath(waypoints, exclusionPoints, stepDistance, exclusionRadius, exclusionRadiusTolerance)

    # 3. Export the Mission to a File
    exportMission(waypointType, flightPath, default_alt, header_name)

# ArgParse Default Values
stepDistanceMin = 50.0 # in feet
exclusionRadiusToleranceMin = 200.0 # in feet
default_alt_min = 150.0 # in feet

# Parser Arguments
parser = argparse.ArgumentParser(description = "Constructs a Flight Path from Given Waypoints and Obstacles")
parser.add_argument("json_file", help = 'Input Mission Data .JSON File')
parser.add_argument("--step", action = "store", dest = "stepDistance", type = float, default = stepDistanceMin, help = "Step Distance between Target Point - Define in ft")
parser.add_argument("--tolerance", action = "store", dest = "exclusionRadiusTolerance", type = float, default = exclusionRadiusToleranceMin, help = "Tolerance for Exclusion Radius around Obstacles - Define in ft")
parser.add_argument("--altitude", action = "store", dest = "default_alt", type = float, default = default_alt_min, help = "Default Altitude for Completing Mission - Define in ft")

results = parser.parse_args()
mission_data = results.json_file
stepDistance = results.stepDistance
exclusionRadiusTolerance = results.exclusionRadiusTolerance
default_alt = results.default_alt

waypointType = 16
header_name = "QGC WPL 110"

main(mission_data, stepDistance, exclusionRadiusTolerance, waypointType, default_alt, header_name)
