"""
    Example demonstrating how to communicate with Microsoft Robotic Developer
    Studio 4 via the Lokarria http interface.

    Author: Erik Billing (billing@cs.umu.se)

    Updated by Ola Ringdahl 204-09-11
    """
from serverIp import *
MRDS_URL = ip
index = 0

import httplib, json, time, sys
from pprint import pprint

from math import sin, cos, pi, atan2, sqrt, acos

HEADERS = {"Content-type": "application/json", "Accept": "text/json"}

class UnexpectedResponse(Exception): pass

def postSpeed(angularSpeed, linearSpeed):
    """Sends a speed command to the MRDS server"""
    mrds = httplib.HTTPConnection(MRDS_URL)
    params = json.dumps({'TargetAngularSpeed': angularSpeed, 'TargetLinearSpeed': linearSpeed})
    mrds.request('POST', '/lokarria/differentialdrive', params, HEADERS)
    response = mrds.getresponse()
    status = response.status
    # response.close()
    if status == 204:
        return response
    else:
        raise UnexpectedResponse(response)

def getPose():
    """Reads the current position and orientation from the MRDS"""
    mrds = httplib.HTTPConnection(MRDS_URL)
    mrds.request('GET', '/lokarria/localization')
    response = mrds.getresponse()
    if (response.status == 200):
        poseData = response.read()
        response.close()
        return json.loads(poseData)
    else:
        return UnexpectedResponse(response)

def bearing(q):
    return rotate(q, {'X': 1.0, 'Y': 0.0, "Z": 0.0})


def rotate(q, v):
    return vector(qmult(qmult(q, quaternion(v)), conjugate(q)))

def quaternion(v):
    q = v.copy()
    q['W'] = 0.0
    return q


def vector(q):
    v = {}
    v["X"] = q["X"]
    v["Y"] = q["Y"]
    v["Z"] = q["Z"]
    return v


def conjugate(q):
    qc = q.copy()
    qc["X"] = -q["X"]
    qc["Y"] = -q["Y"]
    qc["Z"] = -q["Z"]
    return qc


def qmult(q1, q2):
    q = {}
    q["W"] = q1["W"] * q2["W"] - q1["X"] * q2["X"] - q1["Y"] * q2["Y"] - q1["Z"] * q2["Z"]
    q["X"] = q1["W"] * q2["X"] + q1["X"] * q2["W"] + q1["Y"] * q2["Z"] - q1["Z"] * q2["Y"]
    q["Y"] = q1["W"] * q2["Y"] - q1["X"] * q2["Z"] + q1["Y"] * q2["W"] + q1["Z"] * q2["X"]
    q["Z"] = q1["W"] * q2["Z"] + q1["X"] * q2["Y"] - q1["Y"] * q2["X"] + q1["Z"] * q2["W"]
    return q

def getHeading():
    """Returns the XY Orientation as a bearing unit vector"""
    return bearing(getPose()['Pose']['Orientation'])

def getPosition():
    """Returns the XYZ position"""
    return getPose()['Pose']['Position']

"""Pythagoras theorem"""
def pythagorasHyp(x, y):
    return sqrt((x ** 2) + (y ** 2))

"""Add all coordinates in the path to a path stack, and reverse it so it works as a stack"""
def makePath():
    stack = []
    with open(sys.argv[1]) as path_file:
        jsonPath = json.load(path_file)
        for i in range (len(jsonPath)):
            stack.append(jsonPath[i]['Pose']['Position'])
        stack.reverse()
        return stack

"""Get the next goal point from the robot's position from a fixed look-a-head distance"""
def getGoalPoint(path, pos, lookAHead):
    if path:
        for i in range (len(path)):
            # Look at the last index
            p = path[len(path)-1]
            dx = p['X'] - pos['X']
            dy = p['Y'] - pos['Y']

            l = pythagorasHyp(dx,dy)

            if l < lookAHead:
                path.pop()
            else:
                return p
    else:
        print "Stack failed"


"""Convert a coordinate to the robot's coordinate system"""
def convertToRcs(pos, gP):
    list1 = []
    robotHeading = getHeading()

    # Calculate distance to the goal point from the robot
    dx = gP['X'] - pos['X']
    dy = gP['Y'] - pos['Y']
    l = pythagorasHyp(dx,dy)

    # Calculate the angle between the robot's pose and the world coordinate system
    hx = robotHeading['X']
    hy = robotHeading['Y']
    robotAngle = atan2(hy, hx)

    # Calculate the angle between the goal point and the world coordinate system
    pointAngle = atan2(gP['Y']-pos['Y'], gP['X']-pos['X'])

    # Calculate the angle between the goal point and the robot coordinate system
    diffAngle = pointAngle - robotAngle

    # Calculate the goal point's y-coordinate relative to the robot's coordinate system
    yP = sin(diffAngle) / l

    list1.insert(0,yP)
    list1.insert(1,l)

    return list1

"""Calculate the curvature to the goal point for the robot to follow"""
def calculateCurvatureToGp(convertedGp):

    yP = convertedGp[0]
    l = convertedGp[1]

    # Calculate the curvature, gamma
    gamma = (2*yP)/(l**2)
    return gamma

if __name__ == '__main__':
    path = makePath()
    ls = 1
    lookAHead = 0.9
    try:
        while path:
            pos = getPosition()
            orientation = getPose()['Pose']['Orientation']
            gP = getGoalPoint(path, pos, lookAHead)

            if(gP):
                convertedGp = convertToRcs(pos, gP)
                gamma = calculateCurvatureToGp(convertedGp)
                response = postSpeed(gamma*ls, ls)
                #time.sleep(0.01)
        # Stop
        response = postSpeed(0,0)
        print ('Completed!')

    except UnexpectedResponse, ex:
        print 'Unexpected response from server when reading position:', ex