"""
    Example demonstrating how to communicate with Microsoft Robotic Developer
    Studio 4 via the Lokarria http interface.

    Author: Erik Billing (billing@cs.umu.se)

    Updated by Ola Ringdahl 204-09-11
    """
from serverIp import *
MRDS_URL = ip
index = 0

import httplib, json, time
from pprint import pprint

from math import sin, cos, pi, atan2, sqrt

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


def getLaser():
    """Requests the current laser scan from the MRDS server and parses it into a dict"""
    mrds = httplib.HTTPConnection(MRDS_URL)
    mrds.request('GET', '/lokarria/laser/echoes')
    response = mrds.getresponse()
    if (response.status == 200):
        laserData = response.read()
        response.close()
        return json.loads(laserData)
    else:
        return response


def getLaserAngles():
    """Requests the current laser properties from the MRDS server and parses it into a dict"""
    mrds = httplib.HTTPConnection(MRDS_URL)
    mrds.request('GET', '/lokarria/laser/properties')
    response = mrds.getresponse()
    if (response.status == 200):
        laserData = response.read()
        response.close()
        properties = json.loads(laserData)
        beamCount = int((properties['EndAngle'] - properties['StartAngle']) / properties['AngleIncrement'])
        a = properties['StartAngle']  # +properties['AngleIncrement']
        angles = []
        while a <= properties['EndAngle']:
            angles.append(a)
            a += pi / 180  # properties['AngleIncrement']
        # angles.append(properties['EndAngle']-properties['AngleIncrement']/2)
        return angles
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

"""Gets the next point from look-a-head distance of the robot"""
def getGoalPoint():
    list = []
    global index
    with open('path/path-around-table.json') as path_file:
        path = json.load(path_file)

        list.insert(0, path[index]['Pose']['Position']['X'])
        list.insert(1, path[index]['Pose']['Position']['Y'])
        print 'index: ', index
        index += 5
        print 'goalpoint: ', list
        return list

def getHeading():
    """Returns the XY Orientation as a bearing unit vector"""
    return bearing(getPose()['Pose']['Orientation'])

def getPosition():
    """Returns the XYZ position"""
    return getPose()['Pose']['Position']

def getBearing():
    """Returns the XY Orientation as a bearing unit vector"""
    return bearing(getPose()['Pose']['Orientation'])

"""Convert the robot position to RCS"""
def convertToRcs(x0, y0):
    list1 = []
    heading = getHeading()

    xP = heading['X']
    yP = heading['Y']

    # The inverse of the difference of the angles gives the right yaw
    yaw = atan2(1/yP-y0, 1/xP-x0)
    print 'yaw, ', yaw
    # Convert the coordinates using this rule
    x = x0 + xP*cos(yaw) - yP*sin(yaw)
    y = y0 + xP*sin(yaw) + yP*cos(yaw)

    list1.insert(0,x)
    list1.insert(1,y)

    return list1

"""Calculate the curvature to the goal point for the robot to follow"""
def calculateCurvatureToGp():

    # Convert the goal point to the robot's coordinates
    gP = getGoalPoint()
    gPList = convertToRcs(gP[0], gP[1])

    print 'gp converted: ', gPList
    x = gPList[0]
    y = gPList[1]

    # Calculate the tangent to the goal point, l
    l = sqrt(x**2) + sqrt(y**2)

    # Calculate the curvature, gamma
    gamma = 2*x / l**2
    print 'gamma: ', gamma
    print ' '

    return gamma


if __name__ == '__main__':
    print 'Sending commands to MRDS server', MRDS_URL

    try:
        for t in range(1000):
            print 'Robot Current position: X:{X:.3}, Y:{Y:.3}'.format(**getPosition())
            print 'Robot Current heading vector: X:{X:.3}, Y:{Y:.3}'.format(**getHeading())
            pos = getPosition()
            print 'Robots position in RCS: ', convertToRcs(pos['X'], pos['Y'])

            gamma = calculateCurvatureToGp()
            linearSpeed = 0.2
            angularSpeed = gamma * linearSpeed
            response = postSpeed(angularSpeed, linearSpeed)
            time.sleep(0.1)


    except UnexpectedResponse, ex:
        print 'Unexpected response from server when reading position:', ex