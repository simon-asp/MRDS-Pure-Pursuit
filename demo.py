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

def getHeading():
    """Returns the XY Orientation as a bearing unit vector"""
    return bearing(getPose()['Pose']['Orientation'])

def getPosition():
    """Returns the XYZ position"""
    return getPose()['Pose']['Position']

def getBearing():
    """Returns the XY Orientation as a bearing unit vector"""
    return bearing(getPose()['Pose']['Orientation'])

"""Add all coordinates in the path to a path stack, and reverse it so it works as a stack"""
def makePath():
    stack = []
    with open('path/path-around-table.json') as path_file:
        jsonPath = json.load(path_file)
        for i in range (len(jsonPath)):
            stack.append(jsonPath[i]['Pose']['Position'])
        stack.reverse()
        return stack

"""Get the next goal point from the robot's position from a fixed look-a-head distance"""
def getGoalPoint(path, rpos):
    lookAHead = 0.03

    for i in range (len(path)):
        p = path.pop()
        dx = p['X'] - rpos['X']
        dy = p['Y'] - rpos['Y']

        l = sqrt((dx**2)+(dy**2))
        print 'pythagoras: ', l

        print 'popped path: ', p, 'index: ', i

        if l > lookAHead:
            print 'l: ', l
            print 'i: ', i
            return p


"""Convert a coordinate to the robot's coordinate system"""
def convertToRcs(pos, orientation, gP):
    list1 = []
    w = round(orientation['W'],2)
    #robotHeading = getHeading()

    # Calculate the angle between the robot's pose and the world coordinate system
    #hx = robotHeading['X']
    #hy = robotHeading['Y']
    #yaw = atan2(hy-y0, hx-x0)
    angle = 2 * acos(w)
    print 'angle: ', angle

    # Calculate the difference between robot's coordinates and goal point coordinates
    dx = gP['X'] - pos['X']
    dy = gP['Y'] - pos['Y']

    # Calculate the goal point coordinates corresponding to the robot's coordinates
    # using linear algebra rotation matrices
    xP = (cos(angle)*dx) - (sin(angle)*dy)
    yP = (sin(angle)*dx) + (cos(angle)*dy)

    list1.insert(0,xP)
    list1.insert(1,yP)

    return list1

"""Calculate the curvature to the goal point for the robot to follow"""
def calculateCurvatureToGp(robotGp):

    print 'gp converted: ', robotGp
    x = robotGp[0]
    y = robotGp[1]

    # Calculate the tangent to the goal point, l
    l = sqrt((x**2) + (y**2))
    print 'new l: ', l

    # Calculate the curvature, gamma
    gamma = (2*x)/(l**2)

    return gamma


if __name__ == '__main__':
    print 'Sending commands to MRDS server', MRDS_URL
    path = makePath()
    ls = 0.3
    try:
        while(len(path) != 0):
            print 'Robot Current heading vector: X:{X:.3}, Y:{Y:.3}'.format(**getHeading())
            print 'current path size: ', len(path)

            pos = getPosition()
            orientation = getPose()['Pose']['Orientation']
            print 'Robot current pos: ', pos
            print 'Robot current orientation: ', orientation

            gP = getGoalPoint(path, pos)
            print 'Gp original: ', gP
            robotGp = convertToRcs(pos, orientation, gP)
            gamma = calculateCurvatureToGp(robotGp)
            print 'gamma: ', gamma

            response = postSpeed(gamma*ls, ls)
            print ' '
            time.sleep(5)


    except UnexpectedResponse, ex:
        print 'Unexpected response from server when reading position:', ex