from mars_interface import *
from time import clock
from blob_detection import findBlobs
import random
import copy
import sys
import numpy as np;

TURNING_DIF = 3.0
CORNER_DIF = 12.0
MINIMUM_DISTANCE = 2.0
MINIMUM_SIZE = 60
MAXIMUM_SIZE = 500

behaviour = 0
left_cmd = 0
right_cmd = 0

global current_cam

NumberOfBalls = [1, 2, 2, 2]

selected_colour = np.array([0., 0., 0.])

colours = np.array([[0.3, 1., 0.3], [1., 1., 0.2], [1., 0.1, 0.1], [0.2, 0.2, 1.]])

drive_clock = clock()

def findDominantBlob(pixelData, width, height, colours):
    allBlobs = []
    allBlobPixelLists = []
    allBlobCenters = []
    allBlobColours = []

    for colour in colours:
        blobCenters, blobs, blobPixelLists = findBlobs(pixelData, width, height, colour, 120, 121, 1, 10)
        allBlobCenters += blobCenters
        allBlobs += blobs
        allBlobPixelLists += blobPixelLists
        allBlobColours += [colour]*len(blobs)

    if len(allBlobs) == 0:
        return [], [], allBlobPixelLists[0], []

    index = max((len(l) - abs(width/2. - allBlobCenters[i][0])/(float(width)/10.), i) for i, l in enumerate(allBlobs))[1]

    return allBlobCenters[index], allBlobs[index], allBlobPixelLists[index], allBlobColours[index]

def alignWithBlob(center, width):
    global left_cmd, right_cmd, TURNING_DIF

    dif = center[0] - width/2.
    if abs(dif) < TURNING_DIF:
        (left_cmd, right_cmd) = (0.0, 0.0)
        return True
    logMessage("Dif: " + str(abs(dif)) + " > " + str(TURNING_DIF))
    if dif < 0.:
        (left_cmd, right_cmd) = (0.2, -0.2)
    else:
        (left_cmd, right_cmd) = (-0.2, 0.2)
    return False


def findBlob(pixelData, width, height, pixelData2, width2, height2):
    global behaviour, selected_colour, left_cmd, right_cmd, current_cam

    if len(pixelData) > 0:
        logMessage("First Cam: Finding blobs...")
        blobCenter, blobList, blobPixelList, blobColour = findDominantBlob(pixelData, width, height, colours)
        if len(blobCenter) > 0:
            logMessage("Blob at X: " + str(blobCenter[0]) + " Y: " + str(blobCenter[1]) + " Colour: " + str(blobColour) + " Size: " + str(len(blobList)))
            if alignWithBlob(blobCenter, width):
                selected_colour = copy.copy(blobColour)
                behaviour = 1
                (left_cmd, right_cmd) = (-0.9, -0.9)
            logMessage("")
            return [blobPixelList, []]
        (left_cmd, right_cmd) = (-0.2, 0.2)
        logMessage("")
    return [[], []]

def driveTowardsBlob(pixelData, width, height, pixelData2, width2, height2):
    global behaviour, selected_colour, left_cmd, right_cmd, current_cam

    (left_cmd, right_cmd) = (-1.2, -1.2)

    if len(pixelData) > 0:
        blobCenter, blobList, blobPixelList, blobColour = findDominantBlob(pixelData, width, height, np.array([selected_colour]))

        if len(blobCenter) > 0:
            if not alignWithBlob(blobCenter, width):
                behaviour = 0
                (left_cmd, right_cmd) = (0.0, 0.0)
                return [blobPixelList, []]

            logMessage("Size: " + str(len(blobList)))
            logMessage("")

            if len(blobList) > MAXIMUM_SIZE:
                current_cam = "cam1"
                behaviour = 2
                (left_cmd, right_cmd) = (-0.8, 0.8)
        else:
            behaviour = 0
            (left_cmd, right_cmd) = (0.0, 0.0)
            return [blobPixelList,[]]

        return [blobPixelList, []]
    return [[], []]

def findBlobWithSecondCam(pixelData, width, height, pixelData2, width2, height2):
    global behaviour, selected_colour, left_cmd, right_cmd, current_cam

    if len(pixelData) > 0:
        logMessage("Second Cam: Finding blobs...")
        blobCenter, blobList, blobPixelList, blobColour = findDominantBlob(pixelData, width, height, np.array([selected_colour]))
        if len(blobCenter) > 0 and len(blobList) > MAXIMUM_SIZE/2.:
            logMessage("Blob at X: " + str(blobCenter[0]) + " Y: " + str(blobCenter[1]))
            if alignWithBlob(blobCenter, width):
                logMessage("Stopping...")
                (left_cmd, right_cmd) = (-0.9, -0.9)
                behaviour = 3
        else:
            (left_cmd, right_cmd) = (-0.8, 0.8)
        return [blobPixelList, []]
    else:
        return [[], []]

def findBlobWithFirstCam(pixelData, width, height, pixelData2, width2, height2):
    global behaviour, selected_colour, left_cmd, right_cmd, current_cam

    if len(pixelData) > 0:
        logMessage("First Cam: Finding blob...")
        blobCenter, blobList, blobPixelList, blobColour = findDominantBlob(pixelData, width, height, np.array([selected_colour]))
        if len(blobCenter) > 0 and len(blobList) > MAXIMUM_SIZE/2.:
            logMessage("Blob at X: " + str(blobCenter[0]) + " Y: " + str(blobCenter[1]))
            if alignWithBlob(blobCenter, width):
                logMessage("Stopping...")
                (left_cmd, right_cmd) = (-0.9, -0.9)
                behaviour = 5
        else:
            (left_cmd, right_cmd) = (0.8, -0.8)
        return [blobPixelList, []]
    else:
        return [[], []]

def findField(pixelData, width, height, pixelData2, width2, height2):
    global behaviour, selected_colour, left_cmd, right_cmd, current_cam, TURNING_DIF

    if len(pixelData) > 0:
        logMessage("Second Cam: Finding corner...")
        cornerCenter, cornerList, cornerPixelList, cornerColour = findDominantBlob(pixelData2, width2, height2, np.array([selected_colour]))
        if len(cornerCenter) > 0:
            logMessage("Corner at X: " + str(cornerCenter[0]) + " Y: " + str(cornerCenter[1]))
            if abs(cornerCenter[0] - width/2.) < CORNER_DIF:
                logMessage("Turning towards blob...")
                current_cam = "cam0"
                behaviour = 4
                (left_cmd, right_cmd) = (0.8, -0.8)

        blobCenter, blobList, blobPixelList, blobColour = findDominantBlob(pixelData, width, height, np.array([selected_colour]))
        logMessage("")
        if len(blobCenter) > 0:
            (left_cmd, right_cmd) = (-1.2, -1.2)
            dif = blobCenter[0] - width/2.
            #logMessage("Blob at X: " + str(blobCenter[0]) + " Y: " + str(blobCenter[1]) + " Dif: " + str(dif))
            if abs(dif) > TURNING_DIF:
                if dif < 0:
                    (left_cmd, right_cmd) = (0.0, -0.4)
                else:
                    (left_cmd, right_cmd) = (-0.4, 0.0)
        else:
            logMessage("Lost blob, trying to find it again...")
            current_cam = "cam0"
            behaviour = 1
            (left_cmd, right_cmd) = (0.8, -0.8)
        return [blobPixelList, cornerPixelList]
    else:
        return [[], []]

def pushBall(pixelData, width, height, pixelData2, width2, height2):
    global behaviour, selected_colour, left_cmd, right_cmd, current_cam, drive_clock

    (left_cmd, right_cmd) = (-1.2, -1.2)

    if len(pixelData) > 0:
        blobCenter, blobList, blobPixelList, blobColour = findDominantBlob(pixelData, width, height, np.array([selected_colour]))

        if len(blobCenter) > 0:
            if not alignWithBlob(blobCenter, width):
                behaviour = 4
                (left_cmd, right_cmd) = (0.0, 0.0)
                return [blobPixelList, []]

            logMessage("Size: " + str(len(blobList)))
            if len(blobList) > MAXIMUM_SIZE:
                logMessage("Pushing ball...")
                drive_clock = clock()
                behaviour = 6
                (left_cmd, right_cmd) = (-0.8, -0.8)
        else:
            behaviour = 0
            (left_cmd, right_cmd) = (0.0, 0.0)
            return [blobPixelList,[]]

        return [blobPixelList, []]
    return [[], []]

def driveForSomeTime(pixelData, width, height, pixelData2, width2, height2):
    global behaviour, selected_colour, left_cmd, right_cmd, current_cam

    (left_cmd, right_cmd) = (-1.2, -1.2)
    if (clock() - drive_clock) > 12.:
        (left_cmd, right_cmd) = (-1.0, 1.0)
        behaviour = 7
    return [[], []]

def turnAwayFromBlob(pixelData, width, height, pixelData2, width2, height2):
    global behaviour, selected_colour, left_cmd, right_cmd, current_cam

    if len(pixelData) > 0:
        logMessage("First Cam: Turning away...")
        blobCenter, blobList, blobPixelList, blobColour = findDominantBlob(pixelData, width, height, np.array([selected_colour]))
        if len(blobCenter) > 0:
            logMessage("Blob at X: " + str(blobCenter[0]) + " Y: " + str(blobCenter[1]))
        else:
            drive_clock = clock()
            (left_cmd, right_cmd) = (-1.0, 1.0)
            behaviour = 8
        return [blobPixelList, []]
    else:
        return [[], []]

def turnForSomeTime(pixelData, width, height, pixelData2, width2, height2):
    global behaviour, selected_colour, left_cmd, right_cmd, current_cam

    (left_cmd, right_cmd) = (-1.2, 1.2)
    if (clock() - drive_clock) > 4.:
        (left_cmd, right_cmd) = (0.0, 0.0)
        behaviour = 0
    return [[], []]


options = {0 : findBlob,
           1 : driveTowardsBlob,
           2 : findBlobWithSecondCam,
           3 : findField,
           4 : findBlobWithFirstCam,
           5 : pushBall,
           6 : driveForSomeTime,
           7 : turnAwayFromBlob,
           8 : turnForSomeTime
}

def initialBehaviour():
    global behaviour, selected_colour, left_cmd, right_cmd, current_cam

    behaviour = 0
    current_cam = "cam0"
    selected_colour = np.array([0., 0., 0.])
    (left_cmd, right_cmd) = (0., 0.)

def doBehaviour(marsData, pixelData, width, height, pixelData2, width2, height2):
    global behaviour, left_cmd, right_cmd, current_cam

    if len(pixelData) > 0:
        logMessage("Behaviour: " + str(behaviour))

    pixelLists = options[behaviour](pixelData, width, height, pixelData2, width2, height2)

    return (left_cmd, right_cmd, pixelLists[0], pixelLists[1], current_cam)
