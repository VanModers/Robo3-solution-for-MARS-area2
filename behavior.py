from mars_interface import *
from blob_detection import findBlobs
import random
import copy
import sys
import numpy as np;

TURNING_DIF = 5.0
MINIMUM_DISTANCE = 2.0
MAXIMUM_SIZE = 200

behaviour = 0
left_cmd = 0
right_cmd = 0

global current_cam

selected_colour = np.array([0., 0., 0.])

colours = np.array([[0.3, 1., 0.3], [1., 1., 0.2]])

def findDominantBlob(pixelData, width, height, colours):
    allBlobs = []
    allBlobPixelLists = []
    allBlobCenters = []
    allBlobColours = []

    for colour in colours:
        logMessage("Colour: " + str(colour))
        blobCenters, blobs, blobPixelLists = findBlobs(pixelData, width, height, colour, 120, 121, 1, 40)
        allBlobCenters += blobCenters
        allBlobs += blobs
        allBlobPixelLists += blobPixelLists
        allBlobColours += [colour]*len(blobs)

    if len(allBlobs) == 0:
        return [], [], allBlobPixelLists[0], []

    index = 0
    for i in range(len(allBlobs)):
        dif = abs(width/2. - allBlobCenters[i][0])
        size1 = len(allBlobs[i]) - dif/(float(width)/10.)
        dif2 = abs(width/2. - allBlobCenters[index][0])
        size2 = len(allBlobs[index]) - dif2/(float(width)/10.)

        if size1 > size2:
            index = i

    #index = max((len(l) - abs(width/2. - allBlobCenters[i][0])/(float(width)/10.), i) for i, l in enumerate(allBlobs))[1]

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
        (left_cmd, right_cmd) = (-0.8, 0.8)
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

def findField(pixelData, width, height, pixelData2, width2, height2):
    global behaviour, selected_colour, left_cmd, right_cmd, current_cam, TURNING_DIF

    if len(pixelData) > 0:
        logMessage("Second Cam: Finding corner...")
        cornerCenter, cornerList, cornerPixelList, cornerColour = findDominantBlob(pixelData2, width2, height2, np.array([selected_colour]))
        if len(cornerCenter) > 0:
            logMessage("Corner at X: " + str(cornerCenter[0]) + " Y: " + str(cornerCenter[1]))

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
            current_cam = "cam0"
            behaviour = 1
            (left_cmd, right_cmd) = (-0.8, 0.8)
        return [blobPixelList, cornerPixelList]
    else:
        return [[], []]


options = {0 : findBlob,
           1 : driveTowardsBlob,
           2 : findBlobWithSecondCam,
           3 : findField
}

def initialBehavior():
    global behaviour, selected_colour, left_cmd, right_cmd, current_cam

    behaviour = 0
    current_cam = "cam0"
    selected_colour = np.array([0., 0., 0.])
    (left_cmd, right_cmd) = (0., 0.)

def doBehavior(marsData, pixelData, width, height, pixelData2, width2, height2):
    global behaviour, left_cmd, right_cmd, current_cam

    #logMessage(str(colours))

    pixelLists = options[behaviour](pixelData, width, height, pixelData2, width2, height2)

    return (left_cmd, right_cmd, pixelLists[0], pixelLists[1], current_cam)
