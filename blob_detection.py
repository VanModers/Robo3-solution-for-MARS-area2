from mars_interface import *
import copy
import sys
import numpy as np;

blobCenters = np.array([[]])
blobPixels = np.array([[]])

def convertBlobToPixelList(blobList, width, height, colours):
    pixelList = []
    for blobID, blob in enumerate(blobList):
        pixelList.append([0]*width*height*colours)
        for blobPixel in blob:
            for i in range(colours) :
                pixelList[blobID][blobPixel*3 + i] = 255

    if len(blobList) == 0:
        pixelList.append([0]*width*height*colours)
    return pixelList

def Elements(elements, startPosition):
    copiedElements = copy.copy(elements)
    while startPosition < len(copiedElements):
        yield copiedElements[startPosition]
        startPosition += 1

def isPartOfBlob(blob, pixelID):
    for i in range(len(blob)):
        if blob[i] == pixelID:
            return True
    return False

def isPartOfBlobs(blobs, pixelID):
    for i in range(len(blobs)):
        if isPartOfBlob(blobs[i], pixelID):
            return True
    return False

def getMagnitude(vec1, vec2):
    vec3 = vec1 - vec2
    return np.sqrt(vec3.dot(vec3))

def getCenterOfBlob(blob, width):
    center = np.array([0, 0])
    for i in range(len(blob)):
        y = int(blob[i] / width)
        x = int(blob[i] - y*width)
        center[0] += x
        center[1] += y
    center /= len(blob)
    return center

def getDimensionsOfBlob(blob, width, height):
    maxX = 0
    maxY = 0
    minX = width
    minY = height
    for pixel in blob:
        y = int(pixel / width)
        x = int(pixel - y*width)
        maxX = max(x, maxX)
        maxY = max(y, maxY)
        minX = min(x, minX)
        minY = max(y, minY)
    dimensions = np.array([maxX, maxY, minX, minY])
    return dimensions


def mergeBlobs(blob1, blob2):
    blob1Set = set(blob1)
    blob1Set.union(blob2)
    return list(blob1Set)

def findNeighbours(binaryImage, width, height, pixelX, pixelY, blobs, newBlob):
    newBlob.append(int(pixelX + pixelY*width))

    for x in range(-1,2):
        currentPixelX = min(max(pixelX + x, 0), width - 1)
        currentPixelY = min(max(pixelY + 0, 0), height - 1)
        currentPixelID = int(currentPixelX + currentPixelY*width)
        if x != 0 and binaryImage[currentPixelX][currentPixelY] == 1 and not isPartOfBlobs(blobs, currentPixelID) and not isPartOfBlob(newBlob, currentPixelID):
            findNeighbours(binaryImage, width, height, currentPixelX, currentPixelY, blobs, newBlob)

    for y in range(-1,2):
        currentPixelX = min(max(pixelX + 0, 0), width - 1)
        currentPixelY = min(max(pixelY + y, 0), height - 1)
        currentPixelID = int(currentPixelX + currentPixelY*width)
        if y != 0 and binaryImage[currentPixelX][currentPixelY] == 1 and not isPartOfBlobs(blobs, currentPixelID) and not isPartOfBlob(newBlob, currentPixelID):
            findNeighbours(binaryImage, width, height, currentPixelX, currentPixelY, blobs, newBlob)
    return newBlob


def blobExtraction(binaryImage, width, height):
    blobs = []
    for x in range(width):
        for y in range(height):
            if binaryImage[x][y] == 1 and not isPartOfBlobs(blobs, x+y*width):
                blobs.append(findNeighbours(binaryImage, width, height, x, y, blobs, []))
    return blobs


def findBlobs(pixelData, width, height, colour, minThreshold, maxThreshold, thresholdSteps, minDistBetweenBlobs):
    # Filter image by given colour

    dominantColour, index = max((v, i) for i, v in enumerate(colour))

    if dominantColour < 1e-8:
        dominantColour = 1.

    for v in colour:
        v /= dominantColour

    dominantColour *= 255.
    errors = []
    colouredPixelData = np.empty((width, height))
    for x in range(width):
        pixelLine = np.array([])
        for y in range(height):
            #logMessage("R: " + str(pixelData[x][y][0]) + " G: " + str(pixelData[x][y][1]) + " B: " + str(pixelData[x][y][2]))
            error = 0.
            for i in range(3):
                error += (abs(colour[i] -(float(pixelData[x][y][i]) / float(pixelData[x][y][index]))) * 255.)
                #value = value + (pixelData[x][y][i] - minimumColourValue) * (1./(1.-minimumColourPercentage)) * colour[i]
            colouredPixelData[x][y] = int(error)
            errors += [int(error)]

    sys.setrecursionlimit(10000)

    #logMessage("Best threshold: " + str(min(errors)))

    # Create binary images
    blobsOfAllBinaryImages = []
    threshold = minThreshold
    for i in range((maxThreshold - minThreshold)/thresholdSteps):
        binaryImage = np.empty((width, height))
        for x in range(width):
            pixelLine = np.array([])
            for y in range(height):
                if colouredPixelData[x][y] < threshold:
                    binaryImage[x][y] = 1
                else:
                    binaryImage[x][y] = 0
        minThreshold += thresholdSteps
        # Extract blobs for every binary image
        blobs = blobExtraction(binaryImage, width, height)
        blobsOfAllBinaryImages.append(blobs)

    # Put blobs in single array
    singleBlobArray = []
    for i in range(len(blobsOfAllBinaryImages)):
        for j in range(len(blobsOfAllBinaryImages[i])):
            singleBlobArray.append(blobsOfAllBinaryImages[i][j])

    # Merge blobs
    mergedSingleBlobArray = []
    index = 0
    for blob1 in Elements(singleBlobArray, 0):
        index += 1
        center1 = getCenterOfBlob(blob1, width)
        for blob2 in Elements(singleBlobArray, index):
            center2 = getCenterOfBlob(blob2, width)
            if getMagnitude(center1, center2) < minDistBetweenBlobs:
                blob1 = mergeBlobs(blob1, blob2)
                singleBlobArray.remove(blob2)
        mergedSingleBlobArray.append(blob1)

    # Find blob centers
    centers = []
    for i in range(len(mergedSingleBlobArray)):
        centers.append(getCenterOfBlob(mergedSingleBlobArray[i], width))

    return centers, mergedSingleBlobArray, convertBlobToPixelList(mergedSingleBlobArray, width, height, 3)
