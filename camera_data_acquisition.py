from time import clock
from mars_interface import *
import numpy as np;
#import png;

CAMERA_NAMES = ["cam0", "cam1"]
STORAGE_PATH = "./"

cameraData = {}
cameraSize = {}
pixelData = np.array([[[]]])
rangeData = np.array([[]])

def addCameraData(name, data):
# What is here the name for?
    global cameraData, cameraSize
    cameraData[name] = data
    cameraSize[name] = len(data)/4

class CameraDataAcquisition():

    def __init__(self):
        self.start_times = {}
        for camera_name in CAMERA_NAMES:
            requestCameraSensor(camera_name)
            self.start_times[camera_name] = clock()

    def twoone(self, list1):
        list2 = []
        for x in range(len(list1)):
            for y in range(len(list1[x])):
                for i in range(len(list1[x][y])):
                    list2.append(list1[x][y][i])
        return list2

    def toPixelList(self, l, width, height, colours):
        colouredList = [l[i:i+colours] for i in xrange(0, len(l), colours)]
        return [colouredList[i:i+width*height:width] for i in xrange(0, width, 1)]


    def reset_clock(self, timer_name):
        self.start_times[timer_name] = clock()

    def elapsed_time(self, timer_name, s):
        current_time = clock()
        if current_time > self.start_times[timer_name]+s:
            self.start_times[timer_name] = clock()
            return True
        return False

    def getCameraPixelData(self, cam_name, newWidth, newHeight, upperCorner, lowerCorner):
        global cameraData, cameraSize, pixelData, rangeData
        width = 160
        height = 120
        pixelData = np.empty((int(newWidth), int(newHeight), 3), int)
        rangeData = np.empty((int(newWidth), int(newHeight)), int)
        if cameraSize[cam_name] == height*width:
            x1 = 0
            y1 = 0
            for y in np.arange(upperCorner[1], lowerCorner[1], float(lowerCorner[1] - upperCorner[1])/newHeight):
                yy = height-1-int(round(y))
                x1 = 0
                for x in np.arange(upperCorner[0], lowerCorner[0], float(lowerCorner[0] - upperCorner[0])/newWidth):
                    x1 = min(x1, int(newWidth) - 1)
                    y1 = min(y1, int(newHeight) - 1)

                    xx = int(round(x))
                    pixelData[x1][y1][0] = int(cameraData[cam_name][yy*width*4+xx*4]*255)
                    pixelData[x1][y1][1] = int(cameraData[cam_name][yy*width*4+xx*4+1]*255)
                    pixelData[x1][y1][2] = int(cameraData[cam_name][yy*width*4+xx*4+2]*255)
                    rangeData[x1][y1] = int(cameraData[cam_name][yy*width*4+xx*4+3]*100.)
                    x1 += 1
                y1 += 1

    def writeImageDataToFile(self, cam_name, width, height, data):
        global cameraData, cameraSize
        filename = STORAGE_PATH+cam_name+".ppm"
        with open(filename, "w") as f:
            f.write("P3\n")
            f.write(str(width) + " "+ str(height) + "\n")
            f.write("255\n")
            for y in range(height):
                for x in range(width):
                    f.write(str(int(data[x][y][0]))+ " ")
                    f.write(str(int(data[x][y][1]))+ " ")
                    f.write(str(int(data[x][y][2]))+ " ")
                f.write("\n")

    """def writeImagePNGDataToFile(self, cam_name, width, height, data):
        global cameraData, cameraSize
        pngArray = []

        for i in range(height):
            pngRow = []
            for j in range(width):
                pngRow += [data[j][i][0], data[j][i][1], data[j][i][2]]
            pngArray.append(pngRow)

        fileStream = open(cam_name + ".png", "wb")
        writer = png.Writer(width, height, greyscale = False)

        writer.write(fileStream, pngArray)
        fileStream.close()"""

    def writeCameraToFile(self, cam_name):
        global cameraData, cameraSize
        width = 160
        height = 120
        if cameraSize[cam_name] == height*width:
            filename = STORAGE_PATH+cam_name+".ppm"
            with open(filename, "w") as f:
                f.write("P3\n")
                f.write(str(width) + " "+ str(height) + "\n")
                f.write("255\n")
                for y in range(height):
                    yy = height-1-y
                    for x in range(width):
                        f.write(str(int(cameraData[cam_name][yy*width*4+x*4]*255))+ " ")
                        f.write(str(int(cameraData[cam_name][yy*width*4+x*4+1]*255))+ " ")
                        f.write(str(int(cameraData[cam_name][yy*width*4+x*4+2]*255))+ " ")
                    f.write("\n")

    def acquire_images(self):
        global pixelData
        for camera_name in CAMERA_NAMES:
            requestCameraSensor(camera_name)
            if self.elapsed_time(camera_name, 1.0):
                self.writeCameraToFile(camera_name)
        return pixelData

    def acquire_pixeldata(self, cam_name, newWidth, newHeight, upperCorner, lowerCorner, file_name, clock):
        global pixelData, rangeData
        requestCameraSensor(cam_name)
        if self.elapsed_time(cam_name, 0.3) or not clock:
            self.getCameraPixelData(cam_name, newWidth, newHeight, upperCorner, lowerCorner)
            #self.writeImagePNGDataToFile(file_name, int(newWidth), int(newHeight), pixelData)
            self.writeImageDataToFile(file_name, int(newWidth), int(newHeight), pixelData)
            return pixelData, rangeData
        else:
            return np.array([]), np.array([])
