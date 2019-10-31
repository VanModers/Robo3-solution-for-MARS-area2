from mars_interface import *
from euclid import *
import math
import numpy as np;
from camera_data_acquisition import CameraDataAcquisition, addCameraData
from behavior import initialBehaviour, doBehaviour


global cda
current_cam = "cam0"
motor_left_cmd = 0
motor_right_cmd = 0

def init():
    global cda
    clearDict()
    cda = CameraDataAcquisition()
    initialBehaviour()
    setConfig("Robot", "behavior", 0)
    requestConfig("Robot", "behavior")
    setRunning(True)
    logMessage("setup python interface")
    return sendDict()

def update(marsData):
    global cda
    global current_cam, motor_left_cmd, motor_right_cmd
    clearDict()

    setMotor("motor_left", motor_left_cmd)
    setMotor("motor_right", motor_right_cmd)

    #pixelData = cda.acquire_images()
    pixelData, rangeData = cda.acquire_pixeldata(current_cam, 80., 37., [0, 60], [160, 97], current_cam, True)
    pixelData2 = []
    if len(pixelData) > 0:
        pixelData2, rangeData2 = cda.acquire_pixeldata(current_cam, 80, 60, [40, 0], [120, 60], "camZoomed", False)

    (motor_left_cmd, motor_right_cmd, blobPixels, cornerPixels, current_cam) = doBehaviour(marsData, pixelData, 80, 37, pixelData2, 80, 60)
    if len(pixelData) > 0:
        if len(blobPixels) > 0:
            cda.writeImagePNGDataToFile("currentRobotBlobView", 80, 37, cda.toPixelList(blobPixels , 80, 60, 3))
        if len(cornerPixels) > 0:
            cda.writeImagePNGDataToFile("currentRobotCornerView", 80, 60, cda.toPixelList(cornerPixels , 80, 60, 3))
        cda.reset_clock(current_cam)
    setMotor("motor_left", motor_left_cmd)
    setMotor("motor_right", motor_right_cmd)
    return sendDict()
