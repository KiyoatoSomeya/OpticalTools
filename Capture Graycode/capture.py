import PySpin
import glob
import cv2
import numpy as np
import sys
import GetIntrinsicsFromYaml

# parameter
SRC_OFFSET_X = -3000
SRC_OFFSET_Y = 0

CAMERA_WIDTH = 4096
CAMERA_HEIGHT = 3000

IMG_TRIM_WIDTH = 748
IMG_TRIM_HEIGHT = 200

SHUTTER_SPEED = 300225
WAIT_TIME = 100

CAPTURE_POSITION = False;
POSITION_FROM_WORLD = False
MARKER_LENGTH = 0.03885
MARKER_INDEX = 0    # index is defined by opencv.aruco

# const values
WINDOW_SRC = 'src'
WINDOW_CAM = 'cam'
INTRINSICS_VIEW = 'cam_view.yml'
INTRINSICS_WORLD = 'cam_world.yml'
MARKER_FILENAME = 'marker.png'
MARKER_POSITION_PATH = 'position.txt'


# get POSITION_FROM_WORLD from args
args = sys.argv
if len(args) > 1:
    CAPTURE_POSITION = True;
    if args[1] == 'world':
        POSITION_FROM_WORLD = True
    elif args[1] == 'view':
        POSITION_FROM_WOELD = False
else:
    CAPTURE_POSITION = False;

# set Camera
system = PySpin.System.GetInstance()
cam_list = system.GetCameras()
numCameras = cam_list.GetSize()
cam = cam_list.GetByIndex(0)
cam.Init()

# set camera properties
cam.ExposureAuto.SetValue(PySpin.ExposureAuto_Off)
cam.ExposureTime.SetValue(SHUTTER_SPEED)
cam.GainAuto.SetValue(PySpin.GainAuto_Off)
cam.Gain.SetValue(0)

# configure trigger
cam.TriggerMode.SetValue(PySpin.TriggerMode_Off)
cam.TriggerSource.SetValue(PySpin.TriggerSource_Software)
cam.TriggerMode.SetValue(PySpin.TriggerMode_On)
cam.AcquisitionMode.SetValue(PySpin.AcquisitionMode_Continuous)
cam.BeginAcquisition()

# get camera intrinsics
cam_m, dist = GetIntrinsicsFromYaml.get(INTRINSICS_VIEW)

# find bmp
bmp_list = glob.glob('./pattern-*.bmp')

# capture
cv2.namedWindow(WINDOW_SRC, cv2.WND_PROP_FULLSCREEN)
cv2.imshow(WINDOW_SRC, cv2.imread(bmp_list[0]))
cv2.moveWindow(WINDOW_SRC, SRC_OFFSET_X, SRC_OFFSET_Y)
cv2.waitKey(1)
cv2.setWindowProperty(WINDOW_SRC, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
cv2.waitKey(WAIT_TIME)

i = 0
for bmp in bmp_list:
    srcImg = cv2.imread(bmp)
    cv2.imshow(WINDOW_SRC, srcImg)
    cv2.waitKey(WAIT_TIME)
    try:
        cam.TriggerSoftware.Execute()
        capImg = cam.GetNextImage()
    except PyCapture2.Fc2error as fc2Err:
        print("Error retrieving buffer :", fc2Err)
    cv_capImg = np.array(capImg.GetData(), dtype="uint8").reshape((capImg.GetHeight(), capImg.GetWidth()))
    cv_capImg = cv2.cvtColor(cv_capImg, cv2.COLOR_BAYER_BG2BGR)
    cv_capImg = cv2.undistort(cv_capImg, cam_m, dist, None, None)   # distort image from the data of camera intrinsics
    cv_capImg = cv2.flip(cv_capImg, -1)  #flip upside-down and leftside-rigit
    cv_capImg = cv_capImg[IMG_TRIM_HEIGHT:CAMERA_HEIGHT - IMG_TRIM_HEIGHT, IMG_TRIM_WIDTH:CAMERA_WIDTH - IMG_TRIM_WIDTH]    #trimming
    cv2.imwrite('recordedpattern-' + str(i).zfill(3) + '.bmp', cv_capImg)   #save

    cv_capImg = cv2.resize(cv_capImg, None, fx=0.25, fy=0.25)
    cv2.imshow(WINDOW_CAM, cv_capImg)
    cv2.waitKey(1)
    
    i += 1

    capImg.Release()


# finally get marker position
if POSITION_FROM_WORLD:
    print('get position from world camera')
    #finalize
    cam.EndAcquisition()
    cam.TriggerMode.SetValue(PySpin.TriggerMode_Off)

    # set Camera
    cam = cam_list.GetByIndex(1)
    cam.Init()

    # set camera properties
    cam.ExposureAuto.SetValue(PySpin.ExposureAuto_Off)
    cam.ExposureTime.SetValue(SHUTTER_SPEED)
    cam.GainAuto.SetValue(PySpin.GainAuto_Off)
    cam.Gain.SetValue(0)

    # configure trigger
    cam.TriggerMode.SetValue(PySpin.TriggerMode_Off)
    cam.TriggerSource.SetValue(PySpin.TriggerSource_Software)
    cam.TriggerMode.SetValue(PySpin.TriggerMode_On)
    cam.AcquisitionMode.SetValue(PySpin.AcquisitionMode_Continuous)
    cam.BeginAcquisition()

    # get intrinsics
    cam_m, dist = GetIntrinsicsFromYaml.get(INTRINSICS_WORLD)
else:
    print('get position from view camera')


markerImg = cv2.imread(MARKER_FILENAME)
cv2.imshow(WINDOW_SRC, markerImg)
cv2.waitKey(WAIT_TIME)
try:
    cam.TriggerSoftware.Execute()
    capImg = cam.GetNextImage()
except PyCapture2.Fc2error as fc2Err:
    print("Error retrieving buffer :", fc2Err)

cv_capImg = np.array(capImg.GetData(), dtype="uint8").reshape((capImg.GetHeight(), capImg.GetWidth()))
cv_capImg = cv2.cvtColor(cv_capImg, cv2.COLOR_BAYER_BG2BGR)
cv_capImg = cv2.undistort(cv_capImg, cam_m, dist, None, None)   # distort image from the data of camera intrinsics
cv_capImg = cv2.flip(cv_capImg, -1)  #flip upside-down and leftside-rigit
cv2.imwrite('marker_image.png', cv_capImg)
capImg.Release()

# use aruco
aruco = cv2.aruco
dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

corners, ids, rejectedImgPoints = aruco.detectMarkers(cv_capImg, dictionary)

if len(corners) > 0:
    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, MARKER_LENGTH, cam_m, dist)

    # write rotation and transformation
    with open(MARKER_POSITION_PATH, mode='w') as f:
        f.write('transformation: ' + str(tvec[0][0][0]) + ' ' + str(tvec[0][0][1]) + ' ' + str(tvec[0][0][2]))
        f.write('\nrotaion: '+ str(rvec[0][0][0]) + ' ' + str(rvec[0][0][1]) + ' ' + str(rvec[0][0][2]))
else:
    print('not detected marker!')

cv2.waitKey(WAIT_TIME)

# Finalize
cam.EndAcquisition()
cam.TriggerMode.SetValue(PySpin.TriggerMode_Off)
cam_list.Clear()


