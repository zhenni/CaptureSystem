import os
import cv2
from utils.calibrator import generate_intrinsics

def check_and_create_dir(dir_name):
    if not os.path.exists(dir_name):
        os.mkdir(dir_name)


rootDir = "/data/j/zhen_egocentric_pose/capture/Capture_11-15-18_2"
serial_str = ["Kinect01", "Kinect02", "Kinect03","Kinect04","Kinect05","Kinect09"]
imgNamePre = ["img", "img", "img", "img", "img", "img", "img", "img", "img", "img"]

num_cam = 6
startFrame = 38
endFrame = 1500
step = 18

is_show_checkboard = False
wait_time = 0.0

checkboard_size_width = 7
checkboard_size_height = 6
cell_width = 8.99286
cell_height = 8.99286

checkerboardImgDir = os.path.join(rootDir, "SyncData")

detectedCornersDir = os.path.join(checkerboardImgDir, "corners")
intrinsicsDir = os.path.join(checkerboardImgDir, "Intrinsics")

check_and_create_dir(detectedCornersDir)
check_and_create_dir(intrinsicsDir)

error_file = os.path.join(intrinsicsDir, 'error.txt')
f_error= open(error_file,"w+")


for cam_idx in range(num_cam):

    camDirName = serial_str[cam_idx]
    detectedCornersSubFolder = os.path.join(detectedCornersDir, camDirName)
    check_and_create_dir(detectedCornersSubFolder)

    img_base_name = os.path.join(checkerboardImgDir, camDirName, 
                            imgNamePre[cam_idx]+"_{0:05d}.jpg")
    cornerDataBaseName = os.path.join(detectedCornersSubFolder,
                            "corners_{0:05d}.pkl")
    detectedCheckerImgBaseName = os.path.join(detectedCornersSubFolder, 
                            "checkerBoard_{0:05d}.jpg")



    # ====== Get Intrinsics =======
    camera_param, error = generate_intrinsics(img_base_name, 
                            startFrame, endFrame, step,
                            checkboard_size_width, checkboard_size_height,
                            cell_width, cell_height,
                            show_checkboard=is_show_checkboard,
                            wait_time=wait_time,
                            out_basename=detectedCheckerImgBaseName,
                            corner_basename=cornerDataBaseName,
                            )


    print "Intrinsics Camera {}".format(camDirName)
    print "Image Size: ", camera_param.width, camera_param.height
    print "Intrinsics Matrix: ", camera_param.K
    print "Distortion: ", camera_param.dist
    print "Calibration Error:", error

    intrinsics_file = os.path.join(intrinsicsDir, camDirName+".ini")
    camera_param.save_intrinsics_to_ini_file(intrinsics_file)

    f_error.write("Intrinsics Error {}: {}\n".format(camDirName, error))

f_error.close() 




