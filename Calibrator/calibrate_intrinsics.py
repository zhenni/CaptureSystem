import os
import cv2
from utils.calibrator import generate_intrinsics, save_calibration_data_ini_file

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
outDir = os.path.join(checkerboardImgDir, "output")
intrinsicsDir = os.path.join(checkerboardImgDir, "Intrinsics")

check_and_create_dir(detectedCornersDir)
check_and_create_dir(outDir)
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


    # ======= Get Size =========
    for k in range(startFrame, endFrame, step):
        img_name = img_base_name.format(k)
        img = cv2.imread(img_name, cv2.IMREAD_UNCHANGED)
        if img is not None:
            img_height, img_width = img.shape[:2]
            del img
            break


    # ====== Get Intrinsics =======
    intrinsics, distort_coef, error = generate_intrinsics(img_base_name, 
                            startFrame, endFrame, step,
                            img_width, img_height,
                            checkboard_size_width, checkboard_size_height,
                            cell_width, cell_height,
                            show_checkboard=is_show_checkboard,
                            wait_time=wait_time,
                            out_basename=detectedCheckerImgBaseName,
                            corner_basename=cornerDataBaseName,

                            )


    print intrinsics
    print distort_coef
    print error

    f_error.write("Intrinsics Error {}: {}\n".format(camDirName, error))
   
    intrinsics_name = os.path.join(intrinsicsDir, camDirName+".ini")
    save_calibration_data_ini_file(intrinsics_name, intrinsics, distort_coef, img_width, img_height);

f_error.close() 




