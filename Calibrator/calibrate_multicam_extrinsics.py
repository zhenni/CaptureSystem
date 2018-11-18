import os
import cv2
from utils.calibrator import generate_intrinsics

def check_and_create_dir(dir_name):
    if not os.path.exists(dir_name):
        os.mkdir(dir_name)


rootDir = "/data/j/zhen_egocentric_pose/capture/Capture_11-15-18_2"
serial_str = ["Kinect01", "Kinect02", "Kinect03","Kinect04","Kinect05","Kinect09"]
imgNamePre = ["img", "img", "img", "img", "img", "img", "img", "img", "img", "img"]


checkerboardImgDir = os.path.join(rootDir, "SyncData")
intrinsicsDir = os.path.join(checkerboardImgDir, "Intrinsics")

detectedCornersDir = os.path.join(checkerboardImgDir, "corners")
outDir = os.path.join(checkerboardImgDir, "output")
# externalTransDir  tmpdata
# tmpDir tmpdata

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
# bFlipHorizontal
num_pair_for_stereo_calib = 800

check_and_create_dir(detectedCornersDir)
check_and_create_dir(outDir)
check_and_create_dir(intrinsicsDir)

# //load external relative transformation matrix bw cameras
# bool bLoadExternalTransforms = false;
# bool bOnlyUseExternalTransforms = false;

# bool save_tempory_data = true;	

error_file = os.path.join(outDir, 'error.txt')
f_error= open(error_file,"w+")






