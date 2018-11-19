import os
import cv2
import pickle
from utils.calibrator import CameraParam, detect_checkboard_for_sequence, \
                                generate_object_points

import pdb

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
tmpDir = os.path.join(checkerboardImgDir, "tmpdata")

# externalTransDir  tmpdata

num_cam = 6
startFrame = 38
endFrame = 1500
step = 9

is_show_checkboard = False
wait_time = 0.0

checkboard_size_width = 7
checkboard_size_height = 6
cell_width = 8.99286
cell_height = 8.99286
# bFlipHorizontal
num_pair_for_stereo_calib = 800
is_save_tempory_data = True

check_and_create_dir(detectedCornersDir)
check_and_create_dir(intrinsicsDir)
check_and_create_dir(outDir)
check_and_create_dir(tmpDir)

# //load external relative transformation matrix bw cameras
# bool bLoadExternalTransforms = false;
# bool bOnlyUseExternalTransforms = false;

error_file = os.path.join(outDir, 'error.txt')
f_error= open(error_file,"w+")

#########################################
############ Load Intrinsics ############
cameras_param = [CameraParam() for cam_idx in range(num_cam)]
for cam_idx in range(num_cam):
    camDirName = serial_str[cam_idx]
    intrinsics_file = os.path.join(intrinsicsDir, camDirName+".ini")
    cameras_param[cam_idx].read_intrinsics_from_ini_file(intrinsics_file)
    print "Succeed: load intrinsic file for {}".format(camDirName)


cams_image_points_dict = [0]*num_cam
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


    # not bOnlyUseExternalTransforms
    cams_image_points_dict[cam_idx] = \
                detect_checkboard_for_sequence(img_base_name, 
                            startFrame, endFrame, step,
                            cameras_param[cam_idx].width, cameras_param[cam_idx].height, 
                            checkboard_size_width, checkboard_size_height,
                            cell_width, cell_height,
                            show_checkboard=is_show_checkboard,
                            wait_time=wait_time,
                            out_basename=detectedCheckerImgBaseName,
                            corner_basename=cornerDataBaseName,
                            )

######################
# R, T, E, F 
# TODO: move to calibrator

rotationBwCams    = [[0]*num_cam for i in range(num_cam)]
translationBwCams = [[0]*num_cam for i in range(num_cam)]

for cam_idx1 in range(num_cam):
    for cam_idx2 in range(num_cam):
        
        if cam_idx1 == cam_idx2: continue
        camDirName1 = serial_str[cam_idx1]
        camDirName2 = serial_str[cam_idx2]
 
        # Load if detected
        name_R = os.path.join(tmpDir, "RotBwCams_{}_to_{}.pkl".format(camDirName1, camDirName2))
        name_t = os.path.join(tmpDir, "TransBwCams_{}_to_{}.pkl".format(camDirName1, camDirName2))
        if os.path.exists(name_R) and os.path.exists(name_t):
            rotationBwCams   [cam_idx1][cam_idx2] =  pickle.load(open(name_R, "rb"))
            translationBwCams[cam_idx1][cam_idx2] =  pickle.load(open(name_t, "rb"))
            print "Load External Rotation and Translation for [{} - {}]".format(camDirName1, camDirName2)
            continue

        intersect_frames = sorted(set(cams_image_points_dict[cam_idx1].keys()).intersection(
                                      cams_image_points_dict[cam_idx2].keys()))
        image_points1 = [cams_image_points_dict[cam_idx1][idx] for idx in intersect_frames]
        image_points2 = [cams_image_points_dict[cam_idx2][idx] for idx in intersect_frames]

        # TODO: image_used_count, max_pair_image_count
        image_count = len(intersect_frames)
        image_used_count = image_count

        pdb.set_trace()


        ##############################################
        # Stereo calibration for all pairs of the cameras
        object_points = generate_object_points(image_count,
                                checkboard_size_width, checkboard_size_height,
                                cell_width, cell_height)
        error, K1, dist1, K2, dist2, \
            rotationBwCams[cam_idx1][cam_idx2], \
            translationBwCams[cam_idx1][cam_idx2],\
            E, F = cv2.stereoCalibrate(
                object_points, image_points1, image_points2, 
                cameras_param[cam_idx1].K, cameras_param[cam_idx1].dist,
                cameras_param[cam_idx2].K, cameras_param[cam_idx2].dist,
                (0, 0), 
                criteria=(cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-6),
                flags=cv2.CALIB_FIX_INTRINSIC)

        # Print Log
        log_msg = "Reprojection Error between Cam {} and Cam {}: {} , pairs<{} / {}>".format(camDirName1, camDirName2, error, image_used_count, image_count)
        print log_msg
        f_error.write(log_msg)

        if is_save_tempory_data:
            # Save Essential Matrix
            name = os.path.join(tmpDir, "EMatrix_{}_to_{}.pkl".format(camDirName1, camDirName2))
            with open(name, "wb") as fp:   
                pickle.dump(E, fp)

            # Save Fundamental Matrix
            name = os.path.join(tmpDir, "FMatrix_{}_to_{}.pkl".format(camDirName1, camDirName2))
            with open(name, "wb") as fp:   
                pickle.dump(F, fp)

            # Save Rotation
            name = os.path.join(tmpDir, "RotBwCams_{}_to_{}.pkl".format(camDirName1, camDirName2))
            with open(name, "wb") as fp:   
                pickle.dump(rotationBwCams[cam_idx1][cam_idx2], fp)

            # Save Translation
            name = os.path.join(tmpDir, "TransBwCams_{}_to_{}.pkl".format(camDirName1, camDirName2))
            with open(name, "wb") as fp:   
                pickle.dump(translationBwCams[cam_idx1][cam_idx2], fp)

#############################
# the final solution is saved in R and T
# ??? different size of checker board??

