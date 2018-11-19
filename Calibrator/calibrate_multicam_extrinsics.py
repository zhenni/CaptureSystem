import os
import cv2
import pickle
import numpy as np
from utils.calibrator import CameraParam, detect_checkboard_for_sequence, \
                                generate_object_points, detect_checkboard, compliment_rodrigues

import pdb

def check_and_create_dir(dir_name):
    if not os.path.exists(dir_name):
        os.mkdir(dir_name)


rootDir = "/data/j/zhen_egocentric_pose/capture/Capture_11-15-18_2"
serial_str = ["Kinect01", "Kinect02", "Kinect03","Kinect04","Kinect05","Kinect09"]
imgNamePre = ["img", "img", "img", "img", "img", "img", "img", "img", "img", "img"]


checkerboardImgDir = os.path.join(rootDir, "SyncData")
manuallyCaptureDir = os.path.join(rootDir, "SyncData")

intrinsicsDir = os.path.join(checkerboardImgDir, "Intrinsics")
detectedCornersDir = os.path.join(checkerboardImgDir, "corners")
outDir = os.path.join(checkerboardImgDir, "output")
tmpDir = os.path.join(checkerboardImgDir, "tmpdata")

# externalTransDir  tmpdata

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


REF_CAM = 0
ITER_NUM = 100
MIN_PAIR_MATCH = 2
max_num_pair_for_stereo_calib = 800

bFlipHorizontal = False
is_save_tempory_data = True


save_corners = True
save_checkerboardImg = True

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

print "Finish Detecting CheckerBoard for All Images"


######################
# Stereo calibration for all pairs of the cameras
# TODO: move to calibrator

rotationBwCams    = [[None]*num_cam for i in range(num_cam)]
translationBwCams = [[None]*num_cam for i in range(num_cam)]

for cam_idx1 in range(num_cam):
    for cam_idx2 in range(num_cam):
        
        if cam_idx1 == cam_idx2: 

            continue
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

        image_count = len(intersect_frames)
        if image_count <= MIN_PAIR_MATCH: continue

        # if the images two many 
        image_used_count = min(image_count, max_num_pair_for_stereo_calib)
        image_used_indices = range(0, image_count, image_count // image_used_count)

        image_points1 = [image_points1[idx] for idx in image_used_indices]
        image_points2 = [image_points2[idx] for idx in image_used_indices]
        print "Begin compute the relative transformation ({}, {}): {} views are used out of {} views".format(cam_idx1, cam_idx2, image_used_count, image_count)

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

print "Finish stereo calibration for all pairs of the cameras"

#############################
# Final solution is saved in R and T
# Manually Capture a frame that more camera can see
# TODO: use the first frame

##### Initialize

init_flag = [False]*num_cam

for cam_idx in range(num_cam):
    camDirName = serial_str[cam_idx]

    image_file_name = os.path.join(manuallyCaptureDir, '{}.jpg'.format(camDirName))
    corner_saved_file_name = None
    out_image_file_name = None

    if save_corners:
        corner_saved_file_name = os.path.join(manuallyCaptureDir, 'corners_{}.pkl'.format(camDirName))
    if save_checkerboardImg:
        out_image_file_name = os.path.join(manuallyCaptureDir, 'checkerBoard_{}.jpg'.format(camDirName))

    # Generate Extrinsics
    image_points = detect_checkboard(image_file_name, 
                cameras_param[cam_idx].width, cameras_param[cam_idx].height,
                checkboard_size_width, checkboard_size_height, 
                cell_width, cell_height, 
                checkboard_win=None, 
                wait_time=wait_time,
                out_image_file_name=out_image_file_name,
                corner_saved_file_name=corner_saved_file_name)

    if image_points is None: continue

    object_points = generate_object_points(1,
                        checkboard_size_width, checkboard_size_height,
                        cell_width, cell_height)[0]

    retval, rvec, tvec = cv2.solvePnP(object_points, image_points.reshape((-1, 2)), cameras_param[cam_idx].K, cameras_param[cam_idx].dist)  

    image_points_reproject, jac = cv2.projectPoints(object_points, rvec, tvec, cameras_param[cam_idx].K, cameras_param[cam_idx].dist)
    projection_error = cv2.norm(image_points, image_points_reproject)


    cameras_param[cam_idx].R, _ = cv2.Rodrigues(rvec)
    cameras_param[cam_idx].t = tvec

    calib_file_name = os.path.join(outDir, "calib_{}_ori.txt".format(camDirName))
    cameras_param[cam_idx].save_calibration(calib_file_name)

    init_flag[cam_idx] = True

assert(init_flag[REF_CAM]), \
    "Extrinsics cannot be computed for the Reference Camera!"


for cam_idx in range(num_cam):
    if cam_idx == REF_CAM or rotationBwCams[REF_CAM][cam_idx] is None: continue

    cameras_param[cam_idx].R = \
            np.dot(rotationBwCams[REF_CAM][cam_idx], cameras_param[REF_CAM].R)
    cameras_param[cam_idx].t = \
            np.dot(rotationBwCams[REF_CAM][cam_idx], cameras_param[REF_CAM].t) \
            + translationBwCams[REF_CAM][cam_idx]
    init_flag[cam_idx] = True


# If there exist non-initialized cameras, try to find a chain to initilize it

for c in range(num_cam):
    if np.all(init_flag): break

    for cam_idx1 in range(num_cam):
        if init_flag[cam_idx1]: continue

        for cam_idx2 in range(num_cam):
            if cam_idx1 == cam_idx2 or not init_flag[cam_idx2] \
                        or rotationBwCams[cam_idx2][cam_idx1] is None:
                continue

            cameras_param[cam_idx1].R = \
                    np.dot(rotationBwCams[cam_idx2][cam_idx1], cameras_param[cam_idx2].R)
            cameras_param[cam_idx1].t = \
                    np.dot(rotationBwCams[cam_idx2][cam_idx1], cameras_param[cam_idx2].t) \
                    + translationBwCams[cam_idx2][cam_idx1]

            init_flag[cam_idx1] = True
            break

assert (np.all(init_flag)), \
    "Error: camera {} is not initialized because there is no chain found!".format(np.where(np.array(init_flag) == 0))

if is_save_tempory_data:
    for cam_idx in range(num_cam):
        camDirName = serial_str[cam_idx]
        calib_file_name = os.path.join(tmpDir, "calib_{}_0.txt".format(camDirName))
        cameras_param[cam_idx].save_calibration(calib_file_name)



print "Begin updating for Extrinsics..."


R_new = [cameras_param[i].R for i in range(num_cam)]
T_new = [cameras_param[i].t for i in range(num_cam)]

for Iter in range(ITER_NUM):
    
    for cam_idx1 in range(num_cam):
        # keep the [R|T] of reference camera fixed
        if cam_idx1 == REF_CAM: continue

        solutionNum = 0
        R_new[cam_idx1] = np.zeros((3, 3)) 
        R_new_rod       = np.zeros((3, 1))
        T_new[cam_idx1] = np.zeros((3, 1))
        rotation_rod_ref, _ = cv2.Rodrigues(cameras_param[cam_idx1].R)

        for cam_idx2 in range(num_cam):
            if cam_idx1 == cam_idx2: continue

            if rotationBwCams[cam_idx2][cam_idx1] is not None:
                R_tmp =  np.dot(rotationBwCams[cam_idx2][cam_idx1], cameras_param[cam_idx2].R)
                R_tmp_rod, _ = cv2.Rodrigues(R_tmp)
                
                if np.dot(R_tmp_rod.reshape(-1), rotation_rod_ref.reshape(-1)) < 0:
                    R_tmp_rod = compliment_rodrigues(R_tmp_rod)
                R_new_rod += R_tmp_rod

                T_tmp = np.dot(rotationBwCams[cam_idx2][cam_idx1], cameras_param[cam_idx2].t) \
                        + translationBwCams[cam_idx2][cam_idx1]
                T_new[cam_idx1] += T_tmp

                solutionNum +=1

            if rotationBwCams[cam_idx1][cam_idx2] is not None:
                R_tmp =  np.dot(rotationBwCams[cam_idx1][cam_idx2].T, cameras_param[cam_idx2].R)
                R_tmp_rod, _ = cv2.Rodrigues(R_tmp)
                if np.dot(R_tmp_rod.reshape(-1), rotation_rod_ref.reshape(-1)) < 0:
                    R_tmp_rod = compliment_rodrigues(R_tmp_rod)
                R_new_rod += R_tmp_rod

                T_tmp = cameras_param[cam_idx2].t - translationBwCams[cam_idx1][cam_idx2]
                T_tmp = np.dot(rotationBwCams[cam_idx1][cam_idx2].T, T_tmp)
                T_new[cam_idx1] += T_tmp

                solutionNum +=1

        R_new_rod *= (1.0 / solutionNum)
        R_new[cam_idx1], _ = cv2.Rodrigues(R_new_rod)
        T_new[cam_idx1] *= (1.0 / solutionNum)

    for i in range(num_cam):
        print "Iter {}: Camera {} R_delta: {}, t_delta: {}".format(Iter, i, 
                    cv2.norm(cameras_param[i].R-R_new[i]),
                    cv2.norm(cameras_param[i].t-T_new[i]))
        cameras_param[i].R = np.copy(R_new[i])
        cameras_param[i].t = np.copy(T_new[i])

    if is_save_tempory_data:
        for cam_idx in range(num_cam):
            camDirName = serial_str[cam_idx]
            calib_file_name = os.path.join(tmpDir, "calib_{}_{}.txt".format(camDirName, Iter))
            cameras_param[cam_idx].save_calibration(calib_file_name)


for cam_idx in range(num_cam):
    camDirName = serial_str[cam_idx]
    calib_file_name = os.path.join(outDir, "calib_{}.txt".format(camDirName))
    cameras_param[cam_idx].save_calibration(calib_file_name)

print "End"


