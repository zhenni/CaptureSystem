import os
import cv2
import numpy as np
import time
import pickle

import pdb

from visdom import Visdom

class CameraParam():
    def __init__(self, width=0, height=0, 
        K=None, dist=None, R=None, t=None):

        self.width  = width
        self.height = height
        self.K = K
        self.dist = dist
        self.R = R
        self.t = t

    def save_intrinsics_to_ini_file(self, file_name):
        assert(self.w == 0 or self.h == 0)
        assert(self.K is None or self.dist is None)
        f = open(file_name, "w")
        f.write('[Intrinsics]\n')
        f.write('ImageSize= {} {}\n'.format(self.width, self.height))
        f.write('Matrix= {}\n'.format(str(self.K.ravel().tolist())[1:-1]).replace(',', ''))
        f.write('Distortion= {}\n'.format(str(self.dist.ravel().tolist())[1:-1]).replace(',', ''))
        f.close()

    def read_intrinsics_from_ini_file(self, file_name):
        assert(self.R is not None and self.t is not None), \
                'Read Intrinsics before Extrinsics'
        with open(file_name) as f:
            lines = f.readlines()

        assert("Intrinsics" not in lines[0])
        
        line = lines[1].split()[1:]
        self.width  = int(line[0])
        self.height = int(line[1])

        line = lines[2].split()[1:]
        self.K = np.reshape(np.array([float(x) for x in line]), (3, 3))

        line = lines[3].split()[1:]
        self.dist = np.array([float(x) for x in line])





def extract_and_refine_checkboard(img, isBGR, 
            checkboard_size_width, checkboard_size_height,
            border_width=5, bRefine=True, bFastCheck=False, 
            SMALL_WIDTH = 5000, refineMaxIter = 500, refineEpsilon = 1.0E-8):
    '''
    # return None if the image is NULL
    # return corners: num x 1 x 2
    '''

    if img is None:
        print "Warning: input image is NULL!"
        return None

    flag = 0
    if bFastCheck:
        flag = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CV_CALIB_CB_FAST_CHECK
    else:
        flag = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FILTER_QUADS#  + cv2.CALIB_CB_NORMALIZE_IMAGE;

    h, w = img.shape[:2]

    if w > SMALL_WIDTH:
        # image width larger than SMALL_WIDTH, need to resize
        small_img = cv2.resize(img, (SMALL_WIDTH, round(SMALL_WIDTH)/w*h))

        checkBoard_was_found, corners = cv2.findChessboardCorners(small_img, (checkboard_size_width-1, checkboard_size_height-1), flags=flag)

        if not checkBoard_was_found: return None

        corner_count = corners.shape[0]
        ratio_x = w/SMALL_WIDTH
        ratio_y = h/round(SMALL_WIDTH/w*h)
        for i in range(corner_count):
            corners[i, 0, 0] *= ratio_x
            corners[i, 0, 1] *= ratio_y

    else:
        # image width smaller than SMALL_WIDTH
        checkBoard_was_found, corners = cv2.findChessboardCorners(img, (checkboard_size_width-1, checkboard_size_height-1), flags=flag)

        if not checkBoard_was_found: return None
        
        if bRefine:
            
            # color to gray
            if img.shape[2] >= 3:
                if isBGR:
                    img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
                else:
                    img = cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)

            stop_criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, refineMaxIter, refineEpsilon)
            cv2.cornerSubPix(img, corners, (5, 5), (-1, -1), stop_criteria)
    
    # check if corner is at the border   
    one_check_on_border = False
    corner_count = corners.shape[0]
    for i in range(corner_count):
        x, y = corners[i, 0]
        if x < border_width or x >= w - border_width or \
                    y < border_width or y >= h - border_width:
            one_check_on_border = True
    if one_check_on_border: return None

    #  check corners
    point_num_per_image = (checkboard_size_height-1)*(checkboard_size_width-1)
    assert (corners.shape[0]==point_num_per_image,
            corners.shape[1]==1 and corners.shape[2]==2), \
        "Corners is not a qualified corner matrix"


    return corners


def calculate_intrinsics_given_corners(image_corners_list, 
            checkboard_size_width, checkboard_size_height,
            image_size_width, image_size_height,
            cell_width, cell_height, 
            MAX_VIEW_NUMBER_USED=150):
    '''
    return intrinsics, distort_coef, error
    '''
    point_num_per_image = (checkboard_size_height-1)*(checkboard_size_width-1) 

    image_count = len(image_corners_list)
    if (image_count > MAX_VIEW_NUMBER_USED):

        discard_num = image_count - MAX_VIEW_NUMBER_USED
        ratio = image_count // discard_num

        discard_indices = np.arange(discard_num) * ratio
        accept_indices = np.ones((image_count), type=np.bool)
        accept_indices[discard_indices]=False
        image_points_list = [image_points_list[idx] for idx in accept_indices]

        print "Warning: only {} views are used (total:{})!".format(MAX_VIEW_NUMBER_USED, image_count) 
        image_count = MAX_VIEW_NUMBER_USED

    object_points_one_image = np.zeros((point_num_per_image, 3), np.float32)
    object_points_one_image[:,:2] = np.mgrid[0:(checkboard_size_width-1), 0:(checkboard_size_height-1)].T.reshape(-1,2)
    object_points_one_image[:, 0] *= cell_width
    object_points_one_image[:, 1] *= cell_height


    objpoints = [object_points_one_image]*image_count

    print "Begin Compute Intrinsics ({} views are used)...".format(image_count)

    calib_error, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
            objpoints, image_corners_list, 
            (image_size_width, image_size_height),
            None, None, flags= cv2.CALIB_FIX_K3)

    print "error({})...End!".format(calib_error)

    return mtx, dist, calib_error


def generate_intrinsics(image_base_name, start_idx, end_idx, step,
                    checkboard_size_width, checkboard_size_height, 
                    cell_width, cell_height, 
                    show_checkboard=True, 
                    wait_for_key=False, wait_time=0.01,
                    save_checkerboardImg=True, out_basename=None,
                    save_corners=True, corner_basename=None, 
                    isBGR=True, bSaveNullImageCorners=True,
                    bFlipHorizontal=False, bFastCheck=False):
    
    if save_corners: 
        assert corner_basename is not None, \
                "corner_basename need to be set"

    if show_checkboard and not wait_for_key:
        viz = Visdom(port=8095)
        checkboard_win = viz.image( np.random.rand(3, image_size_height, image_size_width),
            opts=dict(title='CheckerBoard', caption='Random'))


    ################################################
    ########### Get Size ##########################
    for idx in range(start_idx, end_idx, step):
        name = image_base_name.format(idx)
        img = cv2.imread(name, cv2.IMREAD_UNCHANGED)
        if img is not None:
            image_size_height, image_size_width = img.shape[:2]
            del img
            break

    #################################################
    # Detect the checkboard from all the input images
    image_points_list = []
    for idx in range(start_idx, end_idx, step):
        print "Reading Image: {0:05d}".format(idx)
        name = image_base_name.format(idx)
        img = cv2.imread(name, cv2.IMREAD_UNCHANGED)

        if img is None:
            print "Warning: {0:05d}-th image is NULL!".format(idx)
            del img; continue

        img_height, img_width = img.shape[:2]
        if not img_width == image_size_width or not img_height == image_size_height:
            print "Warning: {0:05d}-th image is not an qualified image due to its size!".format(idx)
            del img; continue


        if bFlipHorizontal:
            img = cv2.flip(img, 1)

        image_points = extract_and_refine_checkboard(img, isBGR, 
                checkboard_size_width, checkboard_size_height,
                bFastCheck=bFastCheck)

        if image_points is None:
            print "Warning: No checkboard is detected in the {}-th image!".format(idx)

            # save corners even when image_points is None
            if save_corners and bSaveNullImageCorners:            
                name = corner_basename.format(idx)
                with open(name, "wb") as fp:   
                    pickle.dump(image_points, fp)

            if show_checkboard:
                checkboard_win = viz.image( img.transpose(2, 0, 1)[::-1], win=checkboard_win,
                opts=dict(title='CheckerBoard', caption='{}-th image. No checkboard is detected.'.format(idx)))
                time.sleep(wait_time)

            del img; continue

        image_points_list.append(image_points);

        if show_checkboard:
            
            cv2.drawChessboardCorners(img, (checkboard_size_width-1, checkboard_size_height-1), image_points, True)

            checkboard_win = viz.image( img.transpose(2, 0, 1)[::-1] , win=checkboard_win,
                opts=dict(title='CheckerBoard', caption='{}-th image'.format(idx)))

            if wait_for_key:
                raw_input("")
            else:
                time.sleep(wait_time)

        if save_checkerboardImg and out_basename is not None:
            if not show_checkboard: 
                cv2.drawChessboardCorners(img, (checkboard_size_width-1, checkboard_size_height-1), image_points, True)
            name = out_basename.format(idx)
            cv2.imwrite(name,img)

        if save_corners:
            name = corner_basename.format(idx)
            with open(name, "wb") as fp:   
                pickle.dump(image_points, fp)

        del img

    #############################################
    ####### Compute the Intrinsics ##############
    if len(image_points_list) == 0: 
        print "Error: No checkboard detected in all input images"
        return

    print "Finish reading in all images!"


    intrinsics, distort_coef, error =  calculate_intrinsics_given_corners(image_points_list,
                checkboard_size_width, checkboard_size_height,
                image_size_width, image_size_height,
                cell_width, cell_height)

    print "Finish Intrinsics Calibration!"

    camera_param = CameraParam(width=width, height=height, K=intrinsics, dist=distort_coef)
    return camera_param, error



