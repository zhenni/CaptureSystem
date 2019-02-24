import os
import shutil
import subprocess
from glob import glob
from functools import reduce

import pdb

import argparse

# VIDEO_EXT = ".avi"
IMG_EXT = ".jpg"
IMG_NAME_FORMAT = "img_%06d"+IMG_EXT
LOG_NAME_FORMAT = "Log%s.txt"

SYNCED_FOLDER = "SyncData"

serial_numbers = ["18565847", "18565848", "18565849", "18565850", "18565851", "18566303"]

#########################

def read_frameid_for_camera(root_folder, serial_number):
    logfile_name = os.path.join(root_folder, LOG_NAME_FORMAT%serial_number) 
    return read_frameid_from_file(logfile_name)

def read_frameid_from_file(logfile_name):
    with open(logfile_name) as f:
        lines = f.readlines()
    
    num_lines = len(lines)
    
    frameids = {}  # physics: index

    if (num_lines < 3): return frameids
    startid = int(lines[2].split()[2])

    for i in range(num_lines // 11):
        idx = int(lines[i*11].split()[2])
        phy_id = int(lines[i*11+2].split()[2]) - startid
        frameids[phy_id] = idx

    return frameids
    

def move_synced_images(src_folder, tgt_folder, synced_ids, frame_dict):
    for synced_id in synced_ids:
        src_img = os.path.join(src_folder, IMG_NAME_FORMAT % frame_dict[synced_id])
        tgt_img = os.path.join(tgt_folder, IMG_NAME_FORMAT % synced_id)
        shutil.copy2(src_img, tgt_img)

#########################
parser = argparse.ArgumentParser(description='Process to extract videos to images.')
parser.add_argument('folder', metavar='dir', type=str,
                    help='The folder contains the videos')
args = parser.parse_args()

##########################

root_folder = args.folder
frame_ids = [ read_frameid_for_camera(root_folder, serial_number) for serial_number in serial_numbers]

# physics ids
frame_ids_set = [set(ids.keys()) for ids in frame_ids]
intersect_frames = reduce(lambda x, y: x & y, frame_ids_set)
print "%d synced images have been detected"%len(intersect_frames)

synced_folder = os.path.join(root_folder, SYNCED_FOLDER)
if not os.path.exists(synced_folder):
    os.mkdir(synced_folder)

for i, serial_number in enumerate(serial_numbers):
    src_folder = os.path.join(root_folder, serial_number)
    tgt_folder = os.path.join(synced_folder, serial_number)
    if not os.path.exists(tgt_folder):
        os.mkdir(tgt_folder)
    move_synced_images(src_folder, tgt_folder, intersect_frames, frame_ids[i])
 
