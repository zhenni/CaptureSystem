import os
import subprocess
from glob import glob

import argparse

import pdb

VIDEO_EXT = ".avi"
IMG_EXT = ".jpg"
IMG_NAME_FORMAT = "img_%06d"+IMG_EXT

serial_numbers = ["18565847", "18565848", "18565849", "18565850", "18565851", "18566303"]


#########################
def extract_images(video_file_name, tgt_folder, start_frame):
    subprocess.call(["ffmpeg", "-i", video_file_name, "-start_number", "%d"%start_frame, os.path.join(tgt_folder, IMG_NAME_FORMAT) ])


def extract_images_for_camera(root_folder, serial_number):
    img_folder = os.path.join(root_folder, serial_number)
    if not os.path.exists(img_folder):
        os.mkdir(img_folder)
    # else:
    #     print img_folder + " already extracted images"
    #     return
    videos = sorted(glob(os.path.join(root_folder, serial_number+"*"+VIDEO_EXT)))
    for video in videos:
        start_number = len(glob(os.path.join(img_folder, "*"+IMG_EXT)))
        extract_images(video, img_folder, start_number)
    

#########################
parser = argparse.ArgumentParser(description='Process to extract videos to images.')
parser.add_argument('folder', metavar='dir', type=str,
                    help='The folder contains the videos')
args = parser.parse_args()

##########################

root_folder = args.folder
for serial_number in serial_numbers:
    extract_images_for_camera(root_folder, serial_number)
 

