import os
import shutil
from glob import glob
from parse import parse
import pdb

data_folder = '/data/j/zhen_egocentric_pose/Connydata_full/Capture_02-28-18_2-copy/EgoCentericCapture'

input_folder = os.path.join(data_folder, 'Output/SyncImages/Kinect01')
output_folder = os.path.join(data_folder, 'Output/SyncImages/Kinect01-shift')
pattern = 'img_{:05d}.jpg'
shift = 10

if not os.path.exists(output_folder):
    os.mkdir(output_folder)


images = sorted(glob(input_folder + '/*.jpg'))

for src_img in images:

    basename = os.path.basename(src_img)
    idx = parse(pattern, basename)[0]
    tgt_img = os.path.join(output_folder, pattern.format(idx+shift))    
    shutil.copy2(src_img, tgt_img)

# if need to use multiprocessing
# import multiprocessing
# num_cores = multiprocessing.cpu_count()
    
#     p = multiprocessing.Pool(num_cores)
#     args = [[cam_idx, frame_idx, map1, map2, roi] for frame_idx in range(start_frame_idx, end_frame_idx)]
#     p.map(write_undistort_wrapper, args)

