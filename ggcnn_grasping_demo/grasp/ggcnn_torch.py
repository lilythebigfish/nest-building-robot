import sys
import time
import cv2
import torch
import numpy as np
import scipy.ndimage as ndimage
from skimage.draw import disk
from skimage.feature import peak_local_max
import threading

sys.path.append('./ggcnn')
torch.nn.Module.dump_patches = False


# Execution Timing
class TimeIt:
    def __init__(self, s):
        self.s = s
        self.t0 = None
        self.t1 = None
        self.print_output = False

    def __enter__(self):
        self.t0 = time.time()

    def __exit__(self, t, value, traceback):
        self.t1 = time.time()
        if self.print_output:
            print('%s: %s' % (self.s, self.t1 - self.t0))


def process_depth_image(depth, crop_size, out_size=300, return_mask=False, crop_y_inx=-1, crop_x_inx=-1, crop_y_offset=0):
    if crop_y_inx < 0 or crop_x_inx < 0:
        imh, imw = depth.shape
        if crop_y_inx < 0:
            crop_y_inx = max(0, imh - crop_size)  # crop height(y) start index
        if crop_x_inx < 0:
            crop_x_inx = max(0, imw - crop_size)  # crop width(x) start index

    with TimeIt('Crop'):
        depth_crop = depth[crop_y_inx // 2 - crop_y_offset:crop_y_inx // 2 + crop_size - crop_y_offset, crop_x_inx // 2:crop_x_inx // 2 + crop_size]

    with TimeIt('Inpainting_Processing'):
        # open cv inpainting does weird things at the border.
        depth_crop = cv2.copyMakeBorder(depth_crop, 1, 1, 1, 1, cv2.BORDER_DEFAULT)

        depth_nan_mask = np.isnan(depth_crop).astype(np.uint8)
        kernel = np.ones((3, 3),np.uint8)
        depth_nan_mask = cv2.dilate(depth_nan_mask, kernel, iterations=1)
        depth_crop[depth_nan_mask==1] = 0

        # Scale to keep as float, but has to be in bounds -1:1 to keep opencv happy.
        depth_scale = np.abs(depth_crop).max()
        depth_crop = depth_crop.astype(np.float32) / depth_scale  # Has to be float32, 64 not supported.

        with TimeIt('Inpainting'):
            depth_crop = cv2.inpaint(depth_crop, depth_nan_mask, 1, cv2.INPAINT_NS)

        # Back to original size and value range.
        depth_crop = depth_crop[1:-1, 1:-1]
        depth_crop = depth_crop * depth_scale

    with TimeIt('Resizing'):
        # Resize
        depth_crop = cv2.resize(depth_crop, (out_size, out_size), cv2.INTER_AREA)

    if return_mask:
        with TimeIt('Return Mask'):
            depth_nan_mask = depth_nan_mask[1:-1, 1:-1]
            depth_nan_mask = cv2.resize(depth_nan_mask, (out_size, out_size), cv2.INTER_NEAREST)
        return depth_crop, depth_nan_mask
    else:
        return depth_crop


class TorchGGCNN(object):
    def __init__(self, ggcnn_config, depth_img_que, ggcnn_cmd_que):
        self.device = torch.device('cpu')
        self.model = torch.load(ggcnn_config['MODEL_FILE'], map_location=self.device, weights_only=False)
        self.prev_mp = np.array([150, 150])
        self.depth_img_que = depth_img_que
        self.ggcnn_cmd_que = ggcnn_cmd_que
        self.open_loop_height = ggcnn_config['OPEN_LOOP_HEIGHT'] / 1000.0
        self.depth_cam_k = ggcnn_config['DEPTH_CAM_K']
        self.grasp_img = None
        if ggcnn_config['GGCNN_IN_THREAD']:
            self.thread = threading.Thread(target=self.run, daemon=True)
            self.thread.start()
    
    def run(self):
        while True:
            data = self.depth_img_que.get()
            robot_pos, img = data[0], data[1]
            self.grasp_img, result = self.get_grasp_img(img, self.depth_cam_k, robot_pos[2])
            if result:
                if not self.ggcnn_cmd_que.empty():
                    self.ggcnn_cmd_que.get()
                self.ggcnn_cmd_que.put([robot_pos, result])
    
    def get_grasp_img(self, depth_image, depth_cam_k, robot_z):
        crop_size = 300
        crop_y_offset = 0
        out_size = 300

        fx = depth_cam_k[0][0]
        fy = depth_cam_k[1][1]
        cx = depth_cam_k[0][2]
        cy = depth_cam_k[1][2]

        imh, imw = depth_image.shape
        crop_y_inx = max(0, imh - crop_size)  # crop height(y) start index
        crop_x_inx = max(0, imw - crop_size)  # crop width(x) start index

        depth_crop, depth_nan_mask = process_depth_image(depth_image, crop_size, out_size, True, crop_y_inx=crop_y_inx, crop_x_inx=crop_x_inx, crop_y_offset=crop_y_offset)

        with TimeIt('Calculate Depth'):
            # Figure out roughly the depth in mm of the part between the grippers for collision avoidance.
            depth_center = depth_crop[100:141, 130:171].flatten()
            depth_center.sort()
            depth_center = depth_center[:10].mean() * 1000.0
        
        with TimeIt('Inference'):
            depth_crop = np.clip((depth_crop - depth_crop.mean()), -1, 1)

            # input_numpy_array = depth_crop.reshape((1, out_size, out_size, 1)) # numpy ndaray.reshape(new_shape), 1*300*300*1 must be equal to total element number!
            # depthT = torch.as_tensor(input_numpy_array).to(self.device) # shallow copy
            # depthT = depthT.permute(0, 3, 1, 2)

            depthT = torch.from_numpy(depth_crop.reshape(1, 1, out_size, out_size).astype(np.float32)).to(self.device)
            with torch.no_grad():
                pred_out = self.model(depthT)

            points_out = pred_out[0].cpu().numpy().squeeze() # shape is now (300,300)
            points_out[depth_nan_mask] = 0
        
        with TimeIt('Trig'):
            # Calculate the angle map.
            cos_out = pred_out[1].cpu().numpy().squeeze()
            sin_out = pred_out[2].cpu().numpy().squeeze()
            ang_out = np.arctan2(sin_out, cos_out) / 2.0
            width_out = pred_out[3].cpu().numpy().squeeze() * 150.0  # Scaled 0-150:0-1

        with TimeIt('Filter'):
            # Filter the outputs.
            # use ndimage.gaussian_filter replace ndimage.filters.gaussian_filter
            points_out = ndimage.gaussian_filter(points_out, 5.0)
            ang_out = ndimage.gaussian_filter(ang_out, 2.0)
            width_out = ndimage.gaussian_filter(width_out, 2.0)
            points_out = np.clip(points_out, 0.0, 1.0-1e-3)

            # depth_crop = depth_crop.squeeze()
        
        with TimeIt('Control'):
            
             # Threshold to create mask
            mask = points_out > 0.2  # Adjust threshold as needed

            # Connected component analysis
            labeled_mask, num_features = ndimage.label(mask)
            
            if num_features == 0:
                max_pixel = np.unravel_index(np.argmax(points_out), points_out.shape)
            else:
                # Measure object sizes
                sizes = ndimage.sum(mask, labeled_mask, range(1, num_features + 1))
                
                # Choose the largest blob (or pick based on centroid proximity to center)
                largest_label = (np.argmax(sizes) + 1)
                isolated_mask = labeled_mask == largest_label

                # Now compute centroid of isolated object
                center_of_mass = ndimage.center_of_mass(isolated_mask.astype(np.float32))
                max_pixel = np.round(center_of_mass).astype(np.int64)

            max_pixel = np.round(max_pixel).astype(np.int64)
            
            self.prev_mp = max_pixel

            ang = ang_out[max_pixel[0], max_pixel[1]]
            width = width_out[max_pixel[0], max_pixel[1]]
            # Convert max_pixel back to uncropped/resized image coordinates in order to do the camera transform.
            max_pixel = ((np.array(max_pixel) / out_size * crop_size) + np.array([crop_y_inx // 2, crop_x_inx // 2]))
            # use np.int64 replace np.int
            max_pixel = np.round(max_pixel).astype(np.int64)
            # print("max_pixel: original [%d, %d]"%(max_pixel[0], max_pixel[1]))
            point_depth = depth_image[max_pixel[0], max_pixel[1]]

            # These magic numbers are my camera intrinsic parameters. # in camera coordinate system
            x = (max_pixel[1] - cx) / (fx) * point_depth
            y = (max_pixel[0] - cy) / (fy) * point_depth
            z = point_depth
            # print("converted xyz in cam coordinate: [%lf, %lf, %lf]"%(x,y,z))
            if np.isnan(z):
                return points_out, None

        with TimeIt('Draw'):
            # Draw grasp markers on the points_out and publish it. (for visualisation)
            # grasp_img = np.zeros((out_size, out_size, 3), dtype=np.uint8)
            # grasp_img[:,:,2] = (points_out * 255.0)
            grasp_img = cv2.applyColorMap((points_out * 255).astype(np.uint8), cv2.COLORMAP_HOT)

            # grasp_img_plain = grasp_img.copy()

            # use disk replace circle
            # rr, cc = circle(self.prev_mp[0], self.prev_mp[1], 5)
            rr, cc = disk(self.prev_mp, 5, shape=(out_size, out_size, 3))
            grasp_img[rr, cc, 0] = 0
            grasp_img[rr, cc, 1] = 255
            grasp_img[rr, cc, 2] = 0
        
        return grasp_img, [x, y, z, ang, width, depth_center]