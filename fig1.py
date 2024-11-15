# Code used to create figure 1, the actual figure had exposure turned up 1
# will need following packages
import cv2
import matplotlib.pyplot as plt
import numpy as np
import os
from scipy import stats
import open3d as o3d
os.environ["OPENCV_IO_ENABLE_OPENEXR"]="1"

color_path =    "./sample_dataset_images/rgb_14.png"
segment_path =  "./sample_dataset_images/seg_14.png"
depth_path =    "./sample_dataset_images/depth_14.exr"
depth_r_path = "./sample_dataset_images/depth_14.exr"
depth_l_path = "./sample_dataset_images/depth1_14.exr"

semantic_map = {
                "bacterial_spot":       (1, 5),
                "early_blight":         (2, 10),
                "late_blight":          (3, 20),
                "leaf_mold":            (4, 25),
                "septoria_leaf_spot":   (5,30),
                "spider_mites":         (6,35),
                "target_spot":          (7,40),
                "mosaic_virus":         (8,45),
                "yellow_leaf_curl_virus":(9,50),
                "healthy_leaf_pv":      (10, 15),  # plant village healthy leaf
                "healthy_leaf_t":       (10, 255), # texture leaf (healthy)
                "background":           (11, 0),
                "tomato":               (12, 121),
                "stem":                 (13, 111),
                "wood_rod":             (14, 101),
                "red_band":             (15, 140),
                "yellow_flower":        (16, 131)
                }

# used to adjust to correct colors
# since the rendered semantics sometimes blends some colors near the leaves boundary
# basically a mode kernel
def maj_vote(img,x,y,n=3):
    half = n // 2
    x_min, x_max = max(0, x - half), min(img.shape[1], x + half + 1)
    y_min, y_max = max(0, y - half), min(img.shape[0], y + half + 1)
    
    window = img[y_min:y_max, x_min:x_max].flatten()
    window = window[window != 0]
    
    if len(window) > 0:
        # Perform majority voting
        most_common_label = stats.mode(window, keepdims=True)[0][0]
        
        return most_common_label
    else:
        return 11
    
def color_to_id(img_semantic, top_k_disease = 10, semantic_map = semantic_map):
    semantic_id_img = np.zeros(img_semantic.shape)
    disease_counts = []
    for class_name, id_value_map in semantic_map.items():
        print(class_name, np.sum(np.where(img_semantic == id_value_map[1], 1, 0)))
        if id_value_map[1] < 60 and id_value_map[1] > 1:
            disease_counts.append(np.sum(np.where(img_semantic == id_value_map[1], 1, 0)))
        semantic_id_img[img_semantic == id_value_map[1]] = id_value_map[0]
    for i, item_i in enumerate(np.argsort(disease_counts)[::-1]):
        print(i)
        if i >= top_k_disease:
            print(f"Setting {list(semantic_map.items())[item_i][0]} to unknown")
            semantic_id_img[img_semantic == list(semantic_map.items())[item_i][1][1]] = 0
        else:
            print(f"{list(semantic_map.items())[item_i][0]} kept")
            
    # Apply majority voting for unlabeled pixels
    print(f"maj vote: {np.sum(np.where(semantic_id_img == 0, 1, 0))}")
    for y in range(semantic_id_img.shape[0]):
        for x in range(semantic_id_img.shape[1]):
            if semantic_id_img[y, x] == 0:
                semantic_id_img[y, x] = maj_vote(semantic_id_img, x, y)
    return semantic_id_img

depth_image = cv2.imread(depth_path, cv2.IMREAD_ANYCOLOR | cv2.IMREAD_ANYDEPTH) # in cm
depth_image = depth_image * 10 # in mm

img = cv2.cvtColor(cv2.imread(segment_path),cv2.COLOR_BGR2RGB)
rgb = cv2.cvtColor(cv2.imread(color_path),cv2.COLOR_BGR2RGB)
mask = np.where(depth_image < 10000, 1, 0).astype(np.uint8)

# semantic segmentation
top_k_disease = 10 # how many disease classes in the image
mapped_img = color_to_id(img[:,:,0], top_k_disease)
# instance segmentation
instance_img = (img[:,:,1].astype(int)+1) * (img[:,:,2].astype(int)+1)
unique_instances, counts = np.unique(instance_img, return_counts=True)
new_instance_img = np.zeros(instance_img.shape)
inst_id = 0
for i,unique_inst in enumerate(unique_instances):
    if counts[i] > 5000:
        inst_id += 1
        new_instance_img[instance_img == unique_inst] = inst_id

mask_mapped_img = cv2.bitwise_and(mapped_img,mapped_img,mask=mask)
mask_instance_img = cv2.bitwise_and(new_instance_img,new_instance_img,mask=mask)

depth_image_r = cv2.imread(depth_r_path, cv2.IMREAD_ANYCOLOR | cv2.IMREAD_ANYDEPTH) # in cm
depth_image_r = depth_image_r * 10 # to mm
depth_image_r = np.where(depth_image_r > 5000, 0, depth_image_r) # limit to 5m
depth_image_l = cv2.imread(depth_l_path, cv2.IMREAD_ANYCOLOR | cv2.IMREAD_ANYDEPTH)
depth_image_l = depth_image_l * 10
depth_image_l = np.where(depth_image_l > 5000, 0, depth_image_l)

fov = 95.452621 # HFOV from csv/robot_<timestamp>.csv
fx = (2448 / np.tan((fov*np.pi/180.0)/2.0)) / 2
intrinsics = o3d.camera.PinholeCameraIntrinsic(2448,2048,fx,fx,2448/2,2048/2)
o3d_pcd_r = o3d.geometry.PointCloud.create_from_depth_image(o3d.geometry.Image(depth_image_r), intrinsics)
o3d_pcd_l = o3d.geometry.PointCloud.create_from_depth_image(o3d.geometry.Image(depth_image_l), intrinsics)
o3d_pcd_l.translate(np.array([38.8112,0,0])) # baseline in mm, from UE5 simulation (not recorded in csv)
o3d_pcd_l.paint_uniform_color([1,0,0])
o3d_pcd_r.paint_uniform_color([0,1,0])

# computing distances between left/right point clouds and closest neighbor for each point cloud
dist_rl = np.asarray(o3d_pcd_r.compute_point_cloud_distance(o3d_pcd_l))
nn_r = np.asarray(o3d_pcd_r.compute_nearest_neighbor_distance())+1

# get points that have a closer point in the other pointcloud than itself
dist_rl_filter_id = np.where(dist_rl/nn_r < 1.02)[0]

filtered_right_pcd = o3d_pcd_r.select_by_index(dist_rl_filter_id)
filtered_right_pcd.paint_uniform_color([0,0,1])

# transform 3d pointclouds to 2d depth (with occlusion for unobtainable depth from stereo)
def pcd_to_depth(pcd):
    img = np.ones((2048,2448)) * np.inf
    points = np.asarray(pcd.points)
    print(points.shape)
    u = np.round(fx * points[:,0] / points[:,2] + 1224).astype(int)
    v = np.round(fx * points[:,1] / points[:,2] + 1024).astype(int)

    for i, pix in enumerate(zip(u,v)):
        p_u, p_v = pix
        if p_u < 2448 and p_u > 0 and p_v < 2048 and p_v > 0:
            if points[i,2] < img[p_v, p_u]:
                img[p_v,p_u] = points[i,2]
    return img
h,w,c = rgb.shape
mono_depth = np.log(depth_image_r[2*h//3:,2*w//4:])
filtered_right = np.log(pcd_to_depth(filtered_right_pcd))[h//3:2*h//3,2*w//4:]
import matplotlib.cm as cm

filtered_right[np.isinf(filtered_right)] = np.nan

# Normalize depth values, ignoring NaN
norm = plt.Normalize(np.nanmin(filtered_right), np.nanmax(filtered_right))
colors = cm.viridis(norm(filtered_right))
visual = np.zeros((h,w,4), dtype=np.uint8)
visual[2*h//3:,2*w//4:] = cm.viridis(norm(mono_depth))*220
visual[2*h//3:,2*w//4:,3] = 255
visual[2*h//3:,2*w//4:,:3] += (rgb[2*h//3:,2*w//4:]/255.*30).astype(np.uint8)
visual[h//3:2*h//3,2*w//4:] = colors*220
visual[h//3:2*h//3,2*w//4:,:3] += (rgb[h//3:2*h//3,2*w//4:]/255.*30).astype(np.uint8)
visual[h//3:2*h//3,2*w//4:,3] = 255
visual[2*h//3:,:2*w//4,:3] = (rgb[2*h//3:,:2*w//4,:]).astype(np.uint8)
visual[2*h//3:,:2*w//4,3] = 255
visual[h//3:2*h//3,:2*w//4,:3] = (rgb[h//3:2*h//3,:2*w//4,:]/255*100).astype(np.uint8)
visual[h//3:2*h//3,:2*w//4,0] += (np.clip(mask_instance_img[h//3:2*h//3,:2*w//4]-13,0,100)*100).astype(np.uint8)
visual[h//3:2*h//3,:2*w//4,1] += (np.clip(mask_instance_img[h//3:2*h//3,:2*w//4]-12,0,100)*50).astype(np.uint8)
visual[h//3:2*h//3,:2*w//4,2] += (np.clip(mask_instance_img[h//3:2*h//3,:2*w//4]-13,0,100)*100).astype(np.uint8)
visual[h//3:2*h//3,:2*w//4,3] = 255

# set background to 0 for color_map
dise = np.where(mask_mapped_img[:h//3,:]==11, 0, mask_mapped_img[:h//3,:])
color_map = [[128, 64, 128], [244, 35, 232], [70, 70, 70], [102, 102, 156],
            [190, 153, 153], [153, 153, 153], [250, 170, 30], [220, 220, 0],
                 [107, 142, 35], [152, 251, 152], [70, 130, 180],
                 [220, 20, 60], [255, 0, 0], [0, 0, 142], [0, 0, 70],
                 [0, 60, 100], [0, 80, 100], [0, 0, 230], [119, 11, 32]]
# Color the visual using 'dise'
for i in range(int(np.max(dise))+1):
    mask = (dise == i)
    color_value = (np.array(list(color_map[i][:3])+[255])/255*200).astype(np.uint8) # Convert to 0-255 scale
    print(color_value, mask.shape)
    visual[:h//3,:][mask] = color_value  # Assign color to the mask
visual[:h//3,:,:3] += (rgb[:h//3,:] / 255 * 50).astype(np.uint8)
visual[:h//3,:,3] = 255

print("plotting")
plt.imshow(visual)
plt.title("colored")
plt.imsave("fig1.png",cv2.rotate(visual,cv2.ROTATE_90_CLOCKWISE))
plt.show()
