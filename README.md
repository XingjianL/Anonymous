
### Repo for TomatoCastle Supplementary Material

### the entire dataset is around 250 GB after compression and depth reduction to 16-bit

fig1.py: code for creating figure 1 using images in sample_dataset_images

### Sample Dataset Images

RGB: Left image

RGB1: Right images with baseline of around 3.81cm

Depth, Depth1: corresponding depth to RGB, in 32-bit float of centimeters (UE5 unit)

Seg: panoptic segmentation: semantic in red channel, see fig1.py for label classes, instances in B,G channel automatically assigned by the simulator


### Additional
high-res images for fig 8

video: showing the UE5 application GUI and high-resolution images streamed over ROS topics

automation_script: sample bash call for configuring the environment and starts a data collection routine
