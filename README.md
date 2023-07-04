Table of contents
=================

<!--ts-->
   * [The Toronto Warehouse Incremental Change (TorWIC) Mapping Dataset](#The-Toronto-Warehouse-Incremental-Change-(TorWIC)-Mapping-Dataset)
      * [TorWIC-Mapping Description](#TorWIC-Mapping-Description)
      * [Download TorWIC-Mapping dataset](#Download-TorWIC-Mapping-Dataset)
      * [TorWIC-Mapping Data Directory Structure](#TorWIC-Mapping-Data-Directory-Structure)
      * [TorWIC-Mapping Robot and Sensors](#TorWIC-Mapping-Robot-and-sensors)
      * [TorWIC-Mapping Scenarios](#TorWIC-Mapping-Scenarios)
      * [How to Merge Trajectories into ROS Bags](#How-to-Merge-Trajectories-into-ROS-Bags)
      * [TorWIC-Mapping FAQ](#TorWIC-Mapping-FAQ)
      * [POCD Citing](#POCD-Citing)
   * [The Toronto Warehouse Incremental Change SLAM Dataset](#The-Toronto-Warehouse-Incremental-Change-SLAM-Dataset)
      * [TorWIC-SLAM Description](#TorWIC-SLAM-Description)
      * [Download TorWIC-SLAM Dataset](#Download-TorWIC-SLAM-Dataset)
      * [TorWIC-SLAM Data Directory Structure](#TorWIC-SLAM-Data-Directory-Structure)
      * [TorWIC-SLAM Robot and Sensors](#TorWIC-SLAM-Robot-and-Sensors)
      * [TorWIC-SLAM Scenarios](#TorWIC-SLAM-Scenarios)
      * [TorWIC-SLAM FAQ](#TorWIC-SLAM-FAQ)
      * [POV-SLAM Citing](#POV-SLAM-Citing)
   * [Ground-truth Segmentation for Fine-tuning](#Ground-truth-Segmentation-for-Fine-tuning)
   * [Acknowledgements](#Acknowledgements)
<!--te-->

# The Toronto Warehouse Incremental Change (TorWIC) Mapping Dataset 

## TorWIC-Mapping Description
This repository contains the released dataset discussed in **POCD: Probabilistic Object-Level Change Detection and Volumetric Mapping in Semi-Static Scenes**, [[Paper]](https://github.com/Viky397/TorWICDataset/blob/main/Qian_Chatrath_POCD.pdf), [[Supplementary Material]](https://github.com/Viky397/TorWICDataset/blob/main/Qian_Chatrath_POCD_SuppMaterial.pdf). The purpose of this dataset is to evaluate the map mainteneance capabilities in a warehouse environment undergoing incremental changes. This dataset is collected in a [Clearpath Robotics](https://clearpathrobotics.com/) facility.

In the image below is an example of two frames captures by the robot at the AprilTag in two scenarios (Scenario_2-2: top and Scenario_4-1:bottom). Changes include 3 stacks of boxes added in front of the fence, and an additional box wall to the right of the fence.
![comparison](/Figures/compare.png)

## Download TorWIC-Mapping Dataset
The novel dataset taken in the Clearpath Robotics warehouse is located [here](https://drive.google.com/drive/folders/12-h2OPmlmxLk0Y9C3Hr5glkalUp66oEJ?usp=sharing). This dataset contains 18 trajectories that can be merged using the provided script to create a changing environment.

## TorWIC-Mapping Data Directory Structure
The configuration changes can be seen in the following ![layout description document](https://github.com/Viky397/TorWICDataset/blob/main/TorWIC_Dataset.pdf).

Each scenario configuration follows the same folder structure, as seen below. Please see the next section on further details regarding each sensor. 

```
WarehouseSequences
|
|----Baseline configuration
|       +--- rgb                      # 0000.png - xxxx.png      
|       +--- depth                    # 0000.png - xxxx.png
|       +--- segmentation             # 0000.png - xxxx.png    
|       +--- laser scans              # 0000.pcd - xxxx.pcd 
|       --- poses.txt 
|       --- imu.txt 
|       --- odom.txt
|
|---- 1- Box Shifts and Rotations  
|
|-------Sequence 1-1
|       +--- rgb                      # 0000.png - xxxx.png      
|       +--- depth                    # 0000.png - xxxx.png
|       +--- segmentation             # 0000.png - xxxx.png   
|       +--- laser scans              # 0000.pcd - xxxx.pcd
|       --- poses.txt 
|       --- imu.txt 
|       --- odom.txt
|
|-------Sequence 1-X
|
|---- 2- Removing Boxes
|
|-------Sequence 2-1
|       +--- rgb                      # 0000.png - xxxx.png      
|       +--- depth                    # 0000.png - xxxx.png
|       +--- segmentation             # 0000.png - xxxx.png  
|       +--- laser scans              # 0000.pcd - xxxx.pcd
|       --- poses.txt 
|       --- imu.txt 
|       --- odom.txt
|
|-------Sequence 2-X
|
|---- X: Configuration Change
```

## TorWIC-Mapping Robot and Sensors
The dataset was collected on the [OTTO 100 Autonomous Mobile Robot](https://ottomotors.com/100\#stats), remote controlled by a human operator at walking speed. We record sensor measurements from an Intel RealSense D435i RGB-D camera, a wheel encoder, an IMU unit, and a Hokuyo UAM501 2D laser scanner, all rigidly mounted on the platform. The following figure shows the robot platform and the sensor frames, and the following table lists the specifications and formats of the sensor measurements.

![Otto Robot](/Figures/otto.png)
![Table of sensors](/Figures/sensor_table.png)

## TorWIC-Mapping Scenarios
The dataset provides 18 trajectories in 4 scenarios, including the baseline setup. Each trajectory contains the robot traversing through a static  configuration of the environment, starting and finishing at the fixed April-Tag. Users can stitch the trajectories together with the provided script to create routes with structural changes in the scene. A high-level overview of the scenarios and trajectories is listed in the table below. 

![trajs](/Figures/trajs.png)

## How to Merge Trajectories into ROS Bags

1) Clone this repository
2) Ensure the Scenario folders that contain the dataset are in the repository folder
3) Create a folder called ```outputs``` in the repository folder
4) Run: ```python3 utils/create_rosbag_from_trajs.py <traj 1> <traj_2> ... <traj_n>``` \
For example: ```python3 utils/create_rosbag_from_trajs.py 1-2 3-1```

The script relies on ```pypcd``` to proces the laser scans. If you are using Python3, please use the following [version](https://github.com/dimatura/pypcd/pull/35).

## TorWIC-Mapping FAQ
Q) Is the sensor data synchronized? \
A) The sensors on the OTTO 100 platform are not synchronized with each other. For our dataset, we used the RealSense image timestamps as the reference, and take the measurements with the closest timestamp from the LiDAR and the poses. The provided odometry and IMU data is not sub-sampled. Please contact us if you need the unprocessed, raw data (as rosbags). 

Q) How were the poses obtained? \
A) The poses of the robot were obtained offline using a proprietary LiDAR-based SLAM solution. 

Q) How were the segmentation masks obtained? \
A) The provided semantic segmentation masks are not perfect. We trained a semantic segmentation model on thousands of human-labeled warehouse images, and ran inference on the RGB images to obtain the masks. Unfortunately, the training data is proprietary and cannot be released. However, we release a subset of this dataset so users can fine-tune their models. 

Q) Where are the sensor intrinsics and extrinsics? How were they obtained?\
A) This information is provided in the data Google Drive link in the text file. Sensor extrinsics are also provided in the ROS bags under the tf\_static topic. This information is the same for all trajectories. The RealSense camera was calibrated with the Intel's OEM calibration tool. Sensor extrinsics are factory calibrated.

Q) What is the unit of the depth values? \
A) The depth images from RealSense D435i are scaled by a factor of 1000. The uint16 values can be converted into float values and multiplied by 0.001 to get depth values in meters. 

## POCD Citing

When using **POCD** or the dataset in your research, please cite the following publication:

Jingxing Qian, Veronica Chatrath, Jun Yang, James Servos, Angela Schoellig, and Steven L. Waslander, **POCD: Probabilistic Object-Level Change Detection and Volumetric Mapping in Semi-Static Scenes**, _2022 Robotics: Science and Systems (RSS)_, 2022. [[Paper]](https://github.com/Viky397/TorWICDataset/blob/main/Qian_Chatrath_POCD.pdf), [[Supplementary Material]](https://github.com/Viky397/TorWICDataset/blob/main/Qian_Chatrath_POCD_SuppMaterial.pdf) 

```bibtex
@INPROCEEDINGS{QianChatrathPOCD,
  author={Qian, Jingxing and Chatrath, Veronica and Yang, Jun and Servos, James and Schoellig, Angela and Waslander, Steven L.},
  booktitle={2022 Robotics: Science and Systems (RSS)}, 
  title={{POCD: Probabilistic Object-Level Change Detection and Volumetric Mapping in Semi-Static Scenes}}, 
  year={2022},
  volume={},
  number={},
  pages={},
  doi={}}
```

# The Toronto Warehouse Incremental Change SLAM Dataset

## TorWIC-SLAM Description
This repository contains the released dataset discussed in **POV-SLAM: Probabilistic Object-Level Variational SLAM**, [[Paper]](https://github.com/Viky397/TorWICDataset/blob/main/Qian_POV_SLAM.pdf), [[Supplementary Material]](https://github.com/Viky397/TorWICDataset/blob/main/Qian_POV_SLAM_SuppMaterial.pdf). The purpose of this dataset is to evaluate the SLAM capabilities in a warehouse environment undergoing incremental changes. This dataset is collected in a [Clearpath Robotics](https://clearpathrobotics.com/) facility on three data collections days over a four months span.

In the image below is an example of two frames captures by the robot while traversing in the aisles four months apart.
![comparison](/Figures/slam_sample_frame.png)

## Download TorWIC-SLAM Dataset
The real-world dataset taken in the Clearpath Robotics warehouse is located [here](https://drive.google.com/drive/folders/12-h2OPmlmxLk0Y9C3Hr5glkalUp66oEJ?usp=sharing). This dataset contains 20 trajectories. Since the robot all starts close to the same position and orientation (origin of the ground-turth map), users can merge them to create a changing environment.

## TorWIC-SLAM Data Directory Structure
Each scenario configuration follows the same folder structure, as seen below. Please see the next section on further details regarding each sensor. 

```
|--groundtruth_map.ply                       # Ground-truth 3D scan of the warehouse, captured using Leica MS-60 Total Station
|--calibrations.txt                          # Extrinsic and intrinsic calibrations of the Azure RGB-D cameras
|
|--WarehouseSequences
|
|----Scenario 1 Traversal 1
|       +--- image_left                      # 0000.png - xxxx.png, RGB images from left Azure camera
|       +--- image_right                     # 0000.png - xxxx.png, RGB images from right Azure camera
|       +--- depth_left                      # 0000.png - xxxx.png, RGB-aligned depth images from left Azure camera
|       +--- depth_right                     # 0000.png - xxxx.png, RGB-aligned depth images from right Azure camera
|       +--- segmentation_color_left         # 0000.png - xxxx.png, Segmentation masks (colored) from left Azure camera
|       +--- segmentation_color_right        # 0000.png - xxxx.png, Segmentation masks (colored) from right Azure camera
|       +--- segmentation_greyscale_left     # 0000.png - xxxx.png, Segmentation masks (class ID) from left Azure camera
|       +--- segmentation_greyscale_right    # 0000.png - xxxx.png, Segmentation masks (class ID) from right Azure camera
|       +--- lidar                           # 0000.pcd - xxxx.pcd, 3D scans from Ouster OS1-128 LiDAR
|       --- imu_left.txt                     # Raw IMU data from left Azure camera
|       --- imu_right.txt                    # Raw IMU data from right Azure camera
|       --- frame_times.txt                  # Timestamps of RGB, depth, segmentation and lidar frames
|       --- traj_gt.txt                      # Ground-truth poses of Ouster OS1-128 LiDAR
|
|
|----Scenario 1 Traversal 2
|       +--- image_left                      # 0000.png - xxxx.png
|       +--- image_right                     # 0000.png - xxxx.png
|       +--- depth_left                      # 0000.png - xxxx.png
|       +--- depth_right                     # 0000.png - xxxx.png
|       +--- segmentation_color_left         # 0000.png - xxxx.png
|       +--- segmentation_color_right        # 0000.png - xxxx.png
|       +--- segmentation_greyscale_left     # 0000.png - xxxx.png
|       +--- segmentation_greyscale_right    # 0000.png - xxxx.png
|       +--- lidar                           # 0000.pcd - xxxx.pcd
|       --- imu_left.txt                    
|       --- imu_right.txt                   
|       --- frame_times.txt                 
|       --- traj_gt.txt                      
|
|
|
|----Scenario X Traversal Y
|       +--- image_left                      # 0000.png - xxxx.png
|       +--- image_right                     # 0000.png - xxxx.png
|       +--- depth_left                      # 0000.png - xxxx.png
|       +--- depth_right                     # 0000.png - xxxx.png
|       +--- segmentation_color_left         # 0000.png - xxxx.png
|       +--- segmentation_color_right        # 0000.png - xxxx.png
|       +--- segmentation_greyscale_left     # 0000.png - xxxx.png
|       +--- segmentation_greyscale_right    # 0000.png - xxxx.png
|       +--- lidar                           # 0000.pcd - xxxx.pcd
|       --- imu_left.txt                    
|       --- imu_right.txt                   
|       --- frame_times.txt                 
|       --- traj_gt.txt 
```
## TorWIC-SLAM Robot and Sensors
The dataset was collected on the [OTTO 100 Autonomous Mobile Robot](https://ottomotors.com/100\#stats), remote controlled by a human operator at walking speed. We record sensor measurements from two Azure Kinect RGB-D cameras and a Ouster OS1-128 3D LiDAR, all rigidly mounted on the top the platform. The following figure shows the robot platform and the sensor frames, and the following table lists the specifications and formats of the sensor measurements.

![SLAM Otto Robot](/Figures/slam_sensor_frames.png)
![Table of sensors](/Figures/slam_sensors_overview.png)

We further provide centimeter-accurate 3D scan of the warehouse and robot poses. The image below shows a top view of the map.

![Warehouse 3D Map](/Figures/slam_gt_map.png)


## TorWIC-SLAM Scenarios

The dataset provides 20 trajectories in 3 scenarios, captured on 3 data collections days over 4 months. Each trajectory contains the robot traversing through regions of the warehouse, following a predefined path in clockwise and counter-clockwise directions. The robot always starts at the origin of our map frame. Users can stitch the trajectories together to create routes with structural changes in the scene. The graphical illustrations of the scenarios and the dataset breakdown are shown below. 

![SLAM Scenarios](/Figures/slam-scenarios.png)

![SLAM Scenarios Type Breakdown](/Figures/slam_scenarios_type.png)

![SLAM Scenarios Date Breakdown](/Figures/slam_scenarios_date.png)

## TorWIC-SLAM FAQ
Q) Is the sensor data synchronized? \
A) The two Azure Kinect RGB-D cameras are configured in a master (left) - subordinate (right) setup. The cameras and the Ouster OS1-128 LiDAR are synchronized using Precision Time Protocal (PTP). Unfortunately, it was not possible to synchronize the mounted sensors with the OTTO 100 base. The provided raw data (ROS bags) also contain additional robot base sensors such as wheel odometry but they need to be synchonized manually. 

Q) How was the ground-truth 3D map obtained? \
A) We used a Leica MS60 Total Station to scan portions of the warehouse. These sub-scans were processed and stiched together using the Leica Cyclone Register 360 software.

Q) How were the poses obtained? \
A) The poses (of the mounted Ouster OS1-128 LiDAR) were obtained by aligning the 3D scans against the ground-truth 3D map.

Q) How were the segmentation masks obtained? \
A) The provided semantic segmentation masks are not perfect. We trained a semantic segmentation model on thousands of human-labeled images from the same warehouse, and ran inference on the RGB images to obtain the masks. Unfortunately, the training data is proprietary and cannot be released. However, we release a subset of this dataset so users can fine-tune their models. 

Q) How were the sensor intrinsics and extrinsics obtained?\
A) The calibration information is provided with the dataset in a text file. The Azure Kinect RGB cameras were calibrated using the Matlab camera calibration toolbox. Their extrinsics w.r.t. the Ouster LiDAR were obtained by aligning the depth data against the LiDAR scans during a calibration session.

Q) What is the unit of the depth values? \
A) The depth images from the Azure Kinect cameras are scaled by a factor of 1000. The uint16 values can be converted into float values and multiplied by 0.001 to get depth values in meters. 


## POV-SLAM Citing

When using **POV-SLAM** or the dataset in your research, please cite the following publication:

Jingxing Qian, Veronica Chatrath, James Servos, Aaron Mavrinac, Wolfram Burgard, Steven L. Waslander, and Angela Schoellig, **POV-SLAM: Probabilistic Object-Level Variational SLAM**, _2023 Robotics: Science and Systems (RSS)_, 2023. [[Paper]](https://github.com/Viky397/TorWICDataset/blob/main/Qian_Chatrath_POVSLAM.pdf), [[Supplementary Material]](https://github.com/Viky397/TorWICDataset/blob/main/Qian_Chatrath_POVSLAM_SuppMaterial.pdf) 

```bibtex
@INPROCEEDINGS{QianChatrathPOVSLAM,
  author={Qian, Jingxing and Chatrath, Veronica and Servos, James and Mavrinac, Aaron and Burgard, Wolfram and Waslander, Steven L. and Schoellig, Angela},
  booktitle={2023 Robotics: Science and Systems (RSS)}, 
  title={{POV-SLAM: Probabilistic Object-Level Variational SLAM}}, 
  year={2023},
  volume={},
  number={},
  pages={},
  doi={}}
```

# Ground-truth Segmentation for Fine-tuning
A proprietary model was trained to produce the semantic labels for the datasets. Unfortunately, the full training data cannot be released. However, we release a subset of this data such that users can fine-tune their models, if needed. Within the [main folder](https://drive.google.com/file/d/1ovm4ycVrQfpuseI2Kc8TofS-LI0Nly_I/view?usp=sharing), there are 79 folders with unique ID's. Within each folder, there are 3 sets of images, each within its own sub-folder. Each image folder contains an image of the individual semantic mask, the source RGB image, the combined semantic mask image, the combined semantic indexed image, and an annotation ```.json file```. For training purposes, the combined indexed image should be used ```combined_indexedImage.png```. Each pixel holds the class ID of the semantic class corresponding to the table below. The datasets also contain colourized masks that correspond to the Class ID column in the table below. 

| Semantic Class     |  uint16 Class ID |   Colour | RGB     |
|:----      | :----:   |   :----:   |          :----:|
| Background      | 0  |black       | [0,0,0]   |
| Driveable Ground   |1  | white       | [255,255,255]     |
| Ceiling  | 2  |baby blue        | [0,191,255]      |
| Ego Vehicle    | 3  |bright green       | [0,255,0]      |
| Wall/Fence/Pillar | 4  |hot pink        | [255,0,102]      |
| Miscellaneous Static Feature   | 5  |purple       | [153,0,204]     |
| Rack/Shelf   | 6  |dark blue       | [51, 51, 204]    |
| Goods Materials |7  | teal        | [0, 153, 153]     |
| Fixed Machinery   |8  | baby pink        | [255, 204, 255]    |
| Cart/Pallet Jack  |9  |orange        |[255, 153, 0]      |
| Pylons   | 10  |yellow        | [255, 255, 0]     |
| Text Region   | 11  |bright red        | [255, 0, 0]       |
| Miscellaneous Non-Static Feature   | 12  |baby purple        | [204, 102, 255]      |
| Person |13  |watermelon         | [255, 77, 77]      |
| Forklift/Truck | 14  |dark green       | [0, 153, 51]     |
|Miscellaneous Dynamic Feature    | 15  |grey        | [191, 191, 191]      |



# Acknowledgments
This work was supported by the Vector Institute for Artificial Intelligence in Toronto and the NSERC Canadian Robotics Network (NCRN). We would like to thank Clearpath Robotics for providing the facility and the robot platform that made this dataset possible. 
