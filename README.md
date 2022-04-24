# ChangingWarehouseDataset
Releasing a novel dataset taken in a Clearpath Robotics warehouse.

# What to Add
1) Remove the bags that have improper localization (plot poses after extracting txt file)
- Bag 3-3 has broken localization
3) Edit pdf to remove the ones we are not including
4) Add more comments/description to pdf (Explain what baseline is and what changes are)
5) Add finetuning images and description of what labels are and how to finetune (Detectron2)
6) Use synchronization script, then generate rgb (0001.png), depth, txt (ID, time (s and nano-s), x, y, z, qx, qy, qz, qw) and txt of IMU (ID, time (s, ns), acc in xyz and rotation rate in xyz)
7) need camera intrinsics .txt and sensor extrinsics (camera, imu) in a Meta Info txt file


- Create script to automate


### Data directory structure
```
WarehouseSequences
|
|----Default configuration
|       +--- rgb                      # 0000.png - xxxx.png      
|       +--- depth                    # 0000.png - xxxx.png
|       +--- semantic_segmentation    # 0000.png - xxxx.png     
|       --- poses.txt 
|       --- imu.txt 
|
|---- 1- Boxes Move  
|
|-------Sequence 1
|       +--- rgb                      # 0000.png - xxxx.png      
|       +--- depth                    # 0000.png - xxxx.png
|       +--- semantic_segmentation    # 0000.png - xxxx.png     
|       --- poses.txt 
|       --- imu.txt 
|
|-------Sequence N
|
|---- 2- Walls Move
|
|-------Sequence 1
|       +--- rgb                      # 0000.png - xxxx.png      
|       +--- depth                    # 0000.png - xxxx.png
|       +--- semantic_segmentation    # 0000.png - xxxx.png     
|       --- poses.txt 
|       --- imu.txt 
|
|........+-- Seq_N
|
|---- X: Configuration Change
