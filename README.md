# ChangingWarehouseDataset
Releasing a novel dataset taken in a Clearpath Robotics warehouse.

# What to Add
1) Remove the bags that have improper localization (plot poses after extracting txt file)
2) Edit pdf to remove the ones we are not including
3) Add more comments/description to pdf (Explain what baseline is and what changes are)
4) Add finetuning images and description of what labels are and how to finetune (Detectron2)
5) Use synchronization script, then generate rgb (0001.png), depth, txt (ID, time (s and nano-s), x, y, z, qx, qy, qz, qw) and txt of IMU (ID, time (s, ns), acc in xyz and rotation rate in xyz)
6) need camera intrinsics .txt and sensor extrinsics (camera, imu) in a Meta Info txt file


- Create script to automate
