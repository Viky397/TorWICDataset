# ChangingWarehouseDataset
Releasing a novel dataset taken in a Clearpath Robotics warehouse, located [here](https://drive.google.com/drive/folders/12-h2OPmlmxLk0Y9C3Hr5glkalUp66oEJ?usp=sharing).


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
```
## Segmentation Mask Loopup table

| Semantic Class     | Colour | RGB     |
|:----      |    :----:   |          :----:|
| Background      | black       | [0,0,0]   |
| Ceiling   | white       | [255,255,255]     |
| Ego vehicle   | baby blue        | And more      |
| Wall, fench, pillar   | green       | And more      |
| Satic feature   | hot pink        | And more      |
| Rack, shelf   | dark purple       | And more      |
| Goods material   | dark blue       | And more      |
| Fixed machinery  | blue-green        | And more      |
| Cart, pallet, jack   | baby pink        | And more      |
| Pylon  |orange        | And more      |
| Text   | yellow        | And more      |
| Non-staic feature   | red        | And more      |
| Person   | baby purple        | And more      |
| Forklift, truck   | watermelon         | And more      |
| Dynamic feature  | dark green       | And more      |
| Driveable ground   | grey        | And more      |
