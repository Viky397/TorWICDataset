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

## Segmentation Mask Loopup table

| Semantic Class     | Colour | RGB     |
|:----:      |    :----:   |          :----:|
| Background      | Title       | Here's this   |
| Ceiling   | Text        | And more      |
| Ego vehicle   | Text        | And more      |
| Wall, fench, pillar   | Text        | And more      |
| Satic feature   | Text        | And more      |
| Rack, shelf   | Text        | And more      |
| Goods material   | Text        | And more      |
| Fixed machinery  | Text        | And more      |
| Cart, pallet, jack   | Text        | And more      |
| Pylon  | Text        | And more      |
| Text   | Text        | And more      |
| Non-staic feature   | Text        | And more      |
| Person   | Text        | And more      |
| Forklift, truck   | Text        | And more      |
| Dynamic feature  | Text        | And more      |
| Driveable ground   | Text        | And more      |
