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

| Semantic Class     |  uint16 Class ID |   Colour | RGB     |
|:----      | :----:   |   :----:   |          :----:|
| Background      | 0  |black       | [0,0,0]   |
| Ceiling   |1  | white       | [255,255,255]     |
| Ego vehicle   | 2  |baby blue        | [0,191,255]      |
| Wall, fench, pillar   | 3  |bright green       | [0,255,0]      |
| Satic feature   | 4  |hot pink        | [255,0,102]      |
| Rack, shelf   | 5  |purple       | [153,0,204]     |
| Goods material   | 6  |dark blue       | [51, 51, 204]    |
| Fixed machinery  |7  | teal        | [0, 153, 153]     |
| Cart, pallet, jack   |8  | baby pink        | [255, 204, 255]    |
| Pylon  |9  |orange        |[255, 153, 0]      |
| Text   | 10  |yellow        | [255, 255, 0]     |
| Non-static feature   | 11  |bright red        | [255, 0, 0]       |
| Person   | 12  |baby purple        | [204, 102, 255]      |
| Forklift, truck   |13  |watermelon         | [255, 77, 77]      |
| Dynamic feature  | 14  |dark green       | [0, 153, 51]     |
| Driveable ground   | 15  |grey        | [191, 191, 191]      |
