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
| Ego vehicle   | baby blue        | [0,191,255]      |
| Wall, fench, pillar   | bright green       | [0,255,0]      |
| Satic feature   | hot pink        | [255,0,102]      |
| Rack, shelf   | purple       | [153,0,204]     |
| Goods material   | dark blue       | [51, 51, 204]    |
| Fixed machinery  | teal        | [0, 153, 153]     |
| Cart, pallet, jack   | baby pink        | [255, 204, 255]    |
| Pylon  |orange        |[255, 153, 0]      |
| Text   | yellow        | [255, 255, 0]     |
| Non-static feature   | bright red        | [255, 0, 0]       |
| Person   | baby purple        | [204, 102, 255]      |
| Forklift, truck   | watermelon         | [255, 77, 77]      |
| Dynamic feature  | dark green       | [0, 153, 51]     |
| Driveable ground   | grey        | [191, 191, 191]      |
