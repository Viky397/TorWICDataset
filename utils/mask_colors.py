import numpy as np

mask_colors = np.array([[0,0,0], #background #black
[255,255,255], #ceiling # white
[0,191,255], #ego_vehicle #baby blue
[0,255,0], #wall_fence_pillar #green
[255,0,102], #misc_static_feature #hot pink
[153,0,204], #rack_shelf #dark purple
[51, 51, 204], #goods_material #dark blue
[0, 153, 153], #fixed_machinery #blue_green
[255, 204, 255], #cart_pallet_jack #baby pink
[255, 153, 0], #pylon_cone #orange
[255, 255, 0], #text_region #yellow
[255, 0, 0], #misc_non_static_feature #red
[204, 102, 255], #person #baby purple
[255, 77, 77], #fork_truck #watermelon  
[0, 153, 51], #misc_dynamic_feature #dark green
[191, 191, 191]], dtype=np.uint8) #driveable_ground #grey