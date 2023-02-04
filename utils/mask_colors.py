import numpy as np

mask_colors = np.array([[0,0,0], #background #black
[255,255,255], #driveable_ground # white
[0,191,255], #ceiling #baby blue
[0,255,0], #wall_fence_pillar #green
[255,0,102], #fixed_machinery #hot pink
[153,0,204], #rack_shelf #dark purple
[51, 51, 204], #text_region #dark blue
[0, 153, 153], #misc_static_feature #blue_green
[255, 204, 255], #goods_material #baby pink
[255, 153, 0], #carts_pallet_jacks #orange
[255, 255, 0], #pylons_cones #yellow
[255, 0, 0], #misc_non_static_feature #red
[204, 102, 255], #person #baby purple
[255, 77, 77], #fork_truck #watermelon  
[0, 153, 51], #misc_dynamic_feature #dark green
[191, 191, 191],
[204, 102, 255], #person #baby purple
[204, 102, 255], #person #baby purple
[204, 102, 255], #person #baby purple
[204, 102, 255]], dtype=np.uint8) #ego_vehicle #grey