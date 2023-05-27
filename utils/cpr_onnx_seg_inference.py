import numpy as np
import onnxruntime 
import os
import sys
import cv2
from PIL import Image
import matplotlib.pyplot as plt
import scipy
import re
from scipy import special
from mask_colors import mask_colors

def tryint(s):
    try:
        return int(s)
    except:
        return s

def alphanum_key(s):
    """ Turn a string into a list of string and number chunks.
        "z23a" -> ["z", 23, "a"]
    """
    return [ tryint(c) for c in re.split('([0-9]+)', s) ]

def sort_nicely(l):

    l.sort(key=alphanum_key)

print(onnxruntime.get_device())

ONNX_FILE = "deploy_model.onnx"
NUM_CLASSES = 16

target = sys.argv[1]
cam = sys.argv[2]
img_path       = '/home/jqian/Downloads/TORWIC/JUN15/'+target+'/image_'+cam+'/'
img_path_sorted = os.listdir(img_path)
sort_nicely(img_path_sorted)
out_grey = '/home/jqian/Downloads/TORWIC/JUN15/'+target+'/segmentation_greyscale_'+cam+'/'
out_color = '/home/jqian/Downloads/TORWIC/JUN15/'+target+'/segmentation_color_'+cam+'/'

def overlay_class_on_image(image, predicted_classes, opacity=0.5):
    predicted_classes_color = (predicted_classes * 255.0 / (NUM_CLASSES - 1)).astype(np.uint8)
    predicted_classes_color = cv2.applyColorMap(predicted_classes_color, cv2.COLORMAP_RAINBOW)

    overlayed_image = cv2.addWeighted(image, 1 - opacity, predicted_classes_color, opacity, 0)
    return overlayed_image.astype(np.uint8)


# def hisEqulColor(img):
#     ycrcb=cv2.cvtColor(img,cv2.COLOR_BGR2YCR_CB)
#     channels=cv2.split(ycrcb)
#     cv2.equalizeHist(channels[0],channels[0])
#     cv2.merge(channels,ycrcb)
#     cv2.cvtColor(ycrcb,cv2.COLOR_YCR_CB2BGR,img)
#     return img

label_conf = []

def main():

    sess = onnxruntime.InferenceSession(ONNX_FILE, providers=['CUDAExecutionProvider'])

    count = 0
    for img in img_path_sorted:
        print(img)
        input_name = sess.get_inputs()[0].name
        input_shape = sess.get_inputs()[0].shape

        image = cv2.imread(os.path.join(img_path, img))
        height, width, channels = image.shape
        #print(width, height)
        image2 = cv2.imread(os.path.join(img_path, img))
        image = cv2.resize(image, (input_shape[2], input_shape[1]), interpolation=cv2.INTER_LINEAR)
        

        image_f = image.astype(np.float32) / 255.0 - 0.5
        feed = dict([(input_name, [image_f])])
        predictions = sess.run(None, feed)
        predictions = predictions[0][0, :, :, :]

        # pred_norm = scipy.special.softmax(predictions, axis=2)
        # label_conf.append(pred_norm[253,268,12])
        
        predictions = cv2.resize(predictions, (width, height), interpolation=cv2.INTER_LINEAR)
        
        # this image contains the perpixel classes, and would be used for downstream algorithms
        predicted_classes = np.argmax(predictions, axis=2).astype(np.uint8)

        # print(predicted_classes)

        # this image is useful for visualization
        #predicted_classes_color = overlay_class_on_image(image2, predicted_classes)

        #cv2.imshow("Segmentation Prediction", predicted_classes_color)
        #cv2.waitKey(0)
        # im = cv2.resize(predicted_classes, (image2.shape[1], image2.shape[0]), interpolation=cv2.INTER_LINEAR)

        #predicted_classes = cv2.resize(predicted_classes, (width,height), interpolation=cv2.INTER_NEAREST)
        mask_mono = Image.fromarray(predicted_classes)
        
        # plt.imshow(im)
        # plt.show()
        # print(image2.shape)
        mask_rgb = np.zeros((height,width,3), dtype=np.uint8)
        mask_rgb = np.flip(mask_colors[predicted_classes],2)
        #for u in range(width):
        #    for v in range(height):
        #        mask_rgb[v,u,:] = np.flip(mask_colors[predicted_classes[v,u]])

        mask_mono.save(out_grey + str(count).zfill(6) + ".png")
        cv2.imwrite(out_color + str(count).zfill(6) + ".png", mask_rgb)
        
        count +=1

if __name__ == '__main__':
    main()
