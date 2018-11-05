import os
import sys
import random
import math
import numpy as np
import skimage.io
import matplotlib
import matplotlib.pyplot as plt

import coco
import utils
import model as modellib

print 'Initializing Mask RCNN network...'
# Root directory of the project
ROOT_DIR = os.getcwd()
ROOT_DIR = "./src/python"
print(ROOT_DIR)

# Directory to save logs and trained model
MODEL_DIR = os.path.join(ROOT_DIR, "logs")

# Path to trained weights file
COCO_MODEL_PATH = os.path.join(ROOT_DIR, "mask_rcnn_coco.h5")

# Set batch size to 1 since we'll be running inference on
# one image at a time. Batch size = GPU_COUNT * IMAGES_PER_GPU
class InferenceConfig(coco.CocoConfig):
    GPU_COUNT = 1
    IMAGES_PER_GPU = 1

config = InferenceConfig()
config.display()


# Create model object in inference mode.
model = modellib.MaskRCNN(mode="inference", model_dir=MODEL_DIR, config=config)

# Load weights trained on MS-COCO
model.load_weights(COCO_MODEL_PATH, by_name=True)
class_names = ['BG', 'person', 'bicycle', 'car', 'motorcycle', 'airplane',
       'bus', 'train', 'truck', 'boat', 'traffic light',
       'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird',
       'cat', 'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear',
       'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie',
       'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball',
       'kite', 'baseball bat', 'baseball glove', 'skateboard',
       'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup',
       'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
       'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza',
       'donut', 'cake', 'chair', 'couch', 'potted plant', 'bed',
       'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote',
       'keyboard', 'cell phone', 'microwave', 'oven', 'toaster',
       'sink', 'refrigerator', 'book', 'clock', 'vase', 'scissors',
       'teddy bear', 'hair drier', 'toothbrush']
print 'Initialated Mask RCNN network...'

def GetDynSeg(image,image2=None):
	h = image.shape[0]
	w = image.shape[1]
	if len(image.shape) == 2:
		im = np.zeros((h,w,3))
		im[:,:,0]=image
		im[:,:,1]=image
		im[:,:,2]=image
		image = im
	#if image2 is not None:
	#	args+=[image2]
	# Run detection
	results = model.detect([image], verbose=0)
	# Visualize results
	r = results[0]
	i = 0
	mask = np.zeros((h,w))
	for roi in r['rois']:
		if class_names[r['class_ids'][i]] == 'person':
			image_m = r['masks'][:,:,i]
			mask[image_m == 1] = 1.
		if class_names[r['class_ids'][i]] == 'bicycle':
			image_m = r['masks'][:,:,i]
			mask[image_m == 1] = 1.
		if class_names[r['class_ids'][i]] == 'car':
			image_m = r['masks'][:,:,i]
			mask[image_m == 1] = 1.
		if class_names[r['class_ids'][i]] == 'motorcycle':
			image_m = r['masks'][:,:,i]
			mask[image_m == 1] = 1.
		if class_names[r['class_ids'][i]] == 'airplane':
			image_m = r['masks'][:,:,i]
			mask[image_m == 1] = 1.
		if class_names[r['class_ids'][i]] == 'bus':
			image_m = r['masks'][:,:,i]
			mask[image_m == 1] = 1.
		if class_names[r['class_ids'][i]] == 'train':
			image_m = r['masks'][:,:,i]
			mask[image_m == 1] = 1.
		if class_names[r['class_ids'][i]] == 'truck':
			image_m = r['masks'][:,:,i]
			mask[image_m == 1] = 1.
		if class_names[r['class_ids'][i]] == 'boat':
			image_m = r['masks'][:,:,i]
			mask[image_m == 1] = 1.
		if class_names[r['class_ids'][i]] == 'bird':
			image_m = r['masks'][:,:,i]
			mask[image_m == 1] = 1.
		if class_names[r['class_ids'][i]] == 'cat':
			image_m = r['masks'][:,:,i]
			mask[image_m == 1] = 1.
		if class_names[r['class_ids'][i]] == 'dog':
			image_m = r['masks'][:,:,i]
			mask[image_m == 1] = 1.
		if class_names[r['class_ids'][i]] == 'horse':
			image_m = r['masks'][:,:,i]
			mask[image_m == 1] = 1.
		if class_names[r['class_ids'][i]] == 'sheep':
			image_m = r['masks'][:,:,i]
			mask[image_m == 1] = 1.
		if class_names[r['class_ids'][i]] == 'cow':
			image_m = r['masks'][:,:,i]
			mask[image_m == 1] = 1.
		if class_names[r['class_ids'][i]] == 'elephant':
			image_m = r['masks'][:,:,i]
			mask[image_m == 1] = 1.
		if class_names[r['class_ids'][i]] == 'bear':
			image_m = r['masks'][:,:,i]
			mask[image_m == 1] = 1.
		if class_names[r['class_ids'][i]] == 'zebra':
			image_m = r['masks'][:,:,i]
			mask[image_m == 1] = 1.
		if class_names[r['class_ids'][i]] == 'giraffe':
			image_m = r['masks'][:,:,i]
			mask[image_m == 1] = 1.		
		i+=1
	#print('GetSeg mask shape:',mask.shape)

	return mask

im = np.zeros((480,640,3))
mask = GetDynSeg(im)
print("Mask R-CNN is correctly working")

