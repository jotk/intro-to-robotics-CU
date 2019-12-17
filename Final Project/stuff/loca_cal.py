import pdb
import pickle
import random
import copy
import cv2
import numpy as np
import math



color_ranges = []
centroid_group = []
def add_color_range_to_detect(lower_bound, upper_bound):
  global color_ranges
  color_ranges.append([lower_bound, upper_bound]) 

def check_if_color_in_range(bgr_tuple):
	for entry in color_ranges:
		lower, upper = entry[0], entry[1]
	in_range = True

	if bgr_tuple < lower or bgr_tuple > upper:
		in_range = False
	if in_range: return True
	return False





def get_mask(img):
	img_height = img.shape[0]
	img_width = img.shape[1]
	mask = np.zeros([img_height, img_width])
	for y in range(0,img_height):
		for x in range (0,img_width):
			if(check_if_color_in_range(img[y][x])):
				mask[y][x]=1;
	return mask


def expand_nr(img_mask, cur_coord, coordinates_in_blob):
  
  coordinates_in_blob = []
  coordinate_list = [cur_coord] # List of all coordinates to try expanding to
  while len(coordinate_list) > 0:
    cur_coordinate = coordinate_list.pop() # Take the first coordinate in the list and perform 'expand' on it
    if cur_coordinate[0] < 0 or cur_coordinate[1] < 0 or cur_coordinate[0] >= img_mask.shape[0] or cur_coordinate[1] >= img_mask.shape[1]:
      continue
    if img_mask[cur_coordinate[0], cur_coordinate[1]] == 0.0:
      continue
    img_mask[cur_coordinate[0],cur_coordinate[1]] = 0   
    coordinates_in_blob.append(cur_coordinate)
    
    above = [cur_coordinate[0]-1, cur_coordinate[1]]
    below = [cur_coordinate[0]+1, cur_coordinate[1]]
    left = [cur_coordinate[0], cur_coordinate[1]-1]
    right = [cur_coordinate[0], cur_coordinate[1]+1]


    for coord in [above, below, left, right]:
        coordinate_list.append(coord)


  return coordinates_in_blob


def get_blobs(img_mask):
  
  img_mask_height = img_mask.shape[0]
  img_mask_width = img_mask.shape[1]
  mask_dup = copy.copy(img_mask)
  blobs_list = []
  print(mask_dup) 
  for y in range (0,img_mask_height):
      for x in range (0,img_mask_width):
          if (mask_dup[y][x]):
          		blobs = expand_nr(mask_dup, [y,x], [])
          		blobs_list.append(blobs)
  

  return blobs_list

#Todo
def compute_distance(p1,p2):
	distance = math.sqrt(((p1[0]-p2[0])**2)+((p1[1]-p2[1])**2))
	return distance

#Todo
def get_close_centroid(centroid_list):
	global centroid_group
	for i in range (0,len(centroid_list)):
		if(i <= len(centroid_list)-2):
			distance = compute_distance(centroid_list[i], centroid_list[i+1])
			if distance < 5:
				centroid_group.append(centroid_list[i])

	return centroid_group

def get_blob_centroids(blobs_list):

  object_positions_list = []
  for i in blobs_list:
      centroid = np.mean(i,axis=0)
      object_positions_list.append(centroid)

  return object_positions_list


def main():
	global img_height, img_width
  # Read in image using the imread function
	img = cv2.imread('./phoneimg.jpg',cv2.IMREAD_GRAYSCALE)
	add_color_range_to_detect([0], [53]) # Detect red
	img_mask = get_mask(img)
	blobs = get_blobs(img_mask)
	object_positions_list = get_blob_centroids(blobs)
	centroid_group = get_close_centroid(object_positions_list)


	img_markup = img.copy()
	for obj_pos in centroid_group:
	    centroid_group = np.array(obj_pos).astype(np.int32) # In case your object positions weren't numpy arrays
	    img_markup = cv2.circle(img_markup,(centroid_group[1], centroid_group[0]),5,(0,0,0),10)
	    #print("Object pos: " + str(obj_pos_vector))
	

 	centroid_group = get_close_centroid(object_positions_list)
 	print(centroid_group)
	cv2.imshow('orig', img)
	cv2.imshow('img_mask', img_mask)
	cv2.imshow('located', img_markup)
	cv2.waitKey(-1)  # Wait until a key is pressed to exit the program
	cv2.destroyAllWindows() # Close all the windows

main()