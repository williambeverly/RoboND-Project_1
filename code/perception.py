import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

def rock_thresh(img, lower_thresh=(20,100,100), upper_thresh=(40,255,255)):
    # Convert BGR to HSV
    rock_select = np.zeros_like(img[:,:,0])
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

    # Threshold the HSV image to get only yellow and then make binary
    mask = cv2.inRange(hsv, lower_thresh, upper_thresh) > 0    
    rock_select[mask] = 1
    
    return rock_select

# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

def to_polar_coords_bias(Rover, x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    x_pixel *= Rover.explored
    y_pixel *= Rover.explored
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles    

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))                            
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result  
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(Rover, xpix, ypix, xpos, ypos, yaw, world_size, scale, nav_data=False):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)

    # ADDED update the Rover.x_stored & y_stored without casting to integer
    if(nav_data):
    	Rover.x_stored = np.clip(xpix_tran, 0, world_size - 1)
    	Rover.y_stored = np.clip(ypix_tran, 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# ADDED reverse transform and rotation that also includes the explored terrain
def world_to_pix(Rover, xpos, ypos, yaw, world_size, scale):
	# create new lists for unexplored x and y variables
	x_unexplored = []
	y_unexplored = []
	# then for each point in the transformed x and y, check whether the closest
	# pixel in the explored map has already been explored
	for i in range(len(Rover.x_stored)):
		if(Rover.explored[np.int_(Rover.y_stored[i]), np.int_(Rover.x_stored[i])] > 0):
			x_unexplored.append(Rover.x_stored[i])
			y_unexplored.append(Rover.y_stored[i])

	# Convert to numpy arrays
	x_unexplored = np.array(x_unexplored)
	y_unexplored = np.array(y_unexplored)
	# Mirror the transform
	x_pix_rot = (x_unexplored - xpos) * scale
	y_pix_rot = (y_unexplored - ypos) * scale
	# Perform inverse rotation
	yaw_rad = yaw * np.pi / 180
	x = (x_pix_rot * np.cos(-yaw_rad)) - (y_pix_rot * np.sin(-yaw_rad))
	y = (x_pix_rot * np.sin(-yaw_rad)) + (y_pix_rot * np.cos(-yaw_rad))

	return x, y

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    # ADDED - add a valid area mask, so that the obstacle detection does not result in out of range recording
    mask = cv2.warpPerspective(np.ones_like(img[:,:,0]), M, (img.shape[1], img.shape[0]))
    
    return warped, mask


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    dst_size = 5 
    # Set a bottom offset to account for the fact that the bottom of the image 
    # is not the position of the rover but a bit in front of it
    # this is just a rough guess, feel free to change it!
    bottom_offset = 6
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - bottom_offset],
                  [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - bottom_offset],
                  [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset], 
                  [Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset],
                  ])
    # 2) Apply perspective transform
    warped, mask = perspect_transform(Rover.img, source, destination)
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    threshed = color_thresh(warped)
    # Update the obstacles with the mask
    obstacles = np.absolute(np.float32(threshed) - 1) *mask
    rocks = rock_thresh(warped)
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image
    Rover.vision_image[:,:,0] = obstacles * 255
    Rover.vision_image[:,:,1] = rocks * 255
    Rover.vision_image[:,:,2] = threshed * 255
    # 5) Convert map image pixel values to rover-centric coords
    xpix, ypix = rover_coords(threshed)
    xpix_obs, ypix_obs = rover_coords(obstacles)
    xpix_rocks, ypix_rocks = rover_coords(rocks)
    # 6) Convert rover-centric pixel values to world coordinates
    world_size = Rover.worldmap.shape[0]
    scale = 10
    xpos = Rover.pos[0]
    ypos = Rover.pos[1]
    yaw = Rover.yaw
    nav_x_world, nav_y_world = pix_to_world(Rover, xpix, ypix, xpos, ypos, yaw, world_size, scale, nav_data=True)
    obs_x_world, obs_y_world = pix_to_world(Rover, xpix_obs, ypix_obs, xpos, ypos, yaw, world_size, scale)
    rocks_x_world, rocks_y_world = pix_to_world(Rover, xpix_rocks, ypix_rocks, xpos, ypos, yaw, world_size, scale)
    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1

    # only add values if the roll and pitch are near zero
    if(not((Rover.pitch > Rover.pitch_tol) and (Rover.pitch < 360 - Rover.pitch_tol)) and not((Rover.roll > Rover.roll_tol) and (Rover.roll < 360 - Rover.roll_tol))):
    	Rover.worldmap[obs_y_world, obs_x_world, 0] += 1
    	Rover.worldmap[rocks_y_world, rocks_x_world, 1] += 1
    	Rover.worldmap[nav_y_world, nav_x_world, 2] += 10

    # 8) Convert rover-centric pixel positions to polar coordinates

    # ADDED 8.) Convert to rover-centric pixel positions considering already explored area
    x_bias, y_bias = world_to_pix(Rover, xpos, ypos, yaw, world_size, scale)
    x_new = np.concatenate((x_bias, xpix), axis=0)
    y_new = np.concatenate((y_bias, ypix), axis=0)
    dist, angles = to_polar_coords(x_new, y_new)
    #dist, angles = to_polar_coords(xpix, ypix)
    #dist, angles = to_polar_coords_bias(Rover, xpix, ypix)
    # Update Rover pixel distances and angles
    # Rover.nav_dists = rover_centric_pixel_distances
    Rover.nav_dists = dist
    # Rover.nav_angles = rover_centric_angles
    Rover.nav_angles = angles
    
 
    
    
    return Rover