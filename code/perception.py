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

def obstacle_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] <= rgb_thresh[0]) \
                & (img[:,:,1] <= rgb_thresh[1]) \
                & (img[:,:,2] <= rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select


def rock_thresh(img, threshold_low=(100, 100, 0), threshold_high=(160, 160, 40)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:, :, 0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = ~((img[:, :, 0] > threshold_low[0]) & (img[:, :, 0] < threshold_high[0]) \
                     & (img[:, :, 1] > threshold_low[1]) & (img[:, :, 1] < threshold_high[1]) \
                     & (img[:, :, 2] > threshold_low[2]) & (img[:, :, 2] < threshold_high[2]))
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    return color_select

# Define a function to convert to rover-centric coordinates
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = np.absolute(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[0]).astype(np.float)
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

# Define a function to map rover space pixels to world space
def pix_to_world(xpix, ypix, x_rover, y_rover, yaw_rover, world_size, scale):
    # Map pixels from rover space to world coords
    yaw = yaw_rover * np.pi / 180
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_((((xpix * np.cos(yaw)) - (ypix * np.sin(yaw)))/scale) + x_rover),
                            0, world_size - 1)
    y_pix_world = np.clip(np.int_((((xpix * np.sin(yaw)) + (ypix * np.cos(yaw)))/scale) + y_rover),
                            0, world_size - 1)

    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):

    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image

    return warped


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    dst_size = 5
    bottom_offset = 6
    src = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    dst = np.float32([[Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - bottom_offset],
                      [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - bottom_offset],
                      [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset],
                      [Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset],
                      ])


    # 2) Apply perspective transform
    perspective_img = perspect_transform(Rover.img, src, dst)

    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    threshold_terrain=color_thresh(perspective_img)
    threshold_obstacle=obstacle_thresh(perspective_img)
    threshold_rock=rock_thresh(perspective_img)

    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image
    Rover.vision_image[:, :, 0] =threshold_obstacle
    Rover.vision_image[:, :, 1] =threshold_rock
    Rover.vision_image[:, :, 2] =threshold_terrain
	
    # 5) Convert map image pixel values to rover-centric coords
    xpix0, ypix0 = rover_coords(Rover.vision_image[:,:,0])
    xpix1, ypix1 = rover_coords(Rover.vision_image[:,:,1])
    xpix2, ypix2 = rover_coords(Rover.vision_image[:,:,2])
	
    # 6) Convert rover-centric pixel values to world coordinates
    rover_obstacle = pix_to_world(xpix0, ypix0, Rover.pos[0], Rover.pos[1], Rover.yaw, 200, 10)
    rover_rock     = pix_to_world(xpix1, ypix1, Rover.pos[0], Rover.pos[1], Rover.yaw, 200, 10)
    rover_terrain  = pix_to_world(xpix2, ypix2, Rover.pos[0], Rover.pos[1], Rover.yaw, 200, 10)
	
    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 0] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 0] += 1
    Rover.worldmap[rover_obstacle[1], rover_obstacle[0], :]   += [255,0,0]  
    Rover.worldmap[rover_rock[1]    , rover_rock[0]    , :]   += [0,255,0]  
    Rover.worldmap[rover_terrain[1] , rover_terrain[0] , :]   += [0,0,255]  
		
    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles
    Rover.nav_dists, Rover.nav_angles = to_polar_coords(xpix2, ypix2)	
    Rover.terrain = threshold_terrain
    return Rover