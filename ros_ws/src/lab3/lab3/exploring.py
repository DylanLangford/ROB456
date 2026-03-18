#!/usr/bin/env python3

# This assignment lets you both define a strategy for picking the next point to explore and determine how you
#  want to chop up a full path into way points. You'll need path_planning.py as well (for calculating the paths)
#
# Note that there isn't a "right" answer for either of these. This is (mostly) a light-weight way to check
#  your code for obvious problems before trying it in ROS. It's set up to make it easy to download a map and
#  try some robot starting/ending points
#
# Given to you:
#   Image handling
#   plotting
#   Some structure for keeping/changing waypoints and converting to/from the map to the robot's coordinate space
#
# Slides

# The ever-present numpy
import numpy as np
import os

# from ros_ws.src.lab3.lab3.path_planning import eight_connected

# Your path planning code
try:
    import lab3.send_points as send_points
except:
    import send_points as send_points

try:
    import lab3.path_planning as path_planning
except:
    import path_planning as path_planning


# -------------- Showing start and end and path ---------------
def plot_with_explore_points(im_threshhold, zoom=1.0, robot_loc=None, explore_points=None, best_pt=None):
    """Show the map plus, optionally, the robot location and points marked as ones to explore/use as end-points
    @param im - the image of the SLAM map
    @param im_threshhold - the image of the SLAM map
    @param robot_loc - the location of the robot in pixel coordinates
    @param best_pt - The best explore point (tuple, i,j)
    @param explore_points - the proposed places to explore, as a list"""

    # Putting this in here to avoid messing up ROS
    import matplotlib.pyplot as plt

    fig, axs = plt.subplots(1, 1)
    axs.imshow(im_threshhold, origin='lower', cmap="gist_gray")
    axs.set_title("threshold image")

    # Show original and thresholded image
    if explore_points is not None:
        for p in explore_points:
            axs.plot(p[0], p[1], '.b', markersize=2)

    if robot_loc is not None:
        axs.plot(robot_loc[0], robot_loc[1], '+r', markersize=10)
    if best_pt is not None:
        axs.plot(best_pt[0], best_pt[1], '*y', markersize=10)
    axs.axis('equal')

    # Implements a zoom - set zoom to 1.0 if no zoom
    width = im_threshhold.shape[1]
    height = im_threshhold.shape[0]

    axs.set_xlim(width / 2 - zoom * width / 2, width / 2 + zoom * width / 2)
    axs.set_ylim(height / 2 - zoom * height / 2, height / 2 + zoom * height / 2)


# -------------- For converting to the map and back ---------------
def convert_pix_to_x_y(im_size, pix, size_pix):
    """Convert a pixel location [0..W-1, 0..H-1] to a map location (see slides)
    Note: Checks if pix is valid (in map)
    @param im_size - width, height of image
    @param pix - tuple with i, j in [0..W-1, 0..H-1]
    @param size_pix - size of pixel in meters
    @return x,y """
    if not (0 <= pix[0] <= im_size[1]) or not (0 <= pix[1] <= im_size[0]):
        raise ValueError(f"Pixel {pix} not in image, image size {im_size}")

    return [size_pix * pix[i] / im_size[1-i] for i in range(0, 2)]


def convert_x_y_to_pix(im_size, x_y, size_pix):
    """Convert a map location to a pixel location [0..W-1, 0..H-1] in the image/map
    Note: Checks if x_y is valid (in map)
    @param im_size - width, height of image
    @param x_y - tuple with x,y in meters
    @param size_pix - size of pixel in meters
    @return i, j (integers) """
    pix = [int(x_y[i] * im_size[1-i] / size_pix) for i in range(0, 2)]

    if not (0 <= pix[0] <= im_size[1]) or not (0 <= pix[1] <= im_size[0]):
        raise ValueError(f"Loc {x_y} not in image, image size {im_size}")
    return pix


def is_reachable(im, pix):
    """ Is the pixel reachable, i.e., has a neighbor that is free?
    Used for
    @param im - the image
    @param pix - the pixel i,j"""

    # GUIDE: Returns True (the pixel is adjacent to a pixel that is free)
    #  False otherwise
    # You can use four or eight connected - eight will return more points
    # YOUR CODE HERE
    
    adj = path_planning.eight_connected(pix)
    for i in adj:
        if path_planning.is_free(im, i):
            return True
    return False


def find_all_possible_goals(im):
    """ Find all of the places where you have a pixel that is unseen next to a pixel that is free
    It is probably easier to do this, THEN cull it down to some reasonable places to try
    This is because of noise in the map - there may be some isolated pixels
    @param im - thresholded image
    @return list of possible pixel (x,y) locations"""

    #YOUR CODE HERE

    # smaller_im = im[1:-1,1:-1]
    # rows, cols = np.where(smaller_im == 128)
    # goals = list(zip(cols, rows))
    # free = []
    # for i in goals:
    #     for j in path_planning.four_connected(i):
    #         if path_planning.is_free(im, j):
    #             free.append(i)
    # return free

    possible_goals = []
    for i in range(1, im.shape[1]-1):
        for j in range(1, im.shape[0]-1):
            if path_planning.is_unseen(im, (i,j)) and is_reachable(im, (i,j)):
                possible_goals.append((i,j))
    return possible_goals



def find_best_point(im, possible_points : list, robot_loc, last_goal = None):
    """ Pick one of the unseen points to go to
    @param im - thresholded image
    @param possible_points - possible points to chose from (list of tuples)
    @param robot_loc - location of the robot (in case you want to factor that in)
    """
    # YOUR CODE HERE

    # if not possible_points:
    #     return None

    # x = np.sqrt((robot_loc[0]-possible_points[0][0])**2+(robot_loc[1]-possible_points[0][1])**2)
    # min_dist = possible_points[0]
    # for i in possible_points:
    #     distance = np.sqrt((robot_loc[0]-i[0])**2+(robot_loc[1]-i[1])**2)
    #     if distance < x:
    #         x = distance
    #         min_dist = i
    
    # if not path_planning.is_free(im, min_dist):
    #     for neighbor in path_planning.eight_connected(min_dist):
    #         if path_planning.is_free(im, neighbor):
    #             min_dist = neighbor
    # return min_dist

    if not possible_points:
        return None

    # --- Configuration ---
    # resolution is typically 0.05m/pixel. 
    # 1.5 meters / 0.05 = 30 pixels. Adjust this 'deadzone' as needed.
    pixel_deadzone = 40

    # Ensure everything is integer pixel coordinates
    possible_points = [(int(p[0]), int(p[1])) for p in possible_points]
    robot_loc = (int(robot_loc[0]), int(robot_loc[1]))

    min_dist = float('inf')
    best_pt = None

    for pt in possible_points:
        dx = robot_loc[0] - pt[0]
        dy = robot_loc[1] - pt[1]
        dist = np.sqrt(dx*dx + dy*dy)

        # --- THE FIX: Ignore points that are too close (noise/backtracking) ---
        if dist < pixel_deadzone:
            continue
        
        # Ignore points that are too close to point we are currently going to
        if last_goal is not None:
            dist_to_last = np.sqrt((pt[0] - last_goal[0])**2 + (pt[1] - last_goal[1])**2)
            if dist_to_last < (pixel_deadzone / 2):
                continue


        if dist < min_dist:
            # Optional: Check if the point is actually a "good" frontier 
            # (e.g., call your test_best(im, pt) here if you have one)
            min_dist = dist
            best_pt = pt

    # If everything was in the deadzone, just grab the absolute closest 
    # so the robot doesn't stand still.
    if best_pt is None:
        return None

    # --- 2. Ensure the point is actually reachable (is_free) ---
    if not path_planning.is_free(im, best_pt):
        for neighbor in path_planning.eight_connected(best_pt):
            neighbor = (int(neighbor[0]), int(neighbor[1]))
            if (0 <= neighbor[1] < im.shape[0] and 
                0 <= neighbor[0] < im.shape[1]):
                if path_planning.is_free(im, neighbor):
                    best_pt = neighbor
                    break

    return best_pt



 
def find_waypoints(im, path):
    """ Place waypoints along the path
    @param im - the thresholded image
    @param path - the initial path
    @ return - a new path"""

    # Again, no right answer here
    # YOUR CODE HERE
    segmented_path = path[::5]
    return segmented_path


def test_unseen(im, pts):
    for pt in pts:
        count_free = 0
        count_unseen = 0
        for ix in range(-1, 2):
            for iy in range(-1, 2):
                if path_planning.is_free(im, (pt[0] + ix, pt[1] + iy)):
                    count_free += 1
                elif path_planning.is_unseen(im, (pt[0] + ix, pt[1] + iy)):
                    count_unseen += 1
        if count_free == 0 or count_unseen == 0:
            return False
    return True


def test_best(im, pt):
    """ Check that the selected point has at least 3 free neighbors"""
    count_free = 0
    count_unseen = 0
    for ix in range(-1, 2):
        for iy in range(-1, 2):
            if path_planning.is_free(im, (pt[0] + ix, pt[1] + iy)):
                count_free += 1
            elif path_planning.is_unseen(im, (pt[0] + ix, pt[1] + iy)):
                count_unseen += 1
    if count_free < 3:
        return False
    if count_free + count_unseen != 9:
        return False
    return True


if __name__ == '__main__':
    _, im_thresh = path_planning.open_image("map.pgm")

    robot_start_loc = (60, 40)

    all_unseen = find_all_possible_goals(im_thresh)
    best_unseen = find_best_point(im_thresh, all_unseen, robot_loc=robot_start_loc)

    assert test_unseen(im=im_thresh, pts=all_unseen)
    assert test_best(im=im_thresh, pt=best_unseen)

    plot_with_explore_points(im_thresh, zoom=1.0, robot_loc=robot_start_loc, explore_points=all_unseen, best_pt=best_unseen)

    path = path_planning.dijkstra(im_thresh, robot_start_loc, best_unseen)
    waypoints = find_waypoints(im_thresh, path)
    path_planning.plot_with_path(im_thresh, zoom=1.0, robot_loc=robot_start_loc, goal_loc=best_unseen, path=waypoints)

    # Depending on if your mac, windows, linux, and if interactive is true, you may need to call this to get the plt
    # windows to show
    # Putting this in here to avoid messing up ROS
    import matplotlib.pyplot as plt
    plt.show()

    print("Done")
