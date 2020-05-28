"""
Package for drawing the generated map of the world for Pset 4 of CS 189.

2019-03: Created by Mark Petersen
"""

from math import *
import numpy as np
import cv2

class MapDrawer:
    """
    A class for incrementally updating the displayed image of the generated map,
    and saving the image to file.  The map must be an occupancy grid of shape
    (40, 30).
    """

    def __init__(self, start_pos):
        """
        Creates a new MapDrawer object that can be used for visualizing and
        saving your map. Requires the initial position on the map of the robot,
        `start_pos`, for later
        """
        self.map_size = (40, 30)
        self.draw_scale = 16
        self.bot_scale = 14
        self.map = -np.ones(self.map_size).astype(int)
        self.drawn_map = np.zeros((640, 480, 3))
        self.map_colors = [[0, 0, 0], [1, 1, 1], [1, 0, 0],\
                           [1, 0, 1], [1, 1, 0], [0, 1, 1]]
        self.start_pos = np.array(start_pos) * self.draw_scale
        self.start_pos = (int(self.start_pos[1]), int(self.start_pos[0]))

    def UpdateMapDisplay(self, new_map, position, extra_img=None):
        """
        Updates the internally stored map image using `new_map` and displays
        the updated map along with the initial position of the robot, passed in
        the constructor, and the current position of the robot (`position`).
        If `extra_img` is supplied, the image will be displayed alongside the
        map.
        `new_map` must be the same size as the original map (40, 30).
        `extra_img` must be None or be a 3 channel color image.
        """
        assert new_map.shape == self.map_size, "New map size doesn't match old map size"
        for ii in np.ndindex(self.map_size):
            if self.map[ii] == int(new_map[ii]):
                continue
            self.map[ii] = int(new_map[ii])
            pt1 = (int(self.draw_scale*ii[1]), int(self.draw_scale*ii[0]))
            pt2 = (int(self.draw_scale*(ii[1]+ 1)), int(self.draw_scale*(ii[0] + 1)))
            color = self.map_colors[self.map[ii] + 1]
            cv2.rectangle(self.drawn_map, pt1, pt2, color, -1)

        img = np.copy(self.drawn_map)

        current_pos = np.array(position) * self.draw_scale
        current_pos = (int(current_pos[1]), int(current_pos[0]))
        cv2.circle(img, self.start_pos, self.bot_scale, [0, 1, 0], -1)
        cv2.circle(img, current_pos, self.bot_scale, [0, 0, 1], -1)

        if extra_img is not None:
            assert len(extra_img.shape) == 3 and extra_img.shape[2] == 3, "Extra image must be a 3 channel color image"
            h, w, _ = extra_img.shape
            h_start = (self.drawn_map.shape[0] - h) / 2
            padded_img = np.zeros((self.drawn_map.shape[0], w, 3))
            padded_img[h_start:h_start+h] = extra_img
            img = np.hstack((img, padded_img))
            
        #rotated = np.rot90(img, 1) # rotate image CCW
        cv2.imshow('Map', img)
        cv2.waitKey(5)

    def SaveMap(self, filename, position):
        """
        Saves the stored map to file `filename`, with the initial position of
        the robot, passed in the constructor, and the current position of the
        robot, `position`, included on the map.
        """
        img = np.copy(self.drawn_map)

        current_pos = np.array(position) * self.draw_scale
        current_pos = (int(current_pos[1]), int(current_pos[0]))
        cv2.circle(img, self.start_pos, self.bot_scale, [0, 1, 0], -1)
        cv2.circle(img, current_pos, self.bot_scale, [0, 0, 1], -1)

        cv2.imwrite(filename, img * 255)


if __name__ == '__main__':
    import time

    start_pos = (35, 15)
    mapper = MapDrawer(start_pos)
    my_map = -np.ones((40, 30))

    # Sending first image twice as the first image drawn does not fully render
    mapper.UpdateMapDisplay(my_map, start_pos)
    mapper.UpdateMapDisplay(my_map, start_pos)
    time.sleep(3)

    my_map[start_pos[0], start_pos[1]] = 0
    my_map[start_pos[0]-1, start_pos[1]] = 0
    my_map[start_pos[0]-1, start_pos[1]-1] = 0
    my_map[start_pos[0], start_pos[1]-1] = 0

    mapper.UpdateMapDisplay(my_map, start_pos)
    time.sleep(3)

    end_pos = (30, 13)
    my_map[end_pos[0], end_pos[1]] = 0
    my_map[end_pos[0]-1, end_pos[1]] = 0
    my_map[end_pos[0]-1, end_pos[1]-1] = 0
    my_map[end_pos[0], end_pos[1]-1] = 0

    my_map[29:35, 14:16] = 0
    my_map[5:10, 22:27] = 1

    mapper.UpdateMapDisplay(my_map, end_pos)
    time.sleep(3)

    mapper.SaveMap("test_map.png", end_pos)

    try:
        mapper.UpdateMapDisplay(my_map[:20], end_pos)
    except Exception, err:
        print err

    mapper.UpdateMapDisplay(my_map, end_pos, extra_img=np.ones((640, 480, 3)))
    mapper.UpdateMapDisplay(my_map, end_pos, extra_img=np.ones((480, 640, 3)))
    try:
        mapper.UpdateMapDisplay(my_map, end_pos, extra_img=np.zeros((480, 640)))
    except Exception, err:
        print err
