#!/usr/bin/env python3
import numpy as np
from PIL import Image

def create_test_bitmap():

    # ----------------------------
    # 1. Create a fake 2D map
    # ----------------------------
    width = 100
    height = 100

    # Initialize a free map (0 = free, 100 = obstacle, -1 = unknown)
    grid = np.zeros((height, width), dtype=np.int8)

    # Add a fake obstacle wall (horizontal)
    grid[40:45, 10:90] = 100

    # Add some unknown area
    grid[60:80, 0:30] = -1

    # ----------------------------
    # 2. Create a fake goal point
    # ----------------------------
    goal_x = 40
    goal_y = 30
    # (in "pixel" coordinates directly for this test)

    # ----------------------------
    # 3. Convert to RGB image
    # ----------------------------
    img = np.zeros((height, width, 3), dtype=np.uint8)

    # Assign colors based on occupancy
    img[grid == 0] = [255, 255, 255]      # free → white
    img[grid == 100] = [0, 0, 0]          # obstacle → black
    img[grid == -1] = [150, 150, 150]     # unknown → grey

    # ----------------------------
    # 4. Draw goal point in RED
    # ----------------------------
    if 0 <= goal_x < width and 0 <= goal_y < height:
        img[height - goal_y - 1, goal_x] = [255, 0, 0]  # red pixel

    # ----------------------------
    # 5. Save bitmap file
    # ----------------------------
    im = Image.fromarray(img)
    im.save("test_map.bmp")
    print("Saved test_map.bmp")


if __name__ == "__main__":
    create_test_bitmap()
