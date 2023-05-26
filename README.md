# Flight-Cost Rapidly-Exploring Random Tree Star (FC-RRT*): Path Planning Algorithm for UAV in Complex Environment

## Overview
The Flight-Cost Rapidly-Exploring Random Tree Star (FC-RRT*) algorithm is an extension of the well-known RRT* algorithm for efficient path planning in complex environments. The primary enhancement of the FC-RRT* algorithm is the incorporation of an adaptive sampling strategy based on flight cost, which allows the algorithm to efficiently explore the configuration space and quickly converge to an optimal solution.

## Dependencies
To run the code, you need to have the following libraries installed:

- `Pygame` is a Python library for creating video games and multimedia applications.
- `NumPy` is a Python library for numerical computing.
- `math`: A Python standard library for mathematical operations.
- `time`: A standard Python library for manipulating time.
- `trimesh`: A Python library for loading, processing, and visualizing 3D models, such as meshes, point clouds, and voxel grids.
- `Open3D`: A Python library for 3D data processing and visualization, including functions for working with point clouds, meshes, and depth images. It also provides advanced functionality for registration, reconstruction, and segmentation of 3D data.


## Installation 

- To install the dependencies, you can use pip, the Python package installer. Run the following command:
    ```
    pip install pygame numpy trimesh open3d matplotlib
    ```
-  the other standard libraries come pre-installed with Python, so you don't need to install them separately.

## Usage

- Run the script from the command line with optional arguments for RPM values:
    ```
    python main.py
    ```
- A Pygame window will appear, displaying the environment with obstacles. Click on the screen to select a starting point and an ending point for the UAV (avoid the black regions) and close the window (wait for some time for algorithm to process).
- The FC-RRT* algorithm will calculate the optimal path between the selected points, avoiding the obstacles. The path will be displayed on the open3D screen and printed in the terminal.
- To exit the program, close the open3D window.