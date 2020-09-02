# Point-Cloud-Voxelization

### Function Introduction
<br>This is a program that can process point cloud data, realizes points voxelization inside an appointed bounding box or AABB(Axially Aligned Bounding Box), and realizes rendering visualization.

<br>The main program can be divided into two parts, one is processing point data including:
<br>Point data storage from inputing txt file;
<br>Bounding box construction based on the inputing size of voxel grid; 
<br>Point cloud voxelization;
<br>Three-dimentional array generation;
<br>New txt file generation via traversing the three-dimentinal array.

<br>The remaining part focuses on rendering visualization and provides two kinds of bounding boxes along with three different ways of rendering modes:
<br>Two kinds of bounding boxes:
<br>AABB(Axially Aligned Bounding Box);
<br>CUSTOM(fixed 128 * 192 * 128 resolution box).
<br>Three kinds of rendering modes:
<br>Only point cloud rendering mode;
<br>Solid cube rendering mode;
<br>Wireframe and point cloud mixed rendering mode;

<br>You can choose the function of processing the point data instead of waiting for watching the effect of rendering visualization.
<br>However, you can also choose both of them and see what happened when you input different point data or parameters.

### Input and Output
<br>The input of the program is a standard point that is saved in a text file, and this file only contains vertice information.
<br>I put three demo txt files in the build file, you can use these three files to test this program.
<br>
<br>The outputs of the program include a PCD file that saving all of the vertices information using a PCD file format.
<br>In addition, the program will output a three-dimensional array in a text file that corresponds to the occupied voxel grid in assigned bounding space. 

### Installation
<br>The program utilizes a part of PCL libraries content, thus insure you have installed or updated the PCL and Cmake libraries.
<br>Under the Linux operating system:
<br>Launch terminal and install libaries:
<br>sudo apt-get update
<br>sudo apt-get install cmake
<br>sudo apt-get install libpcl-dev pcl-tools libproj-dev

<br>Download my project, and cd the build file in the terminal:
<br>cmake ..
<br>make
<br>If you obtain a file named vexel_grid under the build file, it means you are successful.

### Usage
<br>You must input exact three or five parameters in order to process the program correctly.
<br>Only want processing point data:
<br>Input in the terminal(under the build file): ./voxel_grid YOUR_FILE_NAME VOXEL_SIZE
<br>For example: ./voxel_grid hair1.txt 0.015
<br>Want processing point data and see what happened in the rendering pannel:
<br>Input in the terminal(under the build file): ./voxel_grid YOUR_FILE_NAME VOXEL_SIZE BOX_MODE RENDERING MODE
<br>For example: ./voxel_grid hair1.txt 0.015 AABB/CUSTOM A/B/C
<br>The meaning of the last two parameters just look back please.

By the way, the size of the voxel grid is chosen between 0.011 to 0.016 can generate a better effect for the hairstyle points.
In addition, the standard hair point data can be processed using the program as follows:
https://github.com/ericmaic/Standard-Point




