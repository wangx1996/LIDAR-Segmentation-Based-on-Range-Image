# LIDAR-Segmentation-Based-on-Range-Image

[![build passing](https://img.shields.io/badge/build-passing-brightgreen.svg)](https://github.com/wangx1996/LIDAR-Segmentation-Based-on-Range-Image)[![velodyne_HDL_32E compliant](https://img.shields.io/badge/velodyne_HDL_32E-compliant-red.svg)](https://github.com/wangx1996/LIDAR-Segmentation-Based-on-Range-Image)

## This is a lidar segmentation method based on range-image.

## Method

1. The ground remove method is from ["D. Zermas, I. Izzat and N. Papanikolopoulos, "Fast segmentation of 3D point clouds: A paradigm on LiDAR data for autonomous vehicle applications," 2017 IEEE International Conference on Robotics and Automation (ICRA), Singapore, 2017, pp. 5067-5073, doi: 10.1109/ICRA.2017.7989591."](https://ieeexplore.ieee.org/document/7989591)

2. The scan line compensation method is from ["P. Burger and H. Wuensche, "Fast Multi-Pass 3D Point Segmentation Based on a Structured Mesh Graph for Ground Vehicles," 2018 IEEE Intelligent Vehicles Symposium (IV), Changshu, 2018, pp. 2150-2156, doi: 10.1109/IVS.2018.8500552."](https://ieeexplore.ieee.org/document/8500552)

3. The range image segmentation method is from ["I. Bogoslavskyi and C. Stachniss, "Fast range image-based segmentation of sparse 3D laser scans for online operation," 2016 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Daejeon, 2016, pp. 163-169, doi: 10.1109/IROS.2016.7759050."](https://ieeexplore.ieee.org/document/7759050)

4. The hash table method is inspired by ["S. Park, S. Wang, H. Lim and U. Kang, "Curved-Voxel Clustering for Accurate Segmentation of 3D LiDAR Point Clouds with Real-Time Performance," 2019 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Macau, China, 2019, pp. 6459-6464, doi: 10.1109/IROS40897.2019.8968026."](https://ieeexplore.ieee.org/document/8968026)

5. The process of segmentation is inspired by ["K. Klasing, D. Wollherr and M. Buss, "A clustering method for efficient segmentation of 3D laser data," 2008 IEEE International Conference on Robotics and Automation, Pasadena, CA, 2008, pp. 4043-4048, doi: 10.1109/ROBOT.2008.4543832.](https://ieeexplore.ieee.org/document/4543832)

6. Thec threshhold method is from ["Borges, G.A., Aldon, MJ. Line Extraction in 2D Range Images for Mobile Robotics. Journal of Intelligent and Robotic Systems 40, 267–297 (2004). https://doi.org/10.1023/B:JINT.0000038945.55712.65"](https://link.springer.com/article/10.1023/B:JINT.0000038945.55712.65#citeas)

##### more detail： 

https://blog.csdn.net/weixin_43885544/article/details/111193386


## Code

1.The ground remove code references to the https://github.com/AbangLZU/plane_fit_ground_filter. And I change it to multiplane fitting.

2.The process of segmentation references to the https://github.com/FloatingObjectSegmentation/CppRBNN

## Usage
    
    mkdir build
    cd build
    cmake ..
    make
    ./range forange.pcd
    
## Result

### Line compenstation
![Image text](https://github.com/wangx1996/LIDAR-Segmentation-Based-on-Range-Imag/blob/main/image/linecompensation.png)

### Build range image
![Image text](https://github.com/wangx1996/LIDAR-Segmentation-Based-on-Range-Imag/blob/main/image/buildrange.png)

### segmentation
![Image text](https://github.com/wangx1996/LIDAR-Segmentation-Based-on-Range-Imag/blob/main/image/seg.png)
