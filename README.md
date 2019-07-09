# Pointcloud matching project
Detect the transformations between 2 point-clouds that are scanned from the same place at the different times.

## Prerequisites:

### 1. Setup prerequisites
Install all the prerequisites:
```
sudo apt-get install libtiff5-dev
sudo apt-get install libudev-dev (for openNI insall libusb-dev libusb-1.0-0-dev)
sudo apt -y install doxygen mpi-default-dev openmpi-bin openmpi-common libqhull* libgtest-dev libflann-dev libboost-all-dev libeigen3-dev
sudo apt -y install git-core freeglut3-dev pkg-config build-essential libxmu-dev libxi-dev libphonon-dev libphonon-dev phonon-backend-gstreamer
sudo apt -y install phonon-backend-vlc graphviz mono-complete
sudo apt-get install openjdk-8-jdk openjdk-8-jre
```

### 2. geotiff
Install GeoTIFF (git reset --hard to version 1.4.0) from https://github.com/ufz/geotiff
```
git clone https://github.com/ufz/geotiff.git
cd geotiff
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_BUILD_TYPE=Release ..
make - j8
sudo make install
```

### 3. vtk 8.1.2
Get vtk version 8.1.2 from https://www.vtk.org/download/ or clone from https://github.com/Kitware/VTK.git
```
git clone https://github.com/Kitware/VTK.git
cd VTK
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_BUILD_TYPE=Release ..
make -j8
sudo make install
```

### 4. libLAS 1.8.1
Install libLAS (git reset --hard to version 1.8.1) from https://github.com/libLAS/LibLAS
```
git clone https://github.com/libLAS/libLAS.git
cd LibLAS
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_BUILD_TYPE=Release ..
make - j8
sudo make install
```

### 5. pcl 1.9.1
Install pointcloud library (git reset --hard to version 1.9.1) from https://github.com/PointCloudLibrary/pcl
```
git clone https://github.com/PointCloudLibrary/pcl.git
cd pcl
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_BUILD_TYPE=Release ..
make - j8
sudo make install
```

## Build project
Build project with cmake:
```
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
```

## Run project

### 1. pre_process
pre_process will read pointcloud from las file, filter noise, down sample save remaining points to ply file.

Run pre_process:
```
./pre_process -i <input_las_file> -n <noise_filltering_method> -d <down_sampling_method> -inter <bool> -in <noise_filtered_file> -ids <down_sampled_file> -ofs <offset_file>
```

in which:
```
-n <option> : noise filtering method name
+ stat
+ radius
+ colorbased
+ statcolor
-d <option> : down sampling method name
+ median
+ nearestmed
-inter <option> : save intermediate files
+ Y/y: yes
+ N/n: no
-i: .las file
-in: .ply file
-ids: .ply file
-ofs: .txt file
```

Examples:
```
./pre_process -i data_1_before.las -n stat -d median -inter Yes -in data_1_before_stat.ply -ids data_1_before_stat_median.ply -ofs data_1_before_offset.txt
```

```
./pre_process -i data_1_before.las -n statcolor -d nearestmed -inter Yes -in data_1_before_statcolor.ply -ids data_1_before_statcolor_nearestmed.ply -ofs data_1_before_offset.txt
```

```
./pre_process -i data_1_after.las -n statcolor -d nearestmed -inter Yes -in data_1_after_statcolor.ply -ids data_1_after_statcolor_nearestmed.ply -ofs data_1_after_offset.txt
```

### 2. matching
It will determine all transformations from the old pointcloud to the new one using matching algorithms.

Run matching:
```
./matching -kp <keypoint_method> -des <descriptor_method> -i1 <ply_pointcloud_input_1> -i2 <ply_pointcloud_input_2> -ikp1 <ply_keypoint_1_file> -ikp2 <ply_keypoint_2_file> -o <ouput_file> -inter <bool> -mp <matchingpairs_point2point_file> -ofs1 <offset_file_1> -ofs2 <offset_file_2>
```

in which:
```
-kp <option> : keypoints detection method.
+ iss
+ susan
+ harris3D
-des<descriptor>: descriptor extraction method.
+ 2dmethod
+ icp
+ shot
+ fpfh
+ shotcolor
-inter <option>: save intermediate files
+ Y/y: yes
+ N/n: no
-i1: .ply file
-i2: .ply file
-o: .ply file
-ikp1: .ply file
-ikp2: .ply file
-mp: .txt file
-ofs1: .txt file
-ofs2: .txt file
```

Example:
```
./matching -i1 data_1_before_statcolor_nearestmed.ply -i2 data_1_after_statcolor_nearestmed.ply -kp harris3D -des shot -inter Yes -ikp1 data_1_before_statcolor_nearestmed_harris3D.ply -ikp2 data_1_after_statcolor_nearestmed_harris3D.ply -o data_1_statcolor_nearestmed_harris3D_shot.ply -mp data_1_statcolor_nearestmed_harris3D_shot.txt -ofs1 data_1_before_offset.txt -ofs2 data_1_after_offset.txt
```

If you want to run 2d matching method for, there is no need of keypoint detection, so you can type any of the keypoint_method in the command line:
```
./matching -i1 data_1_before_statcolor_nearestmed.ply -i2 data_1_after_statcolor_nearestmed.ply -kp susan -des 2dmethod -inter No -o data_1_statcolor_nearestmed_0_2dmethod.ply -mp data_1_statcolor_nearestmed_0_2dmethod.txt -ofs1 data_1_before_offset.txt -ofs2 data_1_after_offset.txt
```

### 3. refer_plane
refer_plane will determine all transformations from the old pointcloud to the new one using refer plane algorithm.

Run refer_plane:
```
./refer_plane <before_pointcloud> <after_pointcloud> <offset_file_1> <offset_file_2> <matching_result_file> <matchingpairs_point2point_file>
```

in which:
```
before_pointcloud: ply file
after_pointcloud: ply file
offset_file_1: .txt file
offset_file_2: .txt file
matching_result_file: .ply file
matchingpairs_point2point_file: .txt file
```

Example:
```
./refer_plane data_1_before_statcolor_nearestmed.ply data_1_after_statcolor_nearestmed.ply data_1_before_offset.txt data_1_after_offset.txt data_1_statcolor_nearestmed_referPlane.ply data_1_statcolor_nearestmed_referPlane.txt
```

Note: argument vectors need to be written in the right order as above.
