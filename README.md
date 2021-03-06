# Pointcloud matching project
Detect the transformations between 2 point-clouds that are scanned from the same place at the different times.

## Prerequisite
Install libLAS (git reset --hard to version 1.8.1) from https://github.com/libLAS/LibLAS
```
sudo apt install -y libgeotiff-dev
cd ~/Downloads
git clone git@github.com:libLAS/libLAS.git
cd libLAS
git checkout 1.8.1
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_BUILD_TYPE=Release ..
make - j8
sudo make install
cd ../..
rm -rf libLAS
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
