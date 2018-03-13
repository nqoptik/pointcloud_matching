Environments:
	Ubuntu 16.04 LTS 64-bit

Requirements:
	CMake
	pre-requirements:
		sudo apt-get install libtiff5-dev
		sudo apt-get install libudev-dev (for openNI insall libusb-dev libusb-1.0-0-dev)
		sudo apt -y install doxygen mpi-default-dev openmpi-bin openmpi-common libqhull* libgtest-dev libflann-dev libboost-all-dev libeigen3-dev
		sudo apt -y install git-core freeglut3-dev pkg-config build-essential libxmu-dev libxi-dev libphonon-dev libphonon-dev phonon-backend-gstreamer
		sudo apt -y install phonon-backend-vlc graphviz mono-complete qt-sdk
		sudo apt-get install qt-sdk openjdk-8-jdk openjdk-8-jre
	geotiff
		Install GeoTIFF (git reset --hard to version 1.4.0) from https://github.com/ufz/geotiff with
			cmake -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_BUILD_TYPE=Release ..
	VTK
		Get vtk version 8.1.2 from https://www.vtk.org/download/ with
			cmake -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_BUILD_TYPE=Release .. && make -j8
	LibLAS
		Install libLAS (git reset --hard to version 1.8.1) from https://github.com/libLAS/LibLAS with
			cmake -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_BUILD_TYPE=Release ..
	pcl
		Install pointcloud library (git reset --hard to version 1.9.1) from https://github.com/PointCloudLibrary/pcl with
			cmake -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_BUILD_TYPE=Release ..
	openCV 2.4.x
		Install openCV 2.4.13 form https://www.opencv.org

Build:
	Open terminal in sourse folder and type:
		mkdir build
		cd build
		cmake ..
		make
	Copy "config.ini" to build/
	There are 8 config files corresponding to 8 datasets;
	If you want to run with dataset 1, you need to copy the content of the "config_1.ini" file and paste it to "config.ini" file.

Executors:
	PreProcess: Read pointcloud from las file, filter noise, down sample save remaining points to ply file.
	Matching: Determine all transformations from the old pointcloud to the new one using matching algorithms.
	ReferPlane: Determine all transformations from the old pointcloud to the new one using refer plane algorithm.
Run:
	1.	./PreProcess -i <input_las_file> -n <noise_filltering_method> -d <down_sampling_method> -inter <bool> -in <noise_filtered_file> -ids <down_sampled_file> -ofs <offset_file>

	Options:
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
	eg 1: ./PreProcess -i data_1_before.las -n stat -d median -inter Yes -in data_1_before_stat.ply -ids data_1_before_stat_median.ply -ofs data_1_before_offset.txt
	What it does is to read data_1_before.las -> run stat(statistical outlier removal)
	-> save to file data_1_before_stat_noise.ply (noise group) and data_1_before_stat.ply (remaining group);
	and then run median (a downsampling method) -> save to file data_1_before_stat_median.ply;
	the offset values are saved to file data_1_before_offset.txt as input for matching process
	eg 2: ./PreProcess -i data_1_before.las -n statcolor -d nearestmed -inter Yes -in data_1_before_statcolor.ply -ids data_1_before_statcolor_nearestmed.ply -ofs data_1_before_offset.txt
	eg 3: ./PreProcess -i data_1_after.las -n statcolor -d nearestmed -inter Yes -in data_1_after_statcolor.ply -ids data_1_after_statcolor_nearestmed.ply -ofs data_1_after_offset.txt

	2.	./Matching -kp <keypoint_method> -des <descriptor_method> -i1 <ply_pointcloud_input_1> -i2 <ply_pointcloud_input_2> -ikp1 <ply_keypoint_1_file> -ikp2 <ply_keypoint_2_file> -o <ouput_file> -inter <bool> -mp <matchingpairs_point2point_file> -ofs1 <offset_file_1> -ofs2 <offset_file_2>

	Options:
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
	eg: ./Matching -i1 data_1_before_statcolor_nearestmed.ply -i2 data_1_after_statcolor_nearestmed.ply -kp harris3D -des shot -inter Yes -ikp1 data_1_before_statcolor_nearestmed_harris3D.ply -ikp2 data_1_after_statcolor_nearestmed_harris3D.ply -o data_1_statcolor_nearestmed_harris3D_shot.ply -mp data_1_statcolor_nearestmed_harris3D_shot.txt -ofs1 data_1_before_offset.txt -ofs2 data_1_after_offset.txt
	What it does is to use harris3D to detect keypoints in 2 input pointclouds -> save keypoints to data_1_before_statcolor_nearestmed_harris3D.ply and data_1_after_statcolor_nearestmed_harris3D.ply;
	use shot to extract descriptors of the keypoints and matching those keypoints;
	finally save matching results (and matching pairs) to data_1_statcolor_nearestmed_harris3D_shot.ply and data_1_statcolor_nearestmed_harris3D_shot.txt

Note: If you want to run 2d matching method for, there is no need of keypoint detection, so you can type any of the keypoint_method in the command line.
	eg: ./Matching -i1 data_1_before_statcolor_nearestmed.ply -i2 data_1_after_statcolor_nearestmed.ply -kp susan -des 2dmethod -inter No -o data_1_statcolor_nearestmed_0_2dmethod.ply -mp data_1_statcolor_nearestmed_0_2dmethod.txt -ofs1 data_1_before_offset.txt -ofs2 data_1_after_offset.txt
	The matching results (and matching pairs) will be saved to data_1_statcolor_nearestmed_0_2dmethod.ply and data_1_statcolor_nearestmed_0_2dmethod.txt
	
	3. ./ReferPlane <before_pointcloud> <after_pointcloud> <offset_file_1> <offset_file_2> <matching_result_file> <matchingpairs_point2point_file>
	In which:
		before_pointcloud: ply file
		after_pointcloud: ply file
		offset_file_1: .txt file
		offset_file_2: .txt file
		matching_result_file: .ply file
		matchingpairs_point2point_file: .txt file
	eg: ./ReferPlane data_1_before_statcolor_nearestmed.ply data_1_after_statcolor_nearestmed.ply data_1_statcolor_nearestmed_referPlane.ply data_1_statcolor_nearestmed_referPlane.txt data_1_before_offset.txt data_1_after_offset.txt
	What it does is to use refer plane algorithm to determine all the transformations from data_1_before_statcolor_nearestmed.ply to data_1_after_statcolor_nearestmed.ply,
	offset values are read from data_1_before_offset.txt and data_1_after_offset.txt,	matching results are saved to file data_1_statcolor_nearestmed_referPlane.ply,
	and matching pairs are saved to data_1_statcolor_nearestmed_referPlane.txt
Note: When using command line, need to write argument vector in the right order mentioned above.
