Environtments:
	Ubuntu 16.04 LTS 64-bit

Requirements:
	CMake
	Install pointcloud library 1.8.1 from https://github.com/PointCloudLibrary/pcl
	Install GeoTIFF 1.4.2 from https://github.com/ufz/geotiff
	Install libLAS 1.8.1 from https://github.com/libLAS/LibLAS
	Install openCV 2.4.13 form https://www.opencv.org

Build:
	Open terminal in sourse folder and type:
		mkdir build
		cd build
		cmake ..
		make
	Copy "config.ini" to build/
	There are 8 config files correspoinding to 8 datasets;
	If you want to run with dataset 1, you need to copy the content of the "config_1.ini" file and paste it to "config.ini" file.

Executors:
	PreProcess: Read pointcloud from las file, filter noise, down sample save remaining points to ply file.
	Matching: Determine all transformations from the old pointcloud to the new one.

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
	the offset values are save to file data_1_before_offset.txt as input for matching process
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
	finally save matching result (and matching pairs) to data_1_statcolor_nearestmed_harris3D_shot.ply and data_1_statcolor_nearestmed_harris3D_shot.txt

