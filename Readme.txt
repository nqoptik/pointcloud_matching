Requirements:
	Install pointcloud library from https://github.com/PointCloudLibrary/pcl
	Install libLAS from https://github.com/libLAS/LibLAS

Build:
	Open terminal in sourse folder
	mkdir build
	cd build
	cmake ..
	make

Executors:
	PreProcessing: Read pointcloud from las file, down sample, filter noise, save remaining points to pcl file.
	main: Determine all transformations from the old pointcloud to the new one.

Run:
	./Preprocessing <input.las> <output.pcd>
	./main <old_pointcloud.pcd> <new_pointcloud.pcd>

