#include "plyio.h"

void savePly(std::string fileName, std::vector<pcl::PointXYZRGB> listxyzrgb) {

    std::ofstream ofs_ply(fileName.c_str());
    ofs_ply << "ply\n";
    ofs_ply << "format ascii 1.0\n";
    ofs_ply << "element vertex " <<  listxyzrgb.size() << "\n";
    ofs_ply << "property float x\n";
    ofs_ply << "property float y\n";
    ofs_ply << "property float z\n";
    ofs_ply << "property uchar red\n";
    ofs_ply << "property uchar green\n";
    ofs_ply << "property uchar blue\n";
    ofs_ply << "element face 0\n";
    ofs_ply << "property list uchar int vertex_indices\n";
    ofs_ply << "end_header\n";
    for (size_t i = 0; i < listxyzrgb.size(); i++) {
        ofs_ply << listxyzrgb[i].x << " " << listxyzrgb[i].y << " " << listxyzrgb[i].z << " "
            << (int)listxyzrgb[i].r << " " << (int)listxyzrgb[i].g << " " << (int)listxyzrgb[i].b << "\n";
    }
    ofs_ply.close();
}

