//
// Created by chenjw on 2021/4/16.
//

#ifndef SOFTWARE_ENGINEERING_POINTCLOUD_OP_H
#define SOFTWARE_ENGINEERING_POINTCLOUD_OP_H

#include <iostream>
#include <typeinfo>
#include <string>
#include <vector>
#include <fstream>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/extract_clusters.h>

class pointcloud_op {

public:
    pointcloud_op();
    ~pointcloud_op();
    pcl::PointCloud<pcl::PointXYZ>::Ptr readData(const std::string& filename);
    pcl::visualization::PCLVisualizer::Ptr draw_area(pcl::visualization::PCLVisualizer::Ptr viewer);
    pcl::PointCloud<pcl::PointXYZ>::Ptr decrease_pc(pcl::PointCloud<pcl::PointXYZ> raw_pc, float door);
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr raw_pc, float threshold);
    pcl::PointCloud<pcl::PointXYZ>::Ptr fetch_area(pcl::PointCloud<pcl::PointXYZ> raw_pc);
    pcl::visualization::PCLVisualizer::Ptr draw_bbox(pcl::PointCloud<pcl::PointXYZ> raw_pc, pcl::visualization::PCLVisualizer::Ptr viewer, std::string label);
};


#endif //SOFTWARE_ENGINEERING_POINTCLOUD_OP_H
