#include <iostream>
#include "pointcloud_op.h"

int main() {

    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud;
    pointcloud_op po;
    std::string filename("/home/chenjw/data/2021-04-14-17-04-01-RS-16-Data/2021-04-14-17-04-01-RS-16-Data (Frame 0484).csv");
    raw_cloud = po.readData(filename);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = po.decrease_pc(*raw_cloud, 40);

    // show the point cloud
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Road scene"));
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green_color(cloud, 0, 255, 0); // green
    viewer->addPointCloud(cloud, green_color);

    // mark our interested area
    po.draw_area(viewer);

    // fetch the interested area
    pcl::PointCloud<pcl::PointXYZ>::Ptr area;
    area = po.fetch_area(*cloud);
    // clustering
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> result;
    result = po.clustering(area, 2);
    // show the clustering result
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red_color(cloud, 255, 0, 0); // red
    int count = 0; std::string name = "cluster"; std::string label = "label";
    for (auto i = result.begin(); i != result.end(); i++){
        viewer->addPointCloud(*i, red_color, name.append(std::to_string(count)));
        po.draw_bbox(*(*i), viewer, label.append(std::to_string(count)));
        count ++;
    }
    std::cout << "Find " << count << " objects" << std::endl;

    // keep the viewer
    while (!viewer->wasStopped()){
        viewer->spinOnce(100);
    }

    return 0;
}
