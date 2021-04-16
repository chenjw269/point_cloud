//
// Created by chenjw on 2021/4/16.
//

#include "pointcloud_op.h"

pointcloud_op::pointcloud_op() {}
pointcloud_op::~pointcloud_op() =default;

pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_op::readData(const std::string& filename){

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    std::ifstream inFile(filename, std::ios::in);
    std::string lineStr;
    getline(inFile, lineStr);
    while(getline(inFile, lineStr)) {
        std::istringstream inStr(lineStr);
        std::string itemStr;
        std::vector<float> onepoint;
        while(getline(inStr, itemStr, ',')){
            float tem_data = std::stof(itemStr);
            onepoint.push_back(tem_data);
        }
        //std::cout << "x: " << onepoint[7] << " y: "
        //<< onepoint[8] << " z: " << onepoint[9] << std::endl;
        float x = onepoint[7];
        float y = onepoint[8];
        float z = onepoint[9];
        cloud->push_back(pcl::PointXYZ(x, y, z));
    }

    return cloud;
}

pcl::visualization::PCLVisualizer::Ptr pointcloud_op::draw_area(pcl::visualization::PCLVisualizer::Ptr viewer) {

    pcl::PointXYZ endpt1(-4,5,-5);
    pcl::PointXYZ endpt2(7,5,-5);
    viewer->addLine<pcl::PointXYZ>(endpt1, endpt2, 255, 255, 255, "line1");

    pcl::PointXYZ endpt3(-4,25,-5);
    pcl::PointXYZ endpt4(7,25,-5);
    viewer->addLine<pcl::PointXYZ>(endpt3, endpt4, 255, 255, 255, "line2");

    viewer->addLine<pcl::PointXYZ>(endpt1, endpt3, 255, 255, 255, "line3");

    viewer->addLine<pcl::PointXYZ>(endpt2, endpt4, 255, 255, 255, "line4");

    return viewer;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_op::decrease_pc(pcl::PointCloud<pcl::PointXYZ> raw_pc, float threshold){

    pcl::PointCloud<pcl::PointXYZ>::Ptr new_pc(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < raw_pc.points.size(); i++){
        if (abs(raw_pc.points[i].x) > threshold || abs(raw_pc.points[i].y) > threshold || abs(raw_pc.points[i].z) > threshold){

        }
        else {
            if (raw_pc.points[i].x > -25)
                new_pc->push_back(raw_pc.points[i]);
        }
    }

    return new_pc;
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pointcloud_op::clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr raw_pc, float threshold){

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> result;

    // Create a kdtree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    // Set the input cloud
    tree->setInputCloud (raw_pc);
    // Store indices of each cluster
    std::vector<pcl::PointIndices> cluster_indices;
    // Create a EuclideanCluster object
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    // Set the search radius
    ec.setClusterTolerance (threshold);
    // Set the min num of clustering to 10
    ec.setMinClusterSize (10);
    /// Set the max num of clustering to 25000
    ec.setMaxClusterSize (25000);
    // Set the method of search
    ec.setSearchMethod (tree);
    // Set the input cloud
    ec.setInputCloud (raw_pc);
    // Extract the result as indices
    ec.extract (cluster_indices);
    // Visit each cluster
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        // Store cluster in pointcloud
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
            cloud_cluster->points.push_back(raw_pc->points[*pit]);
            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;
        }
        // Store the cluster result
        result.push_back(cloud_cluster);
    }
    return result;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_op::fetch_area(pcl::PointCloud<pcl::PointXYZ> raw_pc){

    pcl::PointCloud<pcl::PointXYZ>::Ptr new_pc(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < raw_pc.points.size(); i++){
        if (raw_pc.points[i].x > -4 && raw_pc.points[i].x < 7
                && raw_pc.points[i].y > 5 && raw_pc.points[i].y < 25){
            new_pc->push_back(raw_pc.points[i]);
        }
        else {

        }
    }

    return new_pc;
}

pcl::visualization::PCLVisualizer::Ptr pointcloud_op::draw_bbox(pcl::PointCloud<pcl::PointXYZ> raw_pc, pcl::visualization::PCLVisualizer::Ptr viewer, std::string label){

    float maxx = raw_pc.points[0].x; float minx = raw_pc.points[0].x;
    float maxy = raw_pc.points[0].y; float miny = raw_pc.points[0].y;
    float maxz = raw_pc.points[0].z; float minz = raw_pc.points[0].z;
    for (int i = 0; i < raw_pc.points.size(); i++){
        if (raw_pc.points[i].x > maxx)
            maxx = raw_pc.points[i].x;
        if (raw_pc.points[i].x < minx)
            minx = raw_pc.points[i].x;
        if (raw_pc.points[i].y > maxy)
            maxy = raw_pc.points[i].y;
        if (raw_pc.points[i].y < miny)
            miny = raw_pc.points[i].y;
        if (raw_pc.points[i].z > maxz)
            maxz = raw_pc.points[i].z;
        if (raw_pc.points[i].z < minz)
            minz = raw_pc.points[i].z;
    }

    Eigen::Vector3f center((maxx+minx)/2,(maxy+miny)/2,(maxz+minz)/2);
    Eigen::Quaternionf rotation(1,0,0,0);
    float length = maxx - minx; float width = maxy -miny; float height = maxz - minz;
    viewer->addCube(center,rotation,length,width,height, label);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, label);
    return viewer;
}