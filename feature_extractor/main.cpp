#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>

#include <opencv2/opencv.hpp>

#include <iostream>


int main(int argc, char *argv[]) {

    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ>() );

    boost::filesystem::path full_path(boost::filesystem::current_path());
    std::string input_pcl_str =  full_path.string()+"/../feature_extractor/cloud.ply";
    std::ifstream input_pcl( input_pcl_str );
    if(!input_pcl)
        std::cout<<"File Does Not Exist: " + input_pcl_str<<std::endl;

    pcl::io::loadPLYFile<pcl::PointXYZ>(input_pcl_str, *cloud);

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

    ne.setInputCloud (cloud);
    ne.setSearchMethod (tree);
    ne.setRadiusSearch (2);
    ne.compute (*normals);

    //std::cout<<"normals cloud size "<< normals->size ()<<std::endl;

    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());

    fpfh.setInputCloud (cloud);
    fpfh.setInputNormals (normals);
    fpfh.setSearchMethod (tree2);
    fpfh.setRadiusSearch (4);
    fpfh.compute (*fpfhs);

    for(int i=0; i<fpfhs->size();i++){
        for(int j=0; j<33; j++) {
            std::cout << fpfhs->points[i].histogram[j] << " ";
        }
    }
    return 0;
}
