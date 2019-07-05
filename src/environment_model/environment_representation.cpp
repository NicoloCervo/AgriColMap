#include "environment_representation.h"

EnvironmentRepresentation::EnvironmentRepresentation(const std::string& cloudName) : _cloudName(cloudName) {} //, _scaleNoise(1.f, 1.f) {}

void EnvironmentRepresentation::loadFromPCLcloud( const PCLPointCloudXYZRGB::Ptr& pointCloud,
                                                  const float& square_size,
                                                  const Vector2& imgCenter,
                                                  const float& radius,
                                                  const cv::Size& gridMapSize){

    _square_size = square_size;
    pcl::getMinMax3D(*pointCloud, minPt, maxPt);

    _width = gridMapSize.width;
    _height = gridMapSize.height;

    minPt.x = - ( _width / 2 ) * square_size + imgCenter(0);
    maxPt.y = - ( _height / 2 ) * square_size + imgCenter(1);

    x_coord = minPt.x;
    y_coord = maxPt.y;
    altitude_scale = 255.0 / (maxPt.z - minPt.z);

    PCLptXYZRGB init_pt;
    init_pt.x = 0.f; init_pt.y = 0.f; init_pt.z = 0.f;

    _gridMap = std::vector< std::vector<PCLptXYZRGB> >( _width * _height);
    KDTreeXYvector kdTreeXY_points(_width * _height);
    for(unsigned int r = 0; r < _height; ++r){
        for(unsigned int c = 0; c < _width; ++c){
            kdTreeXY_points[c + r * _width](0) = x_coord + (float) c * square_size;
            kdTreeXY_points[c + r * _width](1) = y_coord + (float) r * square_size;
        }
    }

    float leaf_range = 0.1;
    KDTreeXY* kd_tree = new KDTreeXY(kdTreeXY_points, leaf_range);

    for( PCLptXYZRGB pt : pointCloud->points){
      KDTreeXYpoint query_point = pt.getVector3fMap().head(2);
      KDTreeXYpoint answer;
      int index;

      float approx_distance = kd_tree->findNeighbor(answer, index, query_point, radius);
      if (approx_distance > 0) {
          float c_idx = ( (answer(0) - x_coord) / square_size );
          float r_idx = ( (answer(1) - y_coord) / square_size );
          int _gridMapIndex = c_idx + r_idx * _width;
           _gridMap[_gridMapIndex].push_back(pt);
      }
    }

}

PCLptXYZRGB EnvironmentRepresentation::computeAveragePoint(std::vector<PCLptXYZRGB>& ptVec,
                                                           const unsigned int& col,
                                                           const unsigned int& row){

    PCLptXYZRGB out_pt;
    float sum = 0.f; float x = 0.f; float y = 0.f;
    float z = 0.f; float r = 0.f; float g = 0.f; float b = 0.f;
    Vector2 nom_pt(x_coord + col * _square_size, y_coord + row * _square_size);
    float radius = 0.01;
    float std_dev = 0.04;
    float den = 2.f * std_dev * std_dev;
    int iter = 0;
    for( PCLptXYZRGB pt : ptVec ){
        if(pt.getVector3fMap().norm() > 1e-4){
            //float weight = ( nom_pt - pt.getVector3fMap().head(2) ).norm()/radius;
            //float weight =  float( exp ( - (double) ( ( nom_pt - pt.getVector3fMap().head(2) ).norm() ) / ( (double)(2.0 * radius * radius) ) ) );
            float dist = ( nom_pt - pt.getVector3fMap().head(2) ).norm();
            float weight = 1 / exp ( dist / den );
            //setprecision(10);
            //std::cerr << setprecision(10) << nom_pt.transpose() << " " << pt.getVector3fMap().head(2).transpose() << " " << dist << " " << 1 / exp ( dist / (2.0 * 0.04 * 0.04) )  << "\n";
            sum += weight;
            x += pt.x*weight;
            y += pt.y*weight;
            z += pt.z*weight;
            r += (float)pt.r*weight;
            g += (float)pt.g*weight;
            b += (float)pt.b*weight;
            iter++;
        }
    }
    if(iter){
        out_pt.x = x/sum;
        out_pt.y = y/sum;
        out_pt.z = z/sum;
        out_pt.r = (int)(r/sum);
        out_pt.g = (int)(g/sum);
        out_pt.b = (int)(b/sum);
        return out_pt;
    }
}

void EnvironmentRepresentation::computeMMGridMap(){

    // Image size is flipped for the sake of better visualization of the results
    exgImg = cv::Mat( cv::Size(_height, _width), CV_8UC1, cv::Scalar(0) );
    elevImg = cv::Mat( cv::Size(_height, _width), CV_8UC1, cv::Scalar(0) );
    xyzImg = cv::Mat( cv::Size(_height, _width), CV_32FC3, cv::Scalar(0,0,0) );
    exgImgColor = cv::Mat( cv::Size(_height, _width), CV_8UC3, cv::Scalar(0,0,0) );
    rgbImg = cv::Mat( cv::Size(_height, _width), CV_8UC3, cv::Scalar(0,0,0) );

    int iter = 0;
    for(unsigned int r = 0; r < _height; ++r){
        for(unsigned int c = 0; c < _width; ++c, ++iter){
            PCLptXYZRGB pt = computeAveragePoint(_gridMap[iter], c, r);
            if( pt.getVector3fMap().norm() <= 1e-3 )
                continue;
            int ExG = computeExGforXYZRGBPoint(pt);
            exgImg.at<uchar>(c,r) = ExG;
            exgImgColor.at<cv::Vec3b>(c,r)[1] = ExG;
            elevImg.at<uchar>(c,r) = altitude_scale * ( pt.z - minPt.z );
            xyzImg.at<cv::Vec3f>(c,r)[0] = pt.x;
            xyzImg.at<cv::Vec3f>(c,r)[1] = pt.y;
            xyzImg.at<cv::Vec3f>(c,r)[2] = pt.z;
            rgbImg.at<cv::Vec3b>(c,r)[0] = pt.b;
            rgbImg.at<cv::Vec3b>(c,r)[1] = pt.g;
            rgbImg.at<cv::Vec3b>(c,r)[2] = pt.r;
        }
    }

    cv::normalize(xyzImg, xyzImgUChar, 255, 0, cv::NORM_MINMAX);
    xyzImgUChar.convertTo(xyzImgUChar, CV_8UC3);
}
