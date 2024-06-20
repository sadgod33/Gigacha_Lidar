#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include "preprocessor.h"
#include "clustering.h"


class PointCloudGenerator{
public:
    Preprocessor preprocessor;
    Clustering clustering;

    /**
     * @brief exampleCloud를 생성한다.
     * @param 과정: ~~
     *                    기본적으로 포인트클라우드를 직접 매게변수로 받기에는 효율적이지 않아서 수정할 포인트 클라우드 포인터에 데이터를 직접 넣는 방식을 채택했다.
     * @param input_pointCloud 가공 전 포인트 클라우드
     *                         즉, 포인트 클라우드를 새로 생성하기 전에 어느 포인트 클라우드를 기준으로 가공할지 결정한다.
     * @param output_pointCloud 가공 후 저장할 포인트 클라우드
     *                         즉, 포인트 클라우드를 수정하고 나서 저장할 포인트 클라우드를 결정한다.
     */
    void getExampleCloud(pcl::PointCloud<PointType>::Ptr& input_pointCloud, pcl::PointCloud<PointType>::Ptr& output_pointCloud);

    void getAngle1to5(const pcl::PointCloud<PointType>::Ptr& input_pointCloud, const std::shared_ptr<visualization_msgs::MarkerArray>& output_markerArray, double angle);

    void getAngle1to5(const pcl::PointCloud<PointType>::Ptr& input_pointCloud, const std::shared_ptr<visualization_msgs::MarkerArray>& output_markerArray);

    void getFullCloud(const pcl::PointCloud<PointType>::Ptr& input_pointCloud, const pcl::PointCloud<PointType>::Ptr& output_pointCloud, const pcl::PointCloud<PointType>::Ptr& output_infoCloud);


    /**
     * @brief InterestCloud를 생성한다.
     * @param 과정: (범위 지정)
     * @param input_pointCloud 가공 전 포인트 클라우드
     * @param output_pointCloud 가공 후 저장할 포인트 클라우드
     * @param xyz_threshold 포인트 클라우드를 자를 범위
    */
    void getInterestCloud(const pcl::PointCloud<PointType>::Ptr& input_pointCloud, const pcl::PointCloud<PointType>::Ptr& output_pointCloud, const std::pair<double, double> x_threshold, const std::pair<double, double> y_threshold, const std::pair<double, double> z_threshold);
 
    /**
     * @brief InterestCloud를 생성한다.
     * @param 과정: (범위 지정)
     * @param input_pointCloud 가공 전 포인트 클라우드
     * @param output_pointCloud 가공 후 저장할 포인트 클라우드
     * @param xyz_threshold 포인트 클라우드를 자를 범위
    */
    void getFovCloud(const pcl::PointCloud<PointType>::Ptr& input_pointCloud, const pcl::PointCloud<PointType>::Ptr& output_pointCloud, const std::pair<double, double> x_threshold, const std::pair<double, double> y_threshold, const std::pair<double, double> z_threshold, const std::pair<float,  float > xy_angle_threshold);


    void getTransformedClouds(const pcl::PointCloud<PointType>::Ptr& input_pointCloud, const pcl::PointCloud<PointType>::Ptr& output_pointCloud);


    void getGrdRemovalClouds(const pcl::PointCloud<PointType>::Ptr& input_pointCloud, const pcl::PointCloud<PointType>::Ptr& groundCloud, const pcl::PointCloud<PointType>::Ptr& nongroundCloud, const std::pair<double, double> grdRemoval_threshold);

    /**
     * @brief Tob view의 시각에 맞춰 포인트 클라우드를 3d에서 2d로 변환한다.
     * @param 과정: (voxel grid) -> (z축 제거) -> (voxel grid)
     * @param input_pointCloud 가공 전 포인트 클라우드
     * @param output_pointCloud 가공 후 저장할 포인트 클라우드
     * @param remove_floor 라이다로부터 밑으로 자를 범위
     * @param voxel_grid_size 다운셈플링 규격
     * const int remove_floor, const double voxel_grid_size
    */
    void get2DCloud(const pcl::PointCloud<PointType>::Ptr& input_pointCloud, const pcl::PointCloud<PointType>::Ptr& output_pointCloud, const double remove_floor, const double voxel_grid_size);


    /**
     * @brief InterestCloud를 생성한다.
     * @param input_pointCloud 가공 전 포인트 클라우드
     * @param output_pointCloud 가공 후 저장할 포인트 클라우드
    */
    void getConeClusterCloud(const pcl::PointCloud<PointType>::Ptr& input_pointCloud, const pcl::PointCloud<PointType>::Ptr& output_pointCloud);
    void getObjectClusterCloud(const pcl::PointCloud<PointType>::Ptr& input_pointCloud, const std::shared_ptr<std::vector<pcl::PointCloud<PointType>::Ptr>>& smallObject_Cloud_vector, const std::shared_ptr<std::vector<pcl::PointCloud<PointType>::Ptr>>& bigObject_Cloud_vector);
    void getObjectMarkers(const std::shared_ptr<std::vector<pcl::PointCloud<PointType>::Ptr>>& bigObject_Cloud_vector, const std::shared_ptr<visualization_msgs::MarkerArray>& markerarray, const std::shared_ptr<visualization_msgs::MarkerArray>& markerarray_vis);
    void getSavedClusterCloud(const pcl::PointCloud<PointType>::Ptr& input_pointCloud, const pcl::PointCloud<PointType>::Ptr& output_pointCloud, const pcl::PointCloud<PointType>::Ptr& debug_pointCloud);

    void getconeROICloud(const pcl::PointCloud<PointType>::Ptr& input_pointCloud, const pcl::PointCloud<PointType>::Ptr& output_pointCloud, const std::shared_ptr<visualization_msgs::MarkerArray>& ROI_MarkerArray);

    void getLRconeCloud(const pcl::PointCloud<PointType>::Ptr& input_pointCloud, const pcl::PointCloud<PointType>::Ptr& output_Rcone_pointCloud, const pcl::PointCloud<PointType>::Ptr& output_Lcone_pointCloud, const std::shared_ptr<visualization_msgs::MarkerArray>&  debug_Rcone_pointCloud, const std::shared_ptr<visualization_msgs::MarkerArray>&  debug_Lcone_pointCloud);

    /**
     * @brief ROICloud를 생성한다.
     * @param 과정: (z축 제거) -> (KDtree 검색) -> (ROI 범위 지정)
     * @param input_pointCloud 가공 전 포인트 클라우드
     * @param output_pointCloud 가공 후 저장할 포인트 클라우드
     * @param radius ROI 범위인 원기둥의 반지름
    */
    void getROICloud(const pcl::PointCloud<PointType>::Ptr& input_pointCloud, const pcl::PointCloud<PointType>::Ptr& output_pointCloud, const float radius);
};

#endif  //POINTCLOUD_H