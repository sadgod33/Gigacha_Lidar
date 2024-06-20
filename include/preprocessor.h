#ifndef PREPROCESSOR_H
#define PREPROCESSOR_H

#include "utility.h"

class Preprocessor{
public:

    void initCol(std::vector<std::vector<float>>& table, int col_num, int init_num);

    void removeBody(const pcl::PointCloud<PointType>::Ptr& input_pointCloud, const pcl::PointCloud<PointType>::Ptr& output_pointCloud);

    void calibrateLidar(const pcl::PointCloud<PointType>::Ptr& input_pointCloud, const pcl::PointCloud<PointType>::Ptr& output_pointCloud, float Yangle);


    /**
     * @brief 결측치를 제거한다.
     * @param input_pointCloud 가공할 포인트 클라우드
    */
    void removeNaN(const pcl::PointCloud<PointType>::Ptr& input_pointCloud, const pcl::PointCloud<PointType>::Ptr& output_pointCloud);

    /**
     * @brief 지면을 제거한다.
     * @param input_pointCloud 가공할 포인트 클라우드
     * @param z_threshold z값 범위
    */
    void removeFloor(const pcl::PointCloud<PointType>::Ptr& input_pointCloud, const pcl::PointCloud<PointType>::Ptr& output_pointCloud, std::pair<double, double> z_threshold);

    /**
     * @brief voxel_grid로 다운셈플링한다.
     * @param input_pointCloud 가공할 포인트 클라우드
     * @param leaf_size voxel_grid할 규격
    */
    void voxelGride(const pcl::PointCloud<PointType>::Ptr& input_pointCloud, const pcl::PointCloud<PointType>::Ptr& output_pointCloud, double leaf_size);

    /**
     * @brief 3D 포인트 클라우드를 2D 포인트 클라우드로 압축한다.
     * @param input_pointCloud 가공할 포인트 클라우드
     * @param object_size 차원 압축시 2D voxel_grid할 규격
    */
    void convert3Dto2D(const pcl::PointCloud<PointType>::Ptr& input_pointCloud, const pcl::PointCloud<PointType>::Ptr& output_pointCloud, double object_size);

    /**
     * @brief 모든 포인트의 중간값을 노이즈로 추가한다.
     * @param input_point 자를 포인트 
     * @param output_pointCloud 자른 포인트를 저장할 포인트 클라우드
    */
    void addNoise(const pcl::PointCloud<PointType>::Ptr& input_pointCloud, const pcl::PointCloud<PointType>::Ptr& output_pointCloud);

    /**
     * @brief x, y, z 범위 만큼 자른다.
     * @param input_point 자를 포인트 
     * @param output_pointCloud 자른 포인트를 저장할 포인트 클라우드
     * @param xyz_threshold 포인트를 자를 범위 (최대값: -99999, 최소값: 99999)
    */
    void cutPointCloud(const pcl::PointCloud<PointType>::Ptr& input_pointCloud,
                       const pcl::PointCloud<PointType>::Ptr& output_pointCloud,
                       const std::pair<double, double>       x_threshold, 
                       const std::pair<double, double>       y_threshold, 
                       const std::pair<double, double>       z_threshold);


    /**
     * @brief x, y, z 범위와 xy각도 만큼 자른다.
     * @param input_pointCloud 자를 포인트 클라우드
     * @param output_pointCloud 자른 포인트를 저장할 포인트 클라우드
     * @param xyz_threshold 포인트를 자를 범위 (최대값: -99999, 최소값: 99999)
     * @param xy_angle_threshold 포인트를 자를 xy 각도 (가운데로 부터 좌측: 음수, 우측: 양수)
    */
    void cutPointCloud(const pcl::PointCloud<PointType>::Ptr& input_pointCloud,
                       const pcl::PointCloud<PointType>::Ptr& output_pointCloud,
                       const std::pair<double, double>       x_threshold, 
                       const std::pair<double, double>       y_threshold, 
                       const std::pair<double, double>       z_threshold,
                       const std::pair<float,  float >       xy_angle_threshold);


    /**
     * @brief pcl::PointCloud를 std::vector에 저장후 x값이 작은것부터 큰 순서로 정리한다.
     * @param input_pointCloud 자를 포인트 클라우드
     * @param output_vector 자른 포인트를 저장할 컨테이너
    */
    void convertPCLtoVector(const pcl::PointCloud<PointType>::Ptr& input_pointCloud, std::unique_ptr<std::vector<PointType>>& output_vector);


    /**
     * @brief 2개의 점을 이은 선에 대해서 좌측, 우측, 위에 있는지 판단한다.
     * @param firstMidPoint 선이 될 첫번째 점
     * @param secondMidPoint 선이 될 두번째 점
     * @param point 확인할 대상
    */
    float crossLine(std::pair<float, float> firstMidPoint, std::pair<float, float> secondMidPoint, PointType point);


    /**
     * ((x, y), (거리, 각도))에 해당되는 범위 내에 포인트가 있는지 판단한다.
    */
    bool betweenLine(std::pair<std::pair<float, float>, std::pair<float, float>> pointArea, PointType point);

    float distanceTwoPointPower2(std::pair<float, float> midPoint, PointType point);

    bool isWithinAngle(PointType point, std::pair<float, float> midPoint, float midPointAngle, float angle);

    int determineRegion(double a, double b, double m, double c, double d);
};
#endif  //PREPROCESSOR_H