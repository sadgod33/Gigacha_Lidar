#include "pointcloud_generator.h"


/**
 * @brief exampleCloud를 생성한다.
 * @param 과정: ~~
 *                    기본적으로 포인트클라우드를 직접 매게변수로 받기에는 효율적이지 않아서 수정할 포인트 클라우드 포인터에 데이터를 직접 넣는 방식을 채택했다.
 * @param input_pointCloud 가공 전 포인트 클라우드
 *                         즉, 포인트 클라우드를 새로 생성하기 전에 어느 포인트 클라우드를 기준으로 가공할지 결정한다.
 * @param output_pointCloud 가공 후 저장할 포인트 클라우드
 *                         즉, 포인트 클라우드를 수정하고 나서 저장할 포인트 클라우드를 결정한다.
 */
void PointCloudGenerator::getExampleCloud(pcl::PointCloud<PointType>::Ptr& input_pointCloud, pcl::PointCloud<PointType>::Ptr& output_pointCloud)
{
    output_pointCloud->clear();
    // 여기에 내용 추가
    /**
     *? 매게변수로 input_pointCloud를 넣는데 두번째 preprocessor부터는 
     *? input_pointCloud가 아닌 output_pointCloud를 매게변수로 넣어야한다.
     */
}

/**
 * @brief 기울기를 알기 힘든 상황에서 1도부터 5도까지의 선을 준다.
*/
void PointCloudGenerator::getAngle1to5(const pcl::PointCloud<PointType>::Ptr& input_pointCloud, const std::shared_ptr<visualization_msgs::MarkerArray>& output_markerArray)
{
    // 가장 우측에 있는 포인트 클라우드 구하기
    float minY = std::numeric_limits<float>::max();
    PointType minPoint;
    for (const auto& point : input_pointCloud->points) 
    {
        if (-0.2 < point.x && point.x < 0.2)
        {
            if (point.y < minY)
            {
                minY = point.y;
                minPoint = point;
            }
        }
    }

    double length = 7.0; // 선의 길이
    double startX = minPoint.x;
    double startY = minPoint.y;

    for (int i = -5; i <= 5; ++i)
    {
        double angle_deg = static_cast<double>(i); // 각도 (도)
        double angle_rad = angle_deg * DEG_TO_RAD; // 각도를 라디안으로 변환

        // 선의 끝점 계산
        double x_end = length * cos(angle_rad);
        double y_end = length * sin(angle_rad);

        visualization_msgs::Marker marker;
        marker.ns = "lines";
        marker.id = i;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.01; // 선의 두께
        marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0;
        marker.color.a = 1.0;

        geometry_msgs::Point p_start;
        p_start.x = startX; p_start.y = startY; p_start.z = 0.0; // 시작점 (원점)
        geometry_msgs::Point p_end;
        p_end.x = x_end + startX; p_end.y = y_end + startY; p_end.z = 0.0; // 끝점
        marker.points.push_back(p_start);
        marker.points.push_back(p_end);

        output_markerArray->markers.push_back(marker);
    }
}
void PointCloudGenerator::getAngle1to5(const pcl::PointCloud<PointType>::Ptr& input_pointCloud, const std::shared_ptr<visualization_msgs::MarkerArray>& output_markerArray, double angle)
{
    // 가장 우측에 있는 포인트 클라우드 구하기
    float minY = std::numeric_limits<float>::max();
    PointType minPoint;
    minPoint.x = 0;
    minPoint.y = 0;
    minPoint.z = 0;

    double length = 7.0; // 선의 길이
    double startX = minPoint.x;
    double startY = minPoint.y;

    double angle_deg = static_cast<double>(angle); // 각도 (도)
    double angle_rad = angle_deg * DEG_TO_RAD; // 각도를 라디안으로 변환

    // 선의 끝점 계산
    double x_end = length * cos(angle_rad);
    double y_end = length * sin(angle_rad);

    visualization_msgs::Marker marker;
    marker.ns = "lines";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.1; // 선의 두께
    marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0;
    marker.color.a = 1.0;

    geometry_msgs::Point p_start;
    p_start.x = startX; p_start.y = startY; p_start.z = 0.0; // 시작점 (원점)
    geometry_msgs::Point p_end;
    p_end.x = x_end + startX; p_end.y = y_end + startY; p_end.z = 0.0; // 끝점
    marker.points.push_back(p_start);
    marker.points.push_back(p_end);

    output_markerArray->markers.push_back(marker);

}


/**
 * @brief InterestCloud를 생성한다.
 * @param 과정: (범위 지정)
 * @param input_pointCloud 가공 전 포인트 클라우드
 * @param output_pointCloud 가공 후 저장할 포인트 클라우드
 * @param xyz_threshold 포인트 클라우드를 자를 범위
*/
void PointCloudGenerator::getInterestCloud(const pcl::PointCloud<PointType>::Ptr& input_pointCloud,
                                           const pcl::PointCloud<PointType>::Ptr& output_pointCloud,
                                           const std::pair<double, double>       x_threshold, 
                                           const std::pair<double, double>       y_threshold, 
                                           const std::pair<double, double>       z_threshold)
{ 
    output_pointCloud->clear();
    preprocessor.cutPointCloud(input_pointCloud, output_pointCloud, x_threshold, y_threshold, z_threshold);
}


/**
 * @brief InterestCloud를 생성한다.
 * @param 과정: (범위 지정)
 * @param input_pointCloud 가공 전 포인트 클라우드
 * @param output_pointCloud 가공 후 저장할 포인트 클라우드
 * @param xyz_threshold 포인트 클라우드를 자를 범위
*/
void PointCloudGenerator::getFovCloud(const pcl::PointCloud<PointType>::Ptr& input_pointCloud,
                                      const pcl::PointCloud<PointType>::Ptr& output_pointCloud,
                                      const std::pair<double, double>       x_threshold, 
                                      const std::pair<double, double>       y_threshold, 
                                      const std::pair<double, double>       z_threshold,
                                      const std::pair<float,  float >       xy_angle_threshold)
{ 
    output_pointCloud->clear();
    preprocessor.cutPointCloud(input_pointCloud, output_pointCloud, x_threshold, y_threshold, z_threshold, xy_angle_threshold);
}


/**
 * @brief 3D 포인트 클라우드를 2D 포인트 클라우드로 압축한다.
 * @param input_pointCloud 가공할 포인트 클라우드
 * @param object_size 차원 압축시 2D voxel_grid할 규격
*/
void PointCloudGenerator::get2DCloud(const pcl::PointCloud<PointType>::Ptr& input_pointCloud, const pcl::PointCloud<PointType>::Ptr& output_pointCloud, const double remove_floor, const double voxel_grid_size)
{
    preprocessor.removeFloor(input_pointCloud, output_pointCloud, {remove_floor, 0});
    preprocessor.convert3Dto2D(output_pointCloud, output_pointCloud, voxel_grid_size);
    output_pointCloud->height = 1;
    output_pointCloud->width = output_pointCloud->points.size();
}


/**
 * @brief cone 클러스터링을 진행한다.
 * @param input_pointCloud 가공 전 포인트 클라우드
 * @param output_pointCloud 가공 후 저장할 포인트 클라우드
*/
void PointCloudGenerator::getConeClusterCloud(const pcl::PointCloud<PointType>::Ptr& input_pointCloud,
                                              const pcl::PointCloud<PointType>::Ptr& output_pointCloud)
{ 
    output_pointCloud->clear();
    clustering.clusterCone(input_pointCloud, output_pointCloud);
}


/**
 * @brief 다목적 장애물을 클러스터링한다.
*/
void PointCloudGenerator::getObjectClusterCloud(const pcl::PointCloud<PointType>::Ptr& input_pointCloud,
                                                  const std::shared_ptr<visualization_msgs::MarkerArray>& smallObject_MarkerArray,
                                                  const std::shared_ptr<visualization_msgs::MarkerArray>& bigObject_MarkerArray,
                                                  const pcl::PointCloud<PointType>::Ptr& debug_pointCloud)
{
    debug_pointCloud->clear();
    smallObject_MarkerArray->markers.clear();
    bigObject_MarkerArray->markers.clear();

    clustering.clusterObject(input_pointCloud, smallObject_MarkerArray, bigObject_MarkerArray, debug_pointCloud);
}


/**
 * @brief 클러스터링한 대상을 저장한다.
 * @param input_pointCloud 가공 전 포인트 클라우드
 * @param output_pointCloud 가공 후 저장할 포인트 클라우드
 * @param debug_pointCloud 디버깅용 포인트 클라우드
*/
void PointCloudGenerator::getSavedClusterCloud(const pcl::PointCloud<PointType>::Ptr& input_pointCloud,
                                              const pcl::PointCloud<PointType>::Ptr& output_pointCloud,
                                              const pcl::PointCloud<PointType>::Ptr& debug_pointCloud)
{ 
    clustering.saveCluster(input_pointCloud, output_pointCloud);
    
    // 디버깅
    *debug_pointCloud = *output_pointCloud;
    for (PointType& point : debug_pointCloud->points)
    {
        transformAbsToRel(point);
    }
}

/**
 * @brief 콘 위치로 ROI를 설정한다.
 * @param input_pointCloud 가공 전 포인트 클라우드
 * @param output_pointCloud 가공 후 저장할 포인트 클라우드
 * @param debug_pointCloud 디버깅용 포인트 클라우드
*/
void PointCloudGenerator::getconeROICloud(const pcl::PointCloud<PointType>::Ptr& input_pointCloud,
                                          const pcl::PointCloud<PointType>::Ptr& output_pointCloud,
                                          const std::shared_ptr<visualization_msgs::MarkerArray>& ROI_MarkerArray)
{ 
    output_pointCloud->clear();
    clustering.setConeROI(input_pointCloud, output_pointCloud, ROI_MarkerArray);
}


/**
 * @brief 콘의 좌우를 판단한다.
 * @param input_pointCloud 가공 전 포인트 클라우드
 * @param output_Rcone_pointCloud 우측 콘의 포인트 클라우드
 * @param output_Lcone_pointCloud 좌측 콘의 포인트 클라우드
*/
void PointCloudGenerator::getLRconeCloud(const pcl::PointCloud<PointType>::Ptr& input_pointCloud, const pcl::PointCloud<PointType>::Ptr& output_Rcone_pointCloud, const pcl::PointCloud<PointType>::Ptr& output_Lcone_pointCloud)
{
    clustering.identifyLRcone_v2(input_pointCloud, output_Rcone_pointCloud, output_Lcone_pointCloud);
}


/**
 * @brief ROICloud를 생성한다.
 *          z축을 제거해서 검색했기에 연산과정이 적고, 구체가 아닌 원기둥을 중심으로 검색한다고 판단
 * @param 과정: (z축 제거) -> (KDtree 검색) -> (ROI 범위 지정)
 * @param input_pointCloud 가공 전 포인트 클라우드
 * @param output_pointCloud 가공 후 저장할 포인트 클라우드
 * @param radius ROI 범위인 원기둥의 반지름
*/
void PointCloudGenerator::getROICloud(const pcl::PointCloud<PointType>::Ptr& input_pointCloud, const pcl::PointCloud<PointType>::Ptr& output_pointCloud, const float radius)
{
    output_pointCloud->clear();

    // output_pointCloud를 z축 값만 제거한 임시 포인트 클라우드로 사용함
    for (const PointType& point : input_pointCloud->points) {
        PointType newPoint = point;
        newPoint.z = 0.0; // z 좌표를 0으로 설정하여 제거
        output_pointCloud->push_back(newPoint);
    }
    
    // kdtree 알고리즘 검색 대상으로 z축을 제거한 포인트 클라우드로 사용
    pcl::KdTreeFLANN<PointType> kdtree;
    kdtree.setInputCloud(output_pointCloud);
    output_pointCloud->clear();
    
    std::unordered_set<int> all_indices;
    // 범위 설정
    int path_len = global_path.size();
    int min_lookback = curr_index - std::min(curr_index, 200);
    int max_lookforward = curr_index + std::min(200, path_len - curr_index - 1);

    for(int i = min_lookback; i < max_lookforward;++i){
        if (i%10==0){
            std::vector<int> idxes;
            std::vector<float> _;
            pcl::PointXYZI query;
            idxes.clear();
            _.clear();

            double dx = global_path[i].first - ego_x;
            double dy = global_path[i].second - ego_y;

            double cos_heading = std::cos(ego_heading_rad);
            double sin_heading = std::sin(ego_heading_rad);
            
            query.x = static_cast<float>(cos_heading * dx + sin_heading * dy);
            query.y = static_cast<float>(-sin_heading * dx + cos_heading * dy);
            query.z = 0.0f;
            query.intensity = 0.0f;

            kdtree.radiusSearch(query, radius, idxes, _);
            
            for (const auto& idx : idxes) {
                all_indices.insert(idx);
            }
        }
    } 

    for (const auto& idx : all_indices){
        output_pointCloud->points.push_back(input_pointCloud->points[idx]);
    }
}