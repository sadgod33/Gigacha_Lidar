#include "utility.h"




/**
 ** 목적
    ** 1. 상수 및 구조체 저장
    ** 2. preprocessor, clustering ... 등 해당 카테고리에 해당되지 않는 함수
*/


// 상수 선언
    // ring 사용 여부
    const bool useCloudRing = true;                         // ring 사용 여부

    // 기호 상수
    const double PI = 3.14159265;                           // PI
    const int INF = 99999;                                  // 포인트의 범위를 발산으로 하고 싶을 때 넣을 것

    // 라이다 센서 상수
    const int N_SCAN = 32;                                  // 라이다의 row 이자 ring
    const int Horizon_SCAN = 1024;                          // 라이다의 colmun
    const float ang_res_x = 360.0/float(Horizon_SCAN);      // col -> 각도
    const float ang_res_y = 45/float(N_SCAN-1);             // row -> 각도
    const float ang_bottom = 20+0.1;                        // 라이다 시야각 / 2
    const int groundScanInd = 15;                           // 라이다 시야각 상하 분기각

    // 마운트 상수
    const double LI_TO_GPS_X = -1.35;                       // 라이다와 GPS 사이 거리
    const double LI_TO_GND_Z = -0.9;                        // 라이다와 지변 사이 거리
    const double LI_ROT_Y = 0;                              // y축 회전
    const double CAR_TO_GPS_X = -0.1;                       // 차량의 중심과 GPS사이 거리      
    
    // 차량 크기
    const float CAR_SIZE_X = 1.9;                          // 차량의 x축 길이
    const float CAR_SIZE_Y = 4.7;                          // 차량의 y축 길이
    const float CAR_SIZE_Z = 1.7;                          // 차량의 z축 길이

    // 계산 상수
    const double DEG_TO_RAD = PI/180.0;                     // 각도 -> 라이안
    const double RAD_TO_DEG = 180.0/PI;                     // 라이안 -> 각도

    // 콘 클러스터링 상수
    const double Cone_Tolerance = 0.3;                      // 콘 사이 거리
    const int CONE_min_SIZE = 2;                           // 콘에 찍히는 포인트 최소
    const int CONE_MAX_SIZE = 3000;                          // 콘에 찍히는 포인트 최대

    // 기본 클러스터링 상수
    const double OBJECT_Tolerance = 0.75;                      // 객체 사이 거리
    const int OBJECT_min_SIZE = 10;                           // 객체에 찍히는 포인트 최소
    const int OBJECT_MAX_SIZE = 50;                          // 객체에 찍히는 포인트 최대

    // 콘 주행 상수
    const double CONE_BETWEEN = 2;                        // 차량과 양측에 위치한 콘 사이 y방향 거리


// 객체 저장 포인터 클라우드 선언
    pcl::PointCloud<PointType>::Ptr clusterStopOver(new pcl::PointCloud<PointType>());


// 함수 선언
    // 현 시각 저장
    ros::Time rosTime;
    void saveCurrentTime(const sensor_msgs::PointCloud2::ConstPtr& rosCloud)
    {
        rosTime = rosCloud->header.stamp;
    }

    // map을 읽고 global_path에 저장
    std::vector<std::pair<float, float>> global_path = {};
    int curr_index = 0;
    void mapReader(const std::string& map_json) 
    {
        Json::Value root;		
        Json::Reader reader;
        std::ifstream t;
        std::string index;
        t.open(map_json);
        if (!reader.parse(t, root)) {
            cout << "Parsing Failed" << endl;
        }
        for (int k=0;k<root.size();++k){
            index = to_string(k);
            double x = root[index][0].asDouble();
            double y = root[index][1].asDouble();
            global_path.emplace_back(x, y);
        }
        curr_index = 0;
    }

    // global_path내 위치한 curr_index를 구함
    void findCurrIndex()
    {
        double min_dis = -1;
        int min_idx = 0;
        int step_size = 500;
        for (int i = std::max(curr_index-step_size, min_idx); i < (curr_index+step_size); ++i) 
        {
            double dis = std::hypot(global_path[i].first - ego_x, global_path[i].second - ego_y);
            if (min_dis > dis || min_dis == -1)
            {
                min_dis = dis;
                min_idx = i;
            }
        }
        curr_index = min_idx;
    }

    // 현 자동차의 GPS 및 해딩을 전역 상수로 저장
    double ego_x = 0;
    double ego_y = 0;
    double ego_heading_deg = 0;  // 각도
    double ego_heading_rad = 0;  // 라디안
    void setEgo(const geometry_msgs::PoseStampedConstPtr& poseMsg) 
    {   
        ego_x = poseMsg->pose.position.x;
        ego_y = poseMsg->pose.position.y;
        ego_heading_deg = poseMsg->pose.orientation.w;
        ego_heading_rad = poseMsg->pose.orientation.w * DEG_TO_RAD;
    }

    // 터널에서의 해딩값을 전역 상수로 저장
    std::tuple<double, int> setHeadingInTunnel(const pcl::PointCloud<PointType>::Ptr& input_pointCloud, 
                              const std::pair<double, double>        x_threshold, 
                              const std::pair<double, double>        y_threshold, 
                              const std::pair<double, double>        z_threshold)
    {
        pcl::PointCloud<PointType>::Ptr temp_pointCloud(new pcl::PointCloud<PointType>);
        temp_pointCloud->clear();
        for(PointType& point : input_pointCloud->points){
            if(x_threshold.first < point.x && point.x < x_threshold.second &&
                y_threshold.first < point.y && point.y < y_threshold.second &&
                z_threshold.first < point.z && point.z < z_threshold.second)
            {
                temp_pointCloud->points.push_back(point);
            }
        }

    // 다운샘플링을 진행합니다.
        pcl::VoxelGrid<PointType> voxelGrid;
        voxelGrid.setInputCloud(temp_pointCloud);
        voxelGrid.setLeafSize(0.1, 0.1, 0.1);
        voxelGrid.filter(*temp_pointCloud);

    // 8개의 벡터를 생성하여 각 영역의 포인트들을 저장합니다.
        std::unique_ptr<std::vector<pcl::PointCloud<PointType>>> regions (new std::vector<pcl::PointCloud<PointType>>(8));
        for (PointType& point : temp_pointCloud->points)
        {
            int regionIndex = 0;
            if (point.y < -1) {
                // x 값에 따른 인덱스 결정 (0-2.5, 2.5-5, 5-7.5, 7.5-10)
                if (point.x >= 0 && point.x < 2.5) {
                    regionIndex = 0;
                }
                if (point.x >= 1.25 && point.x < 3.75) {
                    regionIndex = 1;
                }
                if (point.x >= 2.5 && point.x < 5) {
                    regionIndex = 2;
                }
                if (point.x >= 3.75 && point.x < 6.25) {
                    regionIndex = 3;
                }
                if (point.x >= 5 && point.x < 7.5) {
                    regionIndex = 4;
                }
                if (point.x >= 6.25 && point.x < 8.75) {
                    regionIndex = 5;
                }
                if (point.x >= 7.5 && point.x <= 10) {
                    regionIndex = 6;
                }
                // 해당 영역에 포인트 추가
                regions->at(regionIndex).push_back(point);
            }
        }

    // 각 영역별 선형회귀를 진행합니다.
        std::shared_ptr<std::vector<float>> x_values = std::make_shared<std::vector<float>>();
        std::shared_ptr<std::vector<float>> y_values = std::make_shared<std::vector<float>>();
        std::shared_ptr<std::vector<double>> coeffs_values = std::make_shared<std::vector<double>>();
        std::shared_ptr<std::vector<double>> coeffs_values_y = std::make_shared<std::vector<double>>();
        
        for (size_t i = 0; i < regions->size(); ++i)
        {
            x_values->clear();
            y_values->clear();
            pcl::PointCloud<PointType>& region_cloud = (*regions)[i];
            for (PointType& point : region_cloud)
            {
                x_values->push_back(point.x);
                y_values->push_back(point.y);
            }

            if (x_values->size() <= 2)
            {
                continue;
            }

            const int n = x_values->size();
            Eigen::MatrixXd X(n, 2);
            Eigen::VectorXd Y(n);

            for (int i = 0; i < n; ++i)
            {
                X(i, 0) = (*x_values)[i];
                X(i, 1) = 1;
                Y(i)    = (*y_values)[i];
            }
            
            Eigen::VectorXd coeffs = X.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Y);
            coeffs_values->push_back(coeffs[0]);
            coeffs_values_y->push_back(coeffs[1]);
        }

    // 터널인지 판단 및 진입 혹은 퇴장 판단
        std::shared_ptr<std::vector<double>> region_dis = std::make_shared<std::vector<double>>();
        double sum_distance = 0.0;
        for (size_t i = 0; i < regions->size(); ++i)
        {
            pcl::PointCloud<PointType>& region_cloud = (*regions)[i];
            for (PointType& point : region_cloud)
            {
                // 완벽히 거리를 구하기 보다는 대략적으로 구하는게 가볍기도 하고, 터널 안에서 기울기가 +-5도 내외라 괜찮을거라고 판단
                sum_distance += std::abs(point.y - ((*coeffs_values)[i]*point.x + (*coeffs_values_y)[i]));
            }
            sum_distance /= region_cloud.size();
            region_dis->push_back(sum_distance);
        }

        unsigned int tunnelCount = 0;
        unsigned int tunnelLoc = 0;
        for (size_t i = 0; i < region_dis->size(); ++i)
        {
            if (region_dis->at(i) <= 0.7)
            {
                tunnelCount++;
                tunnelLoc += pow(10, i);
            }
        }
        if (tunnelCount < 5)
        {
            return {-1, tunnelLoc};
        }

    // 총합 기울기 구하기
        double coeffs_sum = 0;
        if (coeffs_values->size() >= 3)
        {
            std::sort(coeffs_values->begin(), coeffs_values->end());
            for (int i = 1; i < coeffs_values->size()-1; ++i)  // 8개 중 최소 1개, 최대 1개를 제외하고 중간 6개를 합산
            { 
                coeffs_sum += atan((*coeffs_values)[i]);
            }
            coeffs_sum /= (coeffs_values->size()-2);
            return {coeffs_sum, tunnelLoc};
        }
        else
        {
            for (int i = 0; i < coeffs_values->size(); ++i)
            { 
                coeffs_sum += atan((*coeffs_values)[i]);
            }
            coeffs_sum /= coeffs_values->size();
            return {coeffs_sum, tunnelLoc};
        }
    }

    // 한 포인트의 좌표를 상대에서 절대로 변환
    void transformRelToAbs(PointType& point)
    {
        double tempX = cos(ego_heading_rad) * point.x - sin(ego_heading_rad) * point.y + ego_x;
        double tempY = sin(ego_heading_rad) * point.x + cos(ego_heading_rad) * point.y + ego_y;
        
        point.x = tempX;
        point.y = tempY;
    }


    // 한 포인트의 좌표를 절대에서 상대로 변환
    void transformAbsToRel(PointType& point)
    {
        double tempX = cos(ego_heading_rad) * (point.x - ego_x) + sin(ego_heading_rad) * (point.y - ego_y);
        double tempY = -sin(ego_heading_rad) * (point.x - ego_x) + cos(ego_heading_rad) * (point.y - ego_y);
        
        point.x = tempX;
        point.y = tempY;
    }

    // markerArray 초기화
    void deleteAllMarkers(const std::shared_ptr<visualization_msgs::MarkerArray>& markerArray) {
        // 모든 마커의 액션을 DELETE로 설정
        for (size_t i = 0; i < markerArray->markers.size(); ++i) {
            markerArray->markers[i].action = visualization_msgs::Marker::DELETE;
        }
        markerArray->markers.clear();
    }


    // publish 실행
    void publisher(const pcl::PointCloud<PointType>::Ptr& pclCloud, const ros::Publisher& publisher, const std::string frame_id)
    {
        std::unique_ptr<sensor_msgs::PointCloud2> tempCloud(new sensor_msgs::PointCloud2());
        pcl::toROSMsg(*pclCloud, *tempCloud);
        tempCloud->header.frame_id = frame_id;
        tempCloud->header.stamp = rosTime;
        publisher.publish(*tempCloud);
	}


    // markerarray용 publish 실행
    void publisherMarkerArray(const std::shared_ptr<visualization_msgs::MarkerArray>& markerArray, const ros::Publisher& publisher, const std::string frame_id)
    {
        for (auto& marker : markerArray->markers) 
        {
            marker.header.frame_id = frame_id;
            marker.header.stamp = rosTime;
        }
        publisher.publish(*markerArray);
	}


// 칼만필터 클래스 선언
    KalmanFilter::KalmanFilter() 
    {
        identity = Eigen::Matrix2f::Identity();
        errorCovariance = Eigen::Matrix2f::Identity();
        measurementNoise = Eigen::Matrix2f::Identity() * 0.1;
        processNoise = Eigen::Matrix2f::Identity() * 0.1;
    }

    void KalmanFilter::init(Eigen::Vector2f initialState) 
    {
        state = initialState;
    }

    void KalmanFilter::predict() 
    {
        errorCovariance = errorCovariance + processNoise;
    }

    void KalmanFilter::update(Eigen::Vector2f measurement) 
    {
        Eigen::Matrix2f kalmanGain = errorCovariance * (errorCovariance + measurementNoise).inverse();
        state = state + kalmanGain * (measurement - state);
        errorCovariance = (identity - kalmanGain) * errorCovariance;
    } 