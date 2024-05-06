#include "utility.h"

extern std::vector<std::pair<float, float>> global_path;
extern unsigned int curr_index;

void map_reader() {
        Json::Value root;		
        Json::Reader reader;
        std::ifstream t;
        string index;
        t.open("/home/vision/lidar_ws/src/jaejun/maps/obs_small_gwangjang.json");
        if (!reader.parse(t, root)) {
            cout << "Parsing Failed" << endl;
        }
        for (int k=0;k<root.size();++k){
            index = to_string(k);
            double x = root[index][0].asDouble();
            double y = root[index][1].asDouble();
            global_path.emplace_back(x, y);
        }
}


const string pointCloudTopic = "/os_cloud_node/points";
const string imuTopic = "/imu/data";

// Save pcd
const string fileDirectory = "/tmp/";

const bool useCloudRing = true;

const double pi = 3.14159265;

const int N_SCAN = 32;
const int Horizon_SCAN = 1024;
const float ang_res_x = 360.0/float(Horizon_SCAN);
const float ang_res_y = 45/float(N_SCAN-1);
const float ang_bottom = 20+0.1;
const int groundScanInd = 15;

const bool loopClosureEnableFlag = false;
const double mappingProcessInterval = 0.3;

const float scanPeriod = 0.1;
const int systemDelay = 0;
const int imuQueLength = 200;

const float sensorMinimumRange = 1.0;
const float sensorMountAngle = 0.0;
const float segmentTheta = 15.0/180.0*M_PI;
const int segmentValidPointNum = 5;
const int segmentValidLineNum = 2;

const float minimun_dist_x = 50.0;
const float minimun_dist_y = 0.15;
const float segmentAlphaX = ang_res_x / 180.0 * M_PI;
const float segmentAlphaY = ang_res_y / 180.0 * M_PI;

const int coneMin = -1;
const int coneMax = -1;
const float coneEps = -1;

const int edgeFeatureNum = 2;
const int surfFeatureNum = 4;
const int sectionsTotal = 6;
const float edgeThreshold = 0.1;
const float surfThreshold = 0.1;
const float nearestFeatureSearchSqDist = 25;

const float emergencyRange = 5.0;

const float surroundingKeyframeSearchRadius = 50.0;
const int   surroundingKeyframeSearchNum = 50;
const float historyKeyframeSearchRadius = 7.0;
const int   historyKeyframeSearchNum = 25;
const float historyKeyframeFitnessScore = 0.3;

const float globalMapVisualizationSearchRadius = 500.0;

const double DEG_TO_RAD = M_PI / 180.0;

std::tuple<double, double, double, double> euler_to_quaternion(const double &theta){
    double yaw = M_PI/2 + theta;
    double cr = cos(0);
    double sr = sin(0);
    double cp = cos(0);
    double sp = sin(0);
    double cy = cos(yaw*0.5);
    double sy = sin(yaw*0.5);
    
    double ori_w = cr * cp * cy + sr * sp * sy;
    double ori_x = sr * cp * cy - cr * sp * sy;
    double ori_y = cr * sp * cy + sr * cp * sy;
    double ori_z = cr * cp * sy - sr * sp * cy;    
    return std::make_tuple(ori_x, ori_y, ori_z, ori_w);
}

double endpoint_distance(const PointType &point, const std::pair<double, double> minpoint, const std::pair<double, double> maxpoint){
    //Point 에는 cluster안에 들어있는 모든 점의 정보를 담고 있어야 하고
    double dx = maxpoint.first - minpoint.first;
    double dy = maxpoint.second - minpoint.second;
    double diag_dist = sqrt(dx*dx + dy*dy);
    // y = ax + c 여기서 점과 직선 사이의 거리를 반환하면
    double a = (maxpoint.second - minpoint.second)/(maxpoint.first - minpoint.first);
    double c = maxpoint.second - a*maxpoint.first;
    double dist = abs(a*point.x - point.y + c)/sqrt(a*a + 1); //여기 point 받아오는 부분 수정
    return dist;
    }

std::tuple<double, double> find_position(const pcl::PointCloud<PointType> &cloud, double threshold, double min_y, double max_y, double theta){

    double dist_max = 0.0;
    size_t max_dist_ind = 0;
    size_t end = cloud.points.size();
    
    std::pair<double, double> minpoint;
    std::pair<double, double> maxpoint;
    double position_x, position_y;
    for (size_t i=0; i<end; i++){
        if (cloud.points[i].y == min_y){
            minpoint.first = cloud.points[i].x; 
            minpoint.second = cloud.points[i].y;
        }
        if (cloud.points[i].y == max_y){
            maxpoint.first = cloud.points[i].x;
            maxpoint.second = cloud.points[i].y;
        }
    }

    for (size_t i = 0; i<end; i++){
        double dist = endpoint_distance(cloud.points[i], minpoint, maxpoint);
        if (dist > dist_max){
            max_dist_ind =i;
            dist_max = dist;
        } 
    }

    if (dist_max > threshold){
        // slop = (minpoint.first - cloud.points[max_dist_ind].x)/(minpoint.second-cloud.points[max_dist_ind].y);
        position_x = cloud.points[max_dist_ind].x, position_y = cloud.points[max_dist_ind].y;
        
    }
    else
    {
        // slop = FLT_MAX;
        position_x = (maxpoint.first + minpoint.first)/2, position_y = (maxpoint.second + minpoint.second)/2;
    }

    return std::make_tuple(position_x, position_y); // 나중에 slop를 orientation으로 바꾸어 주는 코드 추가 필요
}

bool compare(std::pair<float, float> a, std::pair<float, float> b){
    return a.first < b.first;
}

double variance_criterion(const cv::Mat& c1, const cv::Mat& c2){

    std::vector<double> c1_deep;
    std::vector<double> c2_deep;

    for (size_t i = 0; i<c1.rows; ++i){
        for (size_t j =0; j<c1.cols; ++j){
            c1_deep.push_back(c1.at<double>(i,j));
            c2_deep.push_back(c2.at<double>(i,j));
        }
    }
    sort(c1_deep.begin(), c1_deep.end());
    sort(c2_deep.begin(), c2_deep.end());

    int n_c1 = c1_deep.size();
    int n_c2 = c2_deep.size();

    double c1_min = c1_deep[0];
    double c2_min = c2_deep[0];

    double c1_max = c1_deep[n_c1 - 1];
    double c2_max = c2_deep[n_c2 - 1];

    std::vector<double> D1;
    for (double ic1 : c1_deep){
        double dist1 = std::min(sqrt(pow((c1_max - ic1), 2)), sqrt(pow((ic1 - c1_min), 2)));
        D1.push_back(dist1);
    }

    std::vector<double> D2;
    for (double ic2 : c2_deep){
        double dist2 = std::min(sqrt(pow((c2_max - ic2), 2)), sqrt(pow((ic2 - c2_min), 2)));
        D2.push_back(dist2);
    }

    std::vector<double> E1, E2;
    for (size_t i=0; i<D1.size();++i){
        double d1 = D1[i];
        double d2 = D2[i];
        if (d1<d2){
            E1.push_back(d1);
        }
        else{
            E2.push_back(d2);
        }
    }
    //분산 계산하기
    double V1 = 0.0;
    if (!E1.empty()){
        double mean1 = 0.0;
        for (double e1 : E1){
            mean1 += e1;
        }
        mean1 /=E1.size();

        for (double e1 : E1){
            V1+= std::pow(e1-mean1,2);
        }
        V1 = -V1/E1.size();
    }

    double V2 = 0.0;
    if (!E2.empty()){
        double mean2 = 0.0;
        for (double e2 : E2){
            mean2 += e2;
        }
        mean2 /=E2.size();

        for (double e2 : E2){
            V2+= std::pow(e2-mean2,2);
        }
        V2 = -V2/E2.size();
    }
    //더해서 cost 계산
    double gamma = V1+V2;
    return gamma;
}

std::tuple<double, double, double> get_best_theta(const pcl::PointCloud<PointType> &cloud){

    cv::Mat Matrix_pts = cv::Mat::zeros(cloud.points.size(), 2, CV_64FC1);
    double mean_y = 0;
    double mean_x = 0;
    for (size_t  i=0;i<cloud.size();++i){
        Matrix_pts.at<double>(i,0) = cloud.points[i].x;
        Matrix_pts.at<double>(i,1) = cloud.points[i].y;
        if (cloud.points[i].z < 1.3){
            mean_y += cloud.points[i].y;
            mean_x += cloud.points[i].x;
        }

    }
    mean_x = mean_x / cloud.size();
    mean_y = mean_y / cloud.size();
    // std::cout << mean_x << " : " << mean_y << std::endl;
    double dtheta = M_PI/180;
    double minimal_cost = (-1.0)*FLT_MAX;
    double best_theta = FLT_MAX;
    double second_theta = FLT_MAX;
    double ori_x, ori_y, ori_z, ori_w;
    cv::Mat e1 = cv::Mat::zeros(1,2,CV_64FC1);
    cv::Mat e2 = cv::Mat::zeros(1,2,CV_64FC1);

    for (size_t k=0;k<89;++k){
        double theta = k*M_PI/180;
        double cost = (-1.0)*FLT_MAX;
        if (theta < 89*M_PI/180){
            e1.at<double>(0,0) = cos(theta);
            e1.at<double>(0,1) = sin(theta);
            e2.at<double>(0,0) = -sin(theta);
            e2.at<double>(0,1) = cos(theta);

            cv::Mat c1 = Matrix_pts*e1.t();
            cv::Mat c2 = Matrix_pts*e2.t();

            cost = variance_criterion(c1, c2);
            if (minimal_cost < cost){
                minimal_cost = cost;
                second_theta = best_theta;
                best_theta = theta;
            }
        }
    }
    if (best_theta == FLT_MAX){
        best_theta = -M_PI/2;}
    

    return std::make_tuple(best_theta, mean_x, mean_y);
}

// visualization_msgs::Marker bbox_3d(const pcl::PointCloud<PointType> &cloud)
visualization_msgs::Marker bbox_3d(const pcl::PointCloud<PointType> &cloud, const double &heading, const double &enu_x,const double &enu_y, const int &Id)
{
    pcl::PointXYZI minPoint, maxPoint;
    pcl::getMinMax3D(cloud, minPoint, maxPoint);
    visualization_msgs::Marker marker;
    // if ((maxPoint.x - minPoint.x > 0.2) && (maxPoint.y - minPoint.y < 3.0) ){


    double center_x = (maxPoint.x + minPoint.x)/2;
    double center_y = (maxPoint.y + minPoint.y)/2;
    double center_z = (maxPoint.z + minPoint.z)/2;

    double length_x = fabs(maxPoint.x - minPoint.x);
    double length_y = fabs(maxPoint.y - minPoint.y);
    double length_z = fabs(maxPoint.z - minPoint.z);
    
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();
    
    marker.id = 0;
    
    marker.type = visualization_msgs::Marker::LINE_LIST;
    
    marker.action = visualization_msgs::Marker::ADD;

    // 8개의 pos에 대해서 모두 좌표를 convert해서 geometry에 담자.
    geometry_msgs::Point p[8];
    p[0].x = center_x - length_x / 2; p[0].y = center_y - length_y / 2; p[0].z = center_z - length_z / 2;
    p[1].x = center_x + length_x / 2; p[1].y = center_y - length_y / 2; p[1].z = center_z - length_z / 2;
    p[2].x = center_x - length_x / 2; p[2].y = center_y + length_y / 2; p[2].z = center_z - length_z / 2;
    p[3].x = center_x + length_x / 2; p[3].y = center_y + length_y / 2; p[3].z = center_z - length_z / 2;
    p[4].x = center_x - length_x / 2; p[4].y = center_y - length_y / 2; p[4].z = center_z + length_z / 2;
    p[5].x = center_x + length_x / 2; p[5].y = center_y - length_y / 2; p[5].z = center_z + length_z / 2;
    p[6].x = center_x - length_x / 2; p[6].y = center_y + length_y / 2; p[6].z = center_z + length_z / 2;
    p[7].x = center_x + length_x / 2; p[7].y = center_y + length_y / 2; p[7].z = center_z + length_z / 2;
    

    for(int i=0; i<8; i++) {
        double old_x = p[i].x;
        double old_y = p[i].y;

        p[i].x = old_x*cos(-heading*M_PI/180) + old_y*sin(-heading*M_PI/180) + enu_x;
        p[i].y = -old_x*sin(-heading*M_PI/180) + old_y*cos(-heading*M_PI/180) + enu_y;
    }
    int connections[12][2]={{0,1},{1,3},{3,2},{2,0}, {4,5},{5,7},{7,6},{6,4}, {0,4},{1,5},{3,7},{2,6}};

    for(int i=0;i<12;i++){
        // For each line connection defined above,
        // add both vertices to 'points' array in Marker message.
        marker.points.push_back(p[connections[i][0]]);
        marker.points.push_back(p[connections[i][1]]);
    }
    marker.id = Id;
    marker.lifetime = ros::Duration(0.1);
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.scale.x = 0.05;
    // }        
    return marker;
}