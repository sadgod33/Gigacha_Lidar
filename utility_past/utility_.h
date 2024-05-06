#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_

#include "include.h"

using namespace std;

extern std::vector<std::pair<float, float>> global_path;
extern unsigned int curr_index;

void map_reader();

typedef pcl::PointXYZI PointType;

extern const string pointCloudTopic;
extern const string imuTopic;

// Save pcd
extern const string fileDirectory;

// Using velodyne cloud "ring" channel for image projection (other lidar may have different name for this channel, change "PointXYZIR" below)
extern const bool useCloudRing;

extern const double pi;

// Ouster OS1-64
// for our ouster-32
extern const int N_SCAN;
extern const int Horizon_SCAN;
extern const float ang_res_x;
extern const float ang_res_y;
extern const float ang_bottom;
extern const int groundScanInd;

extern const bool loopClosureEnableFlag;
extern const double mappingProcessInterval;

extern const float scanPeriod;
extern const int systemDelay;
extern const int imuQueLength;

extern const float sensorMinimumRange;
extern const float sensorMountAngle;
extern const float segmentTheta;
extern const int segmentValidPointNum;
extern const int segmentValidLineNum;

extern const float minimun_dist_x;
extern const float minimun_dist_y;
extern const float segmentAlphaX;
extern const float segmentAlphaY;

extern const int coneMin;
extern const int coneMax;
extern const float coneEps;

extern const int edgeFeatureNum;
extern const int surfFeatureNum;
extern const int sectionsTotal;
extern const float edgeThreshold;
extern const float surfThreshold;
extern const float nearestFeatureSearchSqDist;

extern const float emergencyRange;

extern const float surroundingKeyframeSearchRadius;
extern const int surroundingKeyframeSearchNum;
extern const float historyKeyframeSearchRadius;
extern const int historyKeyframeSearchNum;
extern const float historyKeyframeFitnessScore;

extern const float globalMapVisualizationSearchRadius;

extern const double DEG_TO_RAD;

struct OusterPointXYZIRT {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t noise;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint32_t, t, t) (uint16_t, reflectivity, reflectivity)
    (uint8_t, ring, ring) (uint16_t, noise, noise) (uint32_t, range, range)
)


struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;


POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time)
)


typedef PointXYZIRPYT  PointTypePose;

std::tuple<double, double, double, double> euler_to_quaternion(const double &theta);

double endpoint_distance(const PointType &point, const std::pair<double, double> minpoint, const std::pair<double, double> maxpoint);

std::tuple<double, double> find_position(const pcl::PointCloud<PointType> &cloud, double threshold, double min_y, double max_y, double theta);

bool compare(std::pair<float, float> a, std::pair<float, float> b);

double variance_criterion(const cv::Mat& c1, const cv::Mat& c2);

std::tuple<double, double, double> get_best_theta(const pcl::PointCloud<PointType> &cloud);

visualization_msgs::Marker bbox_3d(const pcl::PointCloud<PointType> &cloud, const double &heading, const double &enu_x,const double &enu_y, const int &Id);


#endif  //_UTILITY_LIDAR_ODOMETRY_H_