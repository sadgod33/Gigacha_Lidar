#ifndef CLUSTER_H
#define CLUSTER_H

#include "include.h"
#include "utility.h"
#include "preprocessor.h"


class Clustering{
public:
    Preprocessor preprocessor;
    KalmanFilter kalman;
    HungarianAlgorithm HungAlgo;
    void clusterObject(const pcl::PointCloud<PointType>::Ptr& input_pointCloud, const std::shared_ptr<visualization_msgs::MarkerArray>& smallObject_MarkerArray, const std::shared_ptr<visualization_msgs::MarkerArray>& bigObject_MarkerArray, const pcl::PointCloud<PointType>::Ptr& debug_pointCloud);
    void clusterCone(const pcl::PointCloud<PointType>::Ptr& input_pointCloud, const pcl::PointCloud<PointType>::Ptr& output_pointCloud);
    void saveCluster(const pcl::PointCloud<PointType>::Ptr& input_pointCloud, const pcl::PointCloud<PointType>::Ptr& output_pointCloud);
    void identifyLRcone_v2(const pcl::PointCloud<PointType>::Ptr& input_pointCloud, const pcl::PointCloud<PointType>::Ptr& output_Rcone_pointCloud, const pcl::PointCloud<PointType>::Ptr& output_Lcone_pointCloud, const std::shared_ptr<visualization_msgs::MarkerArray>&  debug_Rcone_pointCloud, const std::shared_ptr<visualization_msgs::MarkerArray>&  debug_Lcone_pointCloud);
    void setConeROI(const pcl::PointCloud<PointType>::Ptr& input_pointCloud, const pcl::PointCloud<PointType>::Ptr& output_pointCloud, const std::shared_ptr<visualization_msgs::MarkerArray>& ROI_MarkerArray);
    void identifyLRcone(const pcl::PointCloud<PointType>::Ptr& input_pointCloud, pcl::PointCloud<PointType>::Ptr& output_Rcone_pointCloud, pcl::PointCloud<PointType>::Ptr& output_Lcone_pointCloud, pcl::PointCloud<PointType>::Ptr& debug_pointCloud);
    //void linearGreressionLRcone(const pcl::PointCloud<PointType>::Ptr& input_pointCloud, pcl::PointCloud<PointType>::Ptr& output_Lcone_pointCloud, pcl::PointCloud<PointType>::Ptr& output_Rcone_pointCloud);
    static bool doIntersect(PointType p1, PointType q1, PointType p2, PointType q2, double L);
private:
    static double distance(const PointType& p1, const PointType& p2);
    static int orientation(const PointType& p, const PointType& q, const PointType& r);
};

#endif  //CLUSTER_H