#include "../include/utility.h"
#include "pointcloud_generator.h"

using namespace std;
using namespace cv;
using namespace message_filters;

class FusionNode {
private:
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub;
    message_filters::Subscriber<sensor_msgs::Image> image_sub;
    message_filters::Subscriber<vision_msgs::Detection2DArray> dynamic_object_sub;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image, vision_msgs::Detection2DArray> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;

    ros::Publisher pubVisionMarkerArray;
    visualization_msgs::MarkerArray visionMarkerArray;

    cv::Mat real_K_MAT;
    cv::Mat TransformMat;

    int c_idx;
    G_CMD g_cmd;

public:
    FusionNode(G_CMD& g_cmd, int& camera_idx, string& image_topic, string& box_topic, string& marker_topic_name): nh("~"), g_cmd(g_cmd) 
    {
        pubVisionMarkerArray = nh.advertise<visualization_msgs::MarkerArray>(marker_topic_name, 1);
        
        pointcloud_sub.subscribe(nh, "/os_cloud_node/points", 10);
        image_sub.subscribe(nh, image_topic, 5);
        dynamic_object_sub.subscribe(nh, box_topic, 5);
        c_idx = camera_idx;

        ParamsLidar params_lidar;
        CameraParams params_camera;
        params_camera.WIDTH = g_cmd.cam[0];
        params_camera.HEIGHT = g_cmd.cam[1];

        // 카메라 파라미터
        real_K_MAT = (cv::Mat_<double>(3, 3) << g_cmd.focal_length[0], 0, params_camera.WIDTH / 2, 0, g_cmd.focal_length[1], params_camera.HEIGHT / 2, 0, 0, 1);
        // 라이다 파라미터
        TransformMat = matrixChanger(g_cmd, params_lidar);

        sync.reset(new Sync(MySyncPolicy(50), pointcloud_sub, image_sub, dynamic_object_sub));
        sync->registerCallback(std::bind(&FusionNode::callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        clearMemory();
    }

    void callback(const sensor_msgs::PointCloud2ConstPtr& pointcloud,
                  const sensor_msgs::ImageConstPtr& image,
                  const vision_msgs::Detection2DArrayConstPtr& dynamicObject) {
        cout << "fusion_object_is_working-> " << c_idx << endl;
        auto start = std::chrono::high_resolution_clock::now();
        detectObject(pointcloud, image, dynamicObject, g_cmd, visionMarkerArray);
        publishPointCloud();
        clearMemory();
        auto finish = std::chrono::high_resolution_clock::now();

        std::chrono::duration<double, std::milli> elapsed = finish - start;
        std::cout << "fusion_object time: " << elapsed.count() << " ms\n";
    }

    void clearMemory() {
        visionMarkerArray.markers.clear();
    }

    void publishPointCloud() {
        pubVisionMarkerArray.publish(visionMarkerArray);
    }

    // --------------------------- 센서퓨전용 변환 행렬 함수 ----------------------------
    cv::Mat getRotMat(vector<double> RPY) 
    {
        double cosR = cos(RPY[0]);
        double cosP = cos(RPY[1]);
        double cosY = cos(RPY[2]);
        double sinR = sin(RPY[0]);
        double sinP = sin(RPY[1]);
        double sinY = sin(RPY[2]);

        cv::Mat rotRoll = (cv::Mat_<double>(3, 3) << 1, 0,    0,
                                                     0, cosR, -sinR,
                                                     0, sinR, cosR);
        cv::Mat rotPitch = (cv::Mat_<double>(3, 3) << cosP,  0, sinP,
                                                      0,     1, 0,
                                                      -sinP, 0, cosP);
        cv::Mat rotYaw = (cv::Mat_<double>(3, 3) << cosY, -sinY, 0,
                                                    sinY, cosY,  0,
                                                    0,    0,     1);
        cv::Mat rotMat = rotYaw * rotPitch * rotRoll;
        return rotMat;
    }

    cv::Mat matrixChanger(G_CMD& g_cmd, ParamsLidar& params_lidar) 
    {
        std::vector<double> camRPY = g_cmd.rvec;
        camRPY[0] += -1.570796;  // -90 degrees in radians
        camRPY[2] += -1.570796;  // -90 degrees in radians

        cv::Mat camRot = getRotMat(camRPY);
        cv::Mat camTransl = (cv::Mat_<double>(3, 1) << g_cmd.t_mat[0], g_cmd.t_mat[1], g_cmd.t_mat[2]);

        cv::Mat Tr_cam_to_vehicle = cv::Mat::zeros(4, 4, CV_64F);
        hconcat(camRot, camTransl, Tr_cam_to_vehicle);
        cv::Mat bottomRow = (cv::Mat_<double>(1, 4) << 0, 0, 0, 1);
        Tr_cam_to_vehicle.push_back(bottomRow);

        std::vector<double> lidarRPY = {0, 0, 0};
        cv::Mat lidarRot = getRotMat(lidarRPY);
        cv::Mat lidarTransl = (cv::Mat_<double>(3, 1) << params_lidar.X, params_lidar.Y, params_lidar.Z);

        cv::Mat Tr_lidar_to_vehicle = cv::Mat::zeros(4, 4, CV_64F);
        hconcat(lidarRot, lidarTransl, Tr_lidar_to_vehicle);
        Tr_lidar_to_vehicle.push_back(bottomRow);

        cv::Mat invTr = Tr_cam_to_vehicle.inv(cv::DECOMP_SVD);
        cv::Mat Tr_lidar_to_cam = invTr * Tr_lidar_to_vehicle;

        return Tr_lidar_to_cam;
    }

    cv::Mat transformLiDARToCamera(const cv::Mat& TransformMat, const cv::Mat& pc_lidar) 
    {
        cv::Mat TransformMat_float;
        TransformMat.convertTo(TransformMat_float, CV_32F); // Convert TransformMat to float
        cv::Mat cam_temp = TransformMat_float * pc_lidar;
        if (cam_temp.rows >= 3) {
            cam_temp = cam_temp.rowRange(0, 3);
        } else {
            std::cerr << "Error: cam_temp does not have enough rows." << std::endl;
            return cv::Mat(); // 빈 행렬 반환
        }
        return cam_temp;
    }

    cv::Mat transformCameraToImage(const cv::Mat& CameraMat, const cv::Mat& pc_camera) 
    {
        cv::Mat CameraMat_float;
        CameraMat.convertTo(CameraMat_float, CV_32F); // Convert CameraMat to float
        cv::Mat img_temp = CameraMat_float * pc_camera;

        for (int i = 0; i < img_temp.cols; ++i) {
            img_temp.at<float>(0, i) /= img_temp.at<float>(2, i);
            img_temp.at<float>(1, i) /= img_temp.at<float>(2, i);
            img_temp.at<float>(2, i) /= img_temp.at<float>(2, i);
        }
        return img_temp;
    }

    pair<float, vector<float>> calc_distance_position2(const vector<vector<float>>& points) 
    {
        cv::Mat mat_points(static_cast<int>(points.size()), static_cast<int>(points[0].size()), CV_32F);
        for (size_t i = 0; i < points.size(); ++i) {
            for (size_t j = 0; j < points[i].size(); ++j) {
                mat_points.at<float>(i, j) = points[i][j];
            }
        }

        mat_points = mat_points.t();
        vector<float> position;
        int index = 0;
        cv::minMaxIdx(mat_points.row(0), 0, 0, &index);
        position.push_back(mat_points.at<float>(0, index));
        position.push_back(mat_points.at<float>(1, index));
        position.push_back(mat_points.at<float>(2, index));
        cv::Mat tmp_position = cv::Mat(position);
        float dist = cv::norm(tmp_position);
        return {dist, position};
    }

    pair<vector<int>, vector<float>> remove_outliers(const vector<float>& arr) 
    {
        vector<int> index;
        vector<float> things;
        vector<float> deleted;
        vector<float> sorted_arr = arr;
        sort(sorted_arr.begin(), sorted_arr.end());
        float q1 = sorted_arr[sorted_arr.size() * 0.45];
        float q3 = sorted_arr[sorted_arr.size() * 0.55];
        for (size_t i = 0; i < arr.size(); i++) {
            if (arr[i] >= q1 && arr[i] <= q3) {
                things.push_back(arr[i]);
            } else {
                deleted.push_back(arr[i]);
                index.push_back(i);
            }
        }
        return {index, things};
    }

    void detectObject(const sensor_msgs::PointCloud2::ConstPtr& ouster, const sensor_msgs::ImageConstPtr& image, const vision_msgs::Detection2DArray::ConstPtr& dynamicObject, G_CMD& g_cmd, visualization_msgs::MarkerArray& visionMarkerArray)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        vector<vector<float>> point_list;
        if (ouster->data.empty())
        {
            std::cout << "(note) Empty cloud" << std::endl;
        } 
        else 
        {
            for (sensor_msgs::PointCloud2ConstIterator<float> it(*ouster, "x"); it != it.end(); ++it) {
                if (-150 < atan2(it[0], it[1]) * RAD_TO_DEG && atan2(it[0], it[1]) * RAD_TO_DEG < 150) {
                    point_list.push_back({it[0], it[1], it[2], 1.0f});
                }   
            }
        }

        if (point_list.empty()) {
            std::cerr << "Error: point_list is empty." << std::endl;
            return;
        }

        cv::Mat pc_np(point_list.size(), 4, CV_32F);
        for (size_t i = 0; i < point_list.size(); ++i) {
            memcpy(pc_np.ptr<float>(i), point_list[i].data(), 4 * sizeof(float));
        }

        cv::Mat filtered_xyz_p = pc_np.colRange(0, 3);
        cv::Mat xyz_p = pc_np.colRange(0, 4).t();
        cv::Mat xyz_c = transformLiDARToCamera(TransformMat, xyz_p);
        cv::Mat xy_i = transformCameraToImage(real_K_MAT, xyz_c);

        xy_i.convertTo(xy_i, CV_32S);

        xy_i = xy_i.t();

        vector<visualization_msgs::Marker> box_list;
        for (const auto& detection : dynamicObject->detections) {
            visualization_msgs::Marker bbox;
            bbox.header = detection.header;
            bbox.pose.position.x = detection.bbox.center.x;
            bbox.pose.position.y = detection.bbox.center.y;
            bbox.scale.x = detection.bbox.size_x;
            bbox.scale.y = detection.bbox.size_y;
            bbox.ns = "o";

            box_list.push_back(bbox);
        }
        
        for (size_t i = 0; i < box_list.size(); ++i) {
            vector<vector<float>> inner_3d_point;
            int scale_x = box_list[i].scale.x;
            int scale_y = box_list[i].scale.y;
            int left_low_x = box_list[i].pose.position.x - scale_x / 2;
            int left_low_y = box_list[i].pose.position.y - scale_y / 2;
            int right_high_x = box_list[i].pose.position.x + scale_x / 2;
            int right_high_y = box_list[i].pose.position.y + scale_y / 2;

            for (int k = 0; k < xy_i.rows; ++k) {
                if (xy_i.at<int>(k, 0) > (left_low_x + 0.45 * scale_x) && xy_i.at<int>(k, 0) < (right_high_x - 0.45 * scale_x) &&
                    xy_i.at<int>(k, 1) > (left_low_y + 0.1 * scale_y) && xy_i.at<int>(k, 1) < (right_high_y - 0.5 * scale_y)) {
                    inner_3d_point.push_back({filtered_xyz_p.at<float>(k, 0), filtered_xyz_p.at<float>(k, 1), filtered_xyz_p.at<float>(k, 2)});
                }
            }

            if (!inner_3d_point.empty()) {
                auto [dist, position] = calc_distance_position2(inner_3d_point);
                visualization_msgs::Marker tmp_pd;
                tmp_pd.header.frame_id = "base_link";
                tmp_pd.header.stamp = ros::Time::now();
                tmp_pd.pose.position.x = position[0];
                tmp_pd.pose.position.y = position[1];
                tmp_pd.pose.position.z = position[2];

                tmp_pd.scale.x = 0.2;
                tmp_pd.scale.y = 0.3;
                tmp_pd.scale.z = 1.0;
                tmp_pd.color.r = 1.0;
                tmp_pd.color.g = 0.0;
                tmp_pd.color.b = 0.0;
                tmp_pd.color.a = 1.0;
                tmp_pd.type = visualization_msgs::Marker::CUBE;
                tmp_pd.action = visualization_msgs::Marker::ADD;

                visionMarkerArray.markers.push_back(tmp_pd);
            }
        }
    }
};


int main(int argc, char** argv) {
    // 메인 프로세스로부터 값 받기
    int camera_idx = 0;
    G_CMD g_cmd;
    uint32_t image_topic_size, box_topic_size, marker_topic_size;
    char image_topic_buffer[256], box_topic_buffer[256], marker_topic_buffer[256];

    // 인덱스
    read(STDIN_FILENO, &camera_idx, sizeof(camera_idx));

    // rvec
    g_cmd.rvec.resize(3);
    read(STDIN_FILENO, g_cmd.rvec.data(), 3 * sizeof(double));
    // t_mat
    g_cmd.t_mat.resize(3);
    read(STDIN_FILENO, g_cmd.t_mat.data(), 3 * sizeof(double));
    // focal_length
    g_cmd.focal_length.resize(2);
    read(STDIN_FILENO, g_cmd.focal_length.data(), 2 * sizeof(double));
    // cam
    g_cmd.cam.resize(2);
    read(STDIN_FILENO, g_cmd.cam.data(), 2 * sizeof(int));

    // string 토픽
    read(STDIN_FILENO, &image_topic_size, sizeof(uint32_t));
    read(STDIN_FILENO, image_topic_buffer, image_topic_size);
    image_topic_buffer[image_topic_size] = '\0'; // Null-terminate the string

    read(STDIN_FILENO, &box_topic_size, sizeof(uint32_t));
    read(STDIN_FILENO, box_topic_buffer, box_topic_size);
    box_topic_buffer[box_topic_size] = '\0'; // Null-terminate the string

    read(STDIN_FILENO, &marker_topic_size, sizeof(uint32_t));
    read(STDIN_FILENO, marker_topic_buffer, marker_topic_size);
    marker_topic_buffer[marker_topic_size] = '\0'; // Null-terminate the string

    string image_topic(image_topic_buffer);
    string box_topic(box_topic_buffer);
    string marker_topic_name(marker_topic_buffer);

    // ROS 실행
    std::string node_name = "fusion_object_" + std::to_string(camera_idx);
    ros::init(argc, argv, node_name);

    FusionNode node(g_cmd, camera_idx, image_topic, box_topic, marker_topic_name);
    ros::spin();

    return 0;
}