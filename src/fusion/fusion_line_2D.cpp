#include "utility.h"
#include "pointcloud_generator.h"

using namespace std;
using namespace cv;
using namespace message_filters;

class FusionLine2DNode {
private:
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub;
    message_filters::Subscriber<sensor_msgs::Image> image_sub;
    message_filters::Subscriber<sensor_msgs::Image> seg_image_sub;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;

    ros::Publisher pubLineMarkerArray;
    visualization_msgs::MarkerArray LineMarkerArray;
    ros::Publisher pubFollowMarkerArray;
    visualization_msgs::MarkerArray FollowMarkerArray;
    ros::Publisher pubLidar;
    pcl::PointCloud<PointType>::Ptr tempPointCloud;


    cv::Mat real_K_MAT;
    cv::Mat TransformMat;
    bool wall_follow;
    float area_ratio;
    
    int c_idx;
    G_CMD g_cmd;
    
public:
    FusionLine2DNode(G_CMD& g_cmd, int& camera_idx, string& image_topic, string& seg_image_topic, string& Line_topic_name, string& Follow_topic_name): nh("~"), g_cmd(g_cmd) 
    {
        pubLineMarkerArray = nh.advertise<visualization_msgs::MarkerArray>(Line_topic_name, 1);
        pubFollowMarkerArray = nh.advertise<visualization_msgs::MarkerArray>(Follow_topic_name, 1);
        pubLidar = nh.advertise<sensor_msgs::PointCloud2>("/seg_lidar_2D", 1);

        pointcloud_sub.subscribe(nh, "/os_cloud_node/points", 10);
        image_sub.subscribe(nh, image_topic, 5);
        seg_image_sub.subscribe(nh, seg_image_topic, 5);
        c_idx = camera_idx;

        ParamsLidar params_lidar;
        CameraParams params_camera;
        params_camera.WIDTH = g_cmd.cam[0];
        params_camera.HEIGHT = g_cmd.cam[1];
        
        // 카메라 파라미터
        real_K_MAT = (cv::Mat_<double>(3, 3) << g_cmd.focal_length[0], 0, params_camera.WIDTH / 2, 0, g_cmd.focal_length[1], params_camera.HEIGHT / 2, 0, 0, 1);
        // 라이다 파라미터
        TransformMat = matrixChanger(g_cmd, params_lidar);  // Extrinsic_param
        // 경로 선정 기준 차선
        bool wall_follow = false;
        // 하단으로부터 볼 영역 비율
        area_ratio = 0.25; // 하단 1/4만 보겠다는 의미

        sync.reset(new Sync(MySyncPolicy(50), pointcloud_sub, image_sub, seg_image_sub));
        sync->registerCallback(std::bind(&FusionLine2DNode::callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        
        allocateMemory();
        clearMemory();
    }

    void allocateMemory(){
        tempPointCloud.reset(new pcl::PointCloud<PointType>());
    }


    void callback(const sensor_msgs::PointCloud2ConstPtr& pointcloud,
                  const sensor_msgs::ImageConstPtr& image,
                  const sensor_msgs::ImageConstPtr& seg_image) {
        cout << "fusion_2D-> " << c_idx << endl;
        cout << "TransformMat = " << getMatSize(TransformMat) << endl;
        cout << "real_K_MAT = " << getMatSize(real_K_MAT) << endl;
        auto start = std::chrono::high_resolution_clock::now();
        detectLine(pointcloud, image, seg_image, g_cmd, LineMarkerArray, FollowMarkerArray);
        publishPointCloud();
        clearMemory();
        auto finish = std::chrono::high_resolution_clock::now();

        std::chrono::duration<double, std::milli> elapsed = finish - start;
        std::cout << "fusion_2D time: " << elapsed.count() << " ms\n";
    }


    void clearMemory() {
        LineMarkerArray.markers.clear();
        FollowMarkerArray.markers.clear();
        tempPointCloud->clear();
    }


    void publishPointCloud() {
        pubLineMarkerArray.publish(LineMarkerArray);
        pubFollowMarkerArray.publish(FollowMarkerArray);
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

    cv::Mat seg_img_filter(const cv::Mat& cv_seg_img) {
        int height = cv_seg_img.rows;
        int width = cv_seg_img.cols;
        int threshold_height = int((1.0f-area_ratio) * float(height));

        cv::Mat kernel = cv::Mat::zeros(height, width, CV_8U);
        kernel.rowRange(threshold_height, height) = 1;

        cv::Mat filtered_seg_img;
        cv::bitwise_and(cv_seg_img, kernel, filtered_seg_img);

        return filtered_seg_img;
    }

    std::pair<float, float> computeSlope(const vector<vector<float>> point_vec)
    {
        int n = point_vec.size();

        Eigen::VectorXd Y(n);
        Eigen::MatrixXd X(n, 2);

        for (int i = 0; i < n; ++i) {
            Y(i) = point_vec[i][2];
            X(i, 0) = point_vec[i][1];
            X(i, 1) = 1.0;
        }

        // 선형 회귀 계산 (X'X)^(-1) X'Y
        Eigen::VectorXd B = (X.transpose() * X).ldlt().solve(X.transpose() * Y);
        
        // 기울기 반환
        return {B(0), B(1)};
    }

    /**
     * 차선이 좌측차선인지 우측차선인지 구분하는 코드인데 짧은거리에서는 크게 관련이야 없겠지만 나중에 보정이 필요할 것이라고 판단됨
     * 걍 그 콘 좌우 구분 알고리즘 적용해도 괜찮을 것이라고 판단됨
     */
    std::pair<std::vector<std::vector<float>>, std::vector<std::vector<float>>> mid_right_start(const std::vector<std::vector<float>>& lane_3d_pts) {
        std::vector<std::vector<float>> right, mid;
        for (const auto& pt : lane_3d_pts) {
            if (pt[1] > 0 && pt[1] < 1.5) {
                mid.push_back(pt);
            } else if (pt[1] < 0 && pt[1] > -1.5) {
                right.push_back(pt);
            }
        }
        return {right, mid};
    }

    visualization_msgs::Marker points_rviz(const std::string& name, int id, const std::vector<std::vector<float>>& points, const std::vector<int>& color) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_link";
        marker.ns = "line";
        marker.id = id;

        marker.action = visualization_msgs::Marker::ADD;
        marker.color.r = color[0] / 255.0;
        marker.color.g = color[1] / 255.0;
        marker.color.b = color[2] / 255.0;
        marker.color.a = 1.0;
        marker.pose.orientation.w = 1.0;

        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0;

        for (const auto& point : points) {
            geometry_msgs::Point p;
            p.x = point[0];
            p.y = point[1];
            p.z = 0;
            marker.points.push_back(p);
        }
        return marker;
    }

    std::vector<std::vector<float>> follow_pts(const std::vector<std::vector<float>>& right, const std::vector<std::vector<float>>& mid) {
        std::vector<std::vector<float>> follow_point;
        size_t min_lane = std::min(right.size(), mid.size());
        for (size_t i = 0; i < min_lane; ++i) {
            follow_point.push_back({mid[i][0], right[i][1] + std::abs(mid[i][1] - right[i][1]) / 2});
        }
        return follow_point;
    }

    std::vector<std::vector<float>> follow_pts_right(const std::vector<std::vector<float>>& right) {
        std::vector<std::vector<float>> follow_point;
        for (const auto& pt : right) {
            follow_point.push_back({pt[0], pt[1] + 1.3});
        }
        return follow_point;
    }

    std::vector<std::vector<float>> follow_pts_mid(const std::vector<std::vector<float>>& mid) {
        std::vector<std::vector<float>> follow_point;
        for (const auto& pt : mid) {
            follow_point.push_back({pt[0], pt[1] - 1.3});
        }
        return follow_point;
    }

    visualization_msgs::MarkerArray marker_array_rviz(const std::vector<visualization_msgs::Marker>& markers) {
        visualization_msgs::MarkerArray marker_array;
        for (const auto& marker : markers) {
            marker_array.markers.push_back(marker);
        }
        return marker_array;
    }

    size_t getMatSize(const cv::Mat& mat) {
        return mat.total() * mat.elemSize();
    }


    void detectLine(const sensor_msgs::PointCloud2::ConstPtr& ouster, const sensor_msgs::ImageConstPtr& image, const sensor_msgs::ImageConstPtr& seg_image, G_CMD& g_cmd, visualization_msgs::MarkerArray& LineMarkerArray, visualization_msgs::MarkerArray& FollowMarkerArray)
    {
        
        cv_bridge::CvImagePtr cv_img;
        cv_bridge::CvImagePtr cv_seg_img;
        try {
            cv_img = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
            cv_seg_img = cv_bridge::toCvCopy(seg_image, sensor_msgs::image_encodings::TYPE_8UC1);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat img = cv_img->image;
        cout << "img = " << getMatSize(img) << endl;
        cv::Mat seg_img_mat = cv_seg_img->image;
        cout << "seg_img_mat = " << getMatSize(seg_img_mat) << endl;
        
        vector<vector<float>> point_list, point_list_slope;
        float camera_to_gnd = 0;
        int count = 0;
        if (ouster->data.empty())
        {
            std::cout << "(note) Empty cloud" << std::endl;
        } 
        else 
        {
            for (sensor_msgs::PointCloud2ConstIterator<float> it(*ouster, "x"); it != it.end(); ++it) {
                if (-30 < atan2(it[0], it[1]) * RAD_TO_DEG && atan2(it[0], it[1]) * RAD_TO_DEG < 30 && 
                    0 < it[0] && it[0] < 3 && it[2] < 0) {
                    point_list_slope.push_back({it[0], it[1], it[2], 1.0f});
                }   
            }
        
        auto [slope, intercept] = computeSlope(point_list_slope);
        for (float y = -3; y < 3; y += 0.05)
        {
            for (float x = 0.5; x < 5; x += 0.1)
            {
                point_list.push_back({x, y, y*slope + intercept, 1.0f});
                }
            }
        
        }
        

        cv::Mat pc_np(point_list.size(), 4, CV_32F);
        cout << "pc_np = " << getMatSize(pc_np) << endl;
        for (size_t i = 0; i < point_list.size(); ++i) {
            memcpy(pc_np.ptr<float>(i), point_list[i].data(), 4 * sizeof(float));
        }
        cout << "pc_np = " << getMatSize(pc_np) << endl;
        cv::Mat filtered_xyz_p = pc_np.colRange(0, 3);
        cv::Mat xyz_p = pc_np.colRange(0, 4).t();                           // world_xyz
        cv::Mat xyz_c = transformLiDARToCamera(TransformMat, xyz_p);        // Ext
        cout << "xyz_c = " << getMatSize(xyz_c) << endl;
        cv::Mat xy_i = transformCameraToImage(real_K_MAT, xyz_c);
        cout << "xy_i = " << getMatSize(xy_i) << endl;
                // fixel_coord: 굳이 전치하지 말고 [{0, 1} , {idx}] 구조 그대로 사용하는게 맘 편할듯

        xy_i.convertTo(xy_i, CV_32S);
        cout << "xy_i = " << getMatSize(xy_i) << endl;

        xy_i = xy_i.t();
        cout << "xy_i = " << getMatSize(xy_i) << endl;


        // ---------------------- lidar_lane --------------------

        cv::Mat filtered_seg_img = seg_img_filter(seg_img_mat);
        std::vector<std::vector<float>> lane_3d_pts;

        for (int i = 0; i < xy_i.rows; ++i) {
            if (xy_i.at<int>(i, 1) < 720 && xy_i.at<int>(i, 1) > 0 && 
                xy_i.at<int>(i, 0) > 0 && xy_i.at<int>(i, 0) < 1280) {
                if (cv_seg_img->image.at<uchar>(xy_i.at<int>(i, 1), xy_i.at<int>(i, 0)) == 1 &&
                    cv_seg_img->image.at<uchar>(xy_i.at<int>(i, 1), xy_i.at<int>(i, 0) - 5) == 0) {
                    lane_3d_pts.push_back({filtered_xyz_p.at<float>(i, 0), filtered_xyz_p.at<float>(i, 1)});
                    cout << filtered_xyz_p.at<float>(i, 0) << " : " << filtered_xyz_p.at<float>(i, 1);
                    PointType point;
                    point.x = filtered_xyz_p.at<float>(i, 0);
                    point.y = filtered_xyz_p.at<float>(i, 1);
                    point.z = 0;
                    tempPointCloud->points.push_back(point);
                }
            }
        }

        // [{x1, y1}, {x2, y2}, {x3, y3}]의 구조임
        auto [right, mid] = mid_right_start(lane_3d_pts);

        // x는 오름차수 -> y는 내림차수
        std::sort(right.begin(), right.end(), [](const std::vector<float>& a, const std::vector<float>& b) { return a[0] < b[0] || (a[0] == b[0] && a[1] > b[1]); });
        std::sort(mid.begin(), mid.end(), [](const std::vector<float>& a, const std::vector<float>& b) { return a[0] > b[0] || (a[0] == b[0] && a[1] < b[1]); });

        // 우측 라인 인식이 잘됨
        if (right.size() < 3) wall_follow = true;
        
        // 인식한 차선 정리
        std::vector<std::vector<std::vector<float>>> lanes(2);
        lanes[0] = right;
        lanes[1] = mid;

        std::vector<visualization_msgs::Marker> markers;
        std::vector<std::vector<int>> colors = {{255, 0, 0}, {0, 255, 0}, {0, 0, 255}};
        for (int k = 0; k < 2; ++k) {
            // 포인트 클라우드 순환
            for (const auto& lane_pt : lane_3d_pts) {
                // 차선이라고 인식된 
                for (auto& lane : lanes[k]) {
                    if (std::pow(lane[0] - lane_pt[0], 2) + std::pow(lane[1] - lane_pt[1], 2) < 0.04) {
                        lane[0] = lane_pt[0];
                        lane[1] = lane_pt[1];
                    }
                }
            }
            visualization_msgs::Marker lane_marker = points_rviz("lane", k, lanes[k], colors[k]);
            markers.push_back(lane_marker);
        }

        std::vector<std::vector<float>> follow_points;
        // 우측차선이 잘 인식이 안될떄
        if (!wall_follow) {
            if (mid.size() > 2 && right.size() > 2) {
                follow_points = follow_pts(right, mid);
            } else if (right.size() > 0) {
                follow_points = follow_pts_right(right);
            } else {
                follow_points = follow_pts_mid(mid);
            }
        // 우측차선 인식이 잘될때
        } else {
            follow_points = follow_pts_right(right);
        }

        LineMarkerArray = marker_array_rviz(markers);
        FollowMarkerArray.markers.push_back(points_rviz("lane", 4, follow_points, colors[2]));
        publisher(tempPointCloud, pubLidar, "/os_sensor");
        
    }
};


int main(int argc, char** argv) {
    // 메인 프로세스로부터 값 받기
    int camera_idx = 0;
    G_CMD g_cmd;
    uint32_t image_topic_size, seg_image_topic_size, Line_topic_size, Follow_topic_size;
    char image_topic_buffer[256], seg_image_topic_buffer[256], Line_topic_buffer[256], Follow_topic_buffer[256];

    /*
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

    read(STDIN_FILENO, &seg_image_topic_size, sizeof(uint32_t));
    read(STDIN_FILENO, seg_image_topic_buffer, seg_image_topic_size);
    seg_image_topic_buffer[seg_image_topic_size] = '\0'; // Null-terminate the string

    read(STDIN_FILENO, &Line_topic_size, sizeof(uint32_t));
    read(STDIN_FILENO, Line_topic_buffer, Line_topic_size);
    Line_topic_buffer[Line_topic_size] = '\0'; // Null-terminate the string

    read(STDIN_FILENO, &Follow_topic_size, sizeof(uint32_t));
    read(STDIN_FILENO, Follow_topic_buffer, Follow_topic_size);
    Follow_topic_buffer[Follow_topic_size] = '\0'; // Null-terminate the string


    string image_topic(image_topic_buffer);
    string seg_image_topic(seg_image_topic_buffer);
    string Line_topic_name(Line_topic_buffer);
    string Follow_topic_name(Follow_topic_buffer);
    */
    string image_topic = "/output_img";
    string seg_image_topic = "/seg_img";
    string Line_topic_name = "/line_marker2D";
    string Follow_topic_name = "/follow_marker2D";

    // ROS 실행
    std::string node_name = "fusion_Line_2D" + std::to_string(camera_idx);
    ros::init(argc, argv, node_name);
    g_cmd.rvec = {0, 0, 0}; g_cmd.t_mat = {0, 0, 0}; g_cmd.focal_length = {820, 820}; g_cmd.cam = {1280, 720};

    FusionLine2DNode node(g_cmd, camera_idx, image_topic, seg_image_topic, Line_topic_name, Follow_topic_name);
    ros::spin();

    return 0;
}