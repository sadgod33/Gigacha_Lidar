#include "pointcloud_generator.h"
#include "utility.h"

using namespace std;

/**
 ** 좌표
        * x: 차량의 정면
        * y: 차량의 좌측
        * z: 차량의 상측
		* 전방시야: x축으로부터 좌측이 음수, 우측이 양수 
        * 회전: 축을 기준으로 오른손의 법칙을 따름
        * 라이다의 row: 아래서부터 위로 숫자가 증가
        * 라이다의 column: 라이다 후면부터 반시계 방향으로 증가
 */

class ImageProjection{
private:
// ros handle
    ros::NodeHandle nh;

// 차량 좌표 및 시간
	geometry_msgs::PoseStamped local;

// ros::subscriber & sync
	message_filters::Subscriber<sensor_msgs::PointCloud2> subLidar;
	message_filters::Subscriber<geometry_msgs::PoseStamped> subLocalMsgs;
	std::vector<message_filters::Subscriber<sensor_msgs::Image>*> subImages;
    std::vector<message_filters::Subscriber<vision_msgs::Detection2DArray>*> subBoxes;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;

    std::vector<std::vector<string>>  camera_box_pairs;
	std::vector<std::vector<string>>  camera_line_pairs;
	std::vector<G_CMD> g_cmd_obj_list;
	std::vector<G_CMD> g_cmd_line_list;
	std::vector<pid_t> pids;
	
// ros::publisher
	ros::Publisher pubOdometry;

	ros::Publisher pubFullCloud;
	ros::Publisher pubFullInfoCloud;
	ros::Publisher pubInterestCloud;
	ros::Publisher pubInterestCloud2D;
	ros::Publisher pubROICloud;
	ros::Publisher pubFovCloud;
	ros::Publisher pubgroundCloud;
	ros::Publisher pubnongroundCloud;

	ros::Publisher pubTransformedCloud;
	ros::Publisher pubTransformedRoiCloud;
	ros::Publisher pubAngleCloud;

	ros::Publisher pubDebugCloud;
	ros::Publisher pubCarMarkerArray;	// 큐브
	ros::Publisher pubCarBboxArray; 	// Bbox
	ros::Publisher pubObjMarkerArray;	// 작은 오브젝트 -> 콘, 배럴
	ros::Publisher pubAngleMarkerArray;

// pcl pointCloud
	// pointCLoud 원본
	pcl::PointCloud<PointType>::Ptr laserCloudIn;

    // Publish할 pointCLoud
	pcl::PointCloud<PointType>::Ptr fullCloud;
	pcl::PointCloud<PointType>::Ptr fullInfoCloud;
	pcl::PointCloud<PointType>::Ptr interestCloud;
	pcl::PointCloud<PointType>::Ptr interestCloud2D;
	pcl::PointCloud<PointType>::Ptr ROICloud;
	pcl::PointCloud<PointType>::Ptr fovCloud;
	pcl::PointCloud<PointType>::Ptr groundCloud;
	pcl::PointCloud<PointType>::Ptr nongroundCloud;
	pcl::PointCloud<PointType>::Ptr transformedCloud;
	pcl::PointCloud<PointType>::Ptr transformedRoiCloud;
	pcl::PointCloud<PointType>::Ptr debugCloud;
	pcl::PointCloud<PointType>::Ptr angleCloud;
	
	// Publish할 MarkerArray
	std::shared_ptr<visualization_msgs::MarkerArray> carMarkerArray = std::make_shared<visualization_msgs::MarkerArray>();
	std::shared_ptr<visualization_msgs::MarkerArray> carBboxArray = std::make_shared<visualization_msgs::MarkerArray>();
	std::shared_ptr<visualization_msgs::MarkerArray> objMarkerArray = std::make_shared<visualization_msgs::MarkerArray>();
	std::shared_ptr<visualization_msgs::MarkerArray> angleMarkerArray = std::make_shared<visualization_msgs::MarkerArray>();

	// 클러스터를 담은 vector
	std::shared_ptr<std::vector<pcl::PointCloud<PointType>::Ptr>> smallObject_Cloud_vector = std::make_shared<std::vector<pcl::PointCloud<PointType>::Ptr>>();
	std::shared_ptr<std::vector<pcl::PointCloud<PointType>::Ptr>> bigObject_Cloud_vector = std::make_shared<std::vector<pcl::PointCloud<PointType>::Ptr>>();

	Preprocessor preprocessor;
	PointCloudGenerator pointCloudGenerator;

	
public:
	// 전처리 클래스 선언
	ImageProjection(): nh("~"){
		// Publish할 토픽 설정
        pubOdometry = nh.advertise<geometry_msgs::PoseStamped>("/local_msg", 10);

		pubFullCloud = nh.advertise<sensor_msgs::PointCloud2>("/full_cloud_projected", 1);
        pubFullInfoCloud = nh.advertise<sensor_msgs::PointCloud2>("/full_cloud_info", 1);

		pubInterestCloud = nh.advertise<sensor_msgs::PointCloud2>("/interest_cloud", 1);
		pubInterestCloud2D = nh.advertise<sensor_msgs::PointCloud2>("/interest_cloud2d", 1);
		pubROICloud = nh.advertise<sensor_msgs::PointCloud2>("/roi_cloud", 1);
		pubFovCloud = nh.advertise<sensor_msgs::PointCloud2>("/fov_cloud", 1);
		pubgroundCloud = nh.advertise<sensor_msgs::PointCloud2>("/ground_cloud", 1);
		pubnongroundCloud = nh.advertise<sensor_msgs::PointCloud2>("/nonground_cloud", 1);
		pubDebugCloud = nh.advertise<sensor_msgs::PointCloud2>("/debug_cloud", 1);
		pubTransformedCloud = nh.advertise<sensor_msgs::PointCloud2>("/transformed_cloud", 1);
		pubTransformedRoiCloud = nh.advertise<sensor_msgs::PointCloud2>("/transformed_roi_cloud", 1);
		pubAngleCloud = nh.advertise<sensor_msgs::PointCloud2>("/angle_cloud", 1);

		pubCarMarkerArray = nh.advertise<visualization_msgs::MarkerArray>("/markers", 1);
		pubCarBboxArray = nh.advertise<visualization_msgs::MarkerArray>("/markers_vis", 1);

		pubObjMarkerArray = nh.advertise<visualization_msgs::MarkerArray>("/obj_marker_array", 1);
		pubAngleMarkerArray = nh.advertise<visualization_msgs::MarkerArray>("/angle_marker_array", 1);

		// Subscribe할 토픽 설정
		subLidar.subscribe(nh, "/os_cloud_node/points", 10);
		subLocalMsgs.subscribe(nh, "/local_msgs_to_vision", 1000);

	// ------------------------------ 카메라 센서퓨전 서브프로세스 -------------------------------
		/*
		 * 아래 vector는 [이미지토픽, 바운딩 박스 or 세그멘테이션 이미지, 출력할 마커 토픽의 이름]으로 구성되어 있다.
		 * 그리고 g_cmd<num>은 각 카메라의 파라미터를 의미하므로 해당 파라미터값을 찾아 입력해준다.
		 * 마지막으로 해당 코드는 rosPackage이기에 상대좌표가 이 코드가 아닌, ~/.ros를 기준으로 작동하기에 절대좌표로 입력해준다.
		 * 방법을 분명 있을텐데 귀찮다...
		*/
        // 카메라별 토픽 및 파라미터 설정
		camera_box_pairs = { {"/usb_cam/static", "/static_bbox", "/vision_marker"}, {"/usb_cam/static", "/static_bbox", "/vision_test"}};

		// 차선 인식 토픽 및 파라미터 설정
		camera_line_pairs = {{"/output_img", "/seg_img", "/line_marker", "/follow_marker"}};

		G_CMD g_cmd0;
		g_cmd0.rvec = {0, 0, 0}; g_cmd0.t_mat = {0, 0, 0}; g_cmd0.focal_length = {636.4573730956954, 667.7077677609984}; g_cmd0.cam = {640, 480};
		g_cmd_obj_list.push_back(g_cmd0);
		G_CMD g_cmd1;
		g_cmd1.rvec = {0, 0, 0}; g_cmd1.t_mat = {0, 0, 0}; g_cmd1.focal_length = {636.4573730956954, 667.7077677609984}; g_cmd1.cam = {640, 480};
		g_cmd_obj_list.push_back(g_cmd1);

		G_CMD g_cmd2;
		g_cmd2.rvec = {0, 0, 0}; g_cmd2.t_mat = {0, 0.05, -0.05}; g_cmd2.focal_length = {820, 820}; g_cmd2.cam = {1280, 720};
		g_cmd_line_list.push_back(g_cmd2);

        for (int camera_idx = 0; camera_idx < camera_box_pairs.size(); ++camera_idx) {
            pid_t pid = subProcess_obj(camera_idx, g_cmd_obj_list[camera_idx], camera_box_pairs[camera_idx]);
            if (pid > 0) {
                pids.push_back(pid);
            }
        }

		for (int camera_idx = 0; camera_idx < camera_line_pairs.size(); ++camera_idx) {
            pid_t pid = subProcess_line(camera_idx, g_cmd_line_list[camera_idx], camera_line_pairs[camera_idx]);
            if (pid > 0) {
                pids.push_back(pid);
            }
        }
	// ------------------------------ 카메라 센서퓨전 서브프로세스 -------------------------------

		
		sync.reset(new Sync(SyncPolicy(50), subLidar, subLocalMsgs));
		sync->registerCallback(boost::bind(&ImageProjection::cloudHandler, this, _1, _2));

		// 													mapReader("json 파일");
		allocateMemory();
		clearMemory();
	}

	~ImageProjection() {
        // 자식 프로세스 종료 대기
        for (pid_t pid : pids) {
            int status;
            waitpid(pid, &status, 0);
        }
    }


    void allocateMemory(){
		laserCloudIn.reset(new pcl::PointCloud<PointType>());

		fullCloud.reset(new pcl::PointCloud<PointType>());
		fullInfoCloud.reset(new pcl::PointCloud<PointType>());
		interestCloud.reset(new pcl::PointCloud<PointType>());
		interestCloud2D.reset(new pcl::PointCloud<PointType>());
		ROICloud.reset(new pcl::PointCloud<PointType>());
		fovCloud.reset(new pcl::PointCloud<PointType>());
		groundCloud.reset(new pcl::PointCloud<PointType>());
		nongroundCloud.reset(new pcl::PointCloud<PointType>());
		debugCloud.reset(new pcl::PointCloud<PointType>());
		transformedCloud.reset(new pcl::PointCloud<PointType>());
		transformedRoiCloud.reset(new pcl::PointCloud<PointType>());
		debugCloud.reset(new pcl::PointCloud<PointType>());
		angleCloud.reset(new pcl::PointCloud<PointType>());
	}


	void clearMemory(){
		laserCloudIn->clear();

		fullCloud->clear();
		fullInfoCloud->clear();
        interestCloud->clear();
		interestCloud2D->clear();
		ROICloud->clear();
		fovCloud->clear();
		groundCloud->clear();
		nongroundCloud->clear();
		debugCloud->clear();
		transformedCloud->clear();
		transformedRoiCloud->clear();
		angleCloud->clear();

		// markerArray 초기화
		deleteAllMarkers(carMarkerArray);
		deleteAllMarkers(carBboxArray);
		deleteAllMarkers(objMarkerArray);
		deleteAllMarkers(angleMarkerArray);

		// vector 초기화
		smallObject_Cloud_vector->clear();
        bigObject_Cloud_vector->clear();

	}


// 라이다와 GPS만 작동시 진행
	void cloudHandler(const sensor_msgs::PointCloud2::ConstPtr& rosCloud, 
					  const geometry_msgs::PoseStampedConstPtr& local_msg)
	{
		cout << "LiDAR is Working" << endl;
		pcl::fromROSMsg(*rosCloud, *laserCloudIn);
		saveCurrentTime(rosCloud);
	// 라이다-GPS 거리 및 라이다 Y축 회전값 보정
		preprocessor.calibrateLidar(laserCloudIn, laserCloudIn, -1.5);
	// 몸체에 찍히는 라이다 포인트 제거
		preprocessor.removeBody(laserCloudIn, laserCloudIn);
	// GPS 및 해딩 불러오기
		setEgo(local_msg, local);
	// 포인트 클라우드 가공 및 생성
        getPointCloud();
	// 포인트 클라우드 publish
		publishPointCloud();
	// 데이터 초기화
		clearMemory();
    }


	void getPointCloud(){
	// 전체 포인트 클라우드 기본 셋팅
		pointCloudGenerator.getFullCloud(laserCloudIn, fullCloud, fullInfoCloud);
	// xyz범위 자르기 + 일정 각도 내만 관심영역으로 설정 
		pointCloudGenerator.getAngleCloud(fullCloud, angleCloud, {-5, 5}, {-5, 5}, {-0.7, 1}, {-45, 45});
	// 관심 영역 설정
		pointCloudGenerator.getInterestCloud(fullCloud, interestCloud, {-2, 15}, {-6, 6}, {-0.7, 1.5});
	// 전방 영역 설정
		//pointCloudGenerator.getFovCloud(fullCloud, fovCloud, {-999, 10}, {-999, 999}, {-999, 999}, {-45, 45});
		
	// 지면제거 영역 분할
		pointCloudGenerator.getGrdRemovalClouds(fullCloud, groundCloud, nongroundCloud, {15.0, -0.85});

	// 절대좌표로 전환
		pointCloudGenerator.getTransformedClouds(fullCloud, transformedCloud);

	// 클러스터링 vector 생성
		pointCloudGenerator.getObjectClusterCloud(interestCloud, smallObject_Cloud_vector, bigObject_Cloud_vector);

	// 클러스터의 크기 및 방향 측정 및 마커 생성
		pointCloudGenerator.getObjectMarkers(bigObject_Cloud_vector, carMarkerArray, carBboxArray);
	}


	void publishPointCloud(){
	// 좌표 및 시간 publish
		pubOdometry.publish(local);

	// MarkArray Publish
		publisherMarkerArray(carMarkerArray, pubCarMarkerArray, "map");
		publisherMarkerArray(carBboxArray, pubCarBboxArray, "map");
		publisherMarkerArray(objMarkerArray, pubObjMarkerArray, "map");
		publisherMarkerArray(angleMarkerArray, pubAngleMarkerArray, "map");
	
	// 포인트 클라우드 Publish
		publisher(fullCloud, pubFullCloud, "map");
		publisher(fullInfoCloud, pubFullInfoCloud, "map");
		publisher(interestCloud, pubInterestCloud, "map");
		publisher(interestCloud2D, pubInterestCloud2D, "map");
		publisher(ROICloud, pubROICloud, "map");
		publisher(fovCloud, pubFovCloud, "map");
		publisher(groundCloud, pubgroundCloud, "map");
		publisher(nongroundCloud, pubnongroundCloud, "map");
		publisher(debugCloud, pubDebugCloud, "map");
		publisher(transformedCloud, pubTransformedCloud, "map");
		publisher(transformedRoiCloud, pubTransformedRoiCloud, "map");
		publisher(angleCloud, pubAngleCloud, "map");
	}
};


int main (int argc, char** argv)
{
	ros::init(argc, argv, "practics_dynamic_main");
	ImageProjection imageProjection;
	ros::spin();
}