#include "pointcloud_generator.h"
#include "utility.h"


using namespace std;


/**
 *! 현재는 콘bag 파일을 기준으로 작성되어 있으며, 실제 적용시 다를 수 있습니다.
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
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;

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

		pubCarMarkerArray = nh.advertise<visualization_msgs::MarkerArray>("/markers", 1);
		pubCarBboxArray = nh.advertise<visualization_msgs::MarkerArray>("/markers_vis", 1);

		pubObjMarkerArray = nh.advertise<visualization_msgs::MarkerArray>("/obj_marker_array", 1);
		pubAngleMarkerArray = nh.advertise<visualization_msgs::MarkerArray>("/angle_marker_array", 1);

		// Subscribe할 토픽 설정
		subLidar.subscribe(nh, "/os_cloud_node/points", 10);
		subLocalMsgs.subscribe(nh, "/local_msgs_to_vision", 1000);

		// callback
		sync.reset(new Sync(MySyncPolicy(50), subLidar, subLocalMsgs));
        sync->registerCallback(boost::bind(&ImageProjection::cloudHandler, this, _1, _2));

		// 													mapReader("json 파일");
		allocateMemory();
		clearMemory();
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

		// markerArray 초기화
		deleteAllMarkers(carMarkerArray);
		deleteAllMarkers(carBboxArray);
		deleteAllMarkers(objMarkerArray);
		deleteAllMarkers(angleMarkerArray);

		// vector 초기화
		smallObject_Cloud_vector->clear();
        bigObject_Cloud_vector->clear();
	}


	void cloudHandler(const sensor_msgs::PointCloud2::ConstPtr& rosCloud, 
					  const geometry_msgs::PoseStampedConstPtr& local_msg)
	{
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
	// 관심 영역 설정
		pointCloudGenerator.getInterestCloud(fullCloud, interestCloud, {-2, 15}, {-6, 6}, {-0.7, 1.5});
	// 전방 영역 설정
		pointCloudGenerator.getFovCloud(fullCloud, fovCloud, {-999, 10}, {-999, 999}, {-999, 999}, {-45, 45});
		
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
		publisherMarkerArray(carMarkerArray, pubCarMarkerArray, "base_link");
		publisherMarkerArray(carBboxArray, pubCarBboxArray, "base_link");
		publisherMarkerArray(objMarkerArray, pubObjMarkerArray, "base_link");
		publisherMarkerArray(angleMarkerArray, pubAngleMarkerArray, "base_link");
	
	// 포인트 클라우드 Publish
		publisher(fullCloud, pubFullCloud, "base_laser");
		publisher(fullInfoCloud, pubFullInfoCloud, "base_laser");
		publisher(interestCloud, pubInterestCloud, "base_laser");
		publisher(interestCloud2D, pubInterestCloud2D, "base_laser");
		publisher(ROICloud, pubROICloud, "base_laser");
		publisher(fovCloud, pubFovCloud, "base_link");
		publisher(groundCloud, pubgroundCloud, "base_laser");
		publisher(nongroundCloud, pubnongroundCloud, "base_laser");
		publisher(debugCloud, pubDebugCloud, "base_laser");
		publisher(transformedCloud, pubTransformedCloud, "base_link");
		publisher(transformedRoiCloud, pubTransformedRoiCloud, "base_link");
	}
};


int main (int argc, char** argv)
{
	ros::init(argc, argv, "practics_dynamic_main");
	ImageProjection imageProjection;
	ros::spin();
}