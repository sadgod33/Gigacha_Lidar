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

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;

	// publisher
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

	ros::Publisher pubConeClusterCloud;
	ros::Publisher pubConeROICloud;
	ros::Publisher pubLconeCloud;
	ros::Publisher pubRconeCloud;
	ros::Publisher pubDebugCloud;
	ros::Publisher pubROIMarkerArray;
	ros::Publisher pubLConeLineMarkerArray;
	ros::Publisher pubRConeLineMarkerArray;

	// subscriber
	message_filters::Subscriber<sensor_msgs::PointCloud2> subLidar;
	message_filters::Subscriber<geometry_msgs::PoseStamped> subLocalMsgs;

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
	pcl::PointCloud<PointType>::Ptr coneClusterCloud;
	pcl::PointCloud<PointType>::Ptr coneROICloud;
	pcl::PointCloud<PointType>::Ptr LconeCloud;
	pcl::PointCloud<PointType>::Ptr RconeCloud;
	pcl::PointCloud<PointType>::Ptr debugCloud;

	// Publish할 MarkerArray
	std::shared_ptr<visualization_msgs::MarkerArray> ROIMarkerArray = std::make_shared<visualization_msgs::MarkerArray>();
	std::shared_ptr<visualization_msgs::MarkerArray> LConeLineMarkerArray = std::make_shared<visualization_msgs::MarkerArray>();
	std::shared_ptr<visualization_msgs::MarkerArray> RConeLineMarkerArray = std::make_shared<visualization_msgs::MarkerArray>();

	// 초기화 X
	pcl::PointCloud<PointType>::Ptr savedClusterCloud;

	Preprocessor preprocessor;
	PointCloudGenerator pointCloudGenerator;


public:
	// 전처리 클래스 선언
	ImageProjection(): nh("~"){
		// Publish할 토픽 설정
		pubOdometry = nh.advertise<geometry_msgs::PoseStamped>("/local_msg", 10);

		pubFullCloud = nh.advertise<sensor_msgs::PointCloud2>("/full_cloud", 1);
		pubFullInfoCloud = nh.advertise<sensor_msgs::PointCloud2>("/full_info_cloud", 1);
		pubInterestCloud = nh.advertise<sensor_msgs::PointCloud2>("/interest_cloud", 1);
		pubInterestCloud2D = nh.advertise<sensor_msgs::PointCloud2>("/interest_cloud2d", 1);
		pubROICloud = nh.advertise<sensor_msgs::PointCloud2>("/roi_cloud", 1);
		pubFovCloud = nh.advertise<sensor_msgs::PointCloud2>("/fov_cloud", 1);
		pubgroundCloud = nh.advertise<sensor_msgs::PointCloud2>("/ground_cloud", 1);
		pubnongroundCloud = nh.advertise<sensor_msgs::PointCloud2>("/nonground_cloud", 1);
		pubTransformedCloud = nh.advertise<sensor_msgs::PointCloud2>("/transformed_cloud", 1);
		pubTransformedRoiCloud = nh.advertise<sensor_msgs::PointCloud2>("/transformed_roi_cloud", 1);
		pubConeClusterCloud = nh.advertise<sensor_msgs::PointCloud2>("/cone_cluster_cloud", 1);
		pubConeROICloud = nh.advertise<sensor_msgs::PointCloud2>("/cone_ROI_cloud", 1);
		pubLconeCloud = nh.advertise<sensor_msgs::PointCloud2>("/Lcone_cloud", 1);
		pubRconeCloud = nh.advertise<sensor_msgs::PointCloud2>("/Rcone_cloud", 1);
		pubDebugCloud = nh.advertise<sensor_msgs::PointCloud2>("/debug_cloud", 1);
		pubROIMarkerArray = nh.advertise<visualization_msgs::MarkerArray>("/ROI_marker_array", 1);
		pubLConeLineMarkerArray = nh.advertise<visualization_msgs::MarkerArray>("/Lcone_line_marker_array", 1);
		pubRConeLineMarkerArray = nh.advertise<visualization_msgs::MarkerArray>("/Rcone_line_marker_array", 1);

		// Subscribe할 토픽 설정
		subLidar.subscribe(nh, "/os_cloud_node/points", 10);
		subLocalMsgs.subscribe(nh, "/local_msgs_to_vision", 1000);

		// 라이다와 GPS 싱크
		sync.reset(new Sync(MySyncPolicy(50), subLidar, subLocalMsgs));
        sync->registerCallback(boost::bind(&ImageProjection::cloudHandler, this, _1, _2));

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
		transformedCloud.reset(new pcl::PointCloud<PointType>());
		transformedRoiCloud.reset(new pcl::PointCloud<PointType>());
		coneClusterCloud.reset(new pcl::PointCloud<PointType>());
		coneROICloud.reset(new pcl::PointCloud<PointType>());
		LconeCloud.reset(new pcl::PointCloud<PointType>());
		RconeCloud.reset(new pcl::PointCloud<PointType>());
		debugCloud.reset(new pcl::PointCloud<PointType>());
		savedClusterCloud.reset(new pcl::PointCloud<PointType>());
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
		transformedCloud->clear();
		transformedRoiCloud->clear();
		coneClusterCloud->clear();
		coneROICloud->clear();
		LconeCloud->clear();
		RconeCloud->clear();
		debugCloud->clear();
		deleteAllMarkers(ROIMarkerArray);
		deleteAllMarkers(LConeLineMarkerArray);
		deleteAllMarkers(RConeLineMarkerArray);

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
		pointCloudGenerator.getFovCloud(fullCloud, fovCloud, {-999, 10}, {-999, 999}, {-999, 999}, {55, 125});
	// 지면제거 영역 분할
		pointCloudGenerator.getGrdRemovalClouds(fullCloud, groundCloud, nongroundCloud, {15.0, -0.85});

	// 절대좌표의 포인트로 전환
		pointCloudGenerator.getTransformedClouds(fullCloud, transformedCloud);
		pointCloudGenerator.getTransformedClouds(ROICloud, transformedRoiCloud);

	// 기본 클러스터링
		pointCloudGenerator.getConeClusterCloud(interestCloud, coneClusterCloud);
	// 콘 ROI 설정
		pointCloudGenerator.getconeROICloud(coneClusterCloud, coneROICloud, ROIMarkerArray);
	// 콘 클러스터링
		pointCloudGenerator.getLRconeCloud(coneROICloud, RconeCloud, LconeCloud, RConeLineMarkerArray, LConeLineMarkerArray);
	}


	void publishPointCloud(){
	// MarkArray Publish
		publisherMarkerArray(ROIMarkerArray, pubROIMarkerArray, "/map");
		publisherMarkerArray(LConeLineMarkerArray, pubLConeLineMarkerArray, "/map");
		publisherMarkerArray(RConeLineMarkerArray, pubRConeLineMarkerArray, "/map");

	// 포인트 클라우드 Publish
		publisher(interestCloud, pubInterestCloud, "/map");
		publisher(interestCloud2D, pubInterestCloud2D, "/map");
		publisher(ROICloud, pubROICloud, "/map");
		publisher(fovCloud, pubFovCloud, "/map");
		publisher(groundCloud, pubgroundCloud, "/map");
		publisher(nongroundCloud, pubnongroundCloud, "/map");
		publisher(transformedCloud, pubTransformedCloud, "/map");
		publisher(transformedRoiCloud, pubTransformedRoiCloud, "/map");
		publisher(coneClusterCloud, pubConeClusterCloud, "/map");
		publisher(coneROICloud, pubConeROICloud, "/map");
		publisher(LconeCloud, pubLconeCloud, "/map");
		publisher(RconeCloud, pubRconeCloud, "/map");
		publisher(debugCloud, pubDebugCloud, "/map");

		// 평시에는 꼭 주석처리 한다!
		//publisher(fullCloud, pubFullCloud, "/map");
	}
};


int main (int argc, char** argv)
{
	ros::init(argc, argv, "practics_dynamic_cone");
	ImageProjection imageProjection;
	ros::spin();
}

