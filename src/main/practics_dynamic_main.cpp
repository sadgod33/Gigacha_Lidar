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

// ros::subscriber & sync
	message_filters::Subscriber<sensor_msgs::PointCloud2> subLidar;
	message_filters::Subscriber<geometry_msgs::PoseStamped> subLocalMsgs;
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;

// ros::publisher
	ros::Publisher pubCarCloud;
	ros::Publisher pubInterestCloud;
	ros::Publisher pubInterestCloud2D;
	ros::Publisher pubROICloud;
	ros::Publisher pubFovCloud;

	ros::Publisher pubConeClusterCloud;
	ros::Publisher pubConeROICloud;
	ros::Publisher pubLconeCloud;
	ros::Publisher pubRconeCloud;
	ros::Publisher pubDebugCloud;
	ros::Publisher pubCarMarkerArray;	// 큰   오브젝트 -> 자동차
	ros::Publisher pubObjMarkerArray;	// 작은 오브젝트 -> 콘, 배럴
	ros::Publisher pubAngleMarkerArray;

// pcl pointCloud
	// pointCLoud 원본
	pcl::PointCloud<PointType>::Ptr fullCloud;

    // Publish할 pointCLoud 
	pcl::PointCloud<PointType>::Ptr interestCloud;
	pcl::PointCloud<PointType>::Ptr interestCloud2D;
	pcl::PointCloud<PointType>::Ptr ROICloud;
	pcl::PointCloud<PointType>::Ptr fovCloud;
	pcl::PointCloud<PointType>::Ptr coneClusterCloud;
	pcl::PointCloud<PointType>::Ptr coneROICloud;
	pcl::PointCloud<PointType>::Ptr LconeCloud;
	pcl::PointCloud<PointType>::Ptr RconeCloud;
	pcl::PointCloud<PointType>::Ptr debugCloud;
	
	// Publish할 MarkerArray
	std::shared_ptr<visualization_msgs::MarkerArray> carMarkerArray = std::make_shared<visualization_msgs::MarkerArray>();
	std::shared_ptr<visualization_msgs::MarkerArray> objMarkerArray = std::make_shared<visualization_msgs::MarkerArray>();
	std::shared_ptr<visualization_msgs::MarkerArray> angleMarkerArray = std::make_shared<visualization_msgs::MarkerArray>();

	Preprocessor preprocessor;
	PointCloudGenerator pointCloudGenerator;


public:
	// 전처리 클래스 선언
	ImageProjection(): nh("~"){
		// Publish할 토픽 설정
		pubInterestCloud = nh.advertise<sensor_msgs::PointCloud2>("/interest_cloud", 1);
		pubInterestCloud2D = nh.advertise<sensor_msgs::PointCloud2>("/interest_cloud2d", 1);
		pubROICloud = nh.advertise<sensor_msgs::PointCloud2>("/roi_cloud", 1);
		pubFovCloud = nh.advertise<sensor_msgs::PointCloud2>("/fov_cloud", 1);
		pubConeClusterCloud = nh.advertise<sensor_msgs::PointCloud2>("/cone_cluster_cloud", 1);
		pubConeROICloud = nh.advertise<sensor_msgs::PointCloud2>("/cone_ROI_cloud", 1);
		pubLconeCloud = nh.advertise<sensor_msgs::PointCloud2>("/Lcone_cloud", 1);
		pubRconeCloud = nh.advertise<sensor_msgs::PointCloud2>("/Rcone_cloud", 1);
		pubDebugCloud = nh.advertise<sensor_msgs::PointCloud2>("/debug_cloud", 1);
		pubCarMarkerArray = nh.advertise<visualization_msgs::MarkerArray>("/car_marker_array", 1);
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
		fullCloud.reset(new pcl::PointCloud<PointType>());

		interestCloud.reset(new pcl::PointCloud<PointType>());
		interestCloud2D.reset(new pcl::PointCloud<PointType>());
		ROICloud.reset(new pcl::PointCloud<PointType>());
		fovCloud.reset(new pcl::PointCloud<PointType>());
		coneClusterCloud.reset(new pcl::PointCloud<PointType>());
		coneROICloud.reset(new pcl::PointCloud<PointType>());
		LconeCloud.reset(new pcl::PointCloud<PointType>());
		RconeCloud.reset(new pcl::PointCloud<PointType>());
		debugCloud.reset(new pcl::PointCloud<PointType>());
	}


	void clearMemory(){
		fullCloud->clear();

        interestCloud->clear();
		interestCloud2D->clear();
		ROICloud->clear();
		fovCloud->clear();
		coneClusterCloud->clear();
		coneROICloud->clear();
		LconeCloud->clear();
		RconeCloud->clear();
		debugCloud->clear();

		// markerArray 초기화
		deleteAllMarkers(carMarkerArray);
		deleteAllMarkers(objMarkerArray);
		deleteAllMarkers(angleMarkerArray);
	}


	void cloudHandler(const sensor_msgs::PointCloud2::ConstPtr& rosCloud, 
					  const geometry_msgs::PoseStampedConstPtr& local_msg)
	{
		pcl::fromROSMsg(*rosCloud, *fullCloud);
	// 현 시간 저장
		saveCurrentTime(rosCloud);
	// 상대좌표: 라이다 -> GPS
		preprocessor.convertLidartoGPS(fullCloud, fullCloud);
	// 몸체에 찍히는 라이다 포인트 제거
		preprocessor.removeBody(fullCloud, fullCloud);
	// GPS 및 해딩 불러오기
		setEgo(local_msg);
		auto [hd, tunnelLoc] = setHeadingInTunnel(fullCloud, {-50, 50}, {-50, 50}, {0.1, 3.5});
		std::cout << endl;
		std::cout << "now heading   : " << hd * RAD_TO_DEG << std::endl;
		std::cout << "tennel in/out : " << std::setfill('0') << std::setw(7) << tunnelLoc << std::endl;
	// 포인트 클라우드 가공 및 생성
        getPointCloud();
	// 포인트 클라우드 publish
		publishPointCloud();
	// 데이터 초기화
		clearMemory();
    }


	void getPointCloud(){
		pointCloudGenerator.getInterestCloud(fullCloud, interestCloud, {0, 15}, {-8, 8}, {-0.6, 0});
		pointCloudGenerator.get2DCloud(interestCloud, interestCloud2D, -0.6, 0.1);
		pointCloudGenerator.getAngle1to5(interestCloud2D, angleMarkerArray);
		// pointCloudGenerator.getROICloud(interestCloud, ROICloud, 2.0);  <- 나중에 GPS 되면 주석 제거 (확인 필요)
	}


	void publishPointCloud(){
	// MarkArray Publish
		publisherMarkerArray(carMarkerArray, pubCarMarkerArray, "/map");
		publisherMarkerArray(objMarkerArray, pubObjMarkerArray, "/map");
		publisherMarkerArray(angleMarkerArray, pubAngleMarkerArray, "/map");
	
	// 포인트 클라우드 Publish
		publisher(interestCloud, pubInterestCloud, "/map");
		publisher(interestCloud2D, pubInterestCloud2D, "/map");
		publisher(ROICloud, pubROICloud, "/map");
		publisher(fovCloud, pubFovCloud, "/map");
		publisher(coneClusterCloud, pubConeClusterCloud, "/map");
		publisher(coneROICloud, pubConeROICloud, "/map");
		publisher(LconeCloud, pubLconeCloud, "/map");
		publisher(RconeCloud, pubRconeCloud, "/map");
		publisher(debugCloud, pubDebugCloud, "/map");
	}
};


int main (int argc, char** argv)
{
	ros::init(argc, argv, "practics_dynamic_main");
	ImageProjection imageProjection;
	ros::spin();
}