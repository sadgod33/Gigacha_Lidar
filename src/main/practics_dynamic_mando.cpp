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
//


class ImageProjection{
private:
// ros handle
    ros::NodeHandle nh;

// 차량 좌표 및 시간
	geometry_msgs::PoseStamped local;

// ros::subscriber
	ros::Subscriber subLidarStop;
	ros::Subscriber subLidar;
	
// ros::publisher
	ros::Publisher pubEmergencyFromLidar;

// pcl pointCloud
	// pointCLoud 원본
	pcl::PointCloud<PointType>::Ptr laserCloudIn;
	pcl::PointCloud<PointType>::Ptr emergencyCloud;

// 긴급 정지
	std_msgs::Bool emergency;

	Preprocessor preprocessor;
	PointCloudGenerator pointCloudGenerator;

	
public:
	// 전처리 클래스 선언
	ImageProjection(): nh("~"){
		// Publish할 토픽 설정
		pubEmergencyFromLidar = nh.advertise<std_msgs::Bool>("/emergency_from_lidar", 10);

		// Subscribe할 토픽 설정
		subLidar = nh.subscribe("/os_cloud_node/points", 10, &ImageProjection::cloudHandler, this);
		subLidarStop = nh.subscribe("/LidarStop", 10, &ImageProjection::LidarStop, this);


		mapReader("/home/sadgod33/catkin_ws/src/pcl_cpp_tutorial/json_map/real_real_final_map.json");
		allocateMemory();
		clearMemory();
	}

	~ImageProjection() {}


    void allocateMemory(){
		laserCloudIn.reset(new pcl::PointCloud<PointType>());
		emergencyCloud.reset(new pcl::PointCloud<PointType>());
	}


	void clearMemory(){
		laserCloudIn->clear();
		emergencyCloud->clear();
	}


// 라이다와 GPS만 작동시 진행
	void cloudHandler(const sensor_msgs::PointCloud2::ConstPtr& rosCloud)
	{
		cout << string(30, '\n');
		cout << "------------------------------------" << endl;

	// ros to pcl
		pcl::fromROSMsg(*rosCloud, *laserCloudIn);
	// 시간 기록
		saveCurrentTime(rosCloud);
	// 라이다-GPS 거리 및 라이다 Y축 회전값 보정
		Eigen::Affine3f transform = Eigen::Affine3f::Identity();
		transform.rotate(Eigen::AngleAxisf(0 * M_PI/180.0, Eigen::Vector3f::UnitY()));
		pcl::transformPointCloud(*laserCloudIn, *laserCloudIn, transform);
	// 포인트 클라우드 가공 및 생성
        getPointCloud();
	// 포인트 클라우드 publish
		publishPointCloud();
	// 데이터 초기화
		clearMemory();

		cout << "------------------------------------" << endl;
    }


	void getPointCloud()
	{
		pointCloudGenerator.getInterestCloud(laserCloudIn, emergencyCloud, {0, 1.5}, {-0.3, 0.3}, {0, 0.5});
		cout << " - emergencyCloud: " << "\033[32m" << emergencyCloud->points.size() << "\033[0m" << endl;
		
		if (emergencyCloud->points.size() > 2)  emergency.data = true;
		else 									emergency.data = false;
	}


	void publishPointCloud()
	{
	// 긴급정지
		pubEmergencyFromLidar.publish(emergency);
	}

	void LidarStop(const std_msgs::Bool::ConstPtr& msg)
	{
		if (msg->data)
		{
			cout << "LiDAR is finished." << endl;
			cout << "2D 라이다로 뭘할 수 있는데^^" << endl;
			ros::shutdown();			
		} 
	}
};


int main (int argc, char** argv)
{
	ros::init(argc, argv, "practics_dynamic_mando");
	ImageProjection imageProjection;
	ros::spin();
}