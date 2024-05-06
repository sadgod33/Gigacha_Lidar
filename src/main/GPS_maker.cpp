#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "GPS_maker");
    ros::NodeHandle nh;

    // "/local_msgs_to_vision" 토픽에 PoseStamped 메시지를 발행하는 퍼블리셔를 설정합니다.
    ros::Publisher local_msgs_pub = nh.advertise<geometry_msgs::PoseStamped>("/local_msgs_to_vision", 1000);

    ros::Rate loop_rate(10); // 10 Hz

    while (ros::ok()) {
        geometry_msgs::PoseStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "base_link";

        // 임의의 위치와 자세(오리엔테이션) 값을 생성
        msg.pose.position.x = rand() % 100 - 50;  // -50에서 49 사이의 값
        msg.pose.position.y = rand() % 100 - 50;
        msg.pose.position.z = rand() % 100 - 50;

        tf::Quaternion q = tf::createQuaternionFromYaw((double)(rand() % 360) * M_PI/180);
        msg.pose.orientation.x = q.x();
        msg.pose.orientation.y = q.y();
        msg.pose.orientation.z = q.z();
        msg.pose.orientation.w = q.w();

        // 메시지를 발행합니다.
        local_msgs_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
