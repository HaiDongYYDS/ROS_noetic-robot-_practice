#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main (int argc, char **argv)
{
    ros::init (argc, argv, "eight_word");
    ros::NodeHandle ph;
    ros::Publisher cmd_pub ;
    cmd_pub = ph.advertise<geometry_msgs::Twist>("/mbot1_vel", 20, true);
    // 8字走路的路径参数
    double linear_speed = 0.4; // m/s
    double angular_speed = 0.5; // rad/s
    double duration1 = 12.58;
    double duration2 = 15.85;
    // 持续时间，秒
    ros::Rate loop_rate(50);
    geometry_msgs::Twist msg;
    //前T时间运动一个正向圆
    msg.linear.x = linear_speed;
    msg.angular.z = angular_speed;
    cmd_pub.publish(msg);
    ros::Duration(duration1).sleep();//后T时间运动一个反向圆
    msg.angular.z =-angular_speed*0.8;
    cmd_pub.publish(msg);
    ros::Duration(duration2).sleep();
    //运动完成8字后停止不动
    msg.linear.x=0;
    msg.angular.z=0;
    cmd_pub.publish(msg);
    while(ros::ok())
    {
    ros::spinOnce();
    loop_rate.sleep();
    }
    return 0;
}
