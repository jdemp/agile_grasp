//
// Created by jensen on 11/16/16.
//
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <agile_grasp/Grasp.h>
#include <agile_grasp/Grasps.h>
#include <agile_grasp/identified_object.h>
#include <geometry_msgs/Vector3.h>

const std::string GRASPS_TOPIC = "find_grasps";
std::vector<agile_grasp::Grasp> hands;
std::vector<agile_grasp::identified_object> objects;
bool new_hand = false;
ros::Subscriber agile_grasp_sub;
ros::Subscriber identified_objects_sub;
ros::Publisher hand_pose_pub;

void graspCallback(const agile_grasp::Grasps msg)
{
            hands.clear();
            for (int i=0;i<msg.grasps.size();i++)
            {
                hands.push_back(msg.grasps[i]);
            }
            new_hand = true;
}

agile_grasp::Grasp getHand(std::string object)
{
    return hands[0];
}

bool validGrasp(geometry_msgs::Vector3 object_center, agile_grasp::Grasp g)
{
    return false;
}




int main(int argc, char** argv)
{
    ros::init(argc, argv, "select_object");
    ros::NodeHandle n("~");

    std::string grasps_topic;

    n.param("grasps_topic", grasps_topic, GRASPS_TOPIC);


    agile_grasp_sub = n.subscribe(grasps_topic, 1, graspCallback);
    hand_pose_pub = n.advertise<agile_grasp::Grasp>("hand_pose",10);

    std::cout <<grasps_topic;

    ros::Rate loop_rate(10);


    while(ros::ok())
    {
        //hand_pose_pub.publish(hand_pose);
        if (new_hand)
        {
            agile_grasp::Grasp hand = getHand("hello");
            hand_pose_pub.publish(hand);
            new_hand=false;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}