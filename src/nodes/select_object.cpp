//
// Created by jensen on 11/16/16.
//
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <agile_grasp/Grasp.h>
#include <agile_grasp/Grasps.h>
#include <agile_grasp/object_grasp.h>
#include <agile_grasp/object_grasp_list.h>
#include <agile_grasp/identified_object.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Float32.h>
#include <tf2_msgs/TFMessage.h>
#include <Eigen/Dense>
#include <math.h>
#include <algorithm>
#include <iostream>


// for the type of object
const int DONT_TRACK = -1;
const int UNKNOWN = 0;
const int BLOCK = 1;
const int CUP = 2;

const int NONE = -1;
const int FINGER_TIP = 0;
const int FULL = 1;

struct object_grasp{
    std::string object_id;
    int object_type;
    geometry_msgs::Vector3 centroid;
    geometry_msgs::Quaternion rotation;

    bool hasGrasp;
    int graspRating;
    float grasp_distance; //store the distance from the centroid to the grasp
    int grasp_type;

    agile_grasp::Grasp grasp;

    //used for visualization and non block comparisons
    geometry_msgs::Vector3 grasp_ur_corner;
    geometry_msgs::Vector3 grasp_ul_corner;
    geometry_msgs::Vector3 grasp_lr_corner;
    geometry_msgs::Vector3 grasp_ll_corner;
};


const std::string GRASPS_TOPIC = "/find_grasps";
const std::string OBJECT_GRASP_TOPIC = "/object_grasp";
const std::string OBJECT_TOPIC = "/tf";
//std::vector<agile_grasp::Grasp> hands;
//std::vector<agile_grasp::identified_object> objects;
bool new_hand = false;
ros::Subscriber agile_grasp_sub;
ros::Subscriber identified_objects_sub;
ros::Publisher object_grasp_pub;
std::vector<object_grasp> objects;



//helpers for finding the smallest and largest x,y,z values in the hands
double getMax(double a, double b, double c, double d)
{
    double max = a;
    if(b>max){max=b;}
    if(c>max){max=c;}
    if(d>max){max=d;}
    return max;
}

double getMin(double a, double b, double c, double d)
{
    double min = a;
    if(b<min){min=b;}
    if(c<min){min=c;}
    if(d<min){min=d;}
    return min;
}/*
//helper functions for grasp points
geometry_msgs::Vector3 getUpperLeftCorner(geometry_msgs::Vector3 v, double distance, geometry_msgs::Vector3 center)
{
    geometry_msgs::Vector3 corner;
    corner.x = center.x + distance*v.x;
    corner.y = center.y + distance*v.y;
    corner.z = center.z + distance*v.z;
    return corner;
}

geometry_msgs::Vector3 getUpperRightCorner(geometry_msgs::Vector3 v, double distance, geometry_msgs::Vector3 center)
{
    geometry_msgs::Vector3 corner;
    corner.x = center.x - distance*v.x;
    corner.y = center.y - distance*v.y;
    corner.z = center.z - distance*v.z;
    return corner;
}

geometry_msgs::Vector3 getLowerLeftCorner(geometry_msgs::Vector3 v, double distance, geometry_msgs::Vector3 surface_center)
{
    geometry_msgs::Vector3 corner;
    corner.x = surface_center.x + distance*v.x;
    corner.y = surface_center.y + distance*v.y;
    corner.z = surface_center.z + distance*v.z;
    return corner;


}

geometry_msgs::Vector3 getLowerRightCorner(geometry_msgs::Vector3 v, double distance, geometry_msgs::Vector3 surface_center)
{
    geometry_msgs::Vector3 corner;
    corner.x = surface_center.x - distance*v.x;
    corner.y = surface_center.y - distance*v.y;
    corner.z = surface_center.z - distance*v.z;
    return corner;
}


//checks if the grasp is in the area of the centroid, the centroids x,y lay somewhere in the grasp
//it gets 1 point for each axis the centroid falls between
//ie if centroid.x is a valid x and a valid y it gets a 2
//may take into account type of object at somepoint (mainly for blocks)
int rateGrasp(full_grasp grasp, geometry_msgs::Vector3 centroid)
{
    //get the min x,y,z for the hand from the corners
    double min_x = getMin(grasp.grasp_lr_corner.x, grasp.grasp_ur_corner.x,grasp.grasp_ll_corner.x, grasp.grasp_ul_corner.x);
    double min_y = getMin(grasp.grasp_lr_corner.y, grasp.grasp_ur_corner.y,grasp.grasp_ll_corner.y, grasp.grasp_ul_corner.y);
    //double min_z = getMin(grasp.grasp_lr_corner.z, grasp.grasp_ur_corner.z,grasp.grasp_ll_corner.z, grasp.grasp_ul_corner.z);

    //get the max x,y,z for the hand from the corners
    double max_x = getMax(grasp.grasp_lr_corner.x, grasp.grasp_ur_corner.x,grasp.grasp_ll_corner.x, grasp.grasp_ul_corner.x);
    double max_y = getMax(grasp.grasp_lr_corner.y, grasp.grasp_ur_corner.y,grasp.grasp_ll_corner.y, grasp.grasp_ul_corner.y);
    //double max_z = getMax(grasp.grasp_lr_corner.z, grasp.grasp_ur_corner.z,grasp.grasp_ll_corner.z, grasp.grasp_ul_corner.z);

    int rating=0;

    if(centroid.x > min_x and centroid.x < max_x){rating++;}
    if(centroid.y > min_y and centroid.y < max_y){rating++;}

    return rating;
}


//checks if the grasp is in the area of the centroid, the centroids x,y lay somewhere in the grasp

void updateObject(agile_grasp::identified_object msg, int index)
{
    objects[index].centroid.x = msg.pose.position.x;
    objects[index].centroid.y = msg.pose.position.y;
    objects[index].centroid.z = msg.pose.position.z;
    //add a way to validate grasp
    int rating = rateGrasp(objects[index].grasp,objects[index].centroid);
    if (rating<2)
    {
        objects[index].hasGrasp =false;
    }

}
*/

int getObjectType(std::string object_name)
{
    if(object_name.compare(0,5,"block")==0 or object_name.compare(0,5,"Block")==0){return BLOCK;}
    else if(object_name.compare(0,3,"cup")==0 or object_name.compare(0,3,"Cup")==0){return CUP;}
    else{return DONT_TRACK;}
}


int findObject(std::string object_name)
{
    for(int i=0;i<objects.size();i++)
    {
        if(object_name.compare(objects[i].object_id)==0)
        {
            return i;
        }
    }
    return -1;
}

//may add a validate grasp after each update
void objectCallback(const tf2_msgs::TFMessage msg)
{
    for(int i=0; i<msg.transforms.size(); i++)
    {
        std::string object_name = msg.transforms[i].child_frame_id;
        int type = getObjectType(object_name);
        if(type>=0)
        {
            if(!objects.empty())
            {
                int index = findObject(object_name);
                if(index>=0)
                {
                    objects[index].centroid = msg.transforms[i].transform.translation;
                    objects[index].rotation = msg.transforms[i].transform.rotation;
                }
                else
                {
                    objects.push_back(object_grasp());
                    int last = objects.size()-1;
                    objects[last].object_id = object_name;
                    objects[last].object_type = type;
                    objects[last].hasGrasp = false;
                    objects[last].centroid = msg.transforms[i].transform.translation;
                    objects[last].rotation = msg.transforms[i].transform.rotation;
                }
            }
            else
            {
                objects.push_back(object_grasp());
                objects[0].object_id = object_name;
                objects[0].object_type = type;
                objects[0].hasGrasp = false;
                objects[0].centroid = msg.transforms[i].transform.translation;
                objects[0].rotation = msg.transforms[i].transform.rotation;
            }
        }
    }
}


agile_grasp::object_grasp getObjectGraspMsg(int index)
{
    agile_grasp::object_grasp m;
    m.object = objects[index].object_id;
    m.centroid = objects[index].centroid;
    m.grasp = objects[index].grasp;
    return m;
}



agile_grasp::object_grasp_list generateObjectGraspListMessage()
{
    //agile_grasp::object_grasp_list msg;
    agile_grasp::object_grasp_list msg;
    for(int i =0;i<objects.size();i++)
    {
        agile_grasp::object_grasp m = getObjectGraspMsg(i);
        msg.list.push_back(m);
    }
    return msg;
}

float distanceCalc(const geometry_msgs::Vector3 center, const geometry_msgs::Vector3 centroid)
{
    float x = center.x-centroid.x;
    float y = center.y-centroid.y;
    float z = center.z-centroid.z;
    return std::sqrt(std::pow(x,2) + std::pow(y,2) + std::pow(z,2));
}


void graspCallback(const agile_grasp::Grasps msg)
{
    for (int i=0;i<msg.grasps.size();i++)
    {
        float min_dist = 10000000.0;
        int best_object_index =-1;
        int grasp_type = -1;
        geometry_msgs::Vector3 center = msg.grasps[i].center;
        for(int j=0;j<objects.size();j++)
        {
            geometry_msgs::Vector3 centroid = objects[j].centroid;
            if(objects[j].object_type==BLOCK)
            {
                float dist = distanceCalc(center,centroid);
                if(dist<min_dist and !objects[j].hasGrasp)
                {
                    min_dist=dist;
                    best_object_index = j;
                    grasp_type = FINGER_TIP;
                }
                else if(dist<min_dist and objects[j].hasGrasp and dist<objects[i].grasp_distance)
                {
                    min_dist=dist;
                    best_object_index = j;
                    grasp_type = FINGER_TIP;
                }
            }
        }

        if(best_object_index>=0)
        {
            objects[best_object_index].grasp = msg.grasps[i];
            objects[best_object_index].hasGrasp = true;
            objects[best_object_index].grasp_distance = min_dist;
            objects[best_object_index].grasp_type = grasp_type;
        }

    }
    if(!objects.empty())
    {
        object_grasp_pub.publish(generateObjectGraspListMessage());
    }
}

//modify later to work with ids
void findGraspCallback(const std_msgs::String msg)
{
    std::string object = msg.data;
    int index = findObject(object);
    if (index>=0)
    {
        std::cout << "Object found";
    }
    else
    {
        std::cout << "Object does not exist";
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "select_object");
    ros::NodeHandle n("~");

    std::string grasps_topic;
    std::string object_grasp_topic;
    std::string object_topic;

    n.param("grasps_topic", grasps_topic, GRASPS_TOPIC);
    n.param("object_grasp_topic", object_grasp_topic, OBJECT_GRASP_TOPIC);
    n.param("object_topic", object_topic, OBJECT_TOPIC);



    agile_grasp_sub = n.subscribe(grasps_topic, 1, graspCallback);
    object_grasp_pub = n.advertise<agile_grasp::object_grasp_list>(object_grasp_topic,10);
    identified_objects_sub = n.subscribe(object_topic,10, objectCallback);


    std::cout << "Grasp Topic: "<<grasps_topic <<"\n";
    std::cout << "Object Topic: " << object_topic <<"\n";
    std::cout << "Object Grasp Topic: " << object_grasp_topic << "\n";

    ros::Rate loop_rate(10);


    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}