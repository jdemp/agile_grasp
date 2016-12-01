//
// Created by jensen on 11/16/16.
//
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <agile_grasp/Grasp.h>
#include <agile_grasp/Grasps.h>
#include <agile_grasp/identified_object.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>
#include <Eigen/Dense>
#include <math.h>
#include <algorithm>




struct full_grasp{
    agile_grasp::Grasp grasp;
    geometry_msgs::Vector3 grasp_ur_corner;
    geometry_msgs::Vector3 grasp_ul_corner;
    geometry_msgs::Vector3 grasp_lr_corner;
    geometry_msgs::Vector3 grasp_ll_corner;
};

struct object_grasp{
    std::string type;
    int objectID;
    geometry_msgs::Vector3 centroid;

    bool hasGrasp;
    int graspRating;

    full_grasp grasp;
};


const std::string GRASPS_TOPIC = "/find_grasps";
const std::string OBJECT_GRASP_TOPIC = "/object_grasp";
const std::string OBJECT_TOPIC = "/objects";
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
}

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

//takes a full grasp with corners, and compares it to the objects and sees if it is a better grasp (based on distances and what not)
void assignGrasp(full_grasp grasp)
{
    for(int i=0; i<objects.size();i++)
    {
        int rating = rateGrasp(objects[i].grasp, objects[i].centroid);
        if (rating>=2)
        {
            if(!objects[i].hasGrasp)
            {
                objects[i].graspRating = rating;
                objects[i].grasp = grasp;
                objects[i].hasGrasp = true;
            }
            else if (rating > objects[i].graspRating)
            {
                objects[i].graspRating = rating;
                objects[i].grasp = grasp;
            }
            else
            {
                //compare grasps
            }
        }

    }
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


void objectCallback(const agile_grasp::identified_object msg)
{
    if(objects.empty())
    {
        objects.push_back(object_grasp());
        objects[0].objectID = msg.object_id;
        objects[0].type = msg.object_identity;
        objects[0].hasGrasp = false;
        objects[0].centroid.x = msg.pose.position.x;
        objects[0].centroid.y = msg.pose.position.y;
        objects[0].centroid.z = msg.pose.position.z;
    }
    else
    {
        bool object_found =false;
        for(int i=0; i<objects.size();i++)
        {
            int id = objects[i].objectID;
            if(id==msg.object_id)
            {
                updateObject(msg,i);
                object_found = true;
                break;
            }
        }
        if(!object_found)
        {
            objects.push_back(object_grasp());
            int index = objects.size()-1;
            objects[index].objectID = msg.object_id;
            objects[index].type = msg.object_identity;
            objects[index].hasGrasp = false;
            objects[index].centroid.x = msg.pose.position.x;
            objects[index].centroid.y = msg.pose.position.y;
            objects[index].centroid.z = msg.pose.position.z;
        }
    }
}

void graspCallback(const agile_grasp::Grasps msg)
{
    for (int i=0;i<msg.grasps.size();i++)
    {
        full_grasp temp;
        temp.grasp = msg.grasps[i];
        double length2 = pow(temp.grasp.axis.x,2) + pow(temp.grasp.axis.y,2) + pow(temp.grasp.axis.z,2);
        double length = sqrt(length2);
        double half_width = temp.grasp.width.data/2;
        geometry_msgs::Vector3 v;
        v.x = temp.grasp.axis.x/length;
        v.y = temp.grasp.axis.y/length;
        v.z = temp.grasp.axis.z/length;
        temp.grasp_ll_corner = getLowerLeftCorner(v,half_width, temp.grasp.surface_center);
        temp.grasp_lr_corner = getLowerRightCorner(v,half_width,temp.grasp.surface_center);
        temp.grasp_ul_corner = getUpperLeftCorner(v,half_width,temp.grasp.center);
        temp.grasp_ur_corner = getUpperRightCorner(v,half_width,temp.grasp.center);
        assignGrasp(temp);

    }
}

//modify later to work with ids
void findGraspCallback(const std_msgs::String msg)
{
    std::string type = msg.data;
    for(int i=0; i<objects.size();i++)
    {
        if(objects[i].type.compare(type)==0)
        {
            if(objects[i].hasGrasp)
            {
                object_grasp_pub.publish(objects[i].grasp.grasp);
                break;
            }
            else{std::cout << "Object found but has no grasp";}
        }
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
    object_grasp_pub = n.advertise<agile_grasp::Grasp>(object_grasp_topic,10);
    identified_objects_sub = n.subscribe(object_topic,10, objectCallback);


    std::cout <<grasps_topic;

    ros::Rate loop_rate(10);


    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}