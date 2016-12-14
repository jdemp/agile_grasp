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
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
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

const double CUP_WIDTH = .08;
const double BLOCK_WIDTH = .025;

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
const std::string CAMERA_FRAME = "/camera_rgb_optical_frame";
const std::string GRASPS_PUB = "/grasps_visual";
//std::vector<agile_grasp::Grasp> hands;
//std::vector<agile_grasp::identified_object> objects;
bool new_hand = false;
ros::Subscriber agile_grasp_sub;
ros::Subscriber identified_objects_sub;
ros::Publisher object_grasp_pub;
ros::Publisher grasp_pub;
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

//helper functions for grasp points for cup
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
int rateGrasp(agile_grasp::Grasp grasp, geometry_msgs::Vector3 centroid)
{
    double length2 = pow(grasp.axis.x,2) + pow(grasp.axis.y,2) + pow(grasp.axis.z,2);
    double length = sqrt(length2);
    double half_width = grasp.width.data/2;
    geometry_msgs::Vector3 v;
    v.x = grasp.axis.x/length;
    v.y = grasp.axis.y/length;
    v.z = grasp.axis.z/length;
    geometry_msgs::Vector3 grasp_ll_corner = getLowerLeftCorner(v,half_width, grasp.surface_center);
    geometry_msgs::Vector3 grasp_lr_corner = getLowerRightCorner(v,half_width,grasp.surface_center);
    geometry_msgs::Vector3 grasp_ul_corner = getUpperLeftCorner(v,half_width,grasp.center);
    geometry_msgs::Vector3 grasp_ur_corner = getUpperRightCorner(v,half_width,grasp.center);

    //get the min x,y,z for the hand from the corners
    double min_x = getMin(grasp_lr_corner.x, grasp_ur_corner.x,grasp_ll_corner.x, grasp_ul_corner.x);
    double min_y = getMin(grasp_lr_corner.y, grasp_ur_corner.y,grasp_ll_corner.y, grasp_ul_corner.y);
    //double min_z = getMin(grasp.grasp_lr_corner.z, grasp.grasp_ur_corner.z,grasp.grasp_ll_corner.z, grasp.grasp_ul_corner.z);

    //get the max x,y,z for the hand from the corners
    double max_x = getMax(grasp_lr_corner.x, grasp_ur_corner.x,grasp_ll_corner.x, grasp_ul_corner.x);
    double max_y = getMax(grasp_lr_corner.y, grasp_ur_corner.y,grasp_ll_corner.y, grasp_ul_corner.y);
    //double max_z = getMax(grasp.grasp_lr_corner.z, grasp.grasp_ur_corner.z,grasp.grasp_ll_corner.z, grasp.grasp_ul_corner.z);

    int rating=0;

    if(centroid.x > min_x and centroid.x < max_x){rating++;}
    if(centroid.y > min_y and centroid.y < max_y){rating++;}

    return rating;
}

geometry_msgs::Vector3 getMidpoint(geometry_msgs::Vector3 point1, geometry_msgs::Vector3 point2)
{
    geometry_msgs::Vector3 midpoint;
    midpoint.x = (point1.x + point2.x)/2;
    midpoint.y = (point1.y + point2.y)/2;
    midpoint.z = (point1.z + point2.z)/2;
    return midpoint;

}

/*
bool validGrasp(agile::Grasp grasp, geometry_msgs::Vector3 centroid)
{

}
*/

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


//Visualization Messages

visualization_msgs::Marker createMarker(const std::string& frame)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame;
    marker.header.stamp = ros::Time::now();
    marker.lifetime = ros::Duration(10);
    marker.action = visualization_msgs::Marker::ADD;
    return marker;
}

visualization_msgs::Marker createApproachMarker(const std::string& frame, const geometry_msgs::Point& center,
                                                      const geometry_msgs::Vector3 approach, int id, double diam)
{
    visualization_msgs::Marker marker = createMarker(frame);
    marker.type = visualization_msgs::Marker::ARROW;
    marker.id = id;
    marker.scale.x = diam; // shaft diameter
    marker.scale.y = diam; // head diameter
    marker.scale.z = 0.01; // head length
    marker.color.r = 0;
    marker.color.g = 1;
    marker.color.b = 0;
    marker.color.a = .4;
    geometry_msgs::Point p, q;
    p.x = center.x;
    p.y = center.y;
    p.z = center.z;
    q.x = p.x - 0.05 * approach.x;
    q.y = p.y - 0.05 * approach.y;
    q.z = p.z - 0.05 * approach.z;
    marker.points.push_back(p);
    marker.points.push_back(q);
    return marker;
}


void publishGraspMarkers()
{
    visualization_msgs::MarkerArray marker_array;
    for(int i =0;i<objects.size();i++)
    {
        geometry_msgs::Point position;
        position.x = objects[i].grasp.surface_center.x;
        position.y = objects[i].grasp.surface_center.y;
        position.z = objects[i].grasp.surface_center.z;
        visualization_msgs::Marker marker = createApproachMarker(CAMERA_FRAME, position, objects[i].grasp.approach, i, 0.008);
        marker.id = i;
        marker_array.markers.push_back(marker);
    }

    grasp_pub.publish(marker_array);
}




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
                if(index>=0 and msg.transforms[i].transform.translation.x==0 and msg.transforms[i].transform.translation.y==0 and msg.transforms[i].transform.translation.z==0)
                {
                    objects.erase(objects.begin()+index);
                }
                else if (index>=0)
                {
                    objects[index].centroid = msg.transforms[i].transform.translation;
                    objects[index].rotation = msg.transforms[i].transform.rotation;
                }
                else if (msg.transforms[i].transform.translation.x!=0 and msg.transforms[i].transform.translation.y!=0 and msg.transforms[i].transform.translation.z!=0)
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
        float width = msg.grasps[i].width.data;
        for(int j=0;j<objects.size();j++)
        {
            geometry_msgs::Vector3 centroid = objects[j].centroid;
            if(objects[j].object_type==BLOCK and width<(BLOCK_WIDTH+.05) and width>(BLOCK_WIDTH-.05))
            {
                float dist = distanceCalc(center,centroid);
                if(dist<min_dist and !objects[j].hasGrasp and dist<.02)
                {
                    min_dist=dist;
                    best_object_index = j;
                    grasp_type = FINGER_TIP;
                }
                else if(dist<min_dist and objects[j].hasGrasp and dist<objects[j].grasp_distance and dist<.02)
                {
                    min_dist=dist;
                    best_object_index = j;
                    grasp_type = FINGER_TIP;
                }
            }
            else if(objects[j].object_type==CUP and width<(CUP_WIDTH+.01) and width>(CUP_WIDTH-.01))
            {
                geometry_msgs::Vector3 midpoint = getMidpoint(msg.grasps[i].surface_center, msg.grasps[i].center);
                float dist = distanceCalc(msg.grasps[i], centroid);
                int rating = rateGrasp(msg.grasps[i],centroid);
                if(!objects[j].hasGrasp and rating>=2 and dist<min_dist and dist<.08)
                {
                    best_object_index=j;
                    min_dist = dist;
                    grasp_type=FULL;
                }
                else if (rating>=2 and dist<min_dist and dist<objects[j].grasp_distance and dist<.08)
                {
                    best_object_index=j;
                    min_dist = dist;
                    grasp_type=FULL;
                }
            }
            else
            {
                /*
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
                 */
                continue;
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
        publishGraspMarkers();
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
    std::string grasp_pub_topic;

    n.param("grasps_topic", grasps_topic, GRASPS_TOPIC);
    n.param("object_grasp_topic", object_grasp_topic, OBJECT_GRASP_TOPIC);
    n.param("object_topic", object_topic, OBJECT_TOPIC);
    n.param("grasps_pub", grasp_pub_topic, GRASPS_PUB);



    agile_grasp_sub = n.subscribe(grasps_topic, 1, graspCallback);
    object_grasp_pub = n.advertise<agile_grasp::object_grasp_list>(object_grasp_topic,10);
    identified_objects_sub = n.subscribe(object_topic,10, objectCallback);
    grasp_pub = n.advertise<visualization_msgs::MarkerArray>(grasp_pub_topic,5);


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