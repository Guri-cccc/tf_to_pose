#include <ros/ros.h>
#include <ros/package.h>
#include <math.h>
#include <vector>
#include <string>
#include <float.h>
#include "yaml-cpp/yaml.h"
#include "typeinfo"
#include <fstream>
#include <iostream>
#include <thread>

#include <fast_tf/fast_tf.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

using namespace ros;
using namespace std;

using std::vector;

class TF_set
{
    public:

    vector<string> tf_names;
    vector<bool> b_tfs;
    vector<geometry_msgs::TransformStamped> m_tfs;
    string topic_name;

};

class TF2Pose
{
public:
    TF2Pose();
    ~TF2Pose();

    bool LoadPrams(std::string path_, vector<TF_set>& tf_set_);

    void TFCallback(const tf2_msgs::TFMessage &msg);
    void TFStaticCallback(const tf2_msgs::TFMessage &msg);

    void run();
    void TrasnfromFT2Pose(vector<geometry_msgs::TransformStamped> transforms_, geometry_msgs::Pose &pose_);

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Subscriber tf_sub, tf_static_sub;
    vector<ros::Publisher> pose_publishers;

    bool b_tf_in, b_is_first;

    vector<vector<bool>> b_tfs;
    vector<vector<geometry_msgs::TransformStamped>> m_tf_i;

    vector<TF_set> tfs;
    string config_path;
};

TF2Pose::TF2Pose() : nh_(""), private_nh_("~")
{
    config_path = ros::package::getPath("tf_to_pose") + "/config/tf_list.yaml";
    cout << "config_path: " << config_path << endl;
    LoadPrams(config_path, tfs);

    tf_sub = nh_.subscribe("/tf", 10, &TF2Pose::TFCallback, this);
    tf_static_sub = nh_.subscribe("/tf_static", 10, &TF2Pose::TFStaticCallback, this);

    for (int i=0; i<tfs.size(); ++i)
    {
        ros::Publisher publisher_buf = nh_.advertise<geometry_msgs::PoseStamped>(tfs[i].topic_name, 1);
        pose_publishers.push_back(publisher_buf);
    }

    b_is_first = true;

    ROS_INFO("TF2Pose is created");
}
TF2Pose::~TF2Pose()
{
    ROS_INFO("TF2Pose is distructed");
}

bool TF2Pose::LoadPrams(std::string path_, vector<TF_set>& tf_set_)
{
    YAML::Node config;
    try{
        config = YAML::LoadFile(path_);
    }
    catch(std::exception &e){
        ROS_ERROR("Failed to load yaml file");
        return false;
    }

    for (const auto& Transformation : config["transformations"]) {
        TF_set tf_buf;
        const YAML::Node& TF = Transformation.second;
        for (const auto& tfFrame : TF["tf_frame_id"])
        {
            tf_buf.tf_names.push_back(tfFrame.as<std::string>());
        }

        tf_buf.b_tfs.resize(tf_buf.tf_names.size()-1, false);
        tf_buf.m_tfs.resize(tf_buf.tf_names.size()-1);
        tf_buf.topic_name = TF["pose_topic_name"].as<std::string>();
        tf_set_.push_back(tf_buf);
    }

    return true;
}

void TF2Pose::TFCallback(const tf2_msgs::TFMessage &msg)
{
    b_tf_in = true;

    for (int j=0; j<tfs.size(); j++)
    {
        for (int i = 0; i < tfs[j].b_tfs.size(); i++)
        {
            geometry_msgs::TransformStamped tf_in;
            if (ftf::FastTF::TFSearch(tfs[j].tf_names[i+1], tfs[j].tf_names[i], msg, tf_in))
            {
                tfs[j].b_tfs[i] = true;
                tfs[j].m_tfs[i] = tf_in;
            }
        }
    }
}

void TF2Pose::TFStaticCallback(const tf2_msgs::TFMessage &msg)
{
    b_tf_in = true;

    for (int j=0; j<tfs.size(); j++)
    {
        for (int i = 0; i < tfs[j].b_tfs.size(); i++)
        {
            geometry_msgs::TransformStamped tf_in;
            if (ftf::FastTF::TFSearch(tfs[j].tf_names[i+1], tfs[j].tf_names[i], msg, tf_in))
            {
                tfs[j].b_tfs[i] = true;
                tfs[j].m_tfs[i] = tf_in;
            }
        }
    }
}

void TF2Pose::run()
{
    if (!b_tf_in)
    {
        cout << "[tag_to_pose_head_cam] /tf topic not in yet" << endl;
        return;
    }

    for (int j=0; j<tfs.size(); j++)
    {
        bool tf_in_check = true;
        for (int i = 0; i < tfs[j].b_tfs.size(); i++)
        {
            if (tfs[j].b_tfs[i] == false)
            {
                tf_in_check = false;
                // if (b_is_first)
                    ROS_WARN_STREAM("tf for " << tfs[j].tf_names[i+1] << " to " << tfs[j].tf_names[i] << " does not exist");
            }
        }

        if (tf_in_check)
        {
            b_is_first = false;

            geometry_msgs::PoseStamped m_pose;
            TrasnfromFT2Pose(tfs[j].m_tfs, m_pose.pose);
            m_pose.header.frame_id = tfs[j].tf_names[tfs[j].tf_names.size()-1];
            m_pose.header.stamp = ros::Time::now();
            pose_publishers[j].publish(m_pose);
        }
    }    
}

void TF2Pose::TrasnfromFT2Pose(vector<geometry_msgs::TransformStamped> transforms_,
                        geometry_msgs::Pose &pose_)
{
    for (int i = 0; i < transforms_.size(); i++)
    {
        ftf::FastTF::TF(pose_, transforms_[i].transform, false);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "TF2Pose");
    ros::NodeHandle _nh;

    printf("Initiate: TF2Pose\n");

    TF2Pose tf_broadcast_;
    ros::Rate loop_rate(1000);
    while (ros::ok())
    {
        ros::spinOnce();
        tf_broadcast_.run();
        loop_rate.sleep();
    }
    printf("Terminate: TF2Pose\n");

    return 0;
}