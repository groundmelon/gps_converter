#include <iostream>
#include <algorithm>
#include <queue>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <quadrotor_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <boost/format.hpp>

#include <uav_utils/utils.h>
#include "GPSUtils.hpp"

using bfmt = boost::format;
using namespace std;

enum class SignalProcessState { Init, Sampling, Normal };

ros::Publisher height_pub;
ros::Publisher gpsodom_pub;
ros::Publisher velo_pub;

double earth_radius = 6378.137 * 1.0e3;
ros::Duration signal_sample_duration(5.0);
double std_factor = 2;

SignalProcessState signal_process_state = SignalProcessState::Init;

struct Coordinate_t {
    double latitude;
    double longitude;
    Coordinate_t(double x, double y) : latitude(x), longitude(y){};
    Coordinate_t() : Coordinate_t(0, 0){};
};

Coordinate_t gps_origin;
// double origin_x = 0.0;
// double origin_y = 0.0;
double origin_yaw = 0.0;
double current_yaw = 0.0;

std::vector<Coordinate_t> coordList;
std::vector<double> yawList;
ros::Time start_time;

void from_gps_to_metric(double lati, double longti, double& x, double& y) {
    // double la = lati / 180.0 * M_PI;
    // double lo = longti / 180.0 * M_PI;
    // return Coordinate_t(std::cos(la) * std::cos(lo) * earth_radius,
    //                     std::cos(la) * std::sin(lo) * earth_radius);
    double x0, y0, z0;
    GPSUtils::GeodeticToEnu(
        lati, longti, 1.0, gps_origin.latitude, gps_origin.longitude, 1.0, x0, y0, z0);
    
    // Transform from ENU to NWU
    x = y0;
    y = -x0;
}

void imu_callback(const sensor_msgs::ImuConstPtr& pMsg) {
    Eigen::Quaterniond q(
        pMsg->orientation.w, pMsg->orientation.x, pMsg->orientation.y, pMsg->orientation.z);
    double yaw = uav_utils::get_yaw_from_quaternion(q);

    if (signal_process_state == SignalProcessState::Sampling) {
        yawList.push_back(yaw);
    } else if (signal_process_state == SignalProcessState::Normal) {
        current_yaw = yaw;
    }
}

void gps_callback(const sensor_msgs::NavSatFixConstPtr& pMsg) {
    if (pMsg->status.status >= 2) {  // health flag from flight controller
        if (signal_process_state == SignalProcessState::Init) {
            start_time = pMsg->header.stamp;
            signal_process_state = SignalProcessState::Sampling;
            ROS_INFO("[GPS] Initializing...");

        } else if (signal_process_state == SignalProcessState::Sampling) {
            coordList.emplace_back(pMsg->latitude, pMsg->longitude);

            if ((pMsg->header.stamp - start_time) > signal_sample_duration) {
                // Data is enough
                ROS_ASSERT(yawList.size());
                ROS_ASSERT(coordList.size());

                double sum_yaw = 0;
                for (auto y : yawList) {
                    sum_yaw += y;
                }
                origin_yaw = sum_yaw / yawList.size();

                double sum_x = 0;
                double sum_y = 0;
                for (auto& c : coordList) {
                    sum_x += c.latitude;
                    sum_y += c.longitude;
                }
                gps_origin.latitude = sum_x / coordList.size();
                gps_origin.longitude = sum_y / coordList.size();

                ROS_INFO("[GPS] Inited. lati:%.3f longti:%.3f Yaw:%.2f",
                         gps_origin.latitude,
                         gps_origin.longitude,
                         origin_yaw);

                current_yaw = origin_yaw;
                signal_process_state = SignalProcessState::Normal;
            }

        } else if (signal_process_state == SignalProcessState::Normal) {
            double x,y;
            from_gps_to_metric(pMsg->latitude, pMsg->longitude, x, y);
            // Current point in ground frame (North-West-Sky)
            Eigen::Vector3d G_p(x, y, 0.0);

            // Rotation from local frame to ground frame
            Eigen::Matrix3d L_R_G = uav_utils::rotz(origin_yaw).transpose();

            // Current point in local frame
            Eigen::Vector3d L_p = L_R_G * G_p;

            // Quaternion represents ground frame in local frame
            Eigen::Quaterniond q(L_R_G.transpose());

            nav_msgs::Odometry odom_msg;
            odom_msg.header.stamp = pMsg->header.stamp;
            odom_msg.header.frame_id = std::string("world");
            odom_msg.pose.pose.position.x = L_p(0);
            odom_msg.pose.pose.position.y = L_p(1);
            odom_msg.pose.pose.position.z = pMsg->altitude;
            odom_msg.pose.pose.orientation.w = q.w();
            odom_msg.pose.pose.orientation.x = q.x();
            odom_msg.pose.pose.orientation.y = q.y();
            odom_msg.pose.pose.orientation.z = q.z();

            double stdGPS = std_factor * 5.0 / static_cast<double>(pMsg->status.status);
            odom_msg.pose.covariance[0 + 0 * 6] = stdGPS * stdGPS;
            odom_msg.pose.covariance[1 + 1 * 6] = stdGPS * stdGPS;
            odom_msg.pose.covariance[2 + 2 * 6] = stdGPS * stdGPS;

            if (static_cast<double>(pMsg->status.status) < 2.0) {
                odom_msg.header.frame_id = std::string("invalid");
            }

            gpsodom_pub.publish(odom_msg);
        }

    } else {
        ROS_INFO_THROTTLE(5.0, "[GPS] Low signal quality...");
        return;
    }

    geometry_msgs::Vector3Stamped height_msg;
    height_msg.header.stamp = pMsg->header.stamp;
    height_msg.header.frame_id = std::string("world");
    height_msg.vector.x = 0.0;
    height_msg.vector.y = 0.0;
    height_msg.vector.z = pMsg->altitude;

    height_pub.publish(height_msg);
}

void velo_callback(const geometry_msgs::Vector3StampedConstPtr& veloMsg,
                   const sensor_msgs::ImuConstPtr& imuMsg) {
    Eigen::Vector3d g_v = uav_utils::from_vector3_msg(veloMsg->vector);
    Eigen::Quaterniond w_q_b = uav_utils::from_quaternion_msg(imuMsg->orientation);

    // dji velocity is in north-east-ground, transform it to north-west-sky
    g_v.y() *= -1;
    g_v.z() *= -1;

    Eigen::Vector3d b_v = w_q_b.conjugate() * g_v;

    geometry_msgs::Vector3Stamped msg;
    msg.header = veloMsg->header;
    msg.header.frame_id = "body";
    msg.vector = uav_utils::to_vector3_msg(b_v);

    velo_pub.publish(msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "gps_converter");
    ros::NodeHandle nh("~");

    double dur;
    nh.param("signal_sample_duration", dur, signal_sample_duration.toSec());
    nh.param("std_factor", std_factor, std_factor);
    signal_sample_duration = ros::Duration(dur);

    ros::Subscriber gps_sub = nh.subscribe("gps", 10, gps_callback);
    ros::Subscriber imu_sub = nh.subscribe("imu", 10, imu_callback);

    message_filters::Subscriber<geometry_msgs::Vector3Stamped> vel_sub(nh, "velo", 10);
    message_filters::Subscriber<sensor_msgs::Imu> ori_sub(nh, "imu", 10);
    message_filters::TimeSynchronizer<geometry_msgs::Vector3Stamped, sensor_msgs::Imu> sync(
        vel_sub, ori_sub, 10);
    sync.registerCallback(boost::bind(&velo_callback, _1, _2));

    height_pub = nh.advertise<geometry_msgs::Vector3Stamped>("height", 10);
    gpsodom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);
    velo_pub = nh.advertise<geometry_msgs::Vector3Stamped>("body_velocity", 10);

    ros::spin();

    return 0;
}
