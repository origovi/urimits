#include <ros/ros.h>

#include "Urimits.hh"

// Publishers are initialized here
ros::Publisher tlPub;
ros::Publisher lapPub;

// TL is initialized here
Urimits urimits;

// This is the map callback
void callback_slam(const dv_msgs::ConeArray::ConstPtr &data) {
  if (not data->cones.empty()) {
    auto ti = chrono::system_clock::now();

    urimits.run(*data, true);
    urimits.publishData(tlPub, lapPub);

    // Elapsed time
    auto tend = chrono::system_clock::now();
    chrono::duration<double> elapsed_seconds = tend - ti;
    ROS_WARN("Urimits ELAPSED TIME: %f ms", elapsed_seconds.count() * 1000);
  }
}
#include "dv_msgs/Cone.h"
#include "geometry_msgs/PoseArray.h"

inline float distSq(const geometry_msgs::Pose &p1, const geometry_msgs::Pose &p2) {
  return (p1.position.x - p2.position.x) * (p1.position.x - p2.position.x) + (p1.position.y - p2.position.y) * (p1.position.y - p2.position.y);
}
// This is the PAU callback
void callback_pau(const geometry_msgs::PoseArray::ConstPtr &data) {
  if (not data->poses.empty()) {
    dv_msgs::ConeArray ca;
    ca.header = data->header;
    list<int> aux;
    for (int i = 0; i < data->poses.size(); ++i) {
      aux.push_back(i);
    }
    // auto it = aux.begin();
    // while (it != aux.end()) {
    //   auto it2 = aux.begin();
    //   while (it2 != aux.end()) {
    //     if (*it != *it2) {
    //       if (distSq(data->poses[*it], data->poses[*it2]) < 0.25) {
    //         it2 = aux.erase(it2);
    //       } else
    //         it2++;
    //     } else
    //       it2++;
    //   }
    //   it++;
    // }
    for (auto it = aux.begin(); it != aux.end(); it++) {
      dv_msgs::Cone c;
      c.x = data->poses[*it].position.x;
      c.y = data->poses[*it].position.y;
      ca.cones.push_back(c);
    }
    auto ti = chrono::system_clock::now();
    urimits.run(ca, true);
    urimits.publishData(tlPub, lapPub);

    // Elapsed time
    auto tend = chrono::system_clock::now();
    chrono::duration<double> elapsed_seconds = tend - ti;
    ROS_WARN("Urimits ELAPSED TIME: %f ms", elapsed_seconds.count() * 1000);
  }
}

// Main
int main(int argc, char **argv) {
  // Init Node:
  ros::init(argc, argv, "urimits");

  // Handle Connections:
  ros::NodeHandle nh;

  nh.param<bool>("urimits/compute_short_tls", urimits.compute_short_tls, false);
  nh.param<bool>("urimits/debug", urimits.debug, false);
  nh.param<float>("urimits/max_radius_to_next_cone", urimits.max_radius_to_next_cone, 5.2);
  nh.param<int>("urimits/max_num_cones_to_consider", urimits.max_num_cones_to_consider, 10);
  nh.param<float>("urimits/first_pseudoPosition_offset", urimits.first_pseudoPosition_offset, 0.5);
  nh.param<float>("urimits/dist_ponderation", urimits.dist_ponderation, 0.5);
  nh.param<float>("urimits/min_angle_between_3_cones", urimits.min_angle_between_3_cones, 0.53);
  nh.param<float>("urimits/min_percentage_of_cones", urimits.min_percentage_of_cones, 90.0);
  nh.param<float>("urimits/max_distSq_betw_trace_centrd", urimits.max_distSq_betw_trace_centrd, 25.0);
  nh.param<int>("urimits/max_trace_length", urimits.max_trace_length, 1e5);
  nh.param<int>("urimits/min_trace_loop_length", urimits.min_trace_loop_length, 10);

  // Publisher & Subscriber:
  ros::Subscriber subMap = nh.subscribe("/cones/opt", 1, callback_slam);
  ros::Subscriber subPau = nh.subscribe("/slam/positions", 1, callback_pau);
  //ros::Subscriber subState = nh.subscribe("/state/car", 1, callback_state);
  tlPub = nh.advertise<dv_msgs::ConeArrayOrdered>("/cones/ordered", 1);
  lapPub = nh.advertise<dv_msgs::ConeArrayOrdered>("/cones/loop", 1);

  ros::spin();
}