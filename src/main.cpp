#include <ros/ros.h>

#include "urimits.hh"

// Publishers are initialized here
ros::Publisher lapPub;

// TL is initialized here
Urimits urimits;

// This is the map callback
void callback_slam(const dv_msgs::ConeArray::ConstPtr &data) {
  // First, states and cones must not be empty, and not closed loop
  if (not data->cones.empty()) {
    auto ti = chrono::system_clock::now();

    urimits.run(data, true);
    urimits.publishData(lapPub);

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

  nh.param<float>("urimits/max_radius_to_next_cone", urimits.max_radius_to_next_cone, 5.2);
  nh.param<int>("urimits/max_num_cones_to_consider", urimits.max_num_cones_to_consider, 10);
  nh.param<float>("urimits/first_pseudoPosition_offset", urimits.first_pseudoPosition_offset, 0.5);
  nh.param<float>("urimits/dist_ponderation", urimits.dist_ponderation, 0.5);
  nh.param<float>("urimits/min_angle_between_3_cones", urimits.min_angle_between_3_cones, 0.53);
  nh.param<int>("urimits/min_trace_loop_length", urimits.min_trace_loop_length, 10);

  // Publisher & Subscriber:
  ros::Subscriber subMap = nh.subscribe("/cones/opt", 1, callback_slam);
  //ros::Subscriber subState = nh.subscribe("/state/car", 1, callback_state);
  lapPub = nh.advertise<dv_msgs::ConeArrayOrdered>("/cones/ordered", 1);

  ros::spin();
}