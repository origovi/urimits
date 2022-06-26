#include <ros/ros.h>

#include "Urimits.hpp"
#include <iostream>

// Publishers are initialized here
ros::Publisher tlPub;
ros::Publisher lapPub;
ros::Publisher fullMarkersPub, partialMarkersPub;

// TL is initialized here
Urimits urimits;

// This is the map callback
void callback_ccat(const as_msgs::ConeArray::ConstPtr &data) {
  if (not data->cones.empty()) {
    auto ti = chrono::system_clock::now();

    urimits.run(*data, true);
    urimits.publishData(tlPub, lapPub, fullMarkersPub, partialMarkersPub);

    // Elapsed time
    static int i = 0;
    static double mean = 0.0;
    auto tend = chrono::system_clock::now();
    chrono::duration<double> elapsed_seconds = tend - ti;
    std::cout << i++ << ',' << elapsed_seconds.count() * 1000 << std::endl;
    mean += elapsed_seconds.count() * 1000;
    if (i %100 == 0) std::cout << "mean: " << mean/(i) << std::endl;
  }
}

// Main
int main(int argc, char **argv) {
  // Init Node:
  ros::init(argc, argv, "urimits");

  // Handle Connections:
  ros::NodeHandle nh;

  nh.param<bool>("urimits/compute_short_tls", urimits.compute_short_tls, false);
  nh.param<bool>("urimits/compute_loop", urimits.compute_loop, false);
  nh.param<bool>("urimits/debug", urimits.debug, false);

  std::string input_topic, output_full_topic, output_partial_topic;
  nh.param<std::string>("urimits/input_topic", input_topic, "/AS/P/ccat/cones");
  nh.param<std::string>("urimits/output_full_topic", output_full_topic, "/AS/P/tracklimits/full");
  nh.param<std::string>("urimits/output_partial_topic", output_partial_topic, "/AS/P/tracklimits/partial");

  nh.param<float>("urimits/max_radius_to_next_cone", urimits.max_radius_to_next_cone, 5.2);
  nh.param<int>("urimits/max_num_cones_to_consider", urimits.max_num_cones_to_consider, 10);
  nh.param<float>("urimits/first_pseudoPosition_offset", urimits.first_pseudoPosition_offset, 0.5);
  nh.param<float>("urimits/dist_ponderation", urimits.dist_ponderation, 0.5);
  nh.param<float>("urimits/min_angle_loop", urimits.min_angle_loop, 0.4);
  nh.param<float>("urimits/min_percentage_of_cones", urimits.min_percentage_of_cones, 90.0);
  nh.param<float>("urimits/max_distSq_betw_trace_centrd", urimits.max_distSq_betw_trace_centrd, 25.0);
  nh.param<float>("urimits/min_angle_short", urimits.min_angle_short, 1.74);
  nh.param<int>("urimits/max_short_trace_length", urimits.max_short_trace_length, 10);
  nh.param<int>("urimits/min_trace_loop_length", urimits.min_trace_loop_length, 10);
  nh.param<bool>("urimits/publish_markers", urimits.publish_markers, false);

  std::string markers_full_topic, markers_partial_topic;
  nh.param<std::string>("urimits/markers_full_topic", markers_full_topic, "/AS/P/urimits/markers/full");
  nh.param<std::string>("urimits/markers_partial_topic", markers_partial_topic, "/AS/P/urimits/markers/partial");

  // Publishers & Subscriber:
  ros::Subscriber subMap = nh.subscribe(input_topic, 1, callback_ccat);

  tlPub = nh.advertise<as_msgs::Tracklimits>(output_full_topic, 1);
  lapPub = nh.advertise<as_msgs::Tracklimits>(output_partial_topic, 1);

  if (urimits.publish_markers) {
    fullMarkersPub = nh.advertise<visualization_msgs::MarkerArray>(markers_full_topic, 1);
    partialMarkersPub = nh.advertise<visualization_msgs::MarkerArray>(markers_partial_topic, 1);
  }

  ros::spin();
}