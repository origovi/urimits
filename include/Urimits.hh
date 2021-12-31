#ifndef URIMITS_HH
#define URIMITS_HH

#include <algorithm>
#include <iostream>
#include <list>
#include <queue>
#include <set>
#include <vector>

#include "dv_msgs/CarState.h"
#include "dv_msgs/ConeArray.h"
#include "dv_msgs/ConeArrayOrdered.h"
#include "Path.hh"
#include "Pos.hh"
#include "visualization_msgs/Marker.h"

using namespace std;

class Urimits {
 private:
  ////////////////
  // STRUCTURES //
  ////////////////
  struct State {
    Pos pos;
    Pos v;
    float angle;
    Path path;
    State(Pos pos, Pos v, float angle) : pos(pos), v(v), angle(angle) {}
  };

  ////////////////
  // ATTRIBUTES //
  ////////////////
  vector<Pos> allCones;
  dv_msgs::ConeArray data;
  Path leftPath, rightPath;
  Path shortLeftPath, shortRightPath;
  set<int> indexesToExclude;
  bool leftLoopClosed, rightLoopClosed;
  bool trackClosedOneTime, trackClosed;

  /////////////////////
  // PRIVATE METHODS //
  /////////////////////
  void reset();
  void computeTrace(Path &output, const bool &leftOrRight, bool isFirst, const int &max_num_cones) const;
  void computeTraceWithCorrection(Path &output, Path &calculatedPath, const bool &leftOrRight, const int &max_num_cones);
  int nextConeIndex(const State &actState, const bool &firstLeft, const bool &firstRight) const;
  inline bool isLoopClosed(const Path &path) const;
  void updateState(State &stateToUpdate, const int &nextConeIndex, const bool &isFirst) const;
  bool segmentIntersectsWithPath(const int &c1, const int &c2, const Path &path) const;
  bool pathIntersectsWithItself(const Path &path) const;
  State stateFromPath(const Path &path) const;
  float getHeuristic(const Pos &nextPos, const State &actState, const bool &firstLeft, const bool &firstRight) const;
  list<int> getPossibleCones(const State &actState) const;
  dv_msgs::ConeArrayOrdered *getTLs(const Path &left, const Path &right) const;
  inline bool stopCondition(const int &nextPossibleIndex, const State &state, const int &max_num_cones) const;
  bool anyIntersection(const Path &path1, const Path &path2) const;

 public:
  // CONSTRUCTOR
  Urimits();

  // PARAMS
  bool compute_short_tls;
  float max_radius_to_next_cone;
  int max_num_cones_to_consider;
  float first_pseudoPosition_offset;
  float dist_ponderation, min_angle_between_3_cones;
  int max_trace_length, min_trace_loop_length;

  ////////////////////
  // PUBLIC METHODS //
  ////////////////////
  void run(const dv_msgs::ConeArray::ConstPtr &, const bool &);
  void publishData(const ros::Publisher &, const ros::Publisher &) const;
};

#endif