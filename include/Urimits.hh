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
#include "Trace.hh"
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
    Trace trace;
    State(Pos pos, Pos v, float angle) : pos(pos), v(v), angle(angle) {}
  };

  ////////////////
  // ATTRIBUTES //
  ////////////////
  vector<Pos> allCones;
  dv_msgs::ConeArray data;
  Trace leftTrace, rightTrace;
  Trace shortLeftTrace, shortRightTrace;
  set<int> indexesToExclude;
  bool leftLoopClosed, rightLoopClosed;
  bool loopTLsValidOneTime, loopTLsValid, shortTLsValid;

  /////////////////////
  // PRIVATE METHODS //
  /////////////////////

  // Main Methods
  void reset();
  void computeTrace(Trace &output, const bool &leftOrRight, bool isFirst, const int &max_num_cones) const;
  void computeTraceWithCorrection(Trace &output, Trace &calculatedTrace, const bool &leftOrRight, const int &max_num_cones);
  float getHeuristic(const Pos &nextPos, const State &actState, const bool &firstLeft, const bool &firstRight) const;
  dv_msgs::ConeArrayOrdered *getTLs(Trace left, Trace right) const;

  // Aux Methods
  int nextConeIndex(const State &actState, const bool &firstLeft, const bool &firstRight) const;
  inline bool isLoopClosed(const Trace &trace) const;
  void updateState(State &stateToUpdate, const int &nextConeIndex, const bool &isFirst) const;
  bool segmentIntersectsWithTrace(const int &c1, const int &c2, const Trace &trace) const;
  bool traceIntersectsWithItself(const Trace &trace) const;
  State stateFromTrace(const Trace &trace) const;
  list<int> getPossibleCones(const State &actState, const bool &isFirst) const;
  inline bool stopCondition(const int &nextPossibleIndex, const State &state, const int &max_num_cones) const;
  bool anyIntersection(const Trace &trace1, const Trace &trace2) const;
  Pos centroidOfTrace(Trace trace) const;
  bool validTLs(const Trace &left, const Trace &right, bool checkingLoop) const;

 public:
  // CONSTRUCTOR
  Urimits();

  ////////////
  // PARAMS //
  ////////////
  
  bool compute_short_tls, compute_loop, debug;

  // First
  float first_pseudoPosition_offset;

  // Search
  int max_num_cones_to_consider;
  float min_angle_between_3_cones;

  // Heuristic
  float max_radius_to_next_cone, dist_ponderation;

  // Closure
  int max_trace_length, min_trace_loop_length;

  // Validation Conditions
  float min_percentage_of_cones, max_distSq_betw_trace_centrd;

  ////////////////////
  // PUBLIC METHODS //
  ////////////////////
  void run(const dv_msgs::ConeArray &data, const bool &leftOrRightFirst);
  dv_msgs::ConeArrayOrdered *getLoop() const;
  dv_msgs::ConeArrayOrdered *getShortTLs() const;
  void publishData(const ros::Publisher &tlPub, const ros::Publisher &loopPub) const;
};

#endif