#ifndef URIMITS_HH
#define URIMITS_HH

#include <algorithm>
#include <iostream>
#include <list>
#include <queue>
#include <set>
#include <vector>

#include "Pos.hh"
#include "Trace.hh"
#include "dv_msgs/CarState.h"
#include "dv_msgs/ConeArray.h"
#include "dv_msgs/ConeArrayOrdered.h"
#include "visualization_msgs/Marker.h"

using namespace std;

class Urimits {
 private:
  typedef pair<float, int> HeurInd;  // first = heuristic, second = coneIndex
  ////////////////
  // STRUCTURES //
  ////////////////
  struct State {
    Pos pos;
    Pos v;
    float angle;
    Trace trace;
    State() {}
    State(Pos pos, Pos v, float angle) : pos(pos), v(v), angle(angle) {}
    inline float getAngleWith(const Pos &conePos) const {
      return Pos::angle(v, pos - conePos);
    }
    static bool compare(const State &s1, const State &s2) {
      return s1.trace.size() < s2.trace.size();
    }
    // static bool compareEnd(const State &s1, const State &s2) {
    //   bool loopClosedS1 = isLoopClosed(s1.trace);
    //   bool loopClosedS2 = isLoopClosed(s2.trace);
    //   if (loopClosedS1 == loopClosedS2) {
    //     return s1.trace.size() < s2.trace.size();
    //   } else
    //     return loopClosedS1;
    // }
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
  void computeTrace(Trace &output, const bool &leftOrRight, bool isRecursive, const int &max_num_cones) const;
  void computeTraceWithCorrection(Trace &output, Trace &calculatedTrace, const bool &leftOrRight, const int &max_num_cones);
  float getHeuristic(const int &nextIndex, const State &actState, const bool &firstLeft, const bool &firstRight) const;
  dv_msgs::ConeArrayOrdered *getTLs(Trace left, Trace right) const;

  // Aux Methods
  State getBestNextCone(const State &actState, const list<HeurInd> &possibleNextCones, const int &max_num_cones, const bool &isFirst) const;
  float getTraceMetric(const Trace &trace) const;
  float getTraceLength(Trace trace) const;
  list<HeurInd> getNextConeIndexes(const State &actState, const bool &leftFirst, const bool &rightFirst) const;
  State getNextState(const State &actState, const HeurInd &uriCone, const bool &isFirst) const;
  int nextConeIndex(const State &actState, const bool &firstLeft, const bool &firstRight) const;
  static inline bool isLoopClosed(const Trace &trace);
  void updateState(State &stateToUpdate, const int &nextConeIndex, const bool &isFirst) const;
  bool segmentIntersectsWithTrace(const int &c1, const int &c2, const Trace &trace) const;
  bool traceIntersectsWithItself(const Trace &trace) const;
  State stateFromTrace(const Trace &trace) const;
  list<int> getPossibleCones(const State &actState, const bool &isFirst) const;
  inline bool stopCondition(const int &nextPossibleIndex, const State &state, const int &max_num_cones) const;
  inline bool stopCondition2(const State &actState, const int &max_num_cones) const;
  bool anyIntersection(const Trace &trace1, const Trace &trace2) const;
  Pos centroidOfTrace(Trace trace) const;
  bool validTLs(const Trace &left, const Trace &right, bool checkingLoop) const;

 public:
  // CONSTRUCTOR
  Urimits();

  ////////////
  // PARAMS //
  ////////////

  bool compute_short_tls, debug;

  // First
  float first_pseudoPosition_offset;

  // Search
  int max_num_cones_to_consider;
  float min_angle_between_3_cones;
  float max_percentage_most_coneHeur;

  // Heuristic
  float max_distSq_to_next_cone, dist_ponderation;

  // Closure
  int max_trace_length, min_trace_loop_length;

  // Validation Conditions
  float min_percentage_of_cones, max_distSq_betw_trace_centrd;

  ////////////////////
  // PUBLIC METHODS //
  ////////////////////
  void run(const dv_msgs::ConeArray &data, const bool &leftOrRightFirst);
  void publishData(const ros::Publisher &tlPub, const ros::Publisher &loopPub) const;
};

#endif