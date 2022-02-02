#include "Urimits.hh"

// Namespaces
using namespace std;

// Constructor
Urimits::Urimits() {
  this->loopTLsValidOneTime = false;
}

////////////////////
// PUBLIC METHODS //
////////////////////

/**
 * Runs the Urimits algorithm having as an input a dv_msgs::ConeArray
 * publishes the tracklimits, and if it detects a the whole track, it
 * also saves the whole tracklimits.
 */
void Urimits::run(const dv_msgs::ConeArray &data, const bool &leftOrRightFirst) {
  if (data.cones.size() == 0) return;
  //if (data->cones.size() == 0) return;

  reset();
  this->data = data;
  //this->data = *data;
  this->allCones.resize(this->data.cones.size());
  for (int i = 0; i < this->data.cones.size(); i++) {
    this->allCones[i] = Pos(this->data.cones[i].x, this->data.cones[i].y);
  }

  Trace *first, *shortFirst, *second, *shortSecond;
  if (leftOrRightFirst) {
    first = &this->leftTrace;
    shortFirst = &this->shortLeftTrace;
    second = &this->rightTrace;
    shortSecond = &this->shortRightTrace;
  } else {
    first = &this->rightTrace;
    shortFirst = &this->shortRightTrace;
    second = &this->leftTrace;
    shortSecond = &this->shortLeftTrace;
  }

  // Compute the short TLs
  if (this->compute_short_tls) {
    computeTrace(*shortFirst, leftOrRightFirst, true, this->max_trace_length);
    computeTraceWithCorrection(*shortSecond, *shortFirst, !leftOrRightFirst, this->max_trace_length);
    this->indexesToExclude.clear();
  }

  // Compute the loop TLs
  if (this->compute_loop) {
    computeTrace(*first, leftOrRightFirst, true, 1e5);
    computeTraceWithCorrection(*second, *first, !leftOrRightFirst, 1e5);
  }

  // Validate the traces
  if (this->compute_loop and (debug or validTLs(this->leftTrace, this->rightTrace, true))) {
    this->loopTLsValid = true;
    this->loopTLsValidOneTime = true;
  }
  
  if (this->compute_short_tls and (debug or (!this->loopTLsValidOneTime and validTLs(this->shortLeftTrace, this->shortRightTrace, false)))) {
    this->shortTLsValid = true;
  }
}

void Urimits::publishData(const ros::Publisher &tlPub, const ros::Publisher &loopPub) const {
  if (this->loopTLsValid) {
    dv_msgs::ConeArrayOrdered *tls = getTLs(this->leftTrace, this->rightTrace);
    ROS_WARN("LOOP PUBLISHED");
    //loopPub.publish(*tls);
    tlPub.publish(*tls);
    delete tls;
  } else if (this->shortTLsValid) {
    dv_msgs::ConeArrayOrdered *tls = getTLs(this->shortLeftTrace, this->shortRightTrace);
    ROS_WARN("TLs PUBLISHED");
    tlPub.publish(*tls);
    delete tls;
  }
}

dv_msgs::ConeArrayOrdered *Urimits::getShortTLs() const {
  return getTLs(this->shortLeftTrace, this->shortRightTrace);
}

dv_msgs::ConeArrayOrdered *Urimits::getLoop() const {
  return getTLs(this->leftTrace, this->rightTrace);
}

/////////////////////
// PRIVATE METHODS //
/////////////////////
void Urimits::reset() {
  this->indexesToExclude.clear();
  this->leftTrace.clear();
  this->shortLeftTrace.clear();
  this->rightTrace.clear();
  this->shortRightTrace.clear();
  this->loopTLsValid = false;
  this->shortTLsValid = false;
}

inline bool Urimits::isLoopClosed(const Trace &trace) const {
  if (trace.size() < 3) return false;
  return trace.first().coneIndex() == trace.coneIndex();
}

/**
 * Checks whether the two traces have any intersection with one another or
 * within themselves
 * O(n²), n=max(this->leftTrace.size(), this->rightTrace.size())
 */
bool Urimits::anyIntersection(const Trace &trace1, const Trace &trace2) const {
  // Check for intersections within the same trace
  if (traceIntersectsWithItself(trace1) or traceIntersectsWithItself(trace2)) return true;

  // Check for intersections between traces
  if (trace1.size() < 2 or trace2.size() < 2) return false;

  int left_ind = trace1.coneIndex();
  Trace left_copy = trace1.before();
  while (!left_copy.empty()) {
    int right_ind = trace2.coneIndex();
    Trace right_copy = trace2.before();
    while (!right_copy.empty()) {
      if (left_ind == right_ind or left_ind == right_copy.coneIndex() or left_copy.coneIndex() == right_ind or left_copy.coneIndex() == right_copy.coneIndex() or
          Pos::intersect(this->allCones[left_ind], this->allCones[left_copy.coneIndex()], this->allCones[right_ind], this->allCones[right_copy.coneIndex()])) return true;
      right_ind = right_copy.coneIndex();
      right_copy = right_copy.before();
    }
    left_ind = left_copy.coneIndex();
    left_copy = left_copy.before();
  }
  return false;
}

/**
 * Stopping condition of the trace computing part. Returns true when a longer trace is impossible to compute.
 */
inline bool Urimits::stopCondition(const int &nextPossibleIndex, const State &state, const int &max_num_cones) const {
  if (isLoopClosed(state.trace))
    return true;
  else
    return nextPossibleIndex == -1 or state.trace.size() >= max_num_cones;
}

/**
 * Computes the first trace of cones, it uses a greedy algorithm with an heuristic, taking at each iteration the
 * best possible cone, until a stopping condition is found.
 * O(n²)
 */
void Urimits::computeTrace(Trace &output, const bool &leftOrRight, bool isFirst, const int &max_num_cones) const {
  State actState(Pos(0, 0), Pos(this->first_pseudoPosition_offset, 0), M_PI);
  if (!isFirst)
    actState = stateFromTrace(output);
  int nextPossibleIndex = nextConeIndex(actState, leftOrRight and isFirst, !leftOrRight and isFirst);
  while (!stopCondition(nextPossibleIndex, actState, max_num_cones)) {
    updateState(actState, nextPossibleIndex, isFirst);
    isFirst = false;
    nextPossibleIndex = nextConeIndex(actState, leftOrRight and isFirst, !leftOrRight and isFirst);
    //cout << "Num cons " << possibleCones.size() << endl;
  }
  //cout << actState.trace << endl;
  output = actState.trace;
}

/**
 * Computes the second trace correcting itself when it the trace wants a cone that is already taken
 * by the first trace, it decides which trace belongs to this cone. If the first trace was computed
 * with missgiven cone, it recomputes the first one and continues with the second one.
 * At most O(n⁴), n=(num cones)
 */
void Urimits::computeTraceWithCorrection(Trace &output, Trace &calculatedTrace, const bool &leftOrRight, const int &max_num_cones) {
  State actState(Pos(0, 0), Pos(this->first_pseudoPosition_offset, 0), M_PI);
  bool isFirst = true;
  int nextPossibleIndex = nextConeIndex(actState, leftOrRight, !leftOrRight);
  while (!stopCondition(nextPossibleIndex, actState, max_num_cones)) {
    if (calculatedTrace.containsCone(nextPossibleIndex)) {
      State possibleState = actState;
      this->indexesToExclude.insert(nextPossibleIndex);
      //cout << "COLISION" << endl;
      updateState(possibleState, nextPossibleIndex, isFirst);
      int next2PossibleIndex = nextConeIndex(possibleState, false, false);
      if (next2PossibleIndex != -1) {
        updateState(possibleState, next2PossibleIndex, false);
        // we compare the angle of this cone with both the computed trace and the trace we are computing,
        // the trace correctly built will be the one with the smallest angle
        if (abs(possibleState.trace.before().angle()) > abs(calculatedTrace.getConeIndexTrace(nextPossibleIndex).angle())) {
          // The new trace is correct, recalculate the other one
          calculatedTrace = calculatedTrace.getConeIndexTrace(nextPossibleIndex).before();
          computeTrace(calculatedTrace, !leftOrRight, false, max_num_cones);
          actState = possibleState;
        }
      }
    } else {
      updateState(actState, nextPossibleIndex, isFirst);
    }
    isFirst = false;
    nextPossibleIndex = nextConeIndex(actState, false, false);
    //cout << "Num cons " << possibleCones.size() << endl;
  }
  //cout << actState.trace << endl;
  output = actState.trace;
}

/**
 * Creates the state that this trace would have
 */
Urimits::State Urimits::stateFromTrace(const Trace &trace) const {
  State res(Pos(0, 0), Pos(1, 0), M_PI);
  if (!trace.empty()) {
    res.pos = this->allCones[trace.coneIndex()];
    res.trace = trace;
    Trace beforeTrace = trace.before();
    if (!beforeTrace.empty()) {
      res.v = res.pos - this->allCones[beforeTrace.coneIndex()];
      Trace before2Trace = beforeTrace.before();
      if (!before2Trace.empty()) {
        res.angle = Pos::angle(this->allCones[beforeTrace.coneIndex()] - this->allCones[before2Trace.coneIndex()], this->allCones[beforeTrace.coneIndex()] - res.pos);
      }
    }
  }
  return res;
}

/**
 * Updates the state adding a new cone to it
 */
void Urimits::updateState(State &stateToUpdate, const int &nextConeIndex, const bool &isFirst) const {
  Pos nextConePos = this->allCones[nextConeIndex];
  stateToUpdate.angle = isFirst ? M_PI : Pos::angle(stateToUpdate.v, stateToUpdate.pos - nextConePos);
  //cout << "Actualitzem " << nextConePos << ", " << stateToUpdate.angle * (180 / M_PI) << endl;
  stateToUpdate.trace.addCone(nextConeIndex, stateToUpdate.angle);
  if (isFirst)
    stateToUpdate.v = Pos(1, 0);
  else
    stateToUpdate.v = nextConePos - stateToUpdate.pos;
  stateToUpdate.pos = nextConePos;
}

/**
 * Returns the heuristic value correspondent to the position we are looking (from our state)
 */
float Urimits::getHeuristic(const Pos &nextPos, const State &actState, const bool &firstLeft, const bool &firstRight) const {
  // DIST HEURISTIC
  float dist = sqrt(Pos::distSq(nextPos, actState.pos));
  float distHeuristic = dist / 5;
  //if (dist > this->max_radius_to_next_cone) distHeuristic *= 10;

  // ANGLE HEURISTIC
  float possibleAngle = Pos::angle(actState.v, actState.pos - nextPos);

  // Minimize angle difference
  //float angleHeuristic = Pos::realAngleDifference(actState.angle, possibleAngle)/M_PI;

  // Minimize absolute angle
  float angleHeuristic = -log(max(0.0, abs(possibleAngle / M_PI) - 0.2));

  // if the angle is negative, we can assume that the cone is located at the left side of the track
  if (firstLeft and possibleAngle < 0) angleHeuristic = 0;
  if (firstRight and possibleAngle > 0) angleHeuristic = 0;

  float heuristic = distHeuristic * this->dist_ponderation + angleHeuristic * (1 - this->dist_ponderation);
  //cout << nextPos << ", Heur: " << heuristic << " AngleHeur: " << angleHeuristic << " Dist: " << dist << endl;
  return heuristic;
}

/**
 * Returns the index of the cone with a smaller heuristic with respect to actState
 */
int Urimits::nextConeIndex(const State &actState, const bool &firstLeft, const bool &firstRight) const {
  list<int> possibleNextCones = getPossibleCones(actState, firstLeft or firstRight);
  if (possibleNextCones.empty()) return -1;
  priority_queue<pair<float, int>, vector<pair<float, int>>, greater<>> nextConesQueue;
  for (const int &possibleNextCone : possibleNextCones) {
    nextConesQueue.push(make_pair(getHeuristic(this->allCones[possibleNextCone], actState, firstLeft, firstRight), possibleNextCone));
  }
  return nextConesQueue.top().second;
}

/**
 * Checks whether the segment determined with positions with indexes c1 and c2 intersects with trace
 * O(n), n=trace.size()
 */
bool Urimits::segmentIntersectsWithTrace(const int &c1, const int &c2, const Trace &trace) const {
  if (trace.size() <= 1)
    return false;
  else if (c1 != trace.coneIndex() and c1 != trace.before().coneIndex() and c2 != trace.coneIndex() and c2 != trace.before().coneIndex()) {
    const Pos &p1 = this->allCones[c1];
    const Pos &p2 = this->allCones[c2];
    const Pos &last_pos = this->allCones[trace.coneIndex()];
    const Pos &beforeLast_pos = this->allCones[trace.before().coneIndex()];
    if (Pos::intersect(p1, p2, last_pos, beforeLast_pos)) return true;
  }
  return segmentIntersectsWithTrace(c1, c2, trace.before());
}

/**
 * Checks whether trace intersects with itself
 * O(n²), n=trace.size()
 */
bool Urimits::traceIntersectsWithItself(const Trace &trace) const {
  if (trace.size() < 4) return false;
  int c1 = trace.coneIndex();
  int c2 = trace.before().coneIndex();
  Trace trace_to_iterate = trace.before().before();
  while (!trace_to_iterate.empty()) {
    if (segmentIntersectsWithTrace(c1, c2, trace)) return true;
    c1 = c2;
    c2 = trace_to_iterate.coneIndex();
    trace_to_iterate = trace_to_iterate.before();
  }
  return false;
}

/**
 * Gets all the cone indexes which are candidates for next cone
 */
list<int> Urimits::getPossibleCones(const State &actState, const bool &isFirst) const {
  list<int> possibleCones;
  priority_queue<pair<float, int>, vector<pair<float, int>>, greater<>> indexes_ordered_by_dist;
  for (int i = 0; i < this->allCones.size(); i++) {
    // only add cone if its not in the exclude list and not in the trace
    if (this->indexesToExclude.find(i) == this->indexesToExclude.end() and (!actState.trace.containsCone(i) or (actState.trace.size() > this->min_trace_loop_length and i == actState.trace.first().coneIndex()))) {
      float dist2 = Pos::distSq(actState.pos, this->allCones[i]);
      if (dist2 < this->max_radius_to_next_cone*this->max_radius_to_next_cone) indexes_ordered_by_dist.push(make_pair(dist2, i));
    }
  }

  // Push to the output, the best cones
  int nItems = min(this->max_num_cones_to_consider, int(indexes_ordered_by_dist.size()));
  for (int i = 0; i < nItems; i++) {
    possibleCones.push_back(indexes_ordered_by_dist.top().second);
    indexes_ordered_by_dist.pop();
  }

  // Remove cones that have a very closed angle
  list<int>::iterator it = possibleCones.begin();
  while (it != possibleCones.end()) {
    float angle = Pos::angle(actState.v, actState.pos - this->allCones[*it]);
    if (abs(angle) < this->min_angle_between_3_cones or (isFirst and this->allCones[*it].x < 0.0)) {
      it = possibleCones.erase(it);
    } else {
      it++;
    }
  }
  return possibleCones;
}

/**
 * Returns the computed tracklimits in dv_msgs format
 */
dv_msgs::ConeArrayOrdered *Urimits::getTLs(Trace left, Trace right) const {
  dv_msgs::ConeArrayOrdered *tls = new dv_msgs::ConeArrayOrdered;

  if (left.size() >= 2) {
    tls->blue.resize(left.size());
    for (int i = 0; i < tls->blue.size(); ++i) {
      tls->blue[i] = this->data.cones[left.coneIndex()];
      left = left.before();
    }
  }
  if (right.size() >= 2) {
    tls->yellow.resize(right.size());
    for (int i = 0; i < tls->yellow.size(); ++i) {
      tls->yellow[i] = this->data.cones[right.coneIndex()];
      right = right.before();
    }
  }
  tls->header = this->data.header;
  tls->state = this->data.state;
  return tls;
}

Pos Urimits::centroidOfTrace(Trace trace) const {
  Pos res(0, 0);
  int num = 0;
  while (!trace.empty()) {
    ++num;
    const Pos &p = this->allCones[trace.coneIndex()];
    res.x += p.x;
    res.y += p.y;
    trace = trace.before();
  }
  return res / max(num, 1);
}

/**
 * Checks that the TLs provided are valid, that means:
 * - Size requirements
 * - The traces do not intersect neither with one another or with itselves
 * - The traces are closed (making a loop) if they have to
 * - The percentage of cones present in the traces are with respect to the total number of
 *   cones is not too small (avoid closing inexistent tracks)
 * - The distance between the centroid of both traces must be within a threshold
 */
bool Urimits::validTLs(const Trace &left, const Trace &right, bool checkingLoop) const {
  // Size
  if (left.size() < 3 or right.size() < 3) return false;

  // Intersection
  if (anyIntersection(left, right)) return false;

  if (checkingLoop) {
    // Closure
    if (!isLoopClosed(left) or !isLoopClosed(right)) return false;

    // Percentage of cones taken
    if (((left.size() + right.size()) / this->allCones.size()) * 100 < this->min_percentage_of_cones) return false;
  }

  // Centroids
  Pos diffCentroids = centroidOfTrace(left) - centroidOfTrace(right);
  if (abs(diffCentroids.x) * abs(diffCentroids.x) + abs(diffCentroids.y) * abs(diffCentroids.y) > this->max_distSq_betw_trace_centrd) return false;

  return true;
}