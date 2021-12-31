#include "Urimits.hh"

// Namespaces
using namespace std;

// Constructor
Urimits::Urimits() {
  this->trackClosedOneTime = false;
}

////////////////////
// PUBLIC METHODS //
////////////////////

/**
 * Runs the Urimits algorithm having as an input a dv_msgs::ConeArray
 * publishes the tracklimits, and if it detects a the whole track, it
 * also saves the whole tracklimits.
 */
void Urimits::run(const dv_msgs::ConeArray::ConstPtr &data, const bool &leftOrRightFirst) {
  if (data->cones.size() == 0) return;

  reset();
  this->data = *data;
  this->allCones.resize(this->data.cones.size());
  for (int i = 0; i < this->data.cones.size(); i++) {
    this->allCones[i] = Pos(this->data.cones[i].x, this->data.cones[i].y);
  }

  Path *first, *shortFirst, *second, *shortSecond;
  if (leftOrRightFirst) {
    first = &this->leftPath;
    shortFirst = &this->shortLeftPath;
    second = &this->rightPath;
    shortSecond = &this->shortRightPath;
  } else {
    first = &this->rightPath;
    shortFirst = &this->shortRightPath;
    second = &this->leftPath;
    shortSecond = &this->shortLeftPath;
  }

  // Compute the short TLs
  if (this->compute_short_tls) {
    computeTrace(*shortFirst, leftOrRightFirst, true, this->max_trace_length);
    computeTraceWithCorrection(*shortSecond, *shortFirst, !leftOrRightFirst, this->max_trace_length);
    this->indexesToExclude.clear();
  }

  // Compute the loop TLs
  computeTrace(*first, leftOrRightFirst, true, 1e5);
  computeTraceWithCorrection(*second, *first, !leftOrRightFirst, 1e5);

  if (this->leftPath.size() >= 3 and this->rightPath.size() >= 3 and isLoopClosed(this->leftPath) and isLoopClosed(this->rightPath) and !anyIntersection(this->leftPath, this->rightPath)) {
    this->trackClosedOneTime = true;
    this->trackClosed = true;
  }
}

void Urimits::publishData(const ros::Publisher &tlPub, const ros::Publisher &loopPub) const {
  if (this->trackClosed) {
    dv_msgs::ConeArrayOrdered* tls = getTLs(this->leftPath, this->rightPath);
    ROS_WARN("LOOP PUBLISHED");
    //loopPub.publish(*tls);
    tlPub.publish(*tls);
    delete tls;
  }
  else if (this->compute_short_tls and !this->trackClosedOneTime and !anyIntersection(this->shortLeftPath, this->shortRightPath)) {
    dv_msgs::ConeArrayOrdered* tls = getTLs(this->shortLeftPath, this->shortRightPath);
    ROS_WARN("TLs PUBLISHED");
    tlPub.publish(*tls);
    delete tls;
  }
}

/////////////////////
// PRIVATE METHODS //
/////////////////////
void Urimits::reset() {
  this->indexesToExclude.clear();
  this->leftPath.clear();
  this->shortLeftPath.clear();
  this->rightPath.clear();
  this->shortRightPath.clear();
  this->trackClosed = false;
}

inline bool Urimits::isLoopClosed(const Path &path) const {
  if (path.size() < 3) return false;
  return path.first().coneIndex() == path.coneIndex();
}

/**
 * Checks whether the two traces have any intersection with one another or
 * within themselves
 * O(n²), n=max(this->leftPath.size(), this->rightPath.size())
 */
bool Urimits::anyIntersection(const Path &path1, const Path &path2) const {
  
  // Check for intersections within the same trace
  if (pathIntersectsWithItself(path1) or pathIntersectsWithItself(path2)) return true;

  // Check for intersections between traces
  if (path1.size() < 2 or path2.size() < 2) return false;

  int left_ind = path1.coneIndex();
  Path left_copy = path1.before();
  while (!left_copy.empty()) {
    int right_ind = path2.coneIndex();
    Path right_copy = path2.before();
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
  if (isLoopClosed(state.path))
    return true;
  else
    return nextPossibleIndex == -1 or state.path.size() >= max_num_cones;
}

/**
 * Computes the first trace of cones, it uses a greedy algorithm with an heuristic, taking at each iteration the
 * best possible cone, until a stopping condition is found.
 * O(n²)
 */
void Urimits::computeTrace(Path &output, const bool &leftOrRight, bool isFirst, const int &max_num_cones) const {
  State actState(Pos(0, 0), Pos(this->first_pseudoPosition_offset, 0), M_PI);
  if (!isFirst)
    actState = stateFromPath(output);
  int nextPossibleIndex = nextConeIndex(actState, leftOrRight and isFirst, !leftOrRight and isFirst);
  while (!stopCondition(nextPossibleIndex, actState, max_num_cones)) {
    updateState(actState, nextPossibleIndex, isFirst);
    isFirst = false;
    nextPossibleIndex = nextConeIndex(actState, leftOrRight and isFirst, !leftOrRight and isFirst);
    //cout << "Num cons " << possibleCones.size() << endl;
  }
  //cout << actState.path << endl;
  output = actState.path;
}

/**
 * Computes the second trace correcting itself when it the trace wants a cone that is already taken
 * by the first trace, it decides which trace belongs to this cone. If the first trace was computed
 * with missgiven cone, it recomputes the first one and continues with the second one.
 * At most O(n⁴), n=(num cones)
 */
void Urimits::computeTraceWithCorrection(Path &output, Path &calculatedPath, const bool &leftOrRight, const int &max_num_cones) {
  State actState(Pos(0, 0), Pos(this->first_pseudoPosition_offset, 0), M_PI);
  bool isFirst = true;
  int nextPossibleIndex = nextConeIndex(actState, leftOrRight, !leftOrRight);
  while (!stopCondition(nextPossibleIndex, actState, max_num_cones)) {
    if (calculatedPath.containsCone(nextPossibleIndex)) {
      State possibleState = actState;
      this->indexesToExclude.insert(nextPossibleIndex);
      //cout << "COLISION" << endl;
      updateState(possibleState, nextPossibleIndex, isFirst);
      int next2PossibleIndex = nextConeIndex(possibleState, false, false);
      if (next2PossibleIndex != -1) {
        updateState(possibleState, next2PossibleIndex, false);
        // we compare the angle of this cone with both the computed path and the path we are computing,
        // the path correctly built will be the one with the smallest angle
        if (abs(possibleState.path.before().angle()) > abs(calculatedPath.getConeIndexPath(nextPossibleIndex).angle())) {
          // The new path is correct, recalculate the other one
          calculatedPath = calculatedPath.getConeIndexPath(nextPossibleIndex).before();
          computeTrace(calculatedPath, !leftOrRight, false, max_num_cones);
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
  //cout << actState.path << endl;
  output = actState.path;
}

/**
 * Creates the state that this path would have
 */
Urimits::State Urimits::stateFromPath(const Path &path) const {
  State res(Pos(0, 0), Pos(1, 0), M_PI);
  if (!path.empty()) {
    res.pos = this->allCones[path.coneIndex()];
    Path beforePath = path.before();
    if (!beforePath.empty()) {
      res.v = res.pos - this->allCones[beforePath.coneIndex()];
      Path before2Path = beforePath.before();
      if (!before2Path.empty()) {
        res.angle = Pos::angle(this->allCones[beforePath.coneIndex()] - this->allCones[before2Path.coneIndex()], this->allCones[beforePath.coneIndex()] - res.pos);
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
  stateToUpdate.path.addCone(nextConeIndex, stateToUpdate.angle);
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
  if (dist > this->max_radius_to_next_cone) distHeuristic *= 10;

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
  list<int> possibleNextCones = getPossibleCones(actState);
  if (possibleNextCones.empty()) return -1;
  priority_queue<pair<float, int>, vector<pair<float, int>>, greater<>> nextConesQueue;
  for (const int &possibleNextCone : possibleNextCones) {
    nextConesQueue.push(make_pair(getHeuristic(this->allCones[possibleNextCone], actState, firstLeft, firstRight), possibleNextCone));
  }
  return nextConesQueue.top().second;
}

/**
 * Checks whether the segment determined with positions with indexes c1 and c2 intersects with path
 * O(n), n=path.size()
 */
bool Urimits::segmentIntersectsWithPath(const int &c1, const int &c2, const Path &path) const {
  if (path.size() <= 1)
    return false;
  else if (c1 != path.coneIndex() and c1 != path.before().coneIndex() and c2 != path.coneIndex() and c2 != path.before().coneIndex()) {
    const Pos &p1 = this->allCones[c1];
    const Pos &p2 = this->allCones[c2];
    const Pos &last_pos = this->allCones[path.coneIndex()];
    const Pos &beforeLast_pos = this->allCones[path.before().coneIndex()];
    if (Pos::intersect(p1, p2, last_pos, beforeLast_pos)) return true;
  }
  return segmentIntersectsWithPath(c1, c2, path.before());
}

/**
 * Checks whether path intersects with itself
 * O(n²), n=path.size()
 */
bool Urimits::pathIntersectsWithItself(const Path &path) const {
  if (path.size() < 4) return false;
  int c1 = path.coneIndex();
  int c2 = path.before().coneIndex();
  Path path_to_iterate = path.before().before();
  while (!path_to_iterate.empty()) {
    if (segmentIntersectsWithPath(c1, c2, path)) return true;
    c1 = c2;
    c2 = path_to_iterate.coneIndex();
    path_to_iterate = path_to_iterate.before();
  }
  return false;
}

/**
 * Gets all the cone indexes which are candidates for next cone
 */
list<int> Urimits::getPossibleCones(const State &actState) const {
  list<int> possibleCones;
  priority_queue<pair<float, int>, vector<pair<float, int>>, greater<>> indexes_ordered_by_dist;
  for (int i = 0; i < this->allCones.size(); i++) {
    // only add cone if its not in the exclude list and not in the path
    if (this->indexesToExclude.find(i) == this->indexesToExclude.end() and (!actState.path.containsCone(i) or (actState.path.size() > this->min_trace_loop_length and i == actState.path.first().coneIndex()))) {
      indexes_ordered_by_dist.push(make_pair(Pos::distSq(actState.pos, this->allCones[i]), i));
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
    if (abs(angle) < this->min_angle_between_3_cones) {
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
dv_msgs::ConeArrayOrdered* Urimits::getTLs(const Path &left, const Path &right) const {
  dv_msgs::ConeArrayOrdered *tls = new dv_msgs::ConeArrayOrdered;

  if (left.size() >= 2) {
    Path left_copy = left;
    tls->blue.resize(left_copy.size());
    for (int i = 0; i < tls->blue.size(); ++i) {
      tls->blue[i] = this->data.cones[left_copy.coneIndex()];
      left_copy = left_copy.before();
    }
  }
  if (right.size() >= 2) {
    Path right_copy = right;
    tls->yellow.resize(right_copy.size());
    for (int i = 0; i < tls->yellow.size(); ++i) {
      tls->yellow[i] = this->data.cones[right_copy.coneIndex()];
      right_copy = right_copy.before();
    }
  }
  tls->header = this->data.header;
  tls->state = this->data.state;
  return tls;
}