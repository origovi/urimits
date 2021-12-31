#include <cmath>
#include <iostream>

struct Pos {
  float x, y;
  Pos(const float &_x = 0.0f, const float &_y = 0.0f) {
    x = _x;
    y = _y;
  }
  static float distSq(const Pos &c1, const Pos &c2) {
    return (c1.x - c2.x) * (c1.x - c2.x) + (c1.y - c2.y) * (c1.y - c2.y);
  }
  Pos operator-(const Pos &p) const {
    return Pos(x - p.x, y - p.y);
  }
  // Compute the angle between two vectors
  static float angle(const Pos &ab, const Pos &cb) {
    float dot = ab.x * cb.x + ab.y * cb.y;
    float det = ab.x * cb.y - ab.y * cb.x;
    return atan2(det, dot);
  }
  static float realAngleDifference(float a1, float a2) {
    if (a1 * a2 < 0)
      return (2 * M_PI - abs(a1) - abs(a2));
    else
      return abs(a1 - a2);
  }

  inline static bool ccw(const Pos &A, const Pos &B, const Pos &C) {
    return (C.y - A.y) * (B.x - A.x) > (B.y - A.y) * (C.x - A.x);
  }
  // Return true if line segments AB and CD intersect
  static bool intersect(const Pos &A, const Pos &B, const Pos &C, const Pos &D) {
    return ccw(A, C, D) != ccw(B, C, D) and ccw(A, B, C) != ccw(A, B, D);
  }

  // Useful to print a Pos
  friend std::ostream &operator<<(std::ostream &os, Pos const &pos) {
    return os << "Pos(" << pos.x << ", " << pos.y << ")";
  }
};
