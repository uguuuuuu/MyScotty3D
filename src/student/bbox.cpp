
#include "../lib/mathlib.h"
#include "debug.h"

#include <optional>

static std::optional<Vec2> intersection(Vec2 a, Vec2 b) {
    Vec2 c(std::max(a.x, b.x), std::min(a.y, b.y));
    if(c.x > c.y) return std::nullopt;
    return c;
}
static bool within_range(float a, Vec2 b) {
    return a >= b.x && a <= b.y;
}
static bool within_range(Vec2 a, Vec2 b) {
    return a.x >= b.x && a.y <= b.y;
}

bool BBox::hit(const Ray& ray, Vec2& times) const {

    // TODO (PathTracer): Task 3
    // Implement ray - bounding box intersection test
    // If the ray intersected the bounding box within the range given by
    // [times.x,times.y], update times with the new intersection times.

    if(empty()) return false;

    Vec2 t;
    std::vector<Vec2> ts(3);
    Vec3 o = ray.point, d = ray.dir;
    for(int axis = 0; axis < 3; axis++) {
        // If ray parallel to axis plane
        if(std::abs(d[axis]) < EPS_F) {
            // If origin within bounds along axis
            if(within_range(o[axis], Vec2(min[axis], max[axis]))) {
                ts[axis].x = -FLT_MAX;
                ts[axis].y = FLT_MAX;
            } else {
                return false;
            }
        } else {
            ts[axis].x = (min[axis] - o[axis]) / d[axis];
            ts[axis].y = (max[axis] - o[axis]) / d[axis];
            if(ts[axis].x > ts[axis].y) {
                std::swap(ts[axis].x, ts[axis].y);
            }
        }
    }
    t.x = std::max(std::max(ts[0].x, ts[1].x), ts[2].x);
    t.y = std::min(std::min(ts[0].y, ts[1].y), ts[2].y);
    if(t.x > t.y) {
        return false;
    }
    if(!intersection(t, ray.dist_bounds)) {
        return false;
    }

    if(within_range(t, times)) times = t;
    return true;
}
