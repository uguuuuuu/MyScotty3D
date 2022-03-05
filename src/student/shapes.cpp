
#include "../rays/shapes.h"
#include "debug.h"

namespace PT {

const char* Shape_Type_Names[(int)Shape_Type::count] = {"None", "Sphere"};

BBox Sphere::bbox() const {

    BBox box;
    box.enclose(Vec3(-radius));
    box.enclose(Vec3(radius));
    return box;
}

static bool within_range(float x, float a, float b) {
    return x >= a && x <= b;
}
static Vec2 intersection(Vec2 a, Vec2 b) {
    Vec2 c(std::max(a.x, b.x), std::min(a.y, b.y));
    return c;
}
Trace Sphere::hit(const Ray& ray) const {

    // TODO (PathTracer): Task 2
    // Intersect this ray with a sphere of radius Sphere::radius centered at the origin.

    // If the ray intersects the sphere twice, ret should
    // represent the first intersection, but remember to respect
    // ray.dist_bounds! For example, if there are two intersections,
    // but only the _later_ one is within ray.dist_bounds, you should
    // return that one!

    Trace ret;
    ret.hit = false;

    Vec3 o = ray.point, d = ray.dir;
    float OdotD = dot(o, d);
    Vec2 t(-OdotD);
    float discrim = OdotD * OdotD - o.norm_squared() + radius * radius;
    if(discrim < 0) {
        return ret;
    }
    discrim = std::sqrt(discrim);
    if(discrim < EPS_F) {
        return ret;
    }
    t += Vec2(-discrim, discrim);
    Vec2 a = intersection(t, ray.dist_bounds);
    if(a.x > a.y) {
        return ret;
    }

    ret.hit = true;
    ret.origin = ray.point;
    ret.distance = a.x > t.x ? t.y : t.x;   // at what distance did the intersection occur?
    ret.position = o + ret.distance * d; // where was the intersection?
    ret.normal = ret.position.unit();       // what was the surface normal at the intersection?
    ray.dist_bounds.y = ret.distance;
    return ret;
}

} // namespace PT
