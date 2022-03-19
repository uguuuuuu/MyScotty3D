
#include "../util/camera.h"
#include "../rays/samplers.h"
#include "debug.h"

#include "../lib/mathlib.h"
#include "../util/rand.h"
#include <sstream>

Ray Camera::generate_ray(Vec2 screen_coord) const {

    // TODO (PathTracer): Task 1
    // compute the position of the input sensor sample coordinate on the
    // canonical sensor plane one unit away from the pinhole.
    // Tip: Compute the ray direction in view space and use
    // the camera transform to transform it back into world space.

    //bool is_logging = RNG::coin_flip(0.0005f);
    //is_logging = false;

    Vec2 xy = screen_coord - Vec2(0.5f, 0.5f);
    //if(is_logging) info("Coordinates in NDC: (%f, %f)", xy.x, xy.y);

    float h = std::tan(Radians(vert_fov / 2.f)) * focal_dist * 2.f;
    float w = aspect_ratio * h;
    //if(is_logging) info("Dimensions of sensor plane: (%f, %f)", w, h);

    xy *= Vec2(w, h);
    Ray r(Vec3(), Vec3(xy.x, xy.y, -1.f));
    //std::stringstream ss;
    //if(is_logging) {
    //    ss << r;
    //    auto s = ss.str();
    //    info("Ray in view space: %s", s.c_str());
    //    std::stringstream().swap(ss);
    //}

    r.transform(iview);
    //if(is_logging) {
    //    ss << r;
    //    info("Ray in world space: %s", ss.str().c_str());
    //}

    return r;
}
