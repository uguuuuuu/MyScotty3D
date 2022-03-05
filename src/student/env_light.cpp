
#include "../rays/env_light.h"

#include <limits>

namespace PT {

template<typename T> static bool within_range(const T& x, const T& a, const T& b) {
    return x >= a && x <= b;
}

template<typename T> static T wrap(const T& x, const T& a) {
    if(x < static_cast<T>(0)) {
        return wrap(x + a, a);
    }
    if(x >= a) {
        return wrap(x - a, a);
    }
    return x;
}
template<typename T> static T wrap(const T& x, const T& a, const T& b) {
    return wrap(x - a, b - a) + a;
}

template<typename T> static T mirror(const T& x, const T& a, const T& b) {
    if(x < a) {
        return wrap(a + (a - x), a, b);
    }
    if(x >= b) {
        return wrap(b - (x - b), a, b);
    }
    return x;
}

Vec3 Env_Map::sample() const {

    // TODO (PathTracer): Task 7

    // First, implement Samplers::Sphere::Uniform so the following line works.
    // Second, implement Samplers::Sphere::Image and swap to image_sampler

    //return uniform_sampler.sample();
    return image_sampler.sample();
}

float Env_Map::pdf(Vec3 dir) const {

    // TODO (PathTracer): Task 7

    // First, return the pdf for a uniform spherical distribution.
    // Second, swap to image_sampler.pdf().

    //return 1.f / (4.f * PI_F);
    return image_sampler.pdf(dir);
}

Spectrum Env_Map::evaluate(Vec3 dir) const {

    // TODO (PathTracer): Task 7

    // Compute emitted radiance along a given direction by finding the corresponding
    // pixels in the enviornment image. You should bi-linearly interpolate the value
    // between the 4 nearest pixels.

    float phi = std::atan2(dir.z, dir.x);
    assert(within_range(phi, -(EPS_F + PI_F), PI_F + EPS_F));
    phi = std::clamp(phi, -PI_F, PI_F);
    phi = wrap(phi, 2.f * PI_F);
    float theta = std::acos(dir.y);
    theta = clamp(theta, 0.f, PI_F);
    Vec2 xy(phi / (2.f * PI_F), theta / PI_F);
    assert(within_range(xy.x, 0.f, 1.f) && within_range(xy.y, 0.f, 1.f));
    //auto v0 = xy;
    //xy = clamp(xy, Vec2(0.f), Vec2(1.f));

    //xy.x = 1.f - xy.x;
    xy.y = 1.f - xy.y;
    auto v1 = xy;
    auto wh = image.dimension();
    Vec2 wh_f(static_cast<float>(wh.first), static_cast<float>(wh.second));
    xy *= wh_f;
    //auto v2 = xy;
    //assert(xy.x >= 0.f && xy.x <= wh_f.x);
    //assert(xy.y >= 0.f && xy.y <= wh_f.y);
    xy -= Vec2(0.5f, 0.5f);
    //auto v3 = xy;
    //assert(xy.x >= -0.5f && xy.x < wh_f.x);
    //assert(xy.y >= -0.5f && xy.y < wh_f.y);
    //xy.x = xy.x < 0.f ? wh_f.x - 0.5f : xy.x;
    //auto v4 = xy;
    //xy.y = xy.y < 0.f ? wh_f.y - 0.5f : xy.y;
    //auto v5 = xy;
    //assert(xy.x >= 0.f && xy.x < wh_f.x);
    //assert(xy.y >= 0.f && xy.y < wh_f.y);
    int x0 = static_cast<int>(std::floor(xy.x));
    int y0 = static_cast<int>(std::floor(xy.y));
    float w_x = xy.x - static_cast<float>(x0);
    float w_y = xy.y - static_cast<float>(y0);
    x0 = static_cast<int>((x0 + wh.first) % wh.first);
    int x1 = (x0 + 1) % wh.first;
    int y1 = y0 + 1;
    y0 = std::max(y0, 0);
    y1 = std::min(y1, static_cast<int>(wh.second - 1));
    auto p0 = lerp(image.at(x0, y0), image.at(x0, y1), w_y);
    auto p1 = lerp(image.at(x1, y0), image.at(x1, y1), w_y);
    auto p = lerp(p0, p1, w_x);

    return p;
}

Vec3 Env_Hemisphere::sample() const {
    return sampler.sample();
}

float Env_Hemisphere::pdf(Vec3 dir) const {
    return 1.0f / (2.0f * PI_F);
}

Spectrum Env_Hemisphere::evaluate(Vec3 dir) const {
    if(dir.y > 0.0f) return radiance;
    return {};
}

Vec3 Env_Sphere::sample() const {
    return sampler.sample();
}

float Env_Sphere::pdf(Vec3 dir) const {
    return 1.0f / (4.0f * PI_F);
}

Spectrum Env_Sphere::evaluate(Vec3) const {
    return radiance;
}

} // namespace PT
