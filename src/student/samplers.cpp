
#include "../rays/samplers.h"
#include "../util/rand.h"

namespace Samplers {

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

static Vec2 omega_to_pt(Vec3 dir) {
    float phi = std::atan2(dir.z, dir.x);
    assert(within_range(phi, -(EPS_F + PI_F), PI_F + EPS_F));
    phi = std::clamp(phi, -PI_F, PI_F);
    phi = wrap(phi, 2.f * PI_F);
    float theta = std::acos(dir.y);
    theta = clamp(theta, 0.f, PI_F);
    return Vec2(phi, theta);
}

Vec2 Rect::sample() const {

    // TODO (PathTracer): Task 1

    // Generate a uniformly random point on a rectangle of size size.x * size.y
    // Tip: RNG::unit()

    //bool is_logging = RNG::coin_flip(0.0005f);
    //is_logging = false;

    Vec2 s{RNG::unit(), RNG::unit()};
    s *= size;
    //if(is_logging)
    //    info("Generated sample (%f, %f) in a %fX%f rectangular region", s.x, s.y, size.x, size.y);

    return s;
}

Vec3 Sphere::Uniform::sample() const {

    // TODO (PathTracer): Task 7

    // Generate a uniformly random point on the unit sphere.
    // Tip: start with Hemisphere::Uniform

    Vec3 p = hemi.sample();
    if(RNG::unit() > 0.5f) {
        p.y = -p.y;
    }

    return p;
}

Sphere::Image::Image(const HDR_Image& image) {

    // TODO (PathTracer): Task 7

    // Set up importance sampling data structures for a spherical environment map image.
    // You may make use of the _pdf, _cdf, and total members, or create your own.

    const auto [_w, _h] = image.dimension();
    w = _w;
    h = _h;
    _pdf.reserve(w * h);
    _cdf.reserve(w * h);

    for(size_t i = 0; i < h; i++) {
        for(size_t j = 0; j < w; j++) {
            float theta = (h - i - 0.5f) / static_cast<float>(h) * PI_F;
            float p = std::sin(theta) * image.at(j, i).luma();
            _pdf.push_back(p);
            total += p;
            _cdf.push_back(total);
        }
    }
}

Vec3 Sphere::Image::sample() const {

    // TODO (PathTracer): Task 7

    // Use your importance sampling data structure to generate a sample direction.
    // Tip: std::upper_bound
    auto i = std::upper_bound(_cdf.begin(), _cdf.end(), RNG::unit() * total) - _cdf.begin();
    assert(i > 0 && static_cast<size_t>(i) < _cdf.size());
    auto x = i % w;
    auto y = i / w;
    Vec2 xy((x + 0.5f) / static_cast<float>(w), (h - y - 0.5f) / static_cast<float>(h));
    Vec2 pt = xy * Vec2(2.f * PI_F, PI_F);

    return Vec3(std::cos(pt.x), std::cos(pt.y), std::sin(pt.x));
}

float Sphere::Image::pdf(Vec3 dir) const {

    // TODO (PathTracer): Task 7

    // What is the PDF of this distribution at a particular direction?

    Vec2 pt = omega_to_pt(dir);
    Vec2 xy = pt / Vec2(2.f * PI_F, PI_F);
    xy.y = 1.f - xy.y;
    xy *= Vec2(static_cast<float>(w), static_cast<float>(h));
    // Nearest neighbor
    size_t x = std::min(static_cast<size_t>(xy.x), w - 1);
    size_t y = std::min(static_cast<size_t>(xy.y), h - 1);
    float j = w * h / 2.f / PI_F / PI_F / std::sin(pt.y);

    return _pdf[y * w + x] * j / total;
}

Vec3 Point::sample() const {
    return point;
}

Vec3 Triangle::sample() const {
    float u = std::sqrt(RNG::unit());
    float v = RNG::unit();
    float a = u * (1.0f - v);
    float b = u * v;
    return a * v0 + b * v1 + (1.0f - a - b) * v2;
}

Vec3 Hemisphere::Uniform::sample() const {

    float Xi1 = RNG::unit();
    float Xi2 = RNG::unit();

    float theta = std::acos(Xi1);
    float phi = 2.0f * PI_F * Xi2;

    float xs = std::sin(theta) * std::cos(phi);
    float ys = std::cos(theta);
    float zs = std::sin(theta) * std::sin(phi);

    return Vec3(xs, ys, zs);
}

Vec3 Hemisphere::Cosine::sample() const {

    float phi = RNG::unit() * 2.0f * PI_F;
    float cos_t = std::sqrt(RNG::unit());

    float sin_t = std::sqrt(1 - cos_t * cos_t);
    float x = std::cos(phi) * sin_t;
    float z = std::sin(phi) * sin_t;
    float y = cos_t;

    return Vec3(x, y, z);
}

} // namespace Samplers
