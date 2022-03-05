
#include "../rays/bsdf.h"
#include "../util/rand.h"

namespace PT {

static Vec3 reflect(Vec3 dir) {

    // TODO (PathTracer): Task 5
    // Return reflection of dir about the surface normal (0,1,0).
    return Vec3(-dir.x, dir.y, -dir.z);
}

static Vec3 refract(Vec3 out_dir, float index_of_refraction, bool& was_internal) {

    // TODO (PathTracer): Task 5
    // Use Snell's Law to refract out_dir through the surface.
    // Return the refracted direction. Set was_internal to true if
    // refraction does not occur due to total internal reflection,
    // and false otherwise.

    // When dot(out_dir,normal=(0,1,0)) is positive, then out_dir corresponds to a
    // ray exiting the surface into vaccum (ior = 1). However, note that
    // you should actually treat this case as _entering_ the surface, because
    // you want to compute the 'input' direction that would cause this output,
    // and to do so you can simply find the direction that out_dir would refract
    // _to_, as refraction is symmetric.

    Vec3 n(0.f, 1.f, 0.f);
    // Treat out_dir as incoming dir
    float cos_ti = dot(out_dir, n);
    float sin_ti = std::sqrt(1.f - cos_ti * cos_ti);
    float sin_tt = sin_ti;
    if(cos_ti > 0) {
        sin_tt /= index_of_refraction;
    } else {
        sin_tt *= index_of_refraction;
    }
    if(sin_tt >= 1.f) {
        was_internal = true;
        return Vec3{};
    }
    float cos_tt = std::sqrt(1.f - sin_tt * sin_tt);
    cos_tt = cos_ti > 0 ? -cos_tt : cos_tt;
    Vec3 phi_i = Vec3(out_dir.x, 0.f, out_dir.z).unit();
    Vec3 phi_t = -phi_i * sin_tt;
    Vec3 in_dir = phi_t + n * cos_tt;

    was_internal = false;
    return in_dir.unit();
}

Scatter BSDF_Lambertian::scatter(Vec3 out_dir) const {

    // TODO (PathTracer): Task 4

    // Sample the BSDF distribution using the cosine-weighted hemisphere sampler.
    // You can use BSDF_Lambertian::evaluate() to compute attenuation.

    Scatter ret;
    ret.direction = sampler.sample();
    ret.attenuation = evaluate(out_dir, ret.direction);

    return ret;
}

Spectrum BSDF_Lambertian::evaluate(Vec3 out_dir, Vec3 in_dir) const {

    // TODO (PathTracer): Task 4

    // Compute the ratio of reflected/incoming radiance when light from in_dir
    // is reflected through out_dir: albedo * cos(theta).

    if(dot(out_dir, Vec3(0.f, 1.f, 0.f)) > 0.f) {
        return clamp(dot(in_dir, Vec3(0.f, 1.f, 0.f)), 0.f, 1.f) * albedo;
    } else {
        return {};
    }
}

float BSDF_Lambertian::pdf(Vec3 out_dir, Vec3 in_dir) const {

    // TODO (PathTracer): Task 4

    // Compute the PDF for sampling in_dir from the cosine-weighted hemisphere distribution.
    if(dot(out_dir, Vec3(0.f, 1.f, 0.f)) > 0.f) {
        return clamp(dot(in_dir, Vec3(0.f, 1.f, 0.f)), 0.f, 1.f) / PI_F;
    } else {
        return 0.f;
    }
}

Scatter BSDF_Mirror::scatter(Vec3 out_dir) const {

    // TODO (PathTracer): Task 5

    Scatter ret;
    ret.direction = Vec3();
    ret.attenuation = Spectrum{};
    if(dot(out_dir, Vec3(0.f, 1.f, 0.f)) > 0.f) {
        ret.direction = reflect(out_dir);
        ret.attenuation = reflectance;
    }
    return ret;
}

Scatter BSDF_Glass::scatter(Vec3 out_dir) const {

    // TODO (PathTracer): Task 5

    // (1) Compute Fresnel coefficient. Tip: Schlick's approximation.
    // (2) Reflect or refract probabilistically based on Fresnel coefficient. Tip: RNG::coin_flip
    // (3) Compute attenuation based on reflectance or transmittance

    // Be wary of your eta1/eta2 ratio - are you entering or leaving the surface?
    // What happens upon total internal reflection?

    Scatter ret;
    ret.direction = Vec3();
    ret.attenuation = Spectrum{};

    Vec3 n(0.f, 1.f, 0.f);
    float cos_tt = dot(out_dir, n);
    float n1 = index_of_refraction, n2 = 1.f;
    if(cos_tt < 0.f) {
        std::swap(n1, n2);
    }
    cos_tt = std::abs(cos_tt);
    bool was_internal = false, refracting = false;
    Vec3 in_dir = refract(out_dir, index_of_refraction, was_internal);
    if(!was_internal) {
        float cos_ti = std::abs(dot(in_dir, n));
        float rs = (n1 * cos_ti - n2 * cos_tt) / (n1 * cos_ti + n2 * cos_tt);
        rs = rs * rs;
        float rp = (n1 * cos_tt - n2 * cos_ti) / (n1 * cos_tt + n2 * cos_ti);
        rp = rp * rp;
        float F = 0.5f * (rs + rp);

        if(RNG::unit() > F) {
            refracting = true;
        }
    }

    if(refracting) {
        ret.direction = in_dir;
        ret.attenuation = transmittance * n2 * n2 / n1 / n1;
    } else {
        ret.direction = reflect(out_dir);
        ret.attenuation = reflectance;
    }
    
    return ret;
}

Scatter BSDF_Refract::scatter(Vec3 out_dir) const {

    // OPTIONAL (PathTracer): Task 5

    // When debugging BSDF_Glass, it may be useful to compare to a pure-refraction BSDF

    Scatter ret;
    ret.direction = Vec3();
    ret.attenuation = Spectrum{};
    bool was_internal = false;
    ret.direction = refract(out_dir, index_of_refraction, was_internal);
    if(!was_internal) {
        ret.attenuation = transmittance;
    }
    return ret;
}

Spectrum BSDF_Diffuse::emissive() const {
    return radiance;
}

} // namespace PT
