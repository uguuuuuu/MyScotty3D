
#include "../scene/particles.h"
#include "../rays/pathtracer.h"

static Vec3 reflect(Vec3 dir) {

    // Return reflection of dir about the surface normal (0,1,0).
    return Vec3(-dir.x, dir.y, -dir.z);
}

bool Scene_Particles::Particle::update(const PT::Object& scene, float dt, float radius) {

    // TODO(Animation): Task 4

    // Compute the trajectory of this particle for the next dt seconds.

    // (1) Build a ray representing the particle's path if it travelled at constant velocity.

    // (2) Intersect the ray with the scene and account for collisions. Be careful when placing
    // collision points using the particle radius. Move the particle to its next position.

    // (3) Account for acceleration due to gravity.

    // (4) Repeat until the entire time step has been consumed.

    // (5) Decrease the particle's age and return whether it should die.

    //info("dt: %f", dt);
    float remaining_time = dt;
    //size_t ctr = 0;
    while(remaining_time > EPS_F) {
        Ray r(pos, velocity, Vec2(EPS_F, std::numeric_limits<float>::max()));
        auto hit = scene.hit(r);
        if(!hit.hit) {
            break;
        }

        float VdotN = dot(velocity.unit(), hit.normal.unit());
        //if(VdotN > 0.f) {
        //    hit.normal = -hit.normal;
        //}
        float t = (hit.distance - radius / std::abs(VdotN)) / velocity.norm();
        if(t > remaining_time) {
            break;
        }
        if(t < 0.f) {
            //info("t = %f", t);
            t = 0.f;
        }

        pos += velocity * t;
        auto local_to_world = Mat4::rotate_to(hit.normal);
        auto world_to_local = local_to_world.inverse();
        auto v = local_to_world.rotate(reflect(-world_to_local.rotate(velocity)));
        //if(std::abs(v.norm() - velocity.norm()) > EPS_F) {
        //    die("reflected velocity not equal to original velocity in magnitude");
        //}
        //if(t == 0.f) {
        //    info("pos: (%f, %f, %f)", pos.x, pos.y, pos.z);
        //    info("hit pos: (%f, %f, %f)", hit.position.x, hit.position.y, hit.position.z);
        //    info("radius: %f", radius);
        //    info("vel: (%f, %f, %f)", velocity.x, velocity.y, velocity.z);
        //    info("reflected: (%f, %f, %f)", v.x, v.y, v.z);
        //    info("normal: (%f, %f, %f)", hit.normal.x, hit.normal.y, hit.normal.z);
        //}
        velocity = v;
        velocity += acceleration * t;

        remaining_time -= t;
        //info("remaining time: %f", remaining_time);
        //ctr++;
        //if(ctr > 20) break;
    }
    pos += velocity * remaining_time;
    velocity += acceleration * remaining_time;
    age -= dt;

    if(age > EPS_F)
        return true;
    else
        return false;
}
