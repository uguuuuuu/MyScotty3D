
#include "../scene/skeleton.h"

Vec3 closest_on_line_segment(Vec3 start, Vec3 end, Vec3 point) {

    // TODO(Animation): Task 3

    // Return the closest point to 'point' on the line segment from start to end

    Vec3 d = end - start;
    float t = dot(d, point - start) / d.norm_squared();

    if(t < 0.f) {
        return start;
    }
    if(t > 1.f) {
        return end;
    }
    return start + t * d;
}

Mat4 Joint::joint_to_bind() const {

    // TODO(Animation): Task 2

    // Return a matrix transforming points in the space of this joint
    // to points in skeleton space in bind position.

    // Bind position implies that all joints have pose = Vec3{0.0f}

    // You will need to traverse the joint heirarchy. This should
    // not take into account Skeleton::base_pos
    
    if(is_root()) {
        return Mat4::I;
    }

    return parent->joint_to_bind() * Mat4::translate(parent->extent);
}

Mat4 Joint::joint_to_posed() const {

    // TODO(Animation): Task 2

    // Return a matrix transforming points in the space of this joint
    // to points in skeleton space, taking into account joint poses.

    // You will need to traverse the joint heirarchy. This should
    // not take into account Skeleton::base_pos

    if(is_root()) {
        return Mat4::euler(pose);
    }

    return parent->joint_to_posed() * Mat4::translate(parent->extent) * Mat4::euler(pose);
}

Vec3 Skeleton::end_of(Joint* j) {

    // TODO(Animation): Task 2

    // Return the bind position of the endpoint of joint j in object space.
    // This should take into account Skeleton::base_pos.

    return joint_to_bind(j) * j->extent;
}

Vec3 Skeleton::posed_end_of(Joint* j) {

    // TODO(Animation): Task 2

    // Return the posed position of the endpoint of joint j in object space.
    // This should take into account Skeleton::base_pos.

    return joint_to_posed(j) * j->extent;
}

Mat4 Skeleton::joint_to_bind(const Joint* j) const {

    // TODO(Animation): Task 2

    // Return a matrix transforming points in joint j's space to object space in
    // bind position. This should take into account Skeleton::base_pos.

    return Mat4::translate(base_pos) * j->joint_to_bind();
}

Mat4 Skeleton::joint_to_posed(const Joint* j) const {

    // TODO(Animation): Task 2

    // Return a matrix transforming points in joint j's space to object space with
    // poses. This should take into account Skeleton::base_pos.

    return Mat4::translate(base_pos) * j->joint_to_posed();
}

void Skeleton::find_joints(const GL::Mesh& mesh, std::vector<std::vector<Joint*>>& map) {

    // TODO(Animation): Task 3

    // Construct a mapping: vertex index -> list of joints that should effect the vertex.
    // A joint should effect a vertex if it is within Joint::radius distance of the
    // bone's line segment in bind position.

    info("find_joints() called");
    const std::vector<GL::Mesh::Vert>& verts = mesh.verts();
    map.resize(verts.size());

    // For each i in [0, verts.size()), map[i] should contain the list of joints that
    // effect vertex i. Note that i is NOT Vert::id! i is the index in verts.

    for_joints([&](Joint* j) {
        // What vertices does joint j effect?
        Vec3 start = Vec3(), end = j->extent;
        auto b_to_j = joint_to_bind(j).inverse();
        for(size_t i = 0; i < verts.size(); i++) {
            auto p0 = b_to_j * verts[i].pos;
            auto p1 = closest_on_line_segment(start, end, p0);
            if((p0 - p1).norm() <= j->radius) {
                map[i].push_back(j);
            }
        }
    });
}

void Skeleton::skin(const GL::Mesh& input, GL::Mesh& output,
                    const std::vector<std::vector<Joint*>>& map) {

    // TODO(Animation): Task 3

    // Apply bone poses & weights to the vertices of the input (bind position) mesh
    // and store the result in the output mesh. See the task description for details.
    // map was computed by find_joints, hence gives a mapping from vertex index to
    // the list of bones the vertex should be effected by.

    // Currently, this just copies the input to the output without modification.
    info("skin() called");

    std::vector<GL::Mesh::Vert> verts = input.verts();

    for(size_t i = 0; i < verts.size(); i++) {

        // Skin vertex i. Note that its position is given in object bind space.
        const auto& joints = map[i];
        float total_weight = 0.f;
        Mat4 transform = Mat4::Zero;
        if(joints.size() == 0) {
            die("A vertex has no associated joints");
        }
        for(const auto j : joints) {
            auto b_to_j = joint_to_bind(j).inverse();
            auto j_to_p = joint_to_posed(j);
            auto p0 = b_to_j * verts[i].pos; 
            auto p1 = closest_on_line_segment(Vec3(), j->extent, p0);
            float w = 1.f / (p0 - p1).norm();
            total_weight += w;
            transform += j_to_p * b_to_j * w;
        }
        transform /= total_weight;
        verts[i].pos = transform * verts[i].pos;
        verts[i].norm = transform.rotate(verts[i].norm).unit();
    }

    std::vector<GL::Mesh::Index> idxs = input.indices();
    output.recreate(std::move(verts), std::move(idxs));
}

void Joint::compute_gradient(Vec3 target, Vec3 current) {

    // TODO(Animation): Task 2

    // Computes the gradient of IK energy for this joint and, should be called
    // recursively upward in the heirarchy. Each call should storing the result
    // in the angle_gradient for this joint.

    // Target is the position of the IK handle in skeleton space.
    // Current is the end position of the IK'd joint in skeleton space.

    auto j_to_p = joint_to_posed();
    auto p = current - j_to_p * Vec3();
    auto j_x = cross(j_to_p.rotate(Vec3(1.f, 0.f, 0.f)), p);
    auto j_y = cross(j_to_p.rotate(Vec3(0.f, 1.f, 0.f)), p);
    auto j_z = cross(j_to_p.rotate(Vec3(0.f, 0.f, 1.f)), p);
    auto j = Mat4(Vec4(j_x, 0.f), Vec4(j_y, 0.f), Vec4(j_z, 0.f), Vec4(Vec3(), 1.f));
    angle_gradient += Mat4::transpose(j) * (current - target);
    if(!is_root()) {
        parent->compute_gradient(target, current);
    }
}

void Skeleton::step_ik(std::vector<IK_Handle*> active_handles) {

    // TODO(Animation): Task 2

    // Do several iterations of Jacobian Transpose gradient descent for IK
    float tau = 0.001f;
    for(size_t i = 0; i < 100; i++) {
        for(auto h : active_handles) {
            h->joint->compute_gradient(h->target, h->joint->joint_to_posed() * h->joint->extent);
        }
        for(auto h : active_handles) {
            auto j = h->joint;
            while(j) {
                j->pose -= tau * j->angle_gradient;
                j->angle_gradient = Vec3();
                j = j->parent;
            }
        }
    }
}
