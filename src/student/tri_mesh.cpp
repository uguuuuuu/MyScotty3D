
#include "../rays/tri_mesh.h"
#include "../rays/samplers.h"

namespace PT {

BBox Triangle::bbox() const {

    // TODO (PathTracer): Task 2
    // Compute the bounding box of the triangle.

    // Beware of flat/zero-volume boxes! You may need to
    // account for that here, or later on in BBox::intersect.
    Vec3 p0 = vertex_list[v0].position;
    Vec3 p1 = vertex_list[v1].position;
    Vec3 p2 = vertex_list[v2].position;

    BBox box(hmin(hmin(p0, p1), p2), hmax(hmax(p0, p1), p2));
    if(box.max.x - box.min.x < EPS_F) box.min.x = box.max.x;
    if(box.max.y - box.min.y < EPS_F) box.min.y = box.max.y;
    if(box.max.z - box.min.z < EPS_F) box.min.z = box.max.z;
    return box;
}

static bool within_range(float x, float a, float b) {
    return x >= a && x <= b;
}
static bool inside_triangle(Vec3 b) {
    return within_range(b.x, 0.f, 1.f) && within_range(b.y, 0.f, 1.f) && within_range(b.z, 0.f, 1.f);
}

Trace Triangle::hit(const Ray& ray) const {

    // Each vertex contains a postion and surface normal
    Tri_Mesh_Vert v_0 = vertex_list[v0];
    Tri_Mesh_Vert v_1 = vertex_list[v1];
    Tri_Mesh_Vert v_2 = vertex_list[v2];

    // TODO (PathTracer): Task 2
    // Intersect the ray with the triangle defined by the three vertices.

    Trace ret;
    ret.hit = false;
    Vec3 e1, e2, s, h;
    e1 = v_1.position - v_0.position;
    e2 = v_2.position - v_0.position;
    float denom = dot(cross(e1, ray.dir), e2);
    if(std::abs(denom) < EPS_F) {
        return ret;
    }
    s = ray.point - v_0.position;
    h.x = -dot(cross(s, e2), ray.dir);
    h.y = dot(cross(e1, ray.dir), s);
    h.z = -dot(cross(s, e2), e1);
    h /= denom;
    Vec3 b(h.x, h.y, 1.f - h.x - h.y);

    if(h.z < 0) {
        return ret;
    }
    if(!inside_triangle(b)) {
        return ret;
    }
    if(!within_range(h.z, ray.dist_bounds.x, ray.dist_bounds.y)) {
        return ret;
    }
    ray.dist_bounds.y = h.z;
    ret.hit = true;
    ret.origin = ray.point;
    ret.distance = h.z;   // at what distance did the intersection occur?
    ret.position = ray.point + ray.dir * h.z; // where was the intersection?
    ret.normal = (v_0.normal * b.x + v_1.normal * b.y + v_2.normal * b.z)
                     .unit(); // what was the surface normal at the intersection?
                           // (this should be interpolated between the three vertex normals)
    return ret;
}

Triangle::Triangle(Tri_Mesh_Vert* verts, unsigned int v0, unsigned int v1, unsigned int v2)
    : vertex_list(verts), v0(v0), v1(v1), v2(v2) {
}

Vec3 Triangle::sample(Vec3 from) const {
    Tri_Mesh_Vert v_0 = vertex_list[v0];
    Tri_Mesh_Vert v_1 = vertex_list[v1];
    Tri_Mesh_Vert v_2 = vertex_list[v2];
    Samplers::Triangle sampler(v_0.position, v_1.position, v_2.position);
    Vec3 pos = sampler.sample();
    return (pos - from).unit();
}

float Triangle::pdf(Ray wray, const Mat4& T, const Mat4& iT) const {

    Ray tray = wray;
    tray.transform(iT);

    Trace trace = hit(tray);
    if(trace.hit) {
        trace.transform(T, iT.T());
        Vec3 v_0 = T * vertex_list[v0].position;
        Vec3 v_1 = T * vertex_list[v1].position;
        Vec3 v_2 = T * vertex_list[v2].position;
        float a = 2.0f / cross(v_1 - v_0, v_2 - v_0).norm();
        float g =
            (trace.position - wray.point).norm_squared() / std::abs(dot(trace.normal, wray.dir));
        return a * g;
    }
    return 0.0f;
}

void Tri_Mesh::build(const GL::Mesh& mesh, bool bvh) {

    use_bvh = bvh;
    verts.clear();
    triangle_bvh.clear();
    triangle_list.clear();

    for(const auto& v : mesh.verts()) {
        verts.push_back({v.pos, v.norm});
    }

    const auto& idxs = mesh.indices();

    std::vector<Triangle> tris;
    for(size_t i = 0; i < idxs.size(); i += 3) {
        tris.push_back(Triangle(verts.data(), idxs[i], idxs[i + 1], idxs[i + 2]));
    }

    if(use_bvh) {
        triangle_bvh.build(std::move(tris), 4);
    } else {
        triangle_list = List<Triangle>(std::move(tris));
    }
}

Tri_Mesh::Tri_Mesh(const GL::Mesh& mesh, bool use_bvh) {
    build(mesh, use_bvh);
}

Tri_Mesh Tri_Mesh::copy() const {
    Tri_Mesh ret;
    ret.verts = verts;
    ret.triangle_bvh = triangle_bvh.copy();
    ret.triangle_list = triangle_list.copy();
    ret.use_bvh = use_bvh;
    return ret;
}

BBox Tri_Mesh::bbox() const {
    if(use_bvh) return triangle_bvh.bbox();
    return triangle_list.bbox();
}

Trace Tri_Mesh::hit(const Ray& ray) const {
    if(use_bvh) return triangle_bvh.hit(ray);
    return triangle_list.hit(ray);
}

size_t Tri_Mesh::visualize(GL::Lines& lines, GL::Lines& active, size_t level,
                           const Mat4& trans) const {
    if(use_bvh) return triangle_bvh.visualize(lines, active, level, trans);
    return 0;
}

Vec3 Tri_Mesh::sample(Vec3 from) const {
    if(use_bvh) {
        die("Sampling BVH-based triangle meshes is not yet supported.");
    }
    return triangle_list.sample(from);
}

float Tri_Mesh::pdf(Ray ray, const Mat4& T, const Mat4& iT) const {
    if(use_bvh) {
        die("Sampling BVH-based triangle meshes is not yet supported.");
    }
    return triangle_list.pdf(ray, T, iT);
}

} // namespace PT
