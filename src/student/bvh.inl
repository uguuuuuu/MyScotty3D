
#include "../rays/bvh.h"
#include "debug.h"
#include <stack>

#include <sstream>
#include <optional>

namespace PT {

struct Bucket {
    BBox bbox;
    size_t n_prims = 0;
};

template<typename Primitive>
void BVH<Primitive>::build(std::vector<Primitive>&& prims, size_t max_leaf_size) {

    // NOTE (PathTracer):
    // This BVH is parameterized on the type of the primitive it contains. This allows
    // us to build a BVH over any type that defines a certain interface. Specifically,
    // we use this to both build a BVH over triangles within each Tri_Mesh, and over
    // a variety of Objects (which might be Tri_Meshes, Spheres, etc.) in Pathtracer.
    //
    // The Primitive interface must implement these two functions:
    //      BBox bbox() const;
    //      Trace hit(const Ray& ray) const;
    // Hence, you may call bbox() and hit() on any value of type Primitive.
    //
    // Finally, also note that while a BVH is a tree structure, our BVH nodes don't
    // contain pointers to children, but rather indicies. This is because instead
    // of allocating each node individually, the BVH class contains a vector that
    // holds all of the nodes. Hence, to get the child of a node, you have to
    // look up the child index in this vector (e.g. nodes[node.l]). Similarly,
    // to create a new node, don't allocate one yourself - use BVH::new_node, which
    // returns the index of a newly added node.

    // Keep these
    nodes.clear();
    primitives = std::move(prims);

    // TODO (PathTracer): Task 3
    // Construct a BVH from the given vector of primitives and maximum leaf
    // size configuration. The starter code builds a BVH with a
    // single leaf node (which is also the root) that encloses all the
    // primitives.

    BBox box;
    for(const Primitive& prim : primitives) box.enclose(prim.bbox());
    root_idx = new_node(box, 0, primitives.size(), 0, 0);

    //info("build() called with %i primitives and max leaf size %i", primitives.size(),
    //     max_leaf_size);

    if(nodes[root_idx].size > max_leaf_size) {
        size_t n_buckets = std::clamp(nodes[root_idx].size / max_leaf_size / 10, (size_t)5, (size_t)20);
        partition(root_idx, n_buckets, max_leaf_size);
    }

    //bvh_print(root_idx, 0);
}

template<typename Primitive> void BVH<Primitive>::bvh_print(size_t p, size_t lv) const {
    if(nodes[p].is_leaf()) {
        info("leaf: %i. level %i", p, lv);
        return;
    }

    info("node: %i. level %i. l: %i. r: %i", p, lv, nodes[p].l, nodes[p].r);
    bvh_print(nodes[p].l, lv + 1);
    bvh_print(nodes[p].r, lv + 1);
}

template<typename Primitive>
void BVH<Primitive>::partition(size_t p, size_t n_buckets, size_t max_leaf_size) {

    //info("partition() called");
    size_t l = new_node();
    size_t r = new_node();
    nodes[p].l = l;
    nodes[p].r = r;
    BBox pbox = nodes[p].bbox;
    std::vector<Bucket> buckets(3 * n_buckets);
    float lowest_cost = FLT_MAX;
    std::pair<size_t, size_t> lowest;

    // Fill buckets
    for(size_t i = nodes[p].start; i < nodes[p].start + nodes[p].size; i++) {
        BBox box = primitives[i].bbox();
        Vec3 c = box.center();
        // Assuming pbox is not flat
        c = (c - pbox.min) / (pbox.max - pbox.min) * static_cast<float>(n_buckets);
        for(int axis = 0; axis < 3; axis++) {
            size_t b = static_cast<size_t>(clamp(c[axis], 0.f, static_cast<float>(n_buckets - 1)));
            assert(b >= 0 && b < n_buckets);
            buckets[axis * n_buckets + b].bbox.enclose(box);
            buckets[axis * n_buckets + b].n_prims++;
        }
    }
    // Find lowest-cost partition
    for(size_t axis = 0; axis < 3; axis++) {
        for(size_t i = 0; i < n_buckets - 1; i++) {
            BBox lbox, rbox;
            size_t ln = 0, rn = 0;
            for(size_t j = 0; j < n_buckets; j++) {
                const Bucket& b = buckets[axis * n_buckets + j];
                if(j <= i) {
                    lbox.enclose(b.bbox);
                    ln += b.n_prims;
                } else {
                    rbox.enclose(b.bbox);
                    rn += b.n_prims;
                }
            }
            if(ln == 0 || rn == 0) continue;
            float cost = lbox.surface_area() * ln + rbox.surface_area() * rn;
            if(cost < lowest_cost) {
                lowest_cost = cost;
                lowest = {axis, i};
                nodes[l].bbox = lbox;
                nodes[l].size = ln;
                nodes[r].bbox = rbox;
                nodes[r].size = rn;
            }
        }
    }
    assert(nodes[l].size > 0 && nodes[r].size > 0);
    std::vector<Bucket>().swap(buckets);

    nodes[l].start = nodes[p].start;
    nodes[r].start = nodes[l].start + nodes[l].size;
    std::partition(primitives.begin() + nodes[p].start,
                   primitives.begin() + nodes[p].start + nodes[p].size, [&](const Primitive& p) {
                       Vec3 c = p.bbox().center();
                       c = (c - pbox.min) / (pbox.max - pbox.min) * static_cast<float>(n_buckets);
                       size_t b = static_cast<size_t>(clamp(c[static_cast<int>(lowest.first)], 0.f,
                                                            static_cast<float>(n_buckets - 1)));
                       assert(b >= 0 && b < n_buckets);
                       return b <= lowest.second;
                   });

    //info("l.size: %i. r.size: %i", nodes[l].size, nodes[r].size);
    //std::stringstream ss;
    //ss << nodes[l].bbox;
    //std::string s = ss.str();
    //std::stringstream().swap(ss);
    //ss << nodes[r].bbox;
    //info("l.bbox: %s", s.c_str());
    //info("r.bbox: %s", ss.str().c_str());

    if(nodes[l].size > max_leaf_size) {
        n_buckets = std::clamp(nodes[l].size / max_leaf_size / 10, (size_t)5, (size_t)20);
        partition(l, n_buckets, max_leaf_size);
    }
    if(nodes[r].size > max_leaf_size) {
        n_buckets = std::clamp(nodes[r].size / max_leaf_size / 10, (size_t)5, (size_t)20);
        partition(r, n_buckets, max_leaf_size);
    }
}

template<typename Primitive> Trace BVH<Primitive>::hit(const Ray& ray) const {

    // TODO (PathTracer): Task 3
    // Implement ray - BVH intersection test. A ray intersects
    // with a BVH aggregate if and only if it intersects a primitive in
    // the BVH that is not an aggregate.

    // The starter code simply iterates through all the primitives.
    // Again, remember you can use hit() on any Primitive value.

     //Trace ret;
     //for(const Primitive& prim : primitives) {
     //    Trace hit = prim.hit(ray);
     //    ret = Trace::min(ret, hit);
     //}
     //return ret;

    return hit(root_idx, ray);

}

template<typename Primitive> Trace BVH<Primitive>::hit(size_t n, const Ray& ray) const {

    auto intersection = [](Vec2 a, Vec2 b) -> std::optional<Vec2> {
        Vec2 i(std::max(a.x, b.x), std::min(a.y, b.y));
        if(i.x > i.y) {
            return std::nullopt;
        } else {
            return i;
        }
    };

    Trace ret;
    ret.hit = false;

    Vec2 t, l_t, r_t;
    t = l_t = r_t = Vec2(-FLT_MAX, FLT_MAX);

    bool h = nodes[n].bbox.hit(ray, t);
    if(!(h && intersection(t, ray.dist_bounds))) {
        return ret;
    }

    //std::stringstream ss;
    //ss << ray;
    //std::string ray_s = ss.str();
    //std::stringstream().swap(ss);
    //ss << nodes[n].bbox;
    //info("%s hit node %i's %s", ray_s.c_str(), n, ss.str().c_str());

    if(nodes[n].is_leaf()) {
        //info("Leaf node %i is hit", n);
        for(size_t i = nodes[n].start; i < nodes[n].start + nodes[n].size; i++) {
            Trace hit = primitives[i].hit(ray);
            ret = Trace::min(ret, hit);
        }
        return ret;
    }
    bool l_h = nodes[nodes[n].l].bbox.hit(ray, l_t);
    bool r_h = nodes[nodes[n].r].bbox.hit(ray, r_t);
    if(!(l_h && intersection(l_t, ray.dist_bounds)) && !(r_h && intersection(r_t, ray.dist_bounds))) {
        return ret;
    }
    if(!(l_h && intersection(l_t, ray.dist_bounds))) {
        return hit(nodes[n].r, ray);
    }
    if(!(r_h && intersection(r_t, ray.dist_bounds))) {
        return hit(nodes[n].l, ray);
    }
    if(l_t.x < r_t.x) {
        Trace l_ret = hit(nodes[n].l, ray);
        if(l_ret.hit && l_ret.distance + EPS_F < r_t.x) {
            return l_ret;
        } else {
            return Trace::min(l_ret, hit(nodes[n].r, ray));
        }
    } else {
        Trace r_ret = hit(nodes[n].r, ray);
        if(r_ret.hit && r_ret.distance + EPS_F < l_t.x) {
            return r_ret;
        } else {
            return Trace::min(hit(nodes[n].l, ray), r_ret);
        }
    }
}

template<typename Primitive>
BVH<Primitive>::BVH(std::vector<Primitive>&& prims, size_t max_leaf_size) {
    build(std::move(prims), max_leaf_size);
}

template<typename Primitive> BVH<Primitive> BVH<Primitive>::copy() const {
    BVH<Primitive> ret;
    ret.nodes = nodes;
    ret.primitives = primitives;
    ret.root_idx = root_idx;
    return ret;
}

template<typename Primitive> bool BVH<Primitive>::Node::is_leaf() const {

    // A node is a leaf if l == r, since all interior nodes must have distinct children
    return l == r;
}

template<typename Primitive>
size_t BVH<Primitive>::new_node(BBox box, size_t start, size_t size, size_t l, size_t r) {
    Node n;
    n.bbox = box;
    n.start = start;
    n.size = size;
    n.l = l;
    n.r = r;
    nodes.push_back(n);
    return nodes.size() - 1;
}

template<typename Primitive> BBox BVH<Primitive>::bbox() const {
    return nodes[root_idx].bbox;
}

template<typename Primitive> std::vector<Primitive> BVH<Primitive>::destructure() {
    nodes.clear();
    return std::move(primitives);
}

template<typename Primitive> void BVH<Primitive>::clear() {
    nodes.clear();
    primitives.clear();
}

template<typename Primitive>
size_t BVH<Primitive>::visualize(GL::Lines& lines, GL::Lines& active, size_t level,
                                 const Mat4& trans) const {

    std::stack<std::pair<size_t, size_t>> tstack;
    tstack.push({root_idx, 0});
    size_t max_level = 0;

    if(nodes.empty()) return max_level;

    while(!tstack.empty()) {

        auto [idx, lvl] = tstack.top();
        max_level = std::max(max_level, lvl);
        const Node& node = nodes[idx];
        tstack.pop();

        Vec3 color = lvl == level ? Vec3(1.0f, 0.0f, 0.0f) : Vec3(1.0f);
        GL::Lines& add = lvl == level ? active : lines;

        BBox box = node.bbox;
        box.transform(trans);
        Vec3 min = box.min, max = box.max;

        auto edge = [&](Vec3 a, Vec3 b) { add.add(a, b, color); };

        edge(min, Vec3{max.x, min.y, min.z});
        edge(min, Vec3{min.x, max.y, min.z});
        edge(min, Vec3{min.x, min.y, max.z});
        edge(max, Vec3{min.x, max.y, max.z});
        edge(max, Vec3{max.x, min.y, max.z});
        edge(max, Vec3{max.x, max.y, min.z});
        edge(Vec3{min.x, max.y, min.z}, Vec3{max.x, max.y, min.z});
        edge(Vec3{min.x, max.y, min.z}, Vec3{min.x, max.y, max.z});
        edge(Vec3{min.x, min.y, max.z}, Vec3{max.x, min.y, max.z});
        edge(Vec3{min.x, min.y, max.z}, Vec3{min.x, max.y, max.z});
        edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, max.y, min.z});
        edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, min.y, max.z});

        if(!node.is_leaf()) {
            tstack.push({node.l, lvl + 1});
            tstack.push({node.r, lvl + 1});
        } else {
            for(size_t i = node.start; i < node.start + node.size; i++) {
                size_t c = primitives[i].visualize(lines, active, level - lvl, trans);
                max_level = std::max(c + lvl, max_level);
            }
        }
    }
    return max_level;
}

} // namespace PT
