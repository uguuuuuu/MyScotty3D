
#include <queue>
#include <set>
#include <unordered_map>
#include <algorithm>
#include <numeric>
#include <functional>

#include "../geometry/halfedge.h"
#include "debug.h"
#include "../lib/log.h"

/* Note on local operation return types:

    The local operations all return a std::optional<T> type. This is used so that your
    implementation can signify that it does not want to perform the operation for
    whatever reason (e.g. you don't want to allow the user to erase the last vertex).

    An optional can have two values: std::nullopt, or a value of the type it is
    parameterized on. In this way, it's similar to a pointer, but has two advantages:
    the value it holds need not be allocated elsewhere, and it provides an API that
    forces the user to check if it is null before using the value.

    In your implementation, if you have successfully performed the operation, you can
    simply return the required reference:

            ... collapse the edge ...
            return collapsed_vertex_ref;

    And if you wish to deny the operation, you can return the null optional:

            return std::nullopt;

    Note that the stubs below all reject their duties by returning the null optional.
*/

std::vector<Halfedge_Mesh::VertexRef> get_neighbors(Halfedge_Mesh::VertexRef v) {

    std::vector<Halfedge_Mesh::VertexRef> neighbors;
    auto h = v->halfedge();
    do {
        neighbors.push_back(h->twin()->vertex());
        h = h->twin()->next();
    } while(h != v->halfedge());

    return neighbors;
}
std::vector<Halfedge_Mesh::HalfedgeRef> get_outgoing_halfedges(
    Halfedge_Mesh::VertexRef v, std::function<bool(Halfedge_Mesh::HalfedgeRef)> pred =
                                    [](Halfedge_Mesh::HalfedgeRef) { return true; }) {
    std::vector<Halfedge_Mesh::HalfedgeRef> outgoing_halfedges;
    auto h = v->halfedge();
    //info("Getting outgoing halfedges of vertex %i", v->id());
    do {
        //info("Outgoing halfedge %i", h->id());
        if(pred(h)) outgoing_halfedges.push_back(h);
        h = h->twin()->next();
    } while(h != v->halfedge());

    return outgoing_halfedges;
}
Halfedge_Mesh::HalfedgeRef get_last_halfedge(Halfedge_Mesh::HalfedgeRef h) {

    auto hh = h;
    do {
        hh = hh->next();
    } while(hh->next() != h);

    return hh;
}
Halfedge_Mesh::VertexRef move(Halfedge_Mesh::VertexRef dst, Halfedge_Mesh::VertexRef src) {
    
    dst->halfedge() = src->halfedge();
    for(auto h : get_outgoing_halfedges(src)) h->vertex() = dst;
    return dst;
}
// Merge the two vertices delimiting the edge of h
// @param h0: Inside halfedge of edge delimited by vertices to be merged
Halfedge_Mesh::VertexRef merge(Halfedge_Mesh::HalfedgeRef h0,
                               Halfedge_Mesh& m) {

    auto h1 = h0->twin();
    auto e0 = h0->edge();
    auto v0 = h0->vertex();
    auto v1 = h1->vertex();
    info("Merging vertices %i and %i", v0->id(), v1->id());
    auto outgoing_halfedges0 =
        get_outgoing_halfedges(v0, [&](Halfedge_Mesh::HalfedgeRef h) { return h != h0; });
    auto outgoing_halfedges1 =
        get_outgoing_halfedges(v1, [&](Halfedge_Mesh::HalfedgeRef h) { return h != h1; });

    auto v = m.new_vertex();
    info("Merged vertex %i", v->id());

    info("Outgoing halfedges of vertex %i", v0->id());
    for(auto h : outgoing_halfedges0) {
        v->halfedge() = h;
        info("%i", h->id());
    }
    info("Outgoing halfedges of vertex %i", v1->id());
    for(auto h : outgoing_halfedges1) {
        v->halfedge() = h;
        info("%i", h->id());
    }

    m.erase(v0);
    m.erase(v1);
    m.erase(e0);
    m.erase(h0);
    m.erase(h1);

    info("Outgoing halfedges of merged vertex %i", v->id());
    for(auto h : get_outgoing_halfedges(v)) {
        info("%i", h->id());
    }

    return v;
}
// @param h0: halfedge pointing outward common vert
// @param h1: halfedge pointing toward common vert
// @return: merge vertex whose halfedge's edge is merge edge
Halfedge_Mesh::VertexRef merge(Halfedge_Mesh::HalfedgeRef h0, Halfedge_Mesh::HalfedgeRef h1,
                             Halfedge_Mesh& m) {

    info("Merging edges %i and %i", h0->id(), h1->id());
    // Face is triangle
    h0 = h0->twin();
    h1 = h1->twin();
    auto h2 = h0->twin();
    auto h3 = h1->twin();
    auto h4 = h2->next();
    auto h5 = h4->twin();
    auto v0 = h0->vertex();
    auto v1 = h3->vertex();
    auto v2 = h1->vertex();
    auto e0 = h4->edge();
    auto e1 = h1->edge();
    auto e2 = h0->edge();
    auto f0 = h2->face();
    auto outgoing_halfedges0 =
        get_outgoing_halfedges(v0, [&](Halfedge_Mesh::HalfedgeRef h) { return h != h4; });
    auto outgoing_halfedges1 = get_outgoing_halfedges(
        v1, [&](Halfedge_Mesh::HalfedgeRef h) { return h != h5 && h != h3; });
    auto outgoing_halfedges2 =
        get_outgoing_halfedges(v2, [&](Halfedge_Mesh::HalfedgeRef h) { return h != h2; });

    auto v = m.new_vertex();
    info("Merged vertex %i", v->id());
    v->halfedge() = h0;
    for(auto h : outgoing_halfedges0) {
        h->vertex() = v;
    }
    for(auto h : outgoing_halfedges1) {
        h->vertex() = v;
    }
    v2->halfedge() = outgoing_halfedges2[0];
    auto e = m.new_edge();
    e->halfedge() = h0;
    h0->set_neighbors(h0->next(), h1, v, e, h0->face());
    h1->set_neighbors(h1->next(), h0, v2, e, h1->face());
    get_last_halfedge(h5)->next() = h5->next();

    m.erase(v0);
    m.erase(v1);
    m.erase(e0);
    m.erase(e1);
    m.erase(e2);
    m.erase(f0);
    m.erase(h2);
    m.erase(h3);
    m.erase(h4);
    m.erase(h5);

    info("Merging finished");
    return v;
}
bool on_boundary(Halfedge_Mesh::VertexRef v) {

    auto outgoing_halfedges = get_outgoing_halfedges(v);
    for(auto h : outgoing_halfedges) {
        if(h->face()->is_boundary() || h->twin()->face()->is_boundary()) return true;
    }
    return false;
}
bool inline on_boundary(Halfedge_Mesh::EdgeRef e) {
    auto h0 = e->halfedge();
    auto h1 = h0->twin();
    return h0->face()->is_boundary() || h1->face()->is_boundary();
}
unsigned int inline num_incident_edges(Halfedge_Mesh::VertexRef v) {

    return on_boundary(v) ? v->degree() + 1 : v->degree();
}
size_t num_edges(Halfedge_Mesh::FaceRef f) {

    auto h = f->halfedge();
    size_t counter = 0;
    do {
        counter++;
        h = h->next();
    } while(h != f->halfedge());
    return counter;
}
std::vector<Halfedge_Mesh::HalfedgeRef> get_boundary_halfedges(Halfedge_Mesh::HalfedgeRef h) {
    std::vector<Halfedge_Mesh::HalfedgeRef> bdry_halfedges;
    auto hh = h;
    do {
        bdry_halfedges.push_back(hh);
        hh = hh->next();
    } while(hh != h);

    return bdry_halfedges;
}

/*
    This method should replace the given vertex and all its neighboring
    edges and faces with a single face, returning the new face.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::erase_vertex(Halfedge_Mesh::VertexRef v) {

    // 0. sanity check
    // 1. create new face
    // 2. reconfigure incident vertices & halfedges & boundary halfedges
    // 3. configure new face
    // 4. delete inside vertex, edges, faces, and halfedges

    if(on_boundary(v)) return std::nullopt;
    std::vector<VertexRef> neighbors = get_neighbors(v);
    for(auto ngbr : neighbors) {
        if(num_incident_edges(ngbr) < 3) return std::nullopt;
    }

    auto face = new_face();

    std::vector<HalfedgeRef> outgoing_halfedges = get_outgoing_halfedges(v);
    for(auto h : outgoing_halfedges) {
        auto h1 = h->next();
        auto twin = h->twin();
        twin->vertex()->halfedge() = h1;

        auto h0 = twin->next();
        while(h0->next() != twin) h0 = h0->next();
        h0->next() = h1;
    }
    auto bdry_halfedges = get_boundary_halfedges(outgoing_halfedges[0]->next());
    for(auto h : bdry_halfedges) {
        h->face() = face;
    }

    face->halfedge() = bdry_halfedges[0];
    
    erase(v);
    for(auto h : outgoing_halfedges) {
        erase(h->edge());
        erase(h->face());
        erase(h->twin());
        erase(h);
    }

    return face;
}

/*
    This method should erase the given edge and return an iterator to the
    merged face.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::erase_edge(Halfedge_Mesh::EdgeRef e) {

    // 0. sanity check
    // 1. new face
    // 2. reconfigure boundary halfedges, incident vertices, & new face
    // 3. delete edge, incident faces, and halfedges

    if(on_boundary(e)) return std::nullopt;
    auto h0 = e->halfedge();
    auto h1 = h0->twin();
    auto v0 = h0->vertex();
    auto v1 = h1->vertex();
    if(on_boundary(v0)) {
        if(v0->degree() <= 1) return std::nullopt;
    } else {
        if(v0->degree() <= 2) return std::nullopt;
    }
    if(on_boundary(v1)) {
        if(v1->degree() <= 1) return std::nullopt;
    } else {
        if(v1->degree() <= 2) return std::nullopt;
    }

    auto face = new_face();

    auto h00 = h0->next();
    auto h10 = h1->next();
    auto h01 = h00;
    while(h01->next() != h0) h01 = h01->next();
    auto h11 = h10;
    while(h11->next() != h1) h11 = h11->next();
    h01->set_neighbors(h10, h01->twin(), h01->vertex(), h01->edge(), face);
    h11->set_neighbors(h00, h11->twin(), h11->vertex(), h11->edge(), face);
    for(auto h = h00; h != h11; h = h->next()) {
        h->face() = face;
    }
    h0->vertex()->halfedge() = h10;
    h1->vertex()->halfedge() = h00;
    face->halfedge() = h00;

    erase(e);
    erase(h0->face());
    erase(h1->face());
    erase(h0);
    erase(h1);

    return face;
}

// return outgoing incident halfedge
std::optional<Halfedge_Mesh::HalfedgeRef> collapse_half(Halfedge_Mesh::HalfedgeRef h,
                                                        Halfedge_Mesh& m) {

    auto h0 = h->next();
    auto h1 = h0->next();
    while(h1->next() != h) h1 = h1->next();
    if(num_edges(h->face()) == 3) {
        m.erase(h0);
        m.erase(h1);
        m.erase(h0->edge());
        m.erase(h1->edge());
        m.erase(h0->face());
        {
            auto hh = h0;
            h0 = h1->twin();
            h1 = hh->twin();
        }
        auto ee = m.new_edge();
        ee->halfedge() = h0;
        h0->set_neighbors(h0->next(), h1, h0->vertex(), ee, h0->face());
        if(h1->next() == h->twin())
            h1->set_neighbors(h1->next()->next(), h0, h1->vertex(), ee, h1->face());
        else
            h1->set_neighbors(h1->next(), h0, h1->vertex(), ee, h1->face());
        h1->vertex()->halfedge() = h1;
    } else {
        h1->next() = h0;
        h->face()->halfedge() = h0;
    }

    return h0;
}
/*
    This method should collapse the given edge and return an iterator to
    the new vertex created by the collapse.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_edge(Halfedge_Mesh::EdgeRef e) {

    auto v0 = e->halfedge()->vertex();
    auto v1 = e->halfedge()->twin()->vertex();
    bool on_bdry = on_boundary(e);
    unsigned int num_triangles = 0;
    std::vector<Halfedge_Mesh::VertexRef> common_neighbors;
    auto h = e->halfedge()->face()->is_boundary() ? e->halfedge()->twin() : e->halfedge();
    if(num_edges(h->face()) == 3) {
        num_triangles++;
        common_neighbors.push_back(h->next()->next()->vertex());
    }
    if(!on_bdry && num_edges(h->twin()->face()) == 3) {
        num_triangles++;
        common_neighbors.push_back(h->twin()->next()->next()->vertex());
    }
    if(num_triangles == 1) {
        if(num_incident_edges(v0) + num_incident_edges(v1) - 3 < 2) return std::nullopt;
        if(num_incident_edges(common_neighbors[0]) - 1 < 2) return std::nullopt;
    }
    if(num_triangles == 2) {
        for(auto v : common_neighbors) {
            if(num_incident_edges(v) - 1 < 2) return std::nullopt;
        }
    }
    auto outgoing_halfedges0 = get_outgoing_halfedges(v0);
    auto outgoing_halfedges1 = get_outgoing_halfedges(v1);

    auto h0 = collapse_half(h, *this);
    if(!on_bdry) h0 = collapse_half(h->twin(), *this);

    auto v = new_vertex();
    v->halfedge() = *h0;

    for(auto hh : outgoing_halfedges0) {
        hh->vertex() = v;
    }
    for(auto hh : outgoing_halfedges1) {
        hh->vertex() = v;
    }

    erase(v0);
    erase(v1);
    erase(e);
    erase(h);
    erase(h->twin());

    return v;
}

//@param h: outside halfedge corresponding to side being collapsed
Halfedge_Mesh::VertexRef collapse_side(Halfedge_Mesh::HalfedgeRef h, Halfedge_Mesh& m) {
    
    info("collapsing side with halfedge %i", h->id());
    Halfedge_Mesh::VertexRef v;
    if(num_edges(h->face()) == 3)
        v = merge(get_last_halfedge(h), h->next(), m);
    else
        v = merge(h, m);

    auto outgoing = get_outgoing_halfedges(v);
    info("merged vertex %i", v->id());
    for(auto n : outgoing) {
        info("outgoing halfedge %i", n->id());
    }
    m.validate();
    info("validate() passed");
    return v;
}
    /*
    This method should collapse the given face and return an iterator to
    the new vertex created by the collapse.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_face(Halfedge_Mesh::FaceRef f) {

    // 0. sanity check
    // 1. create new vertex
    // 2. reconfigure incident faces
    // 3. delete vertices, edges, face, & halfedges

    auto bdry_halfedges = get_boundary_halfedges(f->halfedge());
    for(auto h : bdry_halfedges) {
        if(!on_boundary(h->edge()) && num_edges(h->twin()->face()) == 3) {
            if(num_incident_edges(h->twin()->next()->next()->vertex()) < 3) return std::nullopt;
        }
    }
    std::vector<HalfedgeRef> outgoing_halfedges;
    outgoing_halfedges.reserve(
        std::reduce(bdry_halfedges.begin(), bdry_halfedges.end(), (size_t)0,
                    [](size_t a, HalfedgeRef h) { return a + num_incident_edges(h->vertex()); }) -
        num_edges(f) * 2);
    std::for_each(bdry_halfedges.begin(), bdry_halfedges.end(), [&](HalfedgeRef h) {
        auto v = h->vertex();
        auto halfedges = get_outgoing_halfedges(
            v, [=](HalfedgeRef hh) { return hh->face() != f && hh->twin()->face() != f; });
        auto size = outgoing_halfedges.size();
        outgoing_halfedges.resize(size + halfedges.size());
        std::move(halfedges.begin(), halfedges.end(), outgoing_halfedges.begin() + size);
    });
    if(outgoing_halfedges.size() < 2) return std::nullopt;

    VertexRef v;
    for(auto h : bdry_halfedges) {
            v = collapse_side(h->twin(), *this);
    }
    erase(f);

    return v;
}

/*
    Insets a vertex into the given face, returning a pointer to the new center vertex
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::inset_vertex(FaceRef f) {

    auto v = new_vertex();
    auto bdry_halfedges = get_boundary_halfedges(f->halfedge());
    
    for(size_t i = 0; i < bdry_halfedges.size(); i++) {
        auto h = bdry_halfedges[i];
        auto v0 = h->twin()->vertex();
        auto e0 = new_edge();
        auto f0 = new_face();
        auto h0 = new_halfedge();
        auto h1 = new_halfedge();
        v->halfedge() = h1;
        v0->halfedge() = h0;
        e0->halfedge() = h0;
        f0->halfedge() = h0;
        h->set_neighbors(h0, h->twin(), h->vertex(), h->edge(), f0);
        h0->set_neighbors(h0, h1, v0, e0, f0);
        h1->set_neighbors(bdry_halfedges[(i + 1) % bdry_halfedges.size()], h0, v, e0, f0);
    }
    for(size_t i = 0; i < bdry_halfedges.size(); i++) {
        auto h = bdry_halfedges[i];
        auto h0 = h->next();
        if(i == 0)
            h0->next() = bdry_halfedges[bdry_halfedges.size() - 1]->next()->twin();
        else
            h0->next() = bdry_halfedges[i - 1]->next()->twin();
        auto h1 = h0->twin();
        h1->face() = bdry_halfedges[(i + 1) % bdry_halfedges.size()]->face();
    }

    erase(f);

    return v;
}

/*
    This method should flip the given edge and return an iterator to the
    flipped edge.
*/
std::optional<Halfedge_Mesh::EdgeRef> Halfedge_Mesh::flip_edge(Halfedge_Mesh::EdgeRef e) {

    auto h1 = e->halfedge();
    auto h2 = h1->next();
    auto h0 = get_last_halfedge(h1);
    auto h3 = h1->twin();
    auto h4 = h3->next();
    auto h5 = get_last_halfedge(h3);
    auto v0 = h2->twin()->vertex();
    auto v1 = h1->vertex();
    auto v2 = h4->twin()->vertex();
    auto v3 = h3->vertex();
    auto f0 = h1->face();
    auto f1 = h3->face();

    v0->halfedge() = h3;
    v1->halfedge() = h4;
    v2->halfedge() = h1;
    v3->halfedge() = h2;
    f0->halfedge() = h1;
    f1->halfedge() = h3;
    h0->set_neighbors(h4, h0->twin(), h0->vertex(), h0->edge(), f0);
    h1->set_neighbors(h2->next(), h3, v2, h1->edge(), f0);
    h2->set_neighbors(h3, h2->twin(), h2->vertex(), h2->edge(), f1);
    h3->set_neighbors(h4->next(), h1, v0, h3->edge(), f1);
    h4->set_neighbors(h1, h4->twin(), h4->vertex(), h4->edge(), f0);
    h5->set_neighbors(h2, h5->twin(), h5->vertex(), h5->edge(), f1);

    return e;
}

/*
    This method should split the given edge and return an iterator to the
    newly inserted vertex. The halfedge of this vertex should point along
    the edge that was split, rather than the new edges.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::split_edge(Halfedge_Mesh::EdgeRef e) {

    auto v = new_vertex();
    auto e0 = e;
    auto e1 = new_edge();
    auto e2 = new_edge();
    auto e3 = new_edge();
    auto f0 = e->halfedge()->face();
    auto f1 = e->halfedge()->twin()->face();
    auto f2 = new_face();
    auto f3 = new_face();
    auto h0 = e->halfedge();
    auto h1 = h0->next();
    auto h2 = get_last_halfedge(h0);
    auto h3 = h0->twin();
    auto h4 = h3->next();
    auto h5 = get_last_halfedge(h3);
    auto h6 = new_halfedge();
    auto h7 = new_halfedge();
    auto h8 = new_halfedge();
    auto h9 = new_halfedge();
    auto h10 = new_halfedge();
    auto h11 = new_halfedge();
    auto v0 = h0->vertex();
    auto v1 = h5->vertex();
    auto v2 = h1->vertex();
    auto v3 = h2->vertex();

    v->halfedge() = h0;
    v0->halfedge() = h7;
    v1->halfedge() = h5;
    v2->halfedge() = h1;
    v3->halfedge() = h2;
    e0->halfedge() = h0;
    e1->halfedge() = h8;
    e2->halfedge() = h10;
    e3->halfedge() = h11;
    f0->halfedge() = h0;
    f1->halfedge() = h3;
    f2->halfedge() = h7;
    f3->halfedge() = h10;
    h0->set_neighbors(h1, h3, v, e0, f0);
    h1->set_neighbors(h6, h1->twin(), v2, h1->edge(), f0);
    h2->set_neighbors(h7, h2->twin(), v3, h2->edge(), f2);
    h3->set_neighbors(h11, h0, v2, e0, f1);
    h4->set_neighbors(h9, h4->twin(), v0, h4->edge(), f3);
    h5->set_neighbors(h3, h5->twin(), v1, h5->edge(), f1);
    h6->set_neighbors(h0, h8, v3, e1, f0);
    h7->set_neighbors(h8, h10, v0, e2, f2);
    h8->set_neighbors(h2, h6, v, e1, f2);
    h9->set_neighbors(h10, h11, v1, e3, f3);
    h10->set_neighbors(h4, h7, v, e2, f3);
    h11->set_neighbors(h5, h9, v, e3, f1);

    //TODO: handle boundary case

    return v;
}

/* 
    This method splits the given edge in half, but does not split the
    adjacent faces. Returns an iterator to the new vertex which splits
    the original edge.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::bisect_edge(EdgeRef e) {

    auto h0 = e->halfedge();
    auto h1 = h0->next();
    auto h2 = get_last_halfedge(h0);
    auto h3 = h0->twin();
    auto h4 = h3->next();
    auto h5 = get_last_halfedge(h3);
    auto h6 = new_halfedge();
    auto h7 = new_halfedge();
    auto v = new_vertex();
    auto v0 = h0->vertex();
    auto v1 = h3->vertex();
    auto e0 = e;
    auto e1 = new_edge();
    auto f0 = h0->face();
    auto f1 = h3->face();

    h0->set_neighbors(h1, h3, v, e0, f0);
    h1->set_neighbors(h2, h1->twin(), v1, h1->edge(), f0);
    h2->set_neighbors(h6, h2->twin(), h2->vertex(), h2->edge(), f0);
    h3->set_neighbors(h7, h0, v1, e0, f1);
    h4->set_neighbors(h5, h4->twin(), v0, h4->edge(), f1);
    h5->set_neighbors(h3, h5->twin(), h5->vertex(), h5->edge(), f1);
    h6->set_neighbors(h0, h7, v0, e1, f0);
    h7->set_neighbors(h4, h6, v, e1, f1);
    v->halfedge() = h0;
    v0->halfedge() = h6;
    v1->halfedge() = h3;
    e0->halfedge() = h0;
    e1->halfedge() = h6;
    f0->halfedge() = h0;
    f1->halfedge() = h3;

    return v;
}

std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::inset_face(Halfedge_Mesh::FaceRef f) {

    // hint: use bevel_face positions as a helper function here
    (void)f;
    return std::nullopt;
}

/*
    Bevels a vertex and inserts a vertex into the new vertex, returning a pointer to that vertex
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::extrude_vertex(VertexRef v) {
    (void)v;
    return std::nullopt;
}

/* Note on the beveling process:

    Each of the bevel_vertex, bevel_edge, and bevel_face functions do not represent
    a full bevel operation. Instead, they should update the _connectivity_ of
    the mesh, _not_ the positions of newly created vertices. In fact, you should set
    the positions of new vertices to be exactly the same as wherever they "started from."

    When you click on a mesh element while in bevel mode, one of those three functions
    is called. But, because you may then adjust the distance/offset of the newly
    beveled face, we need another method of updating the positions of the new vertices.

    This is where bevel_vertex_positions, bevel_edge_positions, and
    bevel_face_positions come in: these functions are called repeatedly as you
    move your mouse, the position of which determins the normal and tangent offset
    parameters. These functions are also passed an array of the original vertex
    positions: for bevel_vertex, it has one element, the original vertex position,
    for bevel_edge, two for the two vertices, and for bevel_face, it has the original
    position of each vertex in order starting from face->halfedge. You should use these 
    positions, as well as the normal and tangent offset fields to assign positions to 
    the new vertices.

    Finally, note that the normal and tangent offsets are not relative values - you
    should compute a particular new position from them, not a delta to apply.
*/

/*
    This method should replace the vertex v with a face, corresponding to
    a bevel operation. It should return the new face.  NOTE: This method is
    only responsible for updating the *connectivity* of the mesh---it does not
    need to update the vertex positions. These positions will be updated in
    Halfedge_Mesh::bevel_vertex_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_vertex(Halfedge_Mesh::VertexRef v) {

    // Reminder: You should set the positions of new vertices (v->pos) to be exactly
    // the same as wherever they "started from."

    auto h0 = v->halfedge();
    auto h1 = h0->next();
    auto h2 = get_last_halfedge(h0);
    auto h3 = h2->twin();
    auto h4 = h3->next();
    auto h5 = get_last_halfedge(h3);
    auto h6 = h5->twin();
    auto h7 = h6->next();
    auto h8 = get_last_halfedge(h6);
    auto h9 = new_halfedge();
    auto h10 = new_halfedge();
    auto h11 = new_halfedge();
    auto h12 = new_halfedge();
    auto h13 = new_halfedge();
    auto h14 = new_halfedge();
    auto v0 = v;
    auto v1 = h8->vertex();
    auto v2 = h2->vertex();
    auto v3 = h5->vertex();
    auto v4 = new_vertex();
    auto v5 = new_vertex();
    auto v6 = new_vertex();
    v4->pos = v4->pos = v6->pos = v0->pos;
    auto e0 = h0->edge();
    auto e1 = h2->edge();
    auto e2 = h5->edge();
    auto e3 = new_edge();
    auto e4 = new_edge();
    auto e5 = new_edge();
    auto f0 = h0->face();
    auto f1 = h3->face();
    auto f2 = h6->face();
    auto f3 = new_face();

    h0->set_neighbors(h1, h8, v4, e0, f0);
    h1->set_neighbors(h2, h1->twin(), v1, h1->edge(), f0);
    h2->set_neighbors(h9, h3, v2, e1, f0);
    h3->set_neighbors(h4, h2, v5, e1, f1);
    h4->set_neighbors(h5, h4->twin(), v2, h4->edge(), f1);
    h5->set_neighbors(h10, h6, v3, e2, f1);
    h6->set_neighbors(h7, h5, v6, e2, f2);
    h7->set_neighbors(h8, h7->twin(), v3, h7->edge(), f2);
    h8->set_neighbors(h11, h0, v1, e0, f2);
    h9->set_neighbors(h0, h13, v5, e3, f0);
    h10->set_neighbors(h3, h14, v6, e4, f1);
    h11->set_neighbors(h6, h12, v4, e5, f2);
    h12->set_neighbors(h13, h11, v6, e5, f3);
    h13->set_neighbors(h14, h9, v4, e3, f3);
    h14->set_neighbors(h12, h10, v5, e4, f3);
    erase(v0);
    v1->halfedge() = h1;
    v2->halfedge() = h4;
    v3->halfedge() = h7;
    v4->halfedge() = h13;
    v5->halfedge() = h14;
    v6->halfedge() = h12;
    e0->halfedge() = h0;
    e1->halfedge() = h3;
    e2->halfedge() = h6;
    e3->halfedge() = h13;
    e4->halfedge() = h14;
    e5->halfedge() = h12;
    f0->halfedge() = h0;
    f1->halfedge() = h3;
    f2->halfedge() = h6;
    f3->halfedge() = h14;

    return f3;
}

/*
    This method should replace the edge e with a face, corresponding to a
    bevel operation. It should return the new face. NOTE: This method is
    responsible for updating the *connectivity* of the mesh only---it does not
    need to update the vertex positions. These positions will be updated in
    Halfedge_Mesh::bevel_edge_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_edge(Halfedge_Mesh::EdgeRef e) {

    // Reminder: You should set the positions of new vertices (v->pos) to be exactly
    // the same as wherever they "started from."

    (void)e;
    return std::nullopt;
}

/*
    This method should replace the face f with an additional, inset face
    (and ring of faces around it), corresponding to a bevel operation. It
    should return the new face.  NOTE: This method is responsible for updating
    the *connectivity* of the mesh only---it does not need to update the vertex
    positions. These positions will be updated in
    Halfedge_Mesh::bevel_face_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_face(Halfedge_Mesh::FaceRef f) {

    // Reminder: You should set the positions of new vertices (v->pos) to be exactly
    // the same as wherever they "started from."

    (void)f;
    return std::nullopt;
}

/*
    Compute new vertex positions for the vertices of the beveled vertex.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the original vertex position and its associated outgoing edge
    to compute a new vertex position along the outgoing edge.
*/
void Halfedge_Mesh::bevel_vertex_positions(const std::vector<Vec3>& start_positions,
                                           Halfedge_Mesh::FaceRef face, float tangent_offset) {

    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());

    (void)new_halfedges;
    (void)start_positions;
    (void)face;
    (void)tangent_offset;
}

/*
    Updates the position of v using the given start_position
*/
void Halfedge_Mesh::extrude_vertex_position(const Vec3& start_positions, Halfedge_Mesh::FaceRef face) {
    (void)start_positions;
    (void)face;
}

/*
    Compute new vertex positions for the vertices of the beveled edge.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the preceding and next vertex position from the original mesh
    (in the orig array) to compute an offset vertex position.

    Note that there is a 1-to-1 correspondence between halfedges in
    newHalfedges and vertex positions in start_positions. So, you can write 
    loops of the form:

    for(size_t i = 0; i < new_halfedges.size(); i++)
    {
            Vector3D pi = start_positions[i]; // get the original vertex
            position corresponding to vertex i
    }
*/
void Halfedge_Mesh::bevel_edge_positions(const std::vector<Vec3>& start_positions,
                                         Halfedge_Mesh::FaceRef face, float tangent_offset) {

    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());

    (void)new_halfedges;
    (void)start_positions;
    (void)face;
    (void)tangent_offset;
}

/*
    Compute new vertex positions for the vertices of the beveled face.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the preceding and next vertex position from the original mesh
    (in the start_positions array) to compute an offset vertex
    position.

    Note that there is a 1-to-1 correspondence between halfedges in
    new_halfedges and vertex positions in start_positions. So, you can write 
    loops of the form:

    for(size_t i = 0; i < new_halfedges.size(); i++)
    {
            Vec3 pi = start_positions[i]; // get the original vertex
            position corresponding to vertex i
    }
*/
void Halfedge_Mesh::bevel_face_positions(const std::vector<Vec3>& start_positions,
                                         Halfedge_Mesh::FaceRef face, float tangent_offset,
                                         float normal_offset) {

    if(flip_orientation) normal_offset = -normal_offset;
    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());

    (void)new_halfedges;
    (void)start_positions;
    (void)face;
    (void)tangent_offset;
    (void)normal_offset;
}

/*
    Splits all non-triangular faces into triangles.
*/
void Halfedge_Mesh::triangulate() {

    // For each face...
}

/* Note on the quad subdivision process:

        Unlike the local mesh operations (like bevel or edge flip), we will perform
        subdivision by splitting *all* faces into quads "simultaneously."  Rather
        than operating directly on the halfedge data structure (which as you've
        seen is quite difficult to maintain!) we are going to do something a bit nicer:
           1. Create a raw list of vertex positions and faces (rather than a full-
              blown halfedge mesh).
           2. Build a new halfedge mesh from these lists, replacing the old one.
        Sometimes rebuilding a data structure from scratch is simpler (and even
        more efficient) than incrementally modifying the existing one.  These steps are
        detailed below.

  Step I: Compute the vertex positions for the subdivided mesh.
        Here we're going to do something a little bit strange: since we will
        have one vertex in the subdivided mesh for each vertex, edge, and face in
        the original mesh, we can nicely store the new vertex *positions* as
        attributes on vertices, edges, and faces of the original mesh. These positions
        can then be conveniently copied into the new, subdivided mesh.
        This is what you will implement in linear_subdivide_positions() and
        catmullclark_subdivide_positions().

  Steps II-IV are provided (see Halfedge_Mesh::subdivide()), but are still detailed
  here:

  Step II: Assign a unique index (starting at 0) to each vertex, edge, and
        face in the original mesh. These indices will be the indices of the
        vertices in the new (subdivided) mesh. They do not have to be assigned
        in any particular order, so long as no index is shared by more than one
        mesh element, and the total number of indices is equal to V+E+F, i.e.,
        the total number of vertices plus edges plus faces in the original mesh.
        Basically we just need a one-to-one mapping between original mesh elements
        and subdivided mesh vertices.

  Step III: Build a list of quads in the new (subdivided) mesh, as tuples of
        the element indices defined above. In other words, each new quad should be
        of the form (i,j,k,l), where i,j,k and l are four of the indices stored on
        our original mesh elements.  Note that it is essential to get the orientation
        right here: (i,j,k,l) is not the same as (l,k,j,i).  Indices of new faces
        should circulate in the same direction as old faces (think about the right-hand
        rule).

  Step IV: Pass the list of vertices and quads to a routine that clears
        the internal data for this halfedge mesh, and builds new halfedge data from
        scratch, using the two lists.
*/

/*
    Compute new vertex positions for a mesh that splits each polygon
    into quads (by inserting a vertex at the face midpoint and each
    of the edge midpoints).  The new vertex positions will be stored
    in the members Vertex::new_pos, Edge::new_pos, and
    Face::new_pos.  The values of the positions are based on
    simple linear interpolation, e.g., the edge midpoints and face
    centroids.
*/
void Halfedge_Mesh::linear_subdivide_positions() {

    // For each vertex, assign Vertex::new_pos to
    // its original position, Vertex::pos.

    // For each edge, assign the midpoint of the two original
    // positions to Edge::new_pos.

    // For each face, assign the centroid (i.e., arithmetic mean)
    // of the original vertex positions to Face::new_pos. Note
    // that in general, NOT all faces will be triangles!
}

/*
    Compute new vertex positions for a mesh that splits each polygon
    into quads (by inserting a vertex at the face midpoint and each
    of the edge midpoints).  The new vertex positions will be stored
    in the members Vertex::new_pos, Edge::new_pos, and
    Face::new_pos. The values of the positions are based on
    the Catmull-Clark rules for subdivision.

    Note: this will only be called on meshes without boundary
*/
void Halfedge_Mesh::catmullclark_subdivide_positions() {

    // The implementation for this routine should be
    // a lot like Halfedge_Mesh:linear_subdivide_positions:(),
    // except that the calculation of the positions themsevles is
    // slightly more involved, using the Catmull-Clark subdivision
    // rules. (These rules are outlined in the Developer Manual.)

    // Faces

    // Edges

    // Vertices
}

/*
    This routine should increase the number of triangles in the mesh
    using Loop subdivision. Note: this is will only be called on triangle meshes.
*/
void Halfedge_Mesh::loop_subdivide() {

    // Each vertex and edge of the original mesh can be associated with a
    // vertex in the new (subdivided) mesh.
    // Therefore, our strategy for computing the subdivided vertex locations is to
    // *first* compute the new positions
    // using the connectivity of the original (coarse) mesh. Navigating this mesh
    // will be much easier than navigating
    // the new subdivided (fine) mesh, which has more elements to traverse.  We
    // will then assign vertex positions in
    // the new mesh based on the values we computed for the original mesh.
    
    // Compute new positions for all the vertices in the input mesh using
    // the Loop subdivision rule and store them in Vertex::new_pos.
    //    At this point, we also want to mark each vertex as being a vertex of the
    //    original mesh. Use Vertex::is_new for this.
    
    // Next, compute the subdivided vertex positions associated with edges, and
    // store them in Edge::new_pos.
    
    // Next, we're going to split every edge in the mesh, in any order.
    // We're also going to distinguish subdivided edges that came from splitting 
    // an edge in the original mesh from new edges by setting the boolean Edge::is_new. 
    // Note that in this loop, we only want to iterate over edges of the original mesh.
    // Otherwise, we'll end up splitting edges that we just split (and the
    // loop will never end!)
    
    // Now flip any new edge that connects an old and new vertex.
    
    // Finally, copy new vertex positions into the Vertex::pos.
}

/*
    Isotropic remeshing. Note that this function returns success in a similar
    manner to the local operations, except with only a boolean value.
    (e.g. you may want to return false if this is not a triangle mesh)
*/
bool Halfedge_Mesh::isotropic_remesh() {

    // Compute the mean edge length.
    // Repeat the four main steps for 5 or 6 iterations
    // -> Split edges much longer than the target length (being careful about
    //    how the loop is written!)
    // -> Collapse edges much shorter than the target length.  Here we need to
    //    be EXTRA careful about advancing the loop, because many edges may have
    //    been destroyed by a collapse (which ones?)
    // -> Now flip each edge if it improves vertex degree
    // -> Finally, apply some tangential smoothing to the vertex positions

    // Note: if you erase elements in a local operation, they will not be actually deleted
    // until do_erase or validate is called. This is to facilitate checking
    // for dangling references to elements that will be erased.
    // The rest of the codebase will automatically call validate() after each op,
    // but here simply calling collapse_edge() will not erase the elements.
    // You should use collapse_edge_erase() instead for the desired behavior.

    return false;
}

/* Helper type for quadric simplification */
struct Edge_Record {
    Edge_Record() {
    }
    Edge_Record(std::unordered_map<Halfedge_Mesh::VertexRef, Mat4>& vertex_quadrics,
                Halfedge_Mesh::EdgeRef e)
        : edge(e) {

        // Compute the combined quadric from the edge endpoints.
        // -> Build the 3x3 linear system whose solution minimizes the quadric error
        //    associated with these two endpoints.
        // -> Use this system to solve for the optimal position, and store it in
        //    Edge_Record::optimal.
        // -> Also store the cost associated with collapsing this edge in
        //    Edge_Record::cost.
    }
    Halfedge_Mesh::EdgeRef edge;
    Vec3 optimal;
    float cost;
};

/* Comparison operator for Edge_Records so std::set will properly order them */
bool operator<(const Edge_Record& r1, const Edge_Record& r2) {
    if(r1.cost != r2.cost) {
        return r1.cost < r2.cost;
    }
    Halfedge_Mesh::EdgeRef e1 = r1.edge;
    Halfedge_Mesh::EdgeRef e2 = r2.edge;
    return &*e1 < &*e2;
}

/** Helper type for quadric simplification
 *
 * A PQueue is a minimum-priority queue that
 * allows elements to be both inserted and removed from the
 * queue.  Together, one can easily change the priority of
 * an item by removing it, and re-inserting the same item
 * but with a different priority.  A priority queue, for
 * those who don't remember or haven't seen it before, is a
 * data structure that always keeps track of the item with
 * the smallest priority or "score," even as new elements
 * are inserted and removed.  Priority queues are often an
 * essential component of greedy algorithms, where one wants
 * to iteratively operate on the current "best" element.
 *
 * PQueue is templated on the type T of the object
 * being queued.  For this reason, T must define a comparison
 * operator of the form
 *
 *    bool operator<( const T& t1, const T& t2 )
 *
 * which returns true if and only if t1 is considered to have a
 * lower priority than t2.
 *
 * Basic use of a PQueue might look
 * something like this:
 *
 *    // initialize an empty queue
 *    PQueue<myItemType> queue;
 *
 *    // add some items (which we assume have been created
 *    // elsewhere, each of which has its priority stored as
 *    // some kind of internal member variable)
 *    queue.insert( item1 );
 *    queue.insert( item2 );
 *    queue.insert( item3 );
 *
 *    // get the highest priority item currently in the queue
 *    myItemType highestPriorityItem = queue.top();
 *
 *    // remove the highest priority item, automatically
 *    // promoting the next-highest priority item to the top
 *    queue.pop();
 *
 *    myItemType nextHighestPriorityItem = queue.top();
 *
 *    // Etc.
 *
 *    // We can also remove an item, making sure it is no
 *    // longer in the queue (note that this item may already
 *    // have been removed, if it was the 1st or 2nd-highest
 *    // priority item!)
 *    queue.remove( item2 );
 *
 */
template<class T> struct PQueue {
    void insert(const T& item) {
        queue.insert(item);
    }
    void remove(const T& item) {
        if(queue.find(item) != queue.end()) {
            queue.erase(item);
        }
    }
    const T& top(void) const {
        return *(queue.begin());
    }
    void pop(void) {
        queue.erase(queue.begin());
    }
    size_t size() {
        return queue.size();
    }

    std::set<T> queue;
};

/*
    Mesh simplification. Note that this function returns success in a similar
    manner to the local operations, except with only a boolean value.
    (e.g. you may want to return false if you can't simplify the mesh any
    further without destroying it.)
*/
bool Halfedge_Mesh::simplify() {

    std::unordered_map<VertexRef, Mat4> vertex_quadrics;
    std::unordered_map<FaceRef, Mat4> face_quadrics;
    std::unordered_map<EdgeRef, Edge_Record> edge_records;
    PQueue<Edge_Record> edge_queue;

    // Compute initial quadrics for each face by simply writing the plane equation
    // for the face in homogeneous coordinates. These quadrics should be stored
    // in face_quadrics
    // -> Compute an initial quadric for each vertex as the sum of the quadrics
    //    associated with the incident faces, storing it in vertex_quadrics
    // -> Build a priority queue of edges according to their quadric error cost,
    //    i.e., by building an Edge_Record for each edge and sticking it in the
    //    queue. You may want to use the above PQueue<Edge_Record> for this.
    // -> Until we reach the target edge budget, collapse the best edge. Remember
    //    to remove from the queue any edge that touches the collapsing edge
    //    BEFORE it gets collapsed, and add back into the queue any edge touching
    //    the collapsed vertex AFTER it's been collapsed. Also remember to assign
    //    a quadric to the collapsed vertex, and to pop the collapsed edge off the
    //    top of the queue.

    // Note: if you erase elements in a local operation, they will not be actually deleted
    // until do_erase or validate are called. This is to facilitate checking
    // for dangling references to elements that will be erased.
    // The rest of the codebase will automatically call validate() after each op,
    // but here simply calling collapse_edge() will not erase the elements.
    // You should use collapse_edge_erase() instead for the desired behavior.

    return false;
}
