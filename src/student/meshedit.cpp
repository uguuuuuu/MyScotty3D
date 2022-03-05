
#include <queue>
#include <set>
#include <unordered_map>
#include <algorithm>
#include <numeric>
#include <functional>

#include "../geometry/halfedge.h"
#include "debug.h"
#include "../lib/log.h"

/******************************************************************
*********************** Local Operations **************************
******************************************************************/

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

// Traverses outgoing halfedges in clockwise order
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
// Traverses neighbors in clockwise order
std::vector<Halfedge_Mesh::VertexRef> get_neighbors(Halfedge_Mesh::VertexRef v) {

    auto outgoing_halfedges = get_outgoing_halfedges(v);
    std::vector<Halfedge_Mesh::VertexRef> neighbors(outgoing_halfedges.size());
    std::transform(outgoing_halfedges.begin(), outgoing_halfedges.end(), neighbors.begin(),
                   [](Halfedge_Mesh::HalfedgeRef h) { return h->twin()->vertex(); });

    return neighbors;
}
Halfedge_Mesh::HalfedgeRef get_last_halfedge(Halfedge_Mesh::HalfedgeRef h) {

    auto hh = h;
    do {
        hh = hh->next();
    } while(hh->next() != h);

    return hh;
}
// Traverses incident halfedges in counter-clockwise order
std::vector<Halfedge_Mesh::HalfedgeRef> get_incident_halfedges(Halfedge_Mesh::EdgeRef e) {
    auto he0 = e->halfedge();
    auto he1 = he0->twin();
    auto reorder = [](std::vector<Halfedge_Mesh::HalfedgeRef>&& hs, Halfedge_Mesh::HalfedgeRef h) {
        auto itr = std::find(hs.begin(), hs.end(), h);
        std::rotate(hs.begin(), itr, hs.end());
        std::reverse(hs.begin(), hs.end());
        hs.pop_back();
        return hs;
    };
    auto outgoing_halfedges0 = reorder(get_outgoing_halfedges(he0->vertex()), he0);
    auto outgoing_halfedges1 = reorder(get_outgoing_halfedges(he1->vertex()), he1);
    std::vector<Halfedge_Mesh::HalfedgeRef> incident_halfedges(outgoing_halfedges0.size() +
                                                outgoing_halfedges1.size());
    std::move(outgoing_halfedges1.begin(), outgoing_halfedges1.end(),
              std::move(outgoing_halfedges0.begin(), outgoing_halfedges0.end(),
                        incident_halfedges.begin()));
    return incident_halfedges;
}
std::vector<Halfedge_Mesh::FaceRef> get_incident_faces(Halfedge_Mesh::EdgeRef e) {
    auto h = e->halfedge();
    auto f0 = h->face();
    auto f1 = h->twin()->face();

    return {f0, f1};
}
// Traverses incident faces in clockwise order
std::vector<Halfedge_Mesh::FaceRef> get_incident_faces(Halfedge_Mesh::VertexRef v) {
    auto outgoing_halfedges = get_outgoing_halfedges(v);
    std::vector<Halfedge_Mesh::FaceRef> faces(outgoing_halfedges.size());
    std::transform(outgoing_halfedges.begin(), outgoing_halfedges.end(), faces.begin(),
                   [](Halfedge_Mesh::HalfedgeRef h) { return h->face(); });

    return faces;
}
// Traverses incident edges in clockwise order
std::vector<Halfedge_Mesh::EdgeRef> get_incident_edges(Halfedge_Mesh::VertexRef v) {
    auto outgoing_halfedges = get_outgoing_halfedges(v);
    std::vector<Halfedge_Mesh::EdgeRef> edges(outgoing_halfedges.size());
    std::transform(outgoing_halfedges.begin(), outgoing_halfedges.end(), edges.begin(),
                   [](Halfedge_Mesh::HalfedgeRef h) { return h->edge(); });

    return edges;
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
// Traverses vertices in counter-clockwise order
std::vector<Halfedge_Mesh::VertexRef> get_vertices(Halfedge_Mesh::FaceRef f) {

    auto bdry_halfedges = get_boundary_halfedges(f->halfedge());
    std::vector<Halfedge_Mesh::VertexRef> vertices(bdry_halfedges.size());
    std::transform(bdry_halfedges.begin(), bdry_halfedges.end(), vertices.begin(),
                   [](Halfedge_Mesh::HalfedgeRef h) { return h->vertex(); });

    return vertices;
}
Halfedge_Mesh::VertexRef move(Halfedge_Mesh::VertexRef dst, Halfedge_Mesh::VertexRef src) {
    
    dst->halfedge() = src->halfedge();
    for(auto h : get_outgoing_halfedges(src)) h->vertex() = dst;
    return dst;
}
// Merges the two vertices delimiting the edge of h
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
// @return v: merged vertex whose halfedge's edge is merge edge
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

Halfedge_Mesh::VertexRef collapse_half(Halfedge_Mesh::HalfedgeRef h,
                                                        Halfedge_Mesh& m) {

    auto h0 = h->next();
    auto h1 = get_last_halfedge(h);
    auto v0 = m.new_vertex();

    if(num_edges(h->face()) == 3) {
        m.erase(h);
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

        auto e0 = m.new_edge();
        auto v1 = h1->vertex();
        v0->halfedge() = h0;
        v1->halfedge() = h1;
        e0->halfedge() = h0;
        h0->set_neighbors(h0->next(), h1, v0, e0, h0->face());
        h1->set_neighbors(h1->next(), h0, v1, e0, h1->face());
    } else {
        auto f0 = h->face();
        v0->halfedge() = h0;
        f0->halfedge() = h0;
        h0->vertex() = v0;
        h1->next() = h0;

        m.erase(h);
    }

    return v0;
}
/*
    This method should collapse the given edge and return an iterator to
    the new vertex created by the collapse.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_edge(Halfedge_Mesh::EdgeRef e) {

    auto h = e->halfedge()->face()->is_boundary() ? e->halfedge()->twin() : e->halfedge();
    auto v0 = h->vertex();
    auto v1 = h->twin()->vertex();
    bool on_bdry = on_boundary(e);
    auto p = e->center();

    unsigned int num_triangles = 0;
    std::vector<Halfedge_Mesh::VertexRef> common_neighbors;
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

    auto v = collapse_half(h, *this);
    if(!on_bdry) {
        auto h0 = v->halfedge();
        erase(v);
        v = collapse_half(h->twin(), *this);
        h0->vertex() = v;
    }
    else {
        get_last_halfedge(h->twin())->next() = h->twin()->next();
    }


    for(auto hh : outgoing_halfedges0) {
        hh->vertex() = v;
    }
    for(auto hh : outgoing_halfedges1) {
        hh->vertex() = v;
    }

    //v->pos = v->neighborhood_center();
    v->pos = p;

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
    (void)f;
    return std::nullopt;
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
    v->pos = e->center();
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
    This method should insets a vertex into the given face, returning a pointer to the new center vertex
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::bisect_edge(EdgeRef e) {

    (void)e;
    return std::nullopt;
}

/*
    This method should inset a face into the given face, returning a pointer to the new face.
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::inset_face(Halfedge_Mesh::FaceRef f) {

    // hint: use bevel_face positions as a helper function here
    (void)f;
    return std::nullopt;
}

/*
    This method should bevel a vertex and inserts a vertex into the new vertex, returning a pointer to that vertex
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

    auto outgoing_halfedges = get_outgoing_halfedges(v);
    std::reverse(outgoing_halfedges.begin(), outgoing_halfedges.end());
    auto new_vertices =
        std::reduce(outgoing_halfedges.begin(), outgoing_halfedges.end(), std::vector<VertexRef>(),
                    [&](std::vector<VertexRef> a, HalfedgeRef h) {
                        a.push_back(new_vertex());
                        return a;
                    });
    auto f = new_face();
    for(size_t i = 0; i < new_vertices.size(); i++) {
        auto v0 = new_vertices[i];
        v0->pos = v->pos;
        auto v1 = new_vertices[(i + 1) % new_vertices.size()];
        auto h0 = outgoing_halfedges[i];
        auto e0 = new_edge();
        auto h1 = new_halfedge();
        auto h2 = new_halfedge();
        auto h3 = get_last_halfedge(h0);

        v0->halfedge() = h1;
        e0->halfedge() = h1;
        h0->vertex() = v0;
        h1->set_neighbors(h1, h2, v0, e0, f);
        h2->set_neighbors(h0, h1, v1, e0, h0->face());
        h3->next() = h2;
        f->halfedge() = h1;
    }
    for(size_t i = 0; i < new_vertices.size(); i++) {
        auto v0 = new_vertices[i];
        auto v1 = new_vertices[(i + 1) % new_vertices.size()];
        v0->halfedge()->next() = v1->halfedge();
    }
    erase(v);
    return f;
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

    auto incident_halfedges = get_incident_halfedges(e);
    size_t n_incident_edges0 = num_incident_edges(e->halfedge()->vertex()) - 1;
    auto endpt0 = e->halfedge()->vertex();
    auto endpt1 = e->halfedge()->twin()->vertex();
    auto new_vertices =
        std::reduce(incident_halfedges.begin(), incident_halfedges.end(), std::vector<VertexRef>(),
                    [&](std::vector<VertexRef> a, HalfedgeRef) {
                        a.push_back(new_vertex());
                        return a;
                    });
    auto f = new_face();

    for(size_t i = 0; i < n_incident_edges0; i++) {
        new_vertices[i]->pos = endpt0->pos;
    }
    for(size_t i = n_incident_edges0; i < new_vertices.size(); i++) {
        new_vertices[i]->pos = endpt1->pos;
    }

    for(size_t i = 0; i < new_vertices.size(); i++) {
        auto v0 = new_vertices[i];
        auto v1 = new_vertices[(i + 1) % new_vertices.size()];
        auto e0 = new_edge();
        auto h0 = incident_halfedges[i];
        auto h1 = new_halfedge();
        auto h2 = new_halfedge();
        auto h3 = incident_halfedges[(i + 1) % new_vertices.size()]->twin();
        auto f0 = h0->face();

        v0->halfedge() = h1;
        e0->halfedge() = h1;
        f->halfedge() = h1;
        f0->halfedge() = h0;
        h0->vertex() = v0;
        h1->set_neighbors(h1, h2, v0, e0, f);
        h2->set_neighbors(h0, h1, v1, e0, f0);
        h3->next() = h2;
    }
    for(size_t i = 0; i < new_vertices.size(); i++) {
        auto v0 = new_vertices[i];
        auto v1 = new_vertices[(i + 1) % new_vertices.size()];
        v0->halfedge()->next() = v1->halfedge();
    }

    erase(e->halfedge()->vertex());
    erase(e->halfedge()->twin()->vertex());
    erase(e);
    erase(e->halfedge());
    erase(e->halfedge()->twin());

    return f;
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

    auto bdry_halfedges = get_boundary_halfedges(f->halfedge());
    auto verts = get_vertices(f);
    auto new_vertices = std::reduce(verts.begin(), verts.end(), std::vector<VertexRef>(),
                                    [&](std::vector<VertexRef> a, VertexRef) {
                                        a.push_back(new_vertex());
                                        return a;
                                    });

    auto f0 = new_face();
    for(size_t i = 0; i < new_vertices.size(); i++) {
        new_vertices[i]->pos = verts[i]->pos;
    }
    for(size_t i = 0; i < new_vertices.size(); i++) {
        auto v0 = new_vertices[i];
        auto v1 = verts[i];
        auto v2 = new_vertices[(i + 1) % new_vertices.size()];
        auto e0 = new_edge();
        auto e1 = new_edge();
        auto f1 = new_face();
        auto h0 = new_halfedge();
        auto h1 = new_halfedge();
        auto h2 = new_halfedge();
        auto h3 = new_halfedge();
        auto h4 = bdry_halfedges[i];

        v0->halfedge() = h0;
        v1->halfedge() = h4;
        e0->halfedge() = h0;
        e1->halfedge() = h3;
        f0->halfedge() = h0;
        f1->halfedge() = h1;
        h0->set_neighbors(h0, h1, v0, e0, f0);
        h1->set_neighbors(h2, h0, v2, e0, f1);
        h2->set_neighbors(h4, h3, v0, e1, f1);
        h3->set_neighbors(h3, h2, v1, e1, f0);
        h4->set_neighbors(h4, h4->twin(), v1, h4->edge(), f1);
    }
    for(size_t i = 0; i < new_vertices.size(); i++) {
        // set h0's next
        // set h3's next and face
        // set h4's next

        auto v0 = new_vertices[i];
        auto v1 = new_vertices[(i + 1) % new_vertices.size()];
        auto v2 = new_vertices[(i + new_vertices.size() - 1) % new_vertices.size()];
        auto h0 = v0->halfedge();
        auto h1 = v1->halfedge();
        auto h2 = v2->halfedge()->twin();
        auto h3 = h0->twin()->next()->twin();
        auto h4 = bdry_halfedges[i];
        auto h5 = h1->twin()->next()->twin();
        auto f1 = h2->face();
        
        h0->next() = h1;
        h3->next() = h2;
        h3->face() = f1;
        h4->next() = h5;
    }

    erase(f);

    return f0;
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

    for(size_t i = 0; i < new_halfedges.size(); i++) {
        auto v0 = new_halfedges[i]->vertex();
        auto v1 = new_halfedges[i]->twin()->next()->twin()->vertex();
        auto tangent_vec = (v1->pos - start_positions[i]).normalize();
        v0->pos = tangent_vec * tangent_offset + start_positions[i];
    }

    //info("Start positions");
    //for(auto p : start_positions) {
    //    info("(%d, %d, %d)", p.x, p.y, p.z);
    //}
    //info("----------------------");
    //info("Vertex positions");
    //for(auto h0 : new_halfedges) {
    //    auto v = h0->vertex();
    //    info("(%d, %d, %d)", v->pos.x, v->pos.y, v->pos.z);
    //}
    //info("----------------------");
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

    for(size_t i = 0; i < new_halfedges.size(); i++) {
        auto v0 = new_halfedges[i]->vertex();
        auto v1 = new_halfedges[i]->twin()->next()->twin()->vertex();
        auto tangent_vec = (v1->pos - start_positions[i]).normalize();
        v0->pos = tangent_vec * tangent_offset + start_positions[i];
    }
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

    size_t n_verts = start_positions.size();
    for(size_t i = 0; i < new_halfedges.size(); i++) {
        auto v0 = new_halfedges[i]->vertex();
        auto start_pos0 = start_positions[i];
        auto start_pos1 = start_positions[(i + 1) % n_verts];
        Vec3 start_pos2;
        auto start_pos3 = start_positions[(i + n_verts - 1) % n_verts];
        if(n_verts % 2 == 0) {
            start_pos2 = start_positions[(n_verts / 2 + i) % n_verts];
        } else {
            start_pos2 = (start_positions[(n_verts / 2 + i) % n_verts] +
                          start_positions[(n_verts / 2 + 1 + i) % n_verts]) /
                         2.f;
        }

        auto tangent = (start_pos0 - start_pos2).normalize();
        auto normal = cross(start_pos1 - start_pos0, start_pos3 - start_pos0).normalize();

        v0->pos = start_pos0 + tangent * tangent_offset + normal * normal_offset;
    }
}

/*
    Updates the position of v using the given start_position
*/
void Halfedge_Mesh::extrude_vertex_position(const Vec3& start_positions, Halfedge_Mesh::FaceRef face) {
    (void)start_positions;
    (void)face;
}

/******************************************************************
*********************** Global Operations *************************
******************************************************************/

/*
    Splits all non-triangular faces into triangles.
*/
void Halfedge_Mesh::triangulate() {

    // For each face...
    for(auto f = faces.begin(); f != faces.end(); f++) {
        auto bdry_halfedges = get_boundary_halfedges(f->halfedge());
        auto verts = get_vertices(f);
        auto n_verts = bdry_halfedges.size();
        if(n_verts == 3) {
            continue;
        }
        
        if(n_verts == 4) {
            auto v0 = verts[0];
            auto v1 = verts[1];
            auto v2 = verts[2];
            auto v3 = verts[3];
            auto e0 = new_edge();
            auto f0 = new_face();
            auto h0 = bdry_halfedges[0];
            auto h1 = bdry_halfedges[1];
            auto h2 = new_halfedge();
            auto h3 = new_halfedge();
            auto h4 = bdry_halfedges[2];
            auto h5 = bdry_halfedges[3];

            v0->halfedge() = h0;
            v1->halfedge() = h1;
            v2->halfedge() = h4;
            v3->halfedge() = h5;
            e0->halfedge() = h2;
            f->halfedge() = h3;
            f0->halfedge() = h2;
            h0->face() = f0;
            h1->set_neighbors(h2, h1->twin(), v1, h1->edge(), f0);
            h2->set_neighbors(h0, h3, v2, e0, f0);
            h3->set_neighbors(h4, h2, v0, e0, f);
            h5->next() = h3;

            continue;
        }

        for(size_t i = 0; i < n_verts / 2; i++) {
            auto v0 = verts[i * 2];
            auto v1 = verts[(i * 2 + 1) % n_verts];
            auto v2 = verts[(i * 2 + 2) % n_verts];
            auto e0 = new_edge();
            auto f0 = new_face();
            auto h0 = bdry_halfedges[i * 2];
            auto h1 = bdry_halfedges[(i * 2 + 1) % n_verts];
            auto h2 = new_halfedge();
            auto h3 = new_halfedge();

            v0->halfedge() = h0;
            v1->halfedge() = h1;
            e0->halfedge() = h2;
            f->halfedge() = h3;
            f0->halfedge() = h2;
            h0->set_neighbors(h1, h0->twin(), v0, h0->edge(), f0);
            h1->set_neighbors(h2, h1->twin(), v1, h1->edge(), f0);
            h2->set_neighbors(h0, h3, v2, e0, f0);
            h3->set_neighbors(h3, h2, v0, e0, f);
        }
        if(n_verts % 2 != 0) {
            for(size_t i = 0; i < n_verts / 2 - 1; i++) {
                // set h3's next

                auto v0 = verts[i * 2];
                auto v1 = verts[(i * 2 + 2) % n_verts];
                auto h3 = get_last_halfedge(v0->halfedge())->twin();
                auto h4 = get_last_halfedge(v1->halfedge())->twin();
                h3->next() = h4;
            }
            auto v0 = verts[n_verts - 3];
            auto h3 = get_last_halfedge(v0->halfedge())->twin();
            auto h4 = bdry_halfedges[n_verts - 1];
            h3->next() = h4;
            h4->next() = get_last_halfedge(verts[0]->halfedge())->twin();
        } else {
            for(size_t i = 0; i < n_verts / 2; i++) {
                // set h3's next

                auto v0 = verts[i * 2];
                auto v1 = verts[(i * 2 + 2) % n_verts];
                auto h3 = get_last_halfedge(v0->halfedge())->twin();
                auto h4 = get_last_halfedge(v1->halfedge())->twin();
                h3->next() = h4;
            }
        }
    }
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

    for(auto v = vertices.begin(); v != vertices.end(); v++) {
        v->new_pos = v->pos;
    }

    // For each edge, assign the midpoint of the two original
    // positions to Edge::new_pos.

    for(auto e = edges.begin(); e != edges.end(); e++) {
        e->new_pos = center_of(e);
    }

    // For each face, assign the centroid (i.e., arithmetic mean)
    // of the original vertex positions to Face::new_pos. Note
    // that in general, NOT all faces will be triangles!

    for(auto f = faces.begin(); f != faces.end(); f++) {
        f->new_pos = center_of(f);
    }

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

    for(auto f = faces.begin(); f != faces.end(); f++) {
        f->new_pos = center_of(f);
    }

    // Edges

    for(auto e = edges.begin(); e != edges.end(); e++) {
        auto incident_faces = get_incident_faces(e);
        auto f0 = incident_faces[0];
        auto f1 = incident_faces[1];
        e->new_pos = (((f0->new_pos + f1->new_pos) / 2) + center_of(e)) / 2;
    }

    // Vertices

    for(auto v = vertices.begin(); v != vertices.end(); v++) {
        auto deg = static_cast<float>(v->degree());
        auto incident_faces = get_incident_faces(v);
        auto Q = std::reduce(incident_faces.begin(), incident_faces.end(), Vec3(0.f),
                             [&](Vec3 a, FaceRef f) { return a + f->new_pos; }) /
                 deg;
        auto incident_edges = get_incident_edges(v);
        auto R = std::reduce(incident_edges.begin(), incident_edges.end(), Vec3(0.f),
                             [&](Vec3 a, EdgeRef e) { return a + e->center(); }) /
                 deg;
        auto S = v->pos;
        v->new_pos = (Q + 2 * R + (deg - 3) * S) / deg;
    }
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

    info("Step 1");
    for(auto v = vertices.begin(); v != vertices.end(); v++) {
        v->is_new = false;

        float deg = static_cast<float>(v->degree());
        auto neighbors = get_neighbors(v);
        assert(neighbors.size() == v->degree());
        float u = 0.f;
        if(neighbors.size() == 3) {
            u = 3.f / 16.f;
        } else {
            u = 3.f / (8.f * deg);
        }
        v->new_pos =
            (1.f - deg * u) * v->pos + std::reduce(neighbors.begin(), neighbors.end(), Vec3(0.f),
                                                   [](Vec3 a, VertexRef v) { return a + v->pos; }) *
                                           u;
    }

    // Next, compute the subdivided vertex positions associated with edges, and
    // store them in Edge::new_pos.

    info("Step 2");
    for(auto e = edges.begin(); e != edges.end(); e++) {
        e->is_new = false;

        auto h0 = e->halfedge();
        auto v0 = h0->next()->next()->vertex();
        auto v1 = h0->twin()->next()->next()->vertex();
        e->new_pos = e->center() * 2.f * (3.f / 8.f) + (v0->pos + v1->pos) / 8.f;
    }

    // Next, we're going to split every edge in the mesh, in any order.
    // We're also going to distinguish subdivided edges that came from splitting
    // an edge in the original mesh from new edges by setting the boolean Edge::is_new.
    // Note that in this loop, we only want to iterate over edges of the original mesh.
    // Otherwise, we'll end up splitting edges that we just split (and the
    // loop will never end!)

    info("Step 3");
    auto e = edges.begin();
    for(size_t i = 0, num_edges = n_edges(); i < num_edges; i++) {
        auto next = e;
        next++;

        auto new_pos = e->new_pos;
        auto v = split_edge(e).value();
        auto incident_edges = get_incident_edges(v);
        assert(incident_edges.size() == 4);
        incident_edges[0]->is_new = false;
        incident_edges[1]->is_new = true;
        incident_edges[2]->is_new = false;
        incident_edges[3]->is_new = true;
        v->new_pos = new_pos;
        v->is_new = true;

        e = next;
    }
    
    size_t ctr = 0;
    for(e = edges.begin(); e != edges.end(); e++) {
        if(e->is_new) ctr++;
    }
    info("%i new edges", ctr);
    ctr = 0;

    // Now flip any new edge that connects an old and new vertex.

    info("Step 4");
    for(e = edges.begin(); e != edges.end(); e++) {
        if(e->is_new) {
            auto v0 = e->halfedge()->vertex();
            auto v1 = e->halfedge()->twin()->vertex();
            if(v0->is_new != v1->is_new) {
                flip_edge(e);
                ctr++;
            }
        }
    }
    info("flipped %i edges", ctr);

    // Finally, copy new vertex positions into the Vertex::pos.

    info("Step 5");
    for(auto v = vertices.begin(); v != vertices.end(); v++) {
        v->pos = v->new_pos;
    }

    info("Finished");
}

// Returns deviation of offset plus degree of v from 6 
inline unsigned int dev(Halfedge_Mesh::VertexRef v, int offset = 0) {
    unsigned int deg = v->degree() + offset;
    return deg >= 6u ? deg - 6u : 6u - deg;
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

    for(auto f = faces.begin(); f != faces.end(); f++) {
        if(f->is_boundary()) {
            info("Contains boundary face!");
            return false;
        }
        if(f->degree() != 3) {
            info("Not triangle mesh!");
            return false;
        }
    }

    constexpr float EPSLON = 0.f;
    float L = 0.f;
    for(auto e = edges.begin(); e != edges.end(); e++) {
        L += e->length();
    }
    L /= static_cast<float>(n_edges());

    info("Step 1: splitting");
    auto e = edges.begin();
    for(size_t i = 0, num_edges = n_edges(); i < num_edges; i++) {
        auto next = e;
        next++;
        if(e->length() - EPSLON > 4.f * L / 3.f) {
            split_edge(e);
        }
        e = next;
    }
    info("Step 1 finised. Validating...");
    auto err = validate();
    if(err != std::nullopt) {
        info("%s", err.value().second.c_str());
        return false;
    }
    for(auto v = vertices.begin(); v != vertices.end(); v++) {
        auto N = v->normal();
        if(!(std::isfinite(N.x) && std::isfinite(N.y) && std::isfinite(N.z))) {
            info("N = (%f, %f, %f) of vertex %i is infinite", N.x, N.y, N.z, v->id());
            return false;
        }
    }
    info("Validation finished");

    info("Step 2: collapsing");
    e = edges.begin();
    for(size_t i = 0, num_edges = n_edges(); i < num_edges; i++) {
        auto next = e;
        next++;

        if(e->length() + EPSLON < 4.f * L / 5.f) {
            auto incident_faces = get_incident_faces(e);

            while(std::find_if(incident_faces.begin(), incident_faces.end(), [&](FaceRef f) {
                      return f == next->halfedge()->face() || f == next->halfedge()->twin()->face();
                  }) != incident_faces.end()) {
                next++;
            }
            i += 2;

            auto v = collapse_edge_erase(e);
            if(v == std::nullopt) {
                info("Skipping edge %i", e->id());
                next = e;
                next++;
                i -= 2;
            }
        }

        e = next;
    }
    info("Step 2 finished. Validating...");
    err = validate();
    if(err != std::nullopt) {
        info("%s", err.value().second.c_str());
        return false;
    }
    for(auto v = vertices.begin(); v != vertices.end(); v++) {
        auto N = v->normal();
        if(!(std::isfinite(N.x) && std::isfinite(N.y) && std::isfinite(N.z))) {
            info("N = (%f, %f, %f) of vertex %i is infinite", N.x, N.y, N.z, v->id());
            info("Position: (%f, %f, %f)", v->pos.x, v->pos.y, v->pos.z);
            info("Degree: %i", v->degree());
            auto neighbors = get_neighbors(v);
            info("%i neighbors:", neighbors.size());
            for(auto n : neighbors) {
                info("Position of vertex %i: (%f, %f, %f)", n->id(), n->pos.x, n->pos.y, n->pos.z);
            }
            return false;
        }
        //if(v->pos.x == 0.f && v->pos.y == 0.f && v->pos.z == 0.f) {
        //    info("Vertex %i is 0", v->id());
        //    return false;
        //}
    }
    info("Validation finished");

    info("Step 3: flipping");
    for(e = edges.begin(); e != edges.end(); e++) {
        auto h0 = e->halfedge();
        auto h1 = h0->twin();
        auto v0 = h0->vertex();
        auto v1 = h1->vertex();
        auto v2 = h0->next()->next()->vertex();
        auto v3 = h1->next()->next()->vertex();
        auto deviation = dev(v0) + dev(v1) + dev(v2) + dev(v3);
        if(deviation > dev(v0, -1) + dev(v1, -1) + dev(v2, 1) + dev(v3, 1)) {
            flip_edge(e);
        }
    }
    info("Step 3 finished. Validating...");
    err = validate();
    if(err != std::nullopt) {
        info("%s", err->second.c_str());
        return false;
    }
    for(auto v = vertices.begin(); v != vertices.end(); v++) {
        auto N = v->normal();
        if(!(std::isfinite(N.x) && std::isfinite(N.y) && std::isfinite(N.z))) {
            info("N = (%f, %f, %f) of vertex %i is infinite", N.x, N.y, N.z, v->id());
            return false;
        }
    }
    info("Validation finished");

    info("Step 4: smoothing");
    info("Number of vertices: %i", n_vertices());
    for(int i = 0; i < 15; i++) {
        for(auto v = vertices.begin(); v != vertices.end(); v++) {
            //info("Processing vertex %i", v->id());
            auto c = v->neighborhood_center();
            auto p = v->pos;
            auto d = c - p;
            if(!(std::isfinite(d.x) && std::isfinite(d.y) && std::isfinite(d.z))) {
                info("d = (%f, %f, %f) is infinite (before substracting)", d.x, d.y, d.z);
            }
            auto N = v->normal();
            d = d - dot(d, N) * N;
            if (!(std::isfinite(c.x) && std::isfinite(c.y) && std::isfinite(c.z))) {
                info("c = (%f, %f, %f) is infinite", c.x, c.y, c.z);
            }
            if(!(std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z))) {
                info("p = (%f, %f, %f) is infinite", p.x, p.y, p.z);
            }
            if(!(std::isfinite(N.x) && std::isfinite(N.y) && std::isfinite(N.z))) {
                info("N = (%f, %f, %f) is infinite", N.x, N.y, N.z);
            }
            if(!(std::isfinite(d.x) && std::isfinite(d.y) && std::isfinite(d.z))) {
                info("d = (%f, %f, %f) is infinite", d.x, d.y, d.z);
                info("d dot N = %f", dot(d, N));
            }
            v->new_pos = p + d * (1.f / 5.f);
        }
        for(auto v = vertices.begin(); v != vertices.end(); v++) {
            v->pos = v->new_pos;
        }
    }
    info("Step 4 finished");

    return true;
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
