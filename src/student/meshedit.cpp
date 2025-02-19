
#include <queue>
#include <set>
#include <unordered_map>

#include "../geometry/halfedge.h"
#include "debug.h"
#include <iostream>

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

/*
    This method should replace the given vertex and all its neighboring
    edges and faces with a single face, returning the new face.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::erase_vertex(Halfedge_Mesh::VertexRef v) {

    (void)v;
    return std::nullopt;
}

/*
    This method should erase the given edge and return an iterator to the
    merged face.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::erase_edge(Halfedge_Mesh::EdgeRef e) {

    (void)e;
    return std::nullopt;
}

/*
    This method should collapse the given edge and return an iterator to
    the new vertex created by the collapse.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_edge(Halfedge_Mesh::EdgeRef e) { // consider pyramid examples??
    (void)e;

    HalfedgeRef h0 = e->halfedge();
    VertexRef v0 = h0->vertex();
    FaceRef f0 = h0->face();
    EdgeRef e0 = h0->edge();
    HalfedgeRef twin = h0->twin();
    VertexRef v1 = twin->vertex();
    FaceRef f1 = twin->face();

    // check boundary
    if (f0->is_boundary() || f1->is_boundary()) {
        return std::nullopt;
    }

    v0->pos = (v0->pos + v1->pos) / 2;

    // get neighboring halfedges and edges (get twins -> want to keep outside halfedges and delete inner halfedges)
    HalfedgeRef next0 = h0->next()->twin();
    EdgeRef e1 = next0->edge();
    HalfedgeRef curr0 = h0->next();
    HalfedgeRef prev0 = h0;
    do {
        prev0 = curr0;
        curr0 = curr0->next();
    } while (curr0 != h0);
    prev0 = prev0->twin();
    EdgeRef e2 = prev0->edge();

    HalfedgeRef next1 = twin->next()->twin();
    EdgeRef e3 = next1->edge();
    HalfedgeRef curr1 = twin->next();
    HalfedgeRef prev1 = twin;
    do {
        prev1 = curr1;
        curr1 = curr1->next();
    } while (curr1 != twin);
    prev1 = prev1->twin();
    EdgeRef e4 = prev1->edge();

    // get edge degree of each face
    unsigned int fDegree0 = f0->degree();
    unsigned int fDegree1 = f1->degree();

    // loop through all edges from v1 and move them to v0
    HalfedgeRef start = v1->halfedge();
    do {
        start->vertex() = v0;
        start = start->twin()->next();
    } while (start != v1->halfedge());

    // erase collapsed edge and one of the vertices
    erase(e0);
    erase(v1);
    erase(h0);
    erase(twin);

    if (fDegree0 <= 3) { // delete f0 and collapse neighboring edges by converging to v0
        erase(next0->twin());
        erase(prev0->twin());
        erase(f0);
        erase(e2);

        next0->vertex() = next0->vertex();
        next0->face() = next0->face();
        next0->next() = next0->next();
        next0->twin() = prev0;
        next0->edge() = e1;
        e1->halfedge() = next0;
        next0->vertex()->halfedge() = next0;

        prev0->vertex() = v0;
        prev0->face() = prev0->face();
        prev0->next() = prev0->next();
        prev0->twin() = next0;
        prev0->edge() = e1;

    } else { // do not delete f0, but converge neighboring edges to v0
        next0->vertex() = next0->vertex();
        next0->face() = next0->face();
        next0->next() = next0->next();
        next0->twin() = next0->twin();
        next0->edge() = e1;
        e1->halfedge() = next0;
        next0->vertex()->halfedge() = next0;

        next0->twin()->vertex() = v0;
        next0->twin()->face() = f0;
        next0->twin()->next() = next0->twin()->next();
        next0->twin()->twin() = next0;
        next0->twin()->edge() = e1;

        prev0->vertex() = v0;
        prev0->face() = prev0->face();
        prev0->next() = prev0->next();
        prev0->twin() = prev0->twin();
        prev0->edge() = e2;
        e2->halfedge() = prev0;

        prev0->twin()->vertex() = prev0->twin()->vertex();
        prev0->twin()->face() = f0;
        prev0->twin()->next() = next0->twin();
        prev0->twin()->twin() = prev0;
        prev0->twin()->edge() = e2;
        prev0->twin()->vertex()->halfedge() = prev0->twin();

        f0->halfedge() = next0->twin();
    }

    if (fDegree1 <= 3) { // delete f1 and collapse neighboring edges by converging to v0
        erase(next1->twin());
        erase(prev1->twin());
        erase(f1);
        erase(e4);

        next1->vertex() = next1->vertex();
        next1->face() = next1->face();
        next1->next() = next1->next();
        next1->twin() = prev1;
        next1->edge() = e3;
        e3->halfedge() = next1;
        next1->vertex()->halfedge() = next1;

        prev1->vertex() = v0;
        prev1->face() = prev1->face();
        prev1->next() = prev1->next();
        prev1->twin() = next1;
        prev1->edge() = e3;

    } else { // do not delete f1, but converge neighboring edges to v0
        next1->vertex() = next1->vertex();
        next1->face() = next1->face();
        next1->next() = next1->next();
        next1->twin() = next1->twin();
        next1->edge() = e3;
        e3->halfedge() = next1;
        next1->vertex()->halfedge() = next1;

        next1->twin()->vertex() = v0;
        next1->twin()->face() = f1;
        next1->twin()->next() = next1->twin()->next();
        next1->twin()->twin() = next1;
        next1->twin()->edge() = e3;

        prev1->vertex() = v0;
        prev1->face() = prev1->face();
        prev1->next() = prev1->next();
        prev1->twin() = prev1->twin();
        prev1->edge() = e4;
        e4->halfedge() = prev1;

        prev1->twin()->vertex() = prev1->twin()->vertex();
        prev1->twin()->face() = f1;
        prev1->twin()->next() = next1->twin();
        prev1->twin()->twin() = prev1;
        prev1->twin()->edge() = e4;
        prev1->twin()->vertex()->halfedge() = prev1->twin();

        f1->halfedge() = next1->twin();
    }

    v0->halfedge() = prev0; 

    return v0;
}

/*
    This method should collapse the given face and return an iterator to
    the new vertex created by the collapse.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_face(Halfedge_Mesh::FaceRef f) {

    (void)f;
    return std::nullopt;
}

/*
    This method should flip the given edge and return an iterator to the
    flipped edge.
*/
std::optional<Halfedge_Mesh::EdgeRef> Halfedge_Mesh::flip_edge(Halfedge_Mesh::EdgeRef e) {

    (void)e;
    HalfedgeRef h0 = e->halfedge();
    HalfedgeRef next0 = h0->next();
    HalfedgeRef twin0 = h0->twin();
    VertexRef v0 = h0->vertex();
    FaceRef f0 = h0->face();

    HalfedgeRef next1 = twin0->next();
    VertexRef v1 = twin0->vertex();
    FaceRef f1 = twin0->face();

    // check boundary
    if (f0->is_boundary() || f1->is_boundary()) {
        return e;
    }

    VertexRef v2 = next0->twin()->vertex();
    VertexRef v3 = next1->twin()->vertex();

    // update previous halfedge pointers
    HalfedgeRef curr0 = next0;
    HalfedgeRef prev0 = h0;
    do {
        prev0 = curr0;
        curr0 = curr0->next();
    } while (curr0 != h0);
    prev0->next() = next1;

    HalfedgeRef curr1 = next1;
    HalfedgeRef prev1 = twin0;
    do {
        prev1 = curr1;
        curr1 = curr1->next();
    } while (curr1 != twin0);
    prev1->next() = next0;

    // update current edge pointers
    h0->next() = next0->next();
    h0->vertex() = v3;
    next0->next() = twin0;
    next0->face() = f1;

    twin0->next() = next1->next();
    twin0->vertex() = v2;
    next1->next() = h0;
    next1->face() = f0;

    v0->halfedge() = next1;
    v1->halfedge() = next0;
    v2->halfedge() = v2->halfedge();
    v3->halfedge() = v3->halfedge();

    // go around f0 (inside) - assign face, edge, vertex
    f0->halfedge() = h0;
    HalfedgeRef start0 = h0;
    do {
        start0->face() = f0;
        // EdgeRef edge = start0->edge();
        // edge->halfedge() = start0;
        // VertexRef v = start0->vertex();
        // v->halfedge() = start0;
        start0 = start0->next();
    } while (start0 != h0);

    // go around f1 (inside) - assign face, edge, vertex
    f1->halfedge() = twin0;
    HalfedgeRef start1 = twin0;
    do {
        start1->face() = f1;
        // EdgeRef edge = start1->edge();
        // edge->halfedge() = start1;
        // VertexRef v = start1->vertex();
        // v->halfedge() = start1;
        start1 = start1->next();
    } while (start1 != twin0);

    return h0->edge();
}

/*
    This method should split the given edge and return an iterator to the
    newly inserted vertex. The halfedge of this vertex should point along
    the edge that was split, rather than the new edges.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::split_edge(Halfedge_Mesh::EdgeRef e) {
    (void)e;

    // extract current elements
    HalfedgeRef h0 = e->halfedge();
    HalfedgeRef twin = h0->twin();
    HalfedgeRef next0 = h0->next();
    VertexRef v0 = h0->vertex();
    FaceRef f0 = h0->face();

    HalfedgeRef next1 = twin->next();
    VertexRef v1 = twin->vertex();
    FaceRef f1 = twin->face();
    Vec3 newPos = e->new_pos;

    // check boundary
    if (f0->is_boundary() && f1->is_boundary()) {
        return std::nullopt;
    }
    if (f1->is_boundary()) {
        VertexRef v2 = next0->twin()->vertex();
        HalfedgeRef prev0 = next0->next();

        HalfedgeRef hNew1 = new_halfedge();
        HalfedgeRef hNew2 = new_halfedge();
        HalfedgeRef hNew3 = new_halfedge();
        HalfedgeRef hNew4 = new_halfedge();
        EdgeRef eNew1 = new_edge();
        EdgeRef eNew2 = new_edge();
        FaceRef fNew = new_face();
        VertexRef vNew = new_vertex();

        vNew->pos = (v0->pos + v1->pos) / 2;

        hNew1->vertex() = v0;
        hNew1->edge() = eNew1;
        hNew1->twin() = hNew2;
        hNew1->face() = fNew;
        hNew1->next() = hNew4;

        hNew2->vertex() = vNew;
        hNew2->edge() = eNew1;
        hNew2->twin() = hNew1;
        hNew2->face() = twin->face();
        hNew2->next() = twin->next();

        hNew3->vertex() = v2;
        hNew3->edge() = eNew2;
        hNew3->twin() = hNew4;
        hNew3->face() = f0;
        hNew3->next() = h0;

        hNew4->vertex() = vNew;
        hNew4->edge() = eNew2;
        hNew4->twin() = hNew3;
        hNew4->face() = fNew;
        hNew4->next() = prev0;

        h0->vertex() = vNew;
        h0->edge() = e;
        h0->twin() = twin;
        h0->face() = f0;
        h0->next() = next0;

        twin->vertex() = v1;
        twin->edge() = e;
        twin->twin() = h0;
        twin->face() = twin->face();
        twin->next() = hNew2;

        next0->vertex() = v1;
        next0->edge() = next0->edge();
        next0->twin() = next0->twin();
        next0->face() = f0;
        next0->next() = hNew3;

        prev0->vertex() = v2;
        prev0->edge() = prev0->edge();
        prev0->twin() = prev0->twin();
        prev0->face() = fNew;
        prev0->next() = hNew1;

        eNew1->halfedge() = hNew1;
        eNew2->halfedge() = hNew3;
        fNew->halfedge() = hNew1;
        vNew->halfedge() = h0;
        v0->halfedge() = hNew1;
        v1->halfedge() = twin;
        v2->halfedge() = prev0;
        f0->halfedge() = h0;
        e->halfedge() = h0;

        vNew->is_new = true;
        eNew1->is_new = false;
        eNew2->is_new = true;

        e->new_pos = newPos;

        return vNew;

    } else if (f0->is_boundary()) {
        VertexRef v2 = next1->twin()->vertex();
        HalfedgeRef prev1 = next1->next();

        HalfedgeRef hNew1 = new_halfedge();
        HalfedgeRef hNew2 = new_halfedge();
        HalfedgeRef hNew3 = new_halfedge();
        HalfedgeRef hNew4 = new_halfedge();
        EdgeRef eNew1 = new_edge();
        EdgeRef eNew2 = new_edge();
        FaceRef fNew = new_face();
        VertexRef vNew = new_vertex();

        vNew->pos = (v0->pos + v1->pos) / 2;

        hNew1->vertex() = vNew;
        hNew1->edge() = eNew1;
        hNew1->twin() = hNew2;
        hNew1->face() = h0->face();
        hNew1->next() = next0;

        hNew2->vertex() = vNew;
        hNew2->edge() = eNew1;
        hNew2->twin() = hNew1;
        hNew2->face() = fNew;
        hNew2->next() = hNew3;

        hNew3->vertex() = vNew;
        hNew3->edge() = eNew2;
        hNew3->twin() = hNew4;
        hNew3->face() = fNew;
        hNew3->next() = prev1;

        hNew4->vertex() = v2;
        hNew4->edge() = eNew2;
        hNew4->twin() = hNew3;
        hNew4->face() = f1;
        hNew4->next() = twin;

        h0->vertex() = v0;
        h0->edge() = e;
        h0->twin() = twin;
        h0->face() = f0;
        h0->next() = hNew1;

        twin->vertex() = vNew;
        twin->edge() = e;
        twin->twin() = h0;
        twin->face() = f1;
        twin->next() = next1;

        next1->vertex() = v0;
        next1->edge() = next1->edge();
        next1->twin() = next1->twin();
        next1->face() = f1;
        next1->next() = hNew4;

        prev1->vertex() = v2;
        prev1->edge() = prev1->edge();
        prev1->twin() = prev1->twin();
        prev1->face() = fNew;
        prev1->next() = hNew2;

        eNew1->halfedge() = hNew2;
        eNew2->halfedge() = hNew3;
        fNew->halfedge() = hNew2;
        vNew->halfedge() = twin;
        v0->halfedge() = h0;
        v1->halfedge() = hNew2;
        v2->halfedge() = prev1;

        vNew->is_new = true;
        eNew1->is_new = false;
        eNew2->is_new = true;

        e->new_pos = newPos;
        
        return vNew;
    }

    // check valid split
    unsigned int deg0 = f0->degree();
    unsigned int deg1 = f1->degree();
    if (deg0 != 3 || deg1 != 3) {
        return std::nullopt;
    }

    // get neighboring vertices
    VertexRef v2 = next0->twin()->vertex();
    VertexRef v3 = next1->twin()->vertex();

    // get neighboring halfedges (will be adjacent to new halfedges)
    HalfedgeRef h2 = next0->next(); // tail at v2
    HalfedgeRef h3 = next1->next(); // tail at v3

    // initialize new elements
    VertexRef vNew = new_vertex();
    EdgeRef eNew1 = new_edge();
    EdgeRef eNew2 = new_edge();
    EdgeRef eNew3 = new_edge();
    HalfedgeRef hNew1 = new_halfedge();
    HalfedgeRef hNew2 = new_halfedge();
    HalfedgeRef hNew3 = new_halfedge();
    HalfedgeRef hNew4 = new_halfedge();
    HalfedgeRef hNew5 = new_halfedge();
    HalfedgeRef hNew6 = new_halfedge();
    FaceRef f2 = new_face();
    FaceRef f3 = new_face();

    // set new vertex position
    vNew->pos = (v0->pos + v1->pos) / 2;

    // connect new elements
    hNew1->vertex() = vNew;
    hNew2->vertex() = v1;
    hNew3->vertex() = vNew;
    hNew4->vertex() = v3;
    hNew5->vertex() = vNew;
    hNew6->vertex() = v2;

    hNew1->twin() = hNew2;
    hNew2->twin() = hNew1;
    hNew3->twin() = hNew4;
    hNew4->twin() = hNew3;
    hNew5->twin() = hNew6;
    hNew6->twin() = hNew5;

    hNew1->edge() = eNew1;
    hNew2->edge() = eNew1;
    hNew3->edge() = eNew2;
    hNew4->edge() = eNew2;
    hNew5->edge() = eNew3;
    hNew6->edge() = eNew3;

    hNew1->face() = f3;
    hNew2->face() = f2;
    hNew3->face() = f2;
    hNew4->face() = f1;
    hNew5->face() = f0;
    hNew6->face() = f3;

    // update and reassign pointers
    twin->vertex() = vNew;

    h0->next() = hNew5;
    hNew5->next() = h2;
    next1->next() = hNew4;
    hNew4->next() = twin;
    hNew3->next() = h3;
    h3->next() = hNew2;
    hNew2->next() = hNew3;
    next0->next() = hNew6;
    hNew6->next() = hNew1;
    hNew1->next() = next0;

    // pointers that did not change
    h2->next() = h0;
    twin->next() = next1;
    twin->edge() = e;

    // go around f0 (inside) - assign face and edge
    f0->halfedge() = h0;
    HalfedgeRef start0 = h0;
    do {
        start0->face() = f0;
        EdgeRef edge = start0->edge();
        edge->halfedge() = start0;
        start0 = start0->next();
    } while (start0 != h0);

    // go around f1 (inside) - assign face and edge
    f1->halfedge() = twin;
    HalfedgeRef start1 = twin;
    do {
        start1->face() = f1;
        EdgeRef edge = start1->edge();
        edge->halfedge() = start1;
        start1 = start1->next();
    } while (start1 != twin);

    // go around f2 (inside) - assign face and edge
    f2->halfedge() = hNew2;
    HalfedgeRef start2 = hNew2;
    do {
        start2->face() = f2;
        EdgeRef edge = start2->edge();
        edge->halfedge() = start2;
        start2 = start2->next();
    } while (start2 != hNew2);

    // go around f3 (inside) - assign face and edge
    f3->halfedge() = hNew1;
    HalfedgeRef start3 = hNew1;
    do {
        start3->face() = f3;
        EdgeRef edge = start3->edge();
        edge->halfedge() = start3;
        start3 = start3->next();
    } while (start3 != hNew1);

    // assign halfedge to each vertex
    v0->halfedge() = h0;
    v1->halfedge() = next0;
    v2->halfedge() = h2;
    v3->halfedge() = h3;
    vNew->halfedge() = twin;
    vNew->is_new = true;
    eNew1->is_new = false;
    eNew2->is_new = true;
    eNew3->is_new = true;

    e->new_pos = newPos;

    return vNew;
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
    positions: for  bevel_vertex, it has one element, the original vertex position,
    for bevel_edge,  two for the two vertices, and for bevel_face, it has the original
    position of each vertex in halfedge order. You should use these positions, as well
    as the normal and tangent offset fields to assign positions to the new vertices.

    Finally, note that the normal and tangent offsets are not relative values - you
    should compute a particular new position from them, not a delta to apply.
*/

/*
    This method should replace the vertex v with a face, corresponding to
    a bevel operation. It should return the new face.  NOTE: This method is
    responsible for updating the *connectivity* of the mesh only---it does not
    need to update the vertex positions.  These positions will be updated in
    Halfedge_Mesh::bevel_vertex_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_vertex(Halfedge_Mesh::VertexRef v) {

    // Reminder: You should set the positions of new vertices (v->pos) to be exactly
    // the same as wherever they "started from."

    (void)v;
    return std::nullopt;
}

/*
    This method should replace the edge e with a face, corresponding to a
    bevel operation. It should return the new face. NOTE: This method is
    responsible for updating the *connectivity* of the mesh only---it does not
    need to update the vertex positions.  These positions will be updated in
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

    unsigned int deg = f->degree();
    // FaceRef insetF = new_face();

    VertexRef prevV;
    HalfedgeRef h1prev;
    HalfedgeRef h2prev;
    HalfedgeRef h3prev;
    HalfedgeRef h4prev;
    EdgeRef e1prev;
    EdgeRef e2prev;
    FaceRef prevF;
    HalfedgeRef prevh0;

    HalfedgeRef h0 = f->halfedge();
    VertexRef v0 = h0->vertex();
    VertexRef v = v0;

    for (size_t i = 0; i < deg; i++) {

        // create new elements
        VertexRef newV = new_vertex();
        HalfedgeRef hNew1 = new_halfedge();
        HalfedgeRef hNew2 = new_halfedge();
        HalfedgeRef hNew3 = new_halfedge();
        HalfedgeRef hNew4 = new_halfedge();
        EdgeRef eNew1 = new_edge();
        EdgeRef eNew2 = new_edge();
        FaceRef newF = new_face();

        // assign new vertex position to be original position
        newV->pos = v->pos;

        // adjust h0 face
        h0->face() = newF;

        // assign hNew1 values
        hNew1->vertex() = newV;
        hNew1->edge() = eNew1;
        hNew1->face() = newF;
        hNew1->next() = h0;
        hNew1->twin() = hNew2;

        // assign hNew2 values
        hNew2->vertex() = v;
        hNew2->edge() = eNew1;
        hNew2->next() = hNew3;
        hNew2->twin() = hNew1;

        // assign hNew3 values
        hNew3->vertex() = newV;
        hNew3->edge() = eNew2;

        // assign hNew4 values
        hNew4->vertex() = newV;
        hNew4->face() = f;

        // assign halfedge to edge, face, vertex
        eNew1->halfedge() = hNew1;
        eNew2->halfedge() = hNew3;
        newF->halfedge() = hNew1;
        newV->halfedge() = hNew4;
        f->halfedge() = hNew4;
        v->halfedge() = hNew2;
        
        if (i != 0) { // if not first iteration , connect halfedges with previous elements
            
            hNew2->face() = prevF;
            
            hNew3->face() = prevF;
            hNew3->twin() = h4prev;
            hNew3->next() = h1prev;

            h4prev->next() = hNew4;
            h4prev->edge() = eNew2;
            h4prev->twin() = hNew3;

            prevh0->next() = hNew2;
        }

        // reset previous positions for next iteration
        prevV = newV;
        h1prev = hNew1;
        h2prev = hNew2;
        h3prev = hNew3;
        h4prev = hNew4;
        e1prev = eNew1;
        e2prev = eNew2;
        prevF = newF;
        prevh0 = h0;
        
        h0 = h0->next();
        v = h0->vertex();
    }
    // connect first halfedges with final (previous) elements
    HalfedgeRef h2 = v0->halfedge();
    HalfedgeRef h3 = h2->next();
    VertexRef v1 = h3->vertex();
    HalfedgeRef h4 = v1->halfedge();
    EdgeRef edge = h3->edge();

    h2->face() = prevF;
    h3->face() = prevF;
    h3->twin() = h4prev;
    h3->next() = h1prev;

    h4prev->next() = h4;
    h4prev->edge() = edge;
    h4prev->twin() = h3;
    prevh0->next() = h2;
    
    return f;
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
    Compute new vertex positions for the vertices of the beveled edge.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the preceding and next vertex position from the original mesh
    (in the orig array) to compute an offset vertex position.

    Note that there is a 1-to-1 correspondence between halfedges in
    newHalfedges and vertex positions
    in orig.  So, you can write loops of the form

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
    new_halfedges and vertex positions
    in orig. So, you can write loops of the form

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

    size_t n = new_halfedges.size();

    for (size_t i = 0; i < n; i++) {

        size_t prev = (i + n - 1) % n;
        size_t next = (i + 1) % n;

        Vec3 prevPos = start_positions[prev];
        Vec3 nextPos = start_positions[next];
        Vec3 position = start_positions[i];

        VertexRef v = new_halfedges[i]->vertex();
        
        Vec3 vect1 = prevPos - position;
        Vec3 vect2 = nextPos - position;
        Vec3 tangent = vect1.norm()*vect2 + vect2.norm()*vect1;

        Vec3 newPos = position + (tangent * tangent_offset) - (face->normal() * normal_offset);
        v->pos = newPos;
    }

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
    for (FaceRef f = faces_begin(); f != faces_end(); f++) {
        size_t deg = f->degree();
        HalfedgeRef h0 = f->halfedge();
        VertexRef v0 = h0->vertex();
        VertexRef vStart = v0;
        HalfedgeRef hStart = h0;

        for (size_t count = 0; count != deg-3; count++) {
            
            HalfedgeRef next = hStart->next();
            HalfedgeRef curr = next->next();
            HalfedgeRef prev = next;
            HalfedgeRef prev2 = hStart;
            do {
                prev2 = prev;
                prev = curr;
                curr = curr->next();
            } while (curr != hStart);
            
            VertexRef v1 = hStart->twin()->vertex();
            VertexRef v2 = prev->vertex();
            EdgeRef newE = new_edge();
            HalfedgeRef newH1 = new_halfedge();
            HalfedgeRef newH2 = new_halfedge();
            FaceRef newF = new_face();

            newH1->next() = prev;
            newH1->vertex() = v1;
            newH1->twin() = newH2;
            newH1->edge() = newE;
            newH1->face() = newF;

            newH2->next() = next;
            newH2->vertex() = v2;
            newH2->twin() = newH1;
            newH2->edge() = newE;
            newH2->face() = f;

            hStart->next() = newH1;
            hStart->vertex() = vStart;
            hStart->twin() = hStart->twin();
            hStart->edge() = hStart->edge();
            hStart->face() = newF;

            prev->next() = hStart;
            prev->vertex() = v2;
            prev->twin() = prev->twin();
            prev->edge() = prev->edge();
            prev->face() = newF;

            prev2->next() = newH2;
            prev2->vertex() = prev2->vertex();
            prev2->twin() = prev2->twin();
            prev2->edge() = prev2->edge();
            prev2->face() = prev2->face();

            next->next() = next->next();
            next->vertex() = v1;
            next->twin() = next->twin();
            next->edge() = next->edge();
            next->face() = next->face();

            newE->halfedge() = newH1;
            newF->halfedge() = hStart;
            vStart = v1;
            hStart = next;
            validate();
        } 
        f->halfedge() = hStart;
        hStart->face() = f;
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
        vertices in the new (subdivided mesh).  They do not have to be assigned
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
    for (VertexRef v = vertices_begin(); v != vertices_end(); v++) {
        v->new_pos = v->pos;
    }

    // For each edge, assign the midpoint of the two original
    // positions to Edge::new_pos.
    for (EdgeRef e = edges_begin(); e != edges_end(); e++) {
        VertexRef v1 = e->halfedge()->vertex();
        VertexRef v2 = e->halfedge()->twin()->vertex();
        e->new_pos = (v1->pos + v2->pos) / 2;
    }

    // For each face, assign the centroid (i.e., arithmetic mean)
    // of the original vertex positions to Face::new_pos. Note
    // that in general, NOT all faces will be triangles!
    for (FaceRef f = faces_begin(); f != faces_end(); f++) {
        Vec3 sumPos = Vec3(0,0,0);
        size_t count = 0;
        HalfedgeRef start = f->halfedge();
        do {
            VertexRef v = start->vertex();
            sumPos += v->pos;
            start = start->next();
            count++;
        } while (start != f->halfedge());
        f->new_pos = sumPos / (float) count;
    }
}

/*
    Compute new vertex positions for a mesh that splits each polygon
    into quads (by inserting a vertex at the face midpoint and each
    of the edge midpoints).  The new vertex positions will be stored
    in the members Vertex::new_pos, Edge::new_pos, and
    Face::new_pos.  The values of the positions are based on
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
    for (FaceRef f = faces_begin(); f != faces_end(); f++) {
        Vec3 sumPos = Vec3(0,0,0);
        size_t count = 0;
        HalfedgeRef start = f->halfedge();
        do {
            VertexRef v = start->vertex();
            sumPos += v->pos;
            start = start->next();
            count++;
        } while (start != f->halfedge());
        f->new_pos = sumPos / (float) count;
    }

    // Edges
    for (EdgeRef e = edges_begin(); e != edges_end(); e++) {
        HalfedgeRef h = e->halfedge();
        VertexRef v1 = h->vertex();
        VertexRef v2 = h->twin()->vertex();
        FaceRef f1 = h->face();
        FaceRef f2 = h->face();

        e->new_pos = (v1->pos + v2->pos + f1->new_pos + f2->new_pos) / 4;
    }

    // Vertices
    for (VertexRef v = vertices_begin(); v != vertices_end(); v++) {
        float n = (float) v->degree();
        Vec3 sumQ = Vec3(0,0,0);
        HalfedgeRef start = v->halfedge();
        do {
            sumQ += start->vertex()->pos;
            start = start->twin()->next();
        } while (start != v->halfedge());
        Vec3 Q = sumQ / n;

        Vec3 sumR = Vec3(0,0,0);
        start = v->halfedge();
        do {
            VertexRef v1 = start->vertex();
            VertexRef v2 = start->twin()->vertex();
            sumR += (v1->pos + v2->pos) / 2;
            start = start->twin()->next();
        } while (start != v->halfedge());
        Vec3 R = sumR / n;

        Vec3 newPos = (Q + 2*R + (n-3)*v->pos) / n;
        v->new_pos = newPos;
    }
}

/*
        This routine should increase the number of triangles in the mesh
        using Loop subdivision. Note: this is will only be called on triangle meshes.
*/
void Halfedge_Mesh::loop_subdivide() {

    // mark all curr vertices as NOT is_new and compute updated positions
    for (VertexRef v = vertices_begin(); v != vertices_end(); v++) {
        v->is_new = false;
        int n = v->degree();
        float u;
        if (n == 3) {
            u = (float) 3.0f / 16.0f;
        } else {
            u = (float) 3.0f / (8.0f * (float) n);
        }
        Vec3 neighborPos = Vec3(0, 0, 0);
        HalfedgeRef h = v->halfedge();
        do {
            HalfedgeRef twin = h->twin();
            neighborPos += twin->vertex()->pos;
            h = twin->next();
        } while (h != v->halfedge());
        Vec3 newPos = (Vec3) ((1 - n*u) * v->pos) + (u * neighborPos);
        v->new_pos = newPos;
    }

    // get midpoints of all curr edges (update new_pos) and set NOT is_new
    size_t n = n_edges();
    for (EdgeRef e = edges_begin(); e != edges_end(); e++) {
        e->is_new = false;

        HalfedgeRef h = e->halfedge();
        HalfedgeRef twin = h->twin();
        VertexRef A = h->vertex();
        VertexRef B = twin->vertex();
        VertexRef C = h->next()->twin()->vertex();
        VertexRef D = twin->next()->twin()->vertex();

        Vec3 newPos = ((3.0f / 8.0f) * (A->pos + B->pos)) + ((1.0f / 8.0f) * (C->pos + D->pos));

        e->new_pos = newPos;
    }

    // split current edges
    EdgeRef currE = edges_begin();
    for (size_t i = 0; i < n; i++) {
        EdgeRef nextE = currE;
        nextE++;

        if (!currE->is_new) {
            std::optional<Halfedge_Mesh::VertexRef> x = split_edge(currE);
            if (x.has_value()) {
                VertexRef v = x.value();
                v->new_pos = currE->new_pos;
            }
        }
        currE = nextE;
    }

    // flip every edge that connects an old and new vertex
    for (EdgeRef e = edges_begin(); e != edges_end(); e++) {
        HalfedgeRef h0 = e->halfedge();
        HalfedgeRef h1 = h0->twin();
        VertexRef v0 = h0->vertex();
        VertexRef v1 = h1->vertex();

        if (e->is_new && ((v0->is_new && !v1->is_new) || (!v0->is_new && v1->is_new))) {
            flip_edge(e);

        }
    }

    // update each vertex position
    for (VertexRef v = vertices_begin(); v != vertices_end(); v++) {
        v->pos = v->new_pos;
    }
    
    // Compute new positions for all the vertices in the input mesh, using
    // the Loop subdivision rule, and store them in Vertex::new_pos.
    // -> At this point, we also want to mark each vertex as being a vertex of the
    //    original mesh. Use Vertex::is_new for this.
    // -> Next, compute the updated vertex positions associated with edges, and
    //    store it in Edge::new_pos.
    // -> Next, we're going to split every edge in the mesh, in any order.  For
    //    future reference, we're also going to store some information about which
    //    subdivided edges come from splitting an edge in the original mesh, and
    //    which edges are new, by setting the flat Edge::is_new. Note that in this
    //    loop, we only want to iterate over edges of the original mesh.
    //    Otherwise, we'll end up splitting edges that we just split (and the
    //    loop will never end!)
    // -> Now flip any new edge that connects an old and new vertex.
    // -> Finally, copy the new vertex positions into final Vertex::pos.

    // Each vertex and edge of the original surface can be associated with a
    // vertex in the new (subdivided) surface.
    // Therefore, our strategy for computing the subdivided vertex locations is to
    // *first* compute the new positions
    // using the connectivity of the original (coarse) mesh; navigating this mesh
    // will be much easier than navigating
    // the new subdivided (fine) mesh, which has more elements to traverse.  We
    // will then assign vertex positions in
    // the new mesh based on the values we computed for the original mesh.

    // Compute updated positions for all the vertices in the original mesh, using
    // the Loop subdivision rule.

    // Next, compute the updated vertex positions associated with edges.

    // Next, we're going to split every edge in the mesh, in any order. For
    // future reference, we're also going to store some information about which
    // subdivided edges come from splitting an edge in the original mesh, and
    // which edges are new.
    // In this loop, we only want to iterate over edges of the original
    // mesh---otherwise, we'll end up splitting edges that we just split (and
    // the loop will never end!)

    // Finally, flip any new edge that connects an old and new vertex.

    // Copy the updated vertex positions to the subdivided mesh.
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
    // until do_erase or validate are called. This is to facilitate checking
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
