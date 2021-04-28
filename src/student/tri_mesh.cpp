
#include "../rays/tri_mesh.h"
#include "debug.h"
#include "math.h"
#include <cmath>

namespace PT {

BBox Triangle::bbox() const {

    // TODO (PathTracer): Task 2
    // compute the bounding box of the triangle

    // Beware of flat/zero-volume boxes! You may need to
    // account for that here, or later on in BBox::intersect
    Tri_Mesh_Vert vert0 = vertex_list[v0];
    Tri_Mesh_Vert vert1 = vertex_list[v1];
    Tri_Mesh_Vert vert2 = vertex_list[v2];

    BBox box;
    box.enclose(vert0.position);
    box.enclose(vert1.position);
    box.enclose(vert2.position);
    return box;
}

Trace Triangle::hit(const Ray& ray) const {

    // Vertices of triangle - has postion and surface normal
    Tri_Mesh_Vert v_0 = vertex_list[v0];
    Tri_Mesh_Vert v_1 = vertex_list[v1];
    Tri_Mesh_Vert v_2 = vertex_list[v2];
    (void)v_0;
    (void)v_1;
    (void)v_2;

    // TODO (PathTracer): Task 2
    // Intersect this ray with a triangle defined by the three above points.
    Vec3 origin = ray.point;
    Vec3 dir = ray.dir;

    Vec3 e1 = v_1.position - v_0.position;
    Vec3 e2 = v_2.position - v_0.position;
    Vec3 s = origin - v_0.position;

    Trace ret;
    ret.origin = ray.point;
    ret.hit = false;       // was there an intersection?
    ret.distance = 0.0f;   // at what distance did the intersection occur?
    ret.position = Vec3{}; // where was the intersection?
    ret.normal = Vec3{};   // what was the surface normal at the intersection?
                           // (this should be interpolated between the three vertex normals)

    float denom = dot(cross(e1, dir), e2);
    if (denom != 0.0) {
        Vec3 v = Vec3(-dot(cross(s, e2), dir), dot(cross(e1, dir), s), -dot(cross(s, e2), e1));
        Vec3 soln = (1/denom) * v; // calculate intersection point
        if (soln.x >= 0.0 && soln.y >= 0.0 && (soln.x + soln.y) <= 1.0 && soln.z >= ray.dist_bounds.x && soln.z <= ray.dist_bounds.y) { // check if hit point is inside triangle
            ret.hit = true;
            ret.distance = soln.z;
            ret.position = origin + soln.z*dir;
            ret.normal = (soln.x * v_1.normal) + (soln.y * v_2.normal) + ((1-soln.x-soln.y) * v_0.normal);
            ray.dist_bounds.y = soln.z;
        }
    }

    return ret;
}

Triangle::Triangle(Tri_Mesh_Vert* verts, unsigned int v0, unsigned int v1, unsigned int v2)
    : vertex_list(verts), v0(v0), v1(v1), v2(v2) {
}

void Tri_Mesh::build(const GL::Mesh& mesh) {

    verts.clear();
    triangles.clear();

    for(const auto& v : mesh.verts()) {
        verts.push_back({v.pos, v.norm});
    }

    const auto& idxs = mesh.indices();

    std::vector<Triangle> tris;
    for(size_t i = 0; i < idxs.size(); i += 3) {
        tris.push_back(Triangle(verts.data(), idxs[i], idxs[i + 1], idxs[i + 2]));
    }

    triangles.build(std::move(tris), 4);
}

Tri_Mesh::Tri_Mesh(const GL::Mesh& mesh) {
    build(mesh);
}

Tri_Mesh Tri_Mesh::copy() const {
    Tri_Mesh ret;
    ret.verts = verts;
    ret.triangles = triangles.copy();
    return ret;
}

BBox Tri_Mesh::bbox() const {
    return triangles.bbox();
}

Trace Tri_Mesh::hit(const Ray& ray) const {
    Trace t = triangles.hit(ray);
    return t;
}

size_t Tri_Mesh::visualize(GL::Lines& lines, GL::Lines& active, size_t level,
                           const Mat4& trans) const {
    return triangles.visualize(lines, active, level, trans);
}

} // namespace PT
