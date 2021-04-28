
#include "../rays/shapes.h"
#include "debug.h"
#include "math.h"

namespace PT {

const char* Shape_Type_Names[(int)Shape_Type::count] = {"None", "Sphere"};

BBox Sphere::bbox() const {

    BBox box;
    box.enclose(Vec3(-radius));
    box.enclose(Vec3(radius));
    return box;
}

Trace Sphere::hit(const Ray& ray) const {

    // TODO (PathTracer): Task 2
    // Intersect this ray with a sphere of radius Sphere::radius centered at the origin.

    // If the ray intersects the sphere twice, ret should
    // represent the first intersection, but remember to respect
    // ray.dist_bounds! For example, if there are two intersections,
    // but only the _later_ one is within ray.dist_bounds, you should
    // return that one!
    
    Vec3 origin = ray.point;
    Vec3 dir = ray.dir;

    float soln1 = -dot(origin, dir) + (float) sqrt(std::pow(dot(origin, dir), 2) - std::pow(origin.norm(), 2) + 1);
    float soln2 = -dot(origin, dir) - (float) sqrt(std::pow(dot(origin, dir), 2) - std::pow(origin.norm(), 2) + 1);

    float t1 = std::min(soln1, soln2);
    float t2 = std::max(soln1, soln2);

    Vec3 t1_pos = origin + t1*dir;
    Vec3 t2_pos = origin + t2*dir; 

    float min = ray.dist_bounds.x;
    float max = ray.dist_bounds.y;

    Trace ret;
    ret.origin = ray.point;
    ret.hit = false;       // was there an intersection?
    ret.distance = 0.0f;   // at what distance did the intersection occur?
    ret.position = Vec3{}; // where was the intersection?
    ret.normal = Vec3{};   // what was the surface normal at the intersection?

    if (t1 >= min && t1 <= max) { // t1 is a valid hit point
        ret.hit = true;
        ret.position = t1_pos;
        ret.distance = t1;
        ret.normal = t1_pos.unit();
        ray.dist_bounds.y = t1;
    } else if (t2 >= min && t2 <= max) { // t2 is a valid hit point
        ret.hit = true;
        ret.position = t2_pos;
        ret.distance = t2;
        ret.normal = t2_pos.unit();
        ray.dist_bounds.y = t2;
    }
    
    return ret;
}

} // namespace PT
