
#include "../lib/mathlib.h"
#include "debug.h"

bool BBox::hit(const Ray& ray, Vec2& times) const {

    // TODO (PathTracer):
    // Implement ray - bounding box intersection test
    // If the ray intersected the bounding box within the range given by
    // [times.x,times.y], update times with the new intersection times.

    Vec3 origin = ray.point;
    Vec3 dir = ray.dir;
    Vec3 inv = 1 / dir;

    float tmin, tmax, tymin, tymax, tzmin, tzmax;
    bool xsign = (inv.x < 0);
    bool ysign = (inv.y < 0);
    bool zsign = (inv.z < 0);

    float xmin, xmax, ymin, ymax, zmin, zmax;

    if (xsign == 0) {
        xmin = min.x;
        xmax = max.x;
    } else {
        xmin = max.x;
        xmax = min.x;
    }

    if (ysign == 0) {
        ymin = min.y;
        ymax = max.y;
    } else {
        ymin = max.y;
        ymax = min.y;
    }

    if (zsign == 0) {
        zmin = min.z;
        zmax = max.z;
    } else {
        zmin = max.z;
        zmax = min.z;
    }

    tmin = (xmin - origin.x) * inv.x;
    tmax = (xmax - origin.x) * inv.x;
    tymin = (ymin - origin.y) * inv.y;
    tymax = (ymax - origin.y) * inv.y;
    tzmin = (zmin - origin.z) * inv.z;
    tzmax = (zmax - origin.z) * inv.z;

    if ((tmin > tymax) || (tymin > tmax)) {
        return false;
    } if (tymin > tmin) {
        tmin = tymin;
    } if (tymax < tmax) {
        tmax = tymax;
    }

    if ((tmin > tzmax) || (tzmin > tmax)) {
        return false;
    } if (tzmin > tmin) {
        tmin = tzmin;
    } if (tzmax < tmax) {
        tmax = tzmax;
    }

    times.x = tmin;
    times.y = tmax;

    return true;
}
