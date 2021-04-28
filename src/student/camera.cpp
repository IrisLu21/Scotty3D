
#include "../util/camera.h"
#include "../rays/samplers.h"
#include "debug.h"

Ray Camera::generate_ray(Vec2 screen_coord) const {

    // TODO (PathTracer): Task 1
    // compute position of the input sensor sample coordinate on the
    // canonical sensor plane one unit away from the pinhole.
    // Tip: compute the ray direction in view space and use
    // the camera transform to transform it back into world space.
    float angle = vert_fov / 2;
    float radian = angle * PI_F / 180.0f;
    float sh = std::tan(radian) * 2;
    float sw = sh * aspect_ratio;
    float new_x = (screen_coord.x - 0.5f) * sw;
    float new_y = (screen_coord.y - 0.5f) * sh;

    float pdf;
    Samplers::Rect::Uniform u(Vec2(aperture, aperture));
    Vec2 start = u.sample(pdf);
    Vec3 dir = Vec3(new_x, new_y, -1.0) * focal_dist;
    Ray ray(Vec3(start.x, start.y, 0.0f), dir.normalize());
    ray.transform(iview);

    return ray;
}
