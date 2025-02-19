
#include "../rays/pathtracer.h"
#include "../rays/samplers.h"
#include "../util/rand.h"
#include "debug.h"

namespace PT {

Spectrum Pathtracer::trace_pixel(size_t x, size_t y) {

    Vec2 xy((float)x, (float)y);
    Vec2 wh((float)out_w, (float)out_h);

    float pdf;
    Samplers::Rect::Uniform u(Vec2(1.0f, 1.0f));
    Vec2 rand_ray = u.sample(pdf);
    Vec2 newRay = xy + rand_ray;
    Vec2 ray = newRay / wh;
    Ray out = camera.generate_ray(ray);
    if(RNG::coin_flip(0.0005f)) log_ray(out, 10.0f);
    return trace_ray(out);
}

Spectrum Pathtracer::trace_ray(const Ray& ray) {

    // Trace ray into scene. If nothing is hit, sample the environment
    Trace hit = scene.hit(ray);
    if(!hit.hit) {
        if(env_light.has_value()) {
            return env_light.value().sample_direction(ray.dir);
        }
        return {};
    } 

    // If we're using a two-sided material, treat back-faces the same as front-faces
    const BSDF& bsdf = materials[hit.material];
    if(!bsdf.is_sided() && dot(hit.normal, ray.dir) > 0.0f) {
        hit.normal = -hit.normal;
    }

    // Set up a coordinate frame at the hit point, where the surface normal becomes {0, 1, 0}
    // This gives us out_dir and later in_dir in object space, where computations involving the
    // normal become much easier. For example, cos(theta) = dot(N,dir) = dir.y!
    Mat4 object_to_world = Mat4::rotate_to(hit.normal);
    Mat4 world_to_object = object_to_world.T();
    Vec3 out_dir = world_to_object.rotate(ray.point - hit.position).unit();

    // Debugging: if the normal colors flag is set, return the normal color
    if(debug_data.normal_colors) return Spectrum::direction(hit.normal);

    // Now we can compute the rendering equation at this point.
    // We split it into two stages: sampling lighting (i.e. directly connecting
    // the current path to each light in the scene), then sampling the BSDF
    // to create a new path segment.

    // TODO (PathTracer): Task 5
    // The starter code sets radiance_out to (0.5,0.5,0.5) so that you can test your geometry
    // queries before you implement path tracing. You should change this to (0,0,0) and accumulate
    // the direct and indirect lighting computed below.
    Spectrum radiance_out = Spectrum(0.5f);
    {
        auto sample_light = [&](const auto& light) {
            // If the light is discrete (e.g. a point light), then we only need
            // one sample, as all samples will be equivalent
            int samples = light.is_discrete() ? 1 : (int)n_area_samples;
            for(int i = 0; i < samples; i++) {

                Light_Sample sample = light.sample(hit.position);
                Vec3 in_dir = world_to_object.rotate(sample.direction);

                // If the light is below the horizon, ignore it
                float cos_theta = in_dir.y;
                if(cos_theta <= 0.0f) continue;

                // If the BSDF has 0 throughput in this direction, ignore it.
                // This is another oppritunity to do Russian roulette on low-throughput rays,
                // which would allow us to skip the shadow ray cast, increasing efficiency.
                Spectrum attenuation = bsdf.evaluate(out_dir, in_dir);
                if(attenuation.luma() == 0.0f) continue;

                // TODO (PathTracer): Task 4
                // Construct a shadow ray and compute whether the intersected surface is
                // in shadow. Only accumulate light if not in shadow.

                // Tip: since you're creating the shadow ray at the intersection point, it may
                // intersect the surface at time=0. Similarly, if the ray is allowed to have
                // arbitrary length, it will hit the light it was cast at. Therefore, you should
                // modify the time_bounds of your shadow ray to account for this. Using EPS_F is
                // recommended.

                // Note: that along with the typical cos_theta, pdf factors, we divide by samples.
                // This is because we're  doing another monte-carlo estimate of the lighting from
                // area lights.
                Ray shadow_ray(hit.position, sample.direction);
                shadow_ray.dist_bounds.x = EPS_F;
                shadow_ray.dist_bounds.y = sample.distance + EPS_F;

                Trace shadow_hit = scene.hit(shadow_ray);
                // printf("hit: %d\n", shadow_hit.hit);
                if (!shadow_hit.hit) {
                    radiance_out += (cos_theta / (samples * sample.pdf)) * sample.radiance * attenuation;
                    printf("add to radiance out\n");
                    // printf("radiance_out: %f, %f, %f", radiance_out.r, radiance_out.g, radiance_out.b);
                }
                
            }
        };

        // If the BSDF is discrete (i.e. uses dirac deltas/if statements), then we are never
        // going to hit the exact right direction by sampling lights, so ignore them.
        if(!bsdf.is_discrete()) {
            for(const auto& light : lights) sample_light(light);
            if(env_light.has_value()) sample_light(env_light.value());
        }
    }

    // TODO (PathTracer): Task 5
    // Compute an indirect lighting estimate using pathtracing with Monte Carlo.

    // (1) Ray objects have a depth field; if it reaches max_depth, you should
    // terminate the path.
    if (ray.depth == max_depth) {
        return radiance_out;
    }

    // (2) Randomly select a new ray direction (it may be reflection or transmittance
    // ray depending on surface type) using bsdf.sample()
    BSDF_Sample samp = bsdf.sample(out_dir);

    // (3) Compute the throughput of the recursive ray. This should be the current ray's
    // throughput scaled by the BSDF attenuation, cos(theta), and BSDF sample PDF.
    // Potentially terminate the path using Russian roulette as a function of the new throughput.
    // Note that allowing the termination probability to approach 1 may cause extra speckling.
    Spectrum tp = ray.throughput * samp.attenuation * out_dir.y * samp.pdf;
    float prr = 0.2;
    if (RNG::coin_flip(prr)) { // terminate
        return radiance_out;
    }
    // terminate by russian roulette

    // (4) Create new scene-space ray and cast it to get incoming light. As with shadow rays, you
    // should modify time_bounds so that the ray does not intersect at time = 0. Remember to
    // set the new throughput and depth values.
    Ray new_ray(hit.position, object_to_world.rotate(samp.direction));
    new_ray.throughput = tp;
    new_ray.depth = ray.depth + 1;
    new_ray.dist_bounds.x = EPS_F;
    Spectrum Li = trace_ray(new_ray);

    // (5) Add contribution due to incoming light with proper weighting. Remember to add in
    // the BSDF sample emissive term.
    radiance_out += (Li * out_dir.y * bsdf.evaluate(out_dir, samp.direction) * (1/prr)) + samp.emissive;

    return radiance_out;
}

} // namespace PT
