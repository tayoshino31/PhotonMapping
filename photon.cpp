#include "photon.h"
#include "utils.h"
#include <fstream>
#include <algorithm>
#include <array>

PhotonMapping::PhotonMapping(int num_photons, const Scene& scene) 
: num_photons(num_photons), scene(scene){
}

PhotonMapping::~PhotonMapping(){
}

std::optional<Ray> PhotonMapping::bounce_photon(PathVertex isect, Ray photon_ray,
                                 Spectrum &beta, pcg32_state& rng) {
    const Material& mat = scene.materials[isect.material_id];
    Vector3 dir_view = -photon_ray.dir;
    Vector2 bsdf_rnd_param_uv{ next_pcg32_real<Real>(rng), next_pcg32_real<Real>(rng) };
    Real bsdf_rnd_param_w = next_pcg32_real<Real>(rng);
    std::optional<BSDFSampleRecord> bsdf_sample_ =
        sample_bsdf(mat,
            dir_view,
            isect,
            scene.texture_pool,
            bsdf_rnd_param_uv,
            bsdf_rnd_param_w);
    if (!bsdf_sample_) {
        // BSDF sampling failed. Abort the loop.
        return std::nullopt;
    }
    const BSDFSampleRecord& bsdf_sample = *bsdf_sample_;
    Vector3 dir_bsdf = bsdf_sample.dir_out;
    Ray bsdf_ray{isect.position, dir_bsdf, get_intersection_epsilon(scene), infinity<Real>() };
    return bsdf_ray;
}

void PhotonMapping::photon_tracing(pcg32_state& rng){
    std::array<std::vector<Vector3>, 5> vertexPositions;
    std::array<std::vector<Vector3>, 5> vertexRGB;
    static const std::array<Vector3, 5> bounceColors = {
        Vector3(1.0, 1.0, 1.0),
        Vector3(0.0, 0.7, 1.0),
        Vector3(0.5, 1.0, 0.0),
        Vector3(1.0, 0.0, 1.0),
        Vector3(1.0, 0.5, 0.0)
    };

    for (int i = 0; i < num_photons; i++) {
        //Sample Photon Ray
        Vector2 light_uv{ next_pcg32_real<Real>(rng), next_pcg32_real<Real>(rng) };
        Real light_w = next_pcg32_real<Real>(rng);
        Real shape_w = next_pcg32_real<Real>(rng);
        int light_id = sample_light(scene, light_w);
        const Light& light = scene.lights[light_id];
        Vector3 dummy_ref_point(0, 0, 0);
        PointAndNormal point_on_light =
            sample_point_on_light(light, dummy_ref_point, light_uv, shape_w, scene);
        Vector3 dir = sample_cos_hemisphere(
             Vector2(next_pcg32_real<Real>(rng),next_pcg32_real<Real>(rng)));
        Vector3 pos = point_on_light.position;
        Ray photon_ray{pos, dir, get_shadow_epsilon(scene), infinity<Real>()};

        Spectrum beta = fromRGB(Vector3{ 1, 1, 1 });
        for (int bounce = 0; bounce < 5; bounce++) {
            std::optional<PathVertex> vertex_ = intersect(scene, photon_ray);
            if (!vertex_) {break;}
            PathVertex vertex = *vertex_;

            //print3("pos", vertex.position);
            vertexPositions[bounce].push_back(vertex.position);
            vertexRGB[bounce].push_back(bounceColors[bounce]);

            std::optional<Ray> reflected_ray = bounce_photon(vertex, photon_ray, beta, rng);
            if (!reflected_ray) {
                break;
            }
            else {
                photon_ray = *reflected_ray;
            }
        }
    }
    for (int bounce = 0; bounce < 5; ++bounce) {
        std::string path = "C:\\Users\\tayos\\Desktop\\PLY\\bounce_" + std::to_string(bounce) + ".ply";
        saveToPly(path, vertexPositions[bounce], vertexRGB[bounce]);
    }
}

Spectrum PhotonMapping::camera_traing(int x, int y, pcg32_state& rng) {
    if (x % 10 < 5) {
        return make_const_spectrum(1.0);
    }
    return make_zero_spectrum();
}

Spectrum PhotonMapping::radiance_estimation() {
    return make_zero_spectrum();
}