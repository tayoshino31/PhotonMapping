#pragma once  
#include "photon.h"

PhotonMapping::PhotonMapping(int num_photons, const Scene& scene) 
: num_photons(num_photons), scene(scene){
}
PhotonMapping::~PhotonMapping(){
}

///------------------------------ Photon Tracing ------------------------------------///
void PhotonMapping::build_kdtree() {kdtree.build(photon_pos); }
void PhotonMapping::store_photon(const Vector3 position, 
                                 const Vector3 direction, 
                                 const Spectrum energy) {
    Photon p{position, direction, energy};
    photon_map.push_back(p);
    photon_pos.push_back(position);
}
Real get_area(const Light& light, const Scene& scene){
    const DiffuseAreaLight* areaLight = std::get_if<DiffuseAreaLight>(&light);
    const TriangleMesh* mesh = std::get_if<TriangleMesh>(&scene.shapes[areaLight->shape_id]);
    return mesh->total_area;
}
void PhotonMapping::photon_tracing(pcg32_state& rng){
    for (int i = 0; i < num_photons; i++) {
        // sample position
        Vector2 light_uv{ next_pcg32_real<Real>(rng), next_pcg32_real<Real>(rng) };
        Real light_w = next_pcg32_real<Real>(rng);
        Real shape_w = next_pcg32_real<Real>(rng);
        int light_id = sample_light(scene, light_w);
        const Light& light = scene.lights[light_id];
        Vector3 dummy_ref_point(0,0,0);
        PointAndNormal point_on_light = sample_point_on_light(light, dummy_ref_point, light_uv, shape_w, scene);
        Vector3 pos = point_on_light.position;

        // sample direction
        Vector2 uv(next_pcg32_real<Real>(rng), next_pcg32_real<Real>(rng));
        Vector3 local_dir = sample_cos_hemisphere(uv);
        std::pair<Vector3, Vector3> TB = coordinate_system(point_on_light.normal); //TODO why this conversion?
        Vector3 dir = local_dir.x * TB.first + local_dir.y * TB.second + local_dir.z * point_on_light.normal;

        // create a photon ray
        Ray photon_ray{pos, dir, get_shadow_epsilon(scene), infinity<Real>()};

        // compute del flux of photon
        Real area = get_area(light, scene);
        Spectrum Le = emission(light, dir, 0, point_on_light, scene);
        Spectrum beta = area * Le * c_PI / Real(num_photons);

        // photon mapping
        for (int bounce = 0; bounce < 5; bounce++) {
            std::optional<PathVertex> vertex_ = intersect(scene, photon_ray);
            if (!vertex_) {break;}
            PathVertex vertex = *vertex_;
            store_photon(vertex.position, photon_ray.dir, beta); //TODO //if (bounce > 0) //TODO RR
            std::optional<Ray> reflected_ray = bounce_photon(vertex, photon_ray, beta, rng);
            if (!reflected_ray)break;
            else photon_ray = *reflected_ray;
        }
    }
}
std::optional<Ray> PhotonMapping::bounce_photon(PathVertex isect, Ray photon_ray, Spectrum &beta, pcg32_state& rng) {
    // sample direction
    Vector2 bsdf_rnd_param_uv{next_pcg32_real<Real>(rng), next_pcg32_real<Real>(rng)};
    Real bsdf_rnd_param_w = next_pcg32_real<Real>(rng);
    Vector3 wi = -photon_ray.dir;
    Material mat = scene.materials[isect.material_id];
    std::optional<BSDFSampleRecord> bsdf_sample_ = sample_bsdf(mat, wi, isect, scene.texture_pool, 
                                                               bsdf_rnd_param_uv, bsdf_rnd_param_w);
    if (!bsdf_sample_) return std::nullopt; 
    const BSDFSampleRecord& bsdf_sample = *bsdf_sample_;
    Vector3 wo = bsdf_sample.dir_out;

    // update photon throughput
    Spectrum f = eval(mat, wi, wo, isect, scene.texture_pool);
    Real pdf = pdf_sample_bsdf(mat, wi, wo, isect, scene.texture_pool);
    Real cos_theta = max(0.0, dot(wo, isect.geometric_normal));// TODO
    beta *= f * cos_theta / pdf; 

    // update photon ray
    Ray bsdf_ray{isect.position, wo, get_intersection_epsilon(scene), infinity<Real>() };
    return bsdf_ray;
}



///------------------------------ Rendering ------------------------------------///
Spectrum PhotonMapping::camera_tracing(int x, int y, pcg32_state& rng) {
    // create a camera ray
    int w = scene.camera.width, h = scene.camera.height;
    Vector2 screen_pos((x + next_pcg32_real<Real>(rng)) / w,
                       (y + next_pcg32_real<Real>(rng)) / h);
    Ray ray = sample_primary(scene.camera, screen_pos);

    // find intersection
    std::optional<PathVertex> vertex_ = intersect(scene, ray);
    if (!vertex_) return make_zero_spectrum();
    PathVertex isect = *vertex_;
    const Material& mat = scene.materials[isect.material_id];

    //find N-th nearest neighbors at query point.
    float query[3] = {isect.position.x, isect.position.y, isect.position.z};
    float radius2;
    std::vector<size_t> neighbors = kdtree.findNearestN(query, 500, radius2);

    // direct illumination
    Spectrum direct = make_zero_spectrum();
    //Spectrum direct = dirct_illumination(isect, -ray.dir, rng);

    // indirect illumination
    Spectrum indirect = indirct_illumination(isect, -ray.dir, neighbors, radius2);

    return direct + indirect;
}
Spectrum PhotonMapping::dirct_illumination(PathVertex isect, Vector3 dir_view, pcg32_state& rng) {
    // return emission if isect is light source
    if (is_light(scene.shapes[isect.shape_id])) return emission(isect, dir_view, scene);

    // sample light and create a shadow ray
    Vector2 light_uv{ next_pcg32_real<Real>(rng), next_pcg32_real<Real>(rng) };
    Real light_w = next_pcg32_real<Real>(rng);
    Real shape_w = next_pcg32_real<Real>(rng);
    int light_id = sample_light(scene, light_w);
    const Light& light = scene.lights[light_id];
    PointAndNormal point_on_light = sample_point_on_light(light, isect.position, light_uv, shape_w, scene);
    Vector3 dir_light = normalize(point_on_light.position - isect.position);
    Ray shadow_ray{isect.position, dir_light, get_shadow_epsilon(scene),
                  (1 - get_shadow_epsilon(scene)) * distance(point_on_light.position, isect.position) };

    // return 0 if the shading point occluded
    if (occluded(scene, shadow_ray)) return make_zero_spectrum();

    // diffuse reflection
    const Material& mat = scene.materials[isect.material_id];
    Spectrum Li = emission(light, -dir_light, Real(0), point_on_light, scene);
    Real G = max(-dot(dir_light, point_on_light.normal), Real(0)) /
             distance_squared(point_on_light.position, isect.position);
    Spectrum f = eval(mat, dir_view, dir_light, isect, scene.texture_pool);
    Real pdf = light_pmf(scene, light_id) * 
               pdf_point_on_light(light, point_on_light, isect.position, scene);
    return (Li * f * G) / pdf;
}
Spectrum PhotonMapping::indirct_illumination(PathVertex isect, Vector3 wo, std::vector<size_t> neighbors, Real radius2){
    const Material& mat = scene.materials[isect.material_id];
    Spectrum indirect = make_zero_spectrum();
    for (const size_t& index : neighbors) {
        const Photon& photon = photon_map[index];
        Vector3 wi = -photon.direction;
        Spectrum f = eval(mat, wi, wo, isect, scene.texture_pool); 
        indirect += photon.energy * f; //TODO //Real cos_theta = max(0.0, dot(omega_i, isect.geometric_normal)); 
    }
    indirect /= (c_PI * radius2);
    return indirect;
}

//std::array<std::vector<Vector3>, 5> vertexPositions;
//std::array<std::vector<Vector3>, 5> vertexRGB;
//static const std::array<Vector3, 5> bounceColors = {
//    Vector3(1.0, 1.0, 1.0),
//    Vector3(0.0, 0.7, 1.0),
//    Vector3(0.5, 1.0, 0.0),
//    Vector3(1.0, 0.0, 1.0),
//    Vector3(1.0, 0.5, 0.0)
//};

//std::vector<Vector3> vertexPositions;
//std::vector<Vector3> vertexRGB;


//std::cout << photon_map.size() << std::endl;
//std::string path = "C:\\Users\\tayos\\Desktop\\PLY_RGB\\bounce.ply";
//saveToPly(path, vertexPositions, vertexRGB);

//for (int bounce = 0; bounce < 5; ++bounce) {
//    std::string path = "C:\\Users\\tayos\\Desktop\\PLY_RGB\\bounce_" + std::to_string(bounce) + ".ply";
//    saveToPly(path, vertexPositions[bounce], vertexRGB[bounce]);
//}

//print3("pos", vertex.position);
//vertexPositions[bounce].push_back(vertex.position);
//vertexRGB[bounce].push_back(bounceColors[bounce]);
//vecPositions.push_back(vertex.position);
//vecRGB.push_back(beta);

//vertexPositions.push_back(vertex.position);
//vertexRGB.push_back(beta);


//Russian roulete
//const Material& mat = scene.materials[vertex.material_id];
//const DiffuseAreaLight* areaLight = std::get_if<DiffuseAreaLight>(&light);
//const Lambertian* m = std::get_if<Lambertian>(&mat);
//const auto* const_tex = std::get_if<ConstantTexture<Spectrum>>(&m->reflectance);
//if (const_tex) {
//    Spectrum val = const_tex->value;
//    Real rr_prob = std::min(0.95f, std::max({ float(val[0]), float(val[1]), float(val[2]) }));
//    Real rand = next_pcg32_real<Real>(rng);
//    if (rand > rr_prob) break;
//    else beta /= rr_prob;
//}


//Real wo_ns = max(0.0, dot(dir_view, isect.shading_frame.n));
//Real wi_ng = max(0.0, dot(dir_bsdf, isect.geometric_normal));
//Real wo_ng = max(0.0, dot(dir_view, isect.geometric_normal));
//Real cos_theta = wo_ns * wi_ng / wo_ng;