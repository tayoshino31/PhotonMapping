#pragma once  
#include "photon.h"

PhotonMapping::PhotonMapping(const int num_photons, const Scene& scene, const int n_neighbors, const int max_depth) 
: num_photons(num_photons), scene(scene), n_neighbors(n_neighbors), max_depth(max_depth){
}
PhotonMapping::~PhotonMapping(){
}
void PhotonMapping::build_kdtree() {kdtree.build(photon_pos); }
void PhotonMapping::store_photon(Vector3 position, 
                                 Vector3 direction, 
                                 Spectrum energy) {
    Photon p{position, direction, energy};
    photon_map.push_back(p);
    photon_pos.push_back(position);
}
Real get_area(const Light& light, const Scene& scene){
    const DiffuseAreaLight* areaLight = std::get_if<DiffuseAreaLight>(&light);
    const TriangleMesh* mesh = std::get_if<TriangleMesh>(&scene.shapes[areaLight->shape_id]);
    return mesh->total_area;
}



///------------------------------ Photon Tracing ------------------------------------///
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
        Frame frame(point_on_light.normal);
        Vector3 dir = to_world(frame, sample_cos_hemisphere(uv));
            
        // create a photon ray
        Ray photon_ray{pos, dir, get_shadow_epsilon(scene), infinity<Real>()};

        // compute del flux of photon
        Real area = get_area(light, scene);
        Spectrum Le = emission(light, dir, 0, point_on_light, scene);
        Spectrum beta = area * Le * c_PI; //TODO

        // photon mapping
        Spectrum throughput = make_const_spectrum(1.0);
        for (int bounce = 0; bounce < max_depth; bounce++) {
            std::optional<PathVertex> vertex_ = intersect(scene, photon_ray);
            if (!vertex_) {break;}
            PathVertex vertex = *vertex_;
            if(is_light(scene.shapes[vertex.shape_id])) break; //TODO break if the vertex is light source

            if(bounce >= 1) //TODO for only indirect illumination
                store_photon(vertex.position, -photon_ray.dir, beta * throughput); 
            std::optional<Ray> reflected_ray = bounce_photon(vertex, photon_ray, throughput, rng);

            if (!reflected_ray) break;
            else photon_ray = *reflected_ray;
            
            //Russian roulete
            if(bounce > 1){
                Real rr_prob = min(max(throughput), 0.95);
                Real rand = next_pcg32_real<Real>(rng);
                if (rand >= rr_prob) break; 
                else throughput /= rr_prob;
            }
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
                                                               bsdf_rnd_param_uv, bsdf_rnd_param_w,  TransportDirection::TO_VIEW);
    if (!bsdf_sample_) return std::nullopt; 
    const BSDFSampleRecord& bsdf_sample = *bsdf_sample_;
    Vector3 wo = bsdf_sample.dir_out; 

    // update photon throughput
    Spectrum f = eval(mat, wi, wo, isect, scene.texture_pool, TransportDirection::TO_VIEW);
    Real pdf = pdf_sample_bsdf(mat, wi, wo, isect, scene.texture_pool, TransportDirection::TO_VIEW);
    //Real cos_theta = abs(dot(wo, isect.shading_frame.n));
    Real wo_ns = max(0.0, dot(wo, isect.shading_frame.n));
    Real wi_ng = max(0.0, dot(wi, isect.geometric_normal));
    Real wi_ns = max(0.0, dot(wi, isect.shading_frame.n));
    Real wo_ng = max(0.0, dot(wo, isect.geometric_normal));
    Real cos_theta = abs(wo_ns) * abs(wi_ng) / abs(wo_ng);
    if (wi_ng * wi_ns <= 0 || wo_ng * wo_ns <= 0) {cos_theta = 0; }
    if (pdf <= 0.0) return std::nullopt;


    beta *= f * cos_theta/ pdf; //TODO no cosin=therta

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
    std::vector<size_t> neighbors = kdtree.findNearestN(query, n_neighbors, radius2);
    // float radius = 10.0f;
    // float radius2 = radius * radius;
    // std::vector<size_t> neighbors = kdtree.findPhotonsWithinRadius(query, radius);

    // direct illumination
    //Spectrum direct = make_zero_spectrum();
    Spectrum direct = dirct_illumination(isect, -ray.dir, rng);

    // indirect illumination
    //Spectrum indirect = make_zero_spectrum();
    Spectrum indirect =  indirct_illumination(isect, -ray.dir, neighbors, radius2);
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
    if (is_light(scene.shapes[isect.shape_id])) return emission(isect, wo, scene);
    for (const size_t& index : neighbors) {
        const Photon& photon = photon_map[index];

        Spectrum f = eval(mat, photon.direction, wo, isect, scene.texture_pool, TransportDirection::TO_VIEW); 

        //cancel cosin term
        Frame frame = isect.shading_frame;
        if (dot(frame.n, photon.direction) < 0) {frame = -frame;}
        Real offset = fmax(dot(frame.n, wo), Real(0));
        if(offset > 0.0001) f /= offset;

        indirect += photon.energy * f;
    }
    if(neighbors.size() > 0){
         indirect /= (c_PI * radius2 * Real(num_photons)); //divided by n phton at the end
    }
    return indirect;
}



// Vector2 bsdf_rnd_param_uv{next_pcg32_real<Real>(rng), next_pcg32_real<Real>(rng)};
// Real bsdf_rnd_param_w = next_pcg32_real<Real>(rng);
// std::optional<BSDFSampleRecord> bsdf_sample_ = sample_bsdf(mat, wo, isect, scene.texture_pool, 
//                                                            bsdf_rnd_param_uv, bsdf_rnd_param_w);
// const BSDFSampleRecord& bsdf_sample = *bsdf_sample_;
// Vector3 dir = bsdf_sample.dir_out; 
// Spectrum f = eval(mat, wo, dir, isect, scene.texture_pool);
// Real pdf = pdf_sample_bsdf(mat, wo, dir, isect, scene.texture_pool);
// Real cos_theta = abs(dot(dir, isect.shading_frame.n));
// if (pdf <= 0) return make_zero_spectrum();

//return f * cos_theta * indirect / pdf;
    


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

//std::string path = "C:\\Users\\tayos\\Desktop\\points\\pos.ply";
//save_point_as_ply(positions, colors, path);

//std::cout << photon_map.size() << std::endl;
//std::string path = "C:\\Users\\tayos\\Desktop\\light_emission\\bounce_2.ply";
//save_point_as_ply(positions, power, path);