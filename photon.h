#pragma once
#include "lajolla.h"
#include "ray.h"
#include "spectrum.h"
#include "vector.h"
#include "scene.h"
#include "pcg.h"
#include "utils.h"
#include "kdtree.cpp"
#include <fstream>
#include <algorithm>
#include <array>
#include <vector>

inline Vector3 sample_cos_hemisphere(const Vector2& rnd_param) {
    Real phi = c_TWOPI * rnd_param[0];
    Real tmp = sqrt(std::clamp(1 - rnd_param[1], Real(0), Real(1)));
    return Vector3{
        cos(phi) * tmp, sin(phi) * tmp,
        sqrt(std::clamp(rnd_param[1], Real(0), Real(1)))
    };
}

struct Photon {
    Vector3 position;
    Vector3 direction;
    Spectrum energy;
};

class PhotonMapping {
    private:
        int num_photons;
        int n_neighbors;
        int max_depth;
        const Scene& scene;
        std::vector<Photon> photon_map;
        std::vector<Vector3> photon_pos;
        PhotonKDTree kdtree;
    public:
        PhotonMapping(const int num_photons, const Scene& scene, const int n_neighbors, const int max_depth);
        ~PhotonMapping();
        std::optional<Ray> bounce_photon(PathVertex isect, Ray photon_ray, 
            Spectrum& beta, pcg32_state& rng);
        void store_photon(Vector3 position, Vector3 direction, Spectrum beta);
        void photon_tracing(pcg32_state& rng);
        void build_kdtree();
        Spectrum camera_tracing(int x, int y, pcg32_state& rng);
        Spectrum dirct_illumination(PathVertex isect, Vector3 dir_view, pcg32_state& rng);
        Spectrum indirct_illumination(PathVertex isect, Vector3 wo, std::vector<size_t> neighbors, Real radius2);
};