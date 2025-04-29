#pragma once
#include <vector>
#include "lajolla.h"
#include "ray.h"
#include "spectrum.h"
#include "vector.h"
#include "scene.h"
#include "pcg.h"

struct Sample {
    Vector3 p;
    Vector3 w;
    Spectrum beta;
};

class PhotonMapping {
    private:
        int num_photons;
        std::vector<Sample> photon_map;
        const Scene& scene;
    public:
        PhotonMapping(int num_photons, const Scene& scene);
        ~PhotonMapping();
        std::optional<Ray> bounce_photon(PathVertex isect, Ray photon_ray, 
            Spectrum& beta, pcg32_state& rng);
        void photon_tracing(pcg32_state& rng);
        Spectrum camera_traing(int x, int y, pcg32_state& rng);
        Spectrum radiance_estimation();
};