#pragma once
#include "parsers/parse_scene.h"
#include "parallel.h"
#include "image.h"
#include "render.h"
#include "timer.h"
#include "progress_reporter.h"
#include "pcg.h"
#include "photon.h"
#include <embree4/rtcore.h>
#include <memory>
#include <thread>
#include <vector>
#include <iostream>

extern template float  next_pcg32_real<float >(pcg32_state&);
extern template double next_pcg32_real<double>(pcg32_state&);

Image3 pm_render(const Scene &scene) {
    //initialize
    const int n_photons = 1000000;
    const int n_neighbors = 500;
    const int max_depth = 10;
    const int w = scene.camera.width, 
    const h = scene.camera.height;
    Image3 img(w, h);
    PhotonMapping pm(n_photons, scene, n_neighbors, max_depth);
    
    //photon tracing
    pcg32_state rng = init_pcg32();
    pm.photon_tracing(rng);
    //create kdtree
    pm.build_kdtree();

    //Camera-Ray Tracing
    constexpr int tile_size = 16;
    int num_tiles_x = (w + tile_size - 1) / tile_size;
    int num_tiles_y = (h + tile_size - 1) / tile_size;
    ProgressReporter reporter(num_tiles_x * num_tiles_y);
    parallel_for([&](const Vector2i &tile) {
        pcg32_state rng = init_pcg32(tile[1] * num_tiles_x + tile[0]);
        int x0 = tile[0] * tile_size;
        int x1 = min(x0 + tile_size, w);
        int y0 = tile[1] * tile_size;
        int y1 = min(y0 + tile_size, h);
        for (int y = y0; y < y1; y++) {
            for (int x = x0; x < x1; x++) {
                Spectrum radiance = make_zero_spectrum();
                int spp = scene.options.samples_per_pixel;
                for (int s = 0; s < spp; s++) {
                    radiance += pm.camera_tracing(x, y, rng);
                }
                img(x, y) = radiance / Real(spp);
            }
        }
        reporter.update(1);
    }, Vector2i(num_tiles_x, num_tiles_y));
    reporter.done();
    return img;
}

int main(int argc, char *argv[]) {
    std::cout << "Parsing and constructing scene..." << std::endl;

    if (argc <= 1) {
        std::cout << "[Usage] ./lajolla [-t num_threads] [-o output_file_name] \
                      [-r is_path_tracing]  filename.xml" << std::endl;
        return 0;
    }

    int num_threads = std::thread::hardware_concurrency();
    std::string outputfile = "";
    std::vector<std::string> filenames;
    bool is_path_traing = false;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "-t") {
            num_threads = std::stoi(std::string(argv[++i]));
        } else if (std::string(argv[i]) == "-o") {
            outputfile = std::string(argv[++i]);
        } 
        else if (std::string(argv[i]) == "-r") {
            is_path_traing = bool(argv[++i]);
        }else {
            filenames.push_back(std::string(argv[i]));
        }
    }

    RTCDevice embree_device = rtcNewDevice(nullptr);
    parallel_init(num_threads);

    for (const std::string &filename : filenames) {
        Timer timer;
        tick(timer);
        std::cout << "Parsing and constructing scene " << filename << "." << std::endl;
        std::unique_ptr<Scene> scene = parse_scene(filename, embree_device);
        std::cout << "Done. Took " << tick(timer) << " seconds." << std::endl;
        std::cout << "Rendering..." << std::endl;
        Image3 img;
        if(is_path_traing){
            img = render(*scene);
        }
        else{
            img = pm_render(*scene);
        }
        if (outputfile.compare("") == 0) {outputfile = scene->output_filename;}
        std::cout << "Done. Took " << tick(timer) << " seconds." << std::endl;
        imwrite(outputfile, img);
        std::cout << "Image written to " << outputfile << std::endl;
    }

    parallel_cleanup();
    rtcReleaseDevice(embree_device);
    return 0;
}

