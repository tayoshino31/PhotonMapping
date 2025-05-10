#pragma once  
#include <fstream>
#include "vector.h"

inline void print3(std::string name, Vector3 vec) {
    std::cout << name << " " << vec.x << " " << vec.y << " " << vec.z << std::endl;
}

inline bool saveToPly(const std::string& path, const std::vector<Vector3>& pts,
               const std::vector<Vector3>& rgb)
{
    std::ofstream file(path, std::ios::out | std::ios::trunc);
    if (!file) return false;
    file << "ply\n"
        << "format ascii 1.0\n"
        << "element vertex " << pts.size() << '\n'
        << "property float x\n"
        << "property float y\n"
        << "property float z\n"
        << "property uchar red\n"
        << "property uchar green\n"
        << "property uchar blue\n"
        << "end_header\n";
    for (size_t i = 0; i < pts.size(); ++i) {
        const auto& p = pts[i];
        const auto& c = rgb[i];
        file << p.x << ' ' << p.y << ' ' << p.z << ' '
            << static_cast<int>(std::clamp(static_cast<float>(c.x * 255.0f), 0.0f, 255.0f)) << ' '
            << static_cast<int>(std::clamp(static_cast<float>(c.y * 255.0f), 0.0f, 255.0f)) << ' '
            << static_cast<int>(std::clamp(static_cast<float>(c.z * 255.0f), 0.0f, 255.0f)) << '\n';
    }
    return file.good();
}
