#pragma once  
#include <fstream>
#include "vector.h"

inline void print3(std::string name, Vector3 vec) {
    std::cout << name << " " << vec.x << " " << vec.y << " " << vec.z << std::endl;
}

inline bool save_point_as_ply(const std::vector<Vector3>& pts,
               const std::vector<Vector3>& rgb, const std::string& path)
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


inline void save_vector_as_ply(const std::vector<Vector3>& points,
                         const std::vector<Vector3>& directions,
                         const std::string& filepath,
                         float fixed_length = 0.0f) {
    if (points.size() != directions.size()) {
        std::cerr << "Error: points and directions must have the same size." << std::endl;
        return;
    }

    size_t n = points.size();
    std::ofstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "Error: Cannot open file for writing: " << filepath << std::endl;
        return;
    }

    // Write header
    file << "ply\n";
    file << "format ascii 1.0\n";
    file << "element vertex " << (2 * n) << "\n";
    file << "property float x\nproperty float y\nproperty float z\n";
    file << "element edge " << n << "\n";
    file << "property int vertex1\nproperty int vertex2\n";
    file << "end_header\n";

    // Write vertices
    for (size_t i = 0; i < n; ++i) {
        Vector3 start = points[i];
        Vector3 dir = normalize(directions[i]) * Real(fixed_length);
        Vector3 end = start + dir;
        file << start.x << " " << start.y << " " << start.z << "\n";
        file << end.x << " " << end.y << " " << end.z << "\n";
    }

    // Write edges
    for (size_t i = 0; i < n; ++i) {
        file << (2 * i) << " " << (2 * i + 1) << "\n";
    }

    file.close();
    std::cout << "Saved " << n << " fixed-length vectors to " << filepath << std::endl;
}


inline void save_line_as_ply(const std::vector<Vector3>& starts,
                         const std::vector<Vector3>& ends,
                         const std::string& filepath,
                         float fixed_length = 0.0f) {
    if (starts.size() != ends.size()) {
        std::cerr << "Error: points and directions must have the same size." << std::endl;
        return;
    }

    size_t n = starts.size();
    std::ofstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "Error: Cannot open file for writing: " << filepath << std::endl;
        return;
    }

    // Write header
    file << "ply\n";
    file << "format ascii 1.0\n";
    file << "element vertex " << (2 * n) << "\n";
    file << "property float x\nproperty float y\nproperty float z\n";
    file << "element edge " << n << "\n";
    file << "property int vertex1\nproperty int vertex2\n";
    file << "end_header\n";

    // Write vertices
    for (size_t i = 0; i < n; ++i) {
        Vector3 start = starts[i];
        Vector3 end = ends[i];
        file << start.x << " " << start.y << " " << start.z << "\n";
        file << end.x << " " << end.y << " " << end.z << "\n";
    }

    // Write edges
    for (size_t i = 0; i < n; ++i) {
        file << (2 * i) << " " << (2 * i + 1) << "\n";
    }

    file.close();
    std::cout << "Saved " << n << " fixed-length vectors to " << filepath << std::endl;
}