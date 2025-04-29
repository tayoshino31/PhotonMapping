#include <fstream>
#include "vector.h"

void print3(std::string name, Vector3 vec) {
    std::cout << name << " " << vec.x << " " << vec.y << " " << vec.z << std::endl;
}

bool saveToPly(const std::string& path, const std::vector<Vector3>& pts, 
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

inline Vector3 sample_cos_hemisphere(const Vector2& rnd_param) {
    Real phi = c_TWOPI * rnd_param[0];
    Real tmp = sqrt(std::clamp(1 - rnd_param[1], Real(0), Real(1)));
    return Vector3{
        cos(phi) * tmp, sin(phi) * tmp,
        sqrt(std::clamp(rnd_param[1], Real(0), Real(1)))
    };
}