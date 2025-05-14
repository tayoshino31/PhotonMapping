#pragma once
#include "nanoflann.hpp"
#include "vector.h"
#include <memory>
using namespace nanoflann;

struct PointCloud {
    struct P { float x, y, z; };
    std::vector<P> pts;
    size_t kdtree_get_point_count() const { return pts.size(); }
    float  kdtree_get_pt(size_t idx, size_t dim) const
    {
        return dim == 0 ? pts[idx].x : dim == 1 ? pts[idx].y : pts[idx].z;
    }
    template<class BBOX> bool kdtree_get_bbox(BBOX&) const { return false; }
};

using KDTree = nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<float, PointCloud>,PointCloud,3,  size_t >;

// ---------- wrapper class ----------
class PhotonKDTree {
    PointCloud cloud;
    std::unique_ptr<KDTree> kd_tree; 
public:
    // fill cloud & build index
    void build(const std::vector<Vector3>& pos)
    {
        for (const Vector3& p: pos) {
            PointCloud::P xyz = { p.x, p.y, p.z };
            cloud.pts.push_back(xyz);
        }
        kd_tree = std::make_unique<KDTree>(3, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10));
        kd_tree->buildIndex();
    }
    // query
    std::vector<size_t> findNearestN(const float q[3], size_t n, float& max_dist2) const
    {
        std::vector<size_t> indices(n);
        std::vector<float>   distances(n); 
        nanoflann::KNNResultSet<float> rs(n);
        rs.init(indices.data(), distances.data());
        kd_tree->findNeighbors(rs, q, nanoflann::SearchParameters());
        max_dist2 = *std::max_element(distances.begin(), distances.end());
        return indices;
    }
    std::vector<size_t> findPhotonsWithinRadius(const float q[3], float radius) const {
        std::vector<nanoflann::ResultItem<size_t, float>> matches;
        nanoflann::SearchParameters params;
        kd_tree->radiusSearch(q, radius, matches, params);
        std::vector<size_t> indices;
        indices.reserve(matches.size());
        for (const auto& match : matches) {
            indices.push_back(match.first);
        }

        return indices;
    }
};
