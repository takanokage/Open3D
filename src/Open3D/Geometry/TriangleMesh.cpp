// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2018 www.open3d.org
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
// ----------------------------------------------------------------------------

#include "Open3D/Geometry/TriangleMesh.h"
#include "Open3D/Geometry/IntersectionTest.h"
#include "Open3D/Geometry/KDTreeFlann.h"
#include "Open3D/Geometry/PointCloud.h"

#include <Eigen/Dense>
#include <queue>
#include <random>
#include <tuple>

#include "Open3D/Utility/Console.h"

namespace open3d {
namespace geometry {

void TriangleMesh::Clear() {
    vertices_.clear();
    vertex_normals_.clear();
    vertex_colors_.clear();
    triangles_.clear();
    triangle_normals_.clear();
    adjacency_list_.clear();
}

bool TriangleMesh::IsEmpty() const { return !HasVertices(); }

Eigen::Vector3d TriangleMesh::GetMinBound() const {
    if (!HasVertices()) {
        return Eigen::Vector3d(0.0, 0.0, 0.0);
    }
    auto itr_x = std::min_element(
            vertices_.begin(), vertices_.end(),
            [](const Eigen::Vector3d &a, const Eigen::Vector3d &b) {
                return a(0) < b(0);
            });
    auto itr_y = std::min_element(
            vertices_.begin(), vertices_.end(),
            [](const Eigen::Vector3d &a, const Eigen::Vector3d &b) {
                return a(1) < b(1);
            });
    auto itr_z = std::min_element(
            vertices_.begin(), vertices_.end(),
            [](const Eigen::Vector3d &a, const Eigen::Vector3d &b) {
                return a(2) < b(2);
            });
    return Eigen::Vector3d((*itr_x)(0), (*itr_y)(1), (*itr_z)(2));
}

Eigen::Vector3d TriangleMesh::GetMaxBound() const {
    if (!HasVertices()) {
        return Eigen::Vector3d(0.0, 0.0, 0.0);
    }
    auto itr_x = std::max_element(
            vertices_.begin(), vertices_.end(),
            [](const Eigen::Vector3d &a, const Eigen::Vector3d &b) {
                return a(0) < b(0);
            });
    auto itr_y = std::max_element(
            vertices_.begin(), vertices_.end(),
            [](const Eigen::Vector3d &a, const Eigen::Vector3d &b) {
                return a(1) < b(1);
            });
    auto itr_z = std::max_element(
            vertices_.begin(), vertices_.end(),
            [](const Eigen::Vector3d &a, const Eigen::Vector3d &b) {
                return a(2) < b(2);
            });
    return Eigen::Vector3d((*itr_x)(0), (*itr_y)(1), (*itr_z)(2));
}

TriangleMesh &TriangleMesh::Transform(const Eigen::Matrix4d &transformation) {
    for (auto &vertex : vertices_) {
        Eigen::Vector4d new_point =
                transformation *
                Eigen::Vector4d(vertex(0), vertex(1), vertex(2), 1.0);
        vertex = new_point.block<3, 1>(0, 0);
    }
    for (auto &vertex_normal : vertex_normals_) {
        Eigen::Vector4d new_normal =
                transformation * Eigen::Vector4d(vertex_normal(0),
                                                 vertex_normal(1),
                                                 vertex_normal(2), 0.0);
        vertex_normal = new_normal.block<3, 1>(0, 0);
    }
    for (auto &triangle_normal : triangle_normals_) {
        Eigen::Vector4d new_normal =
                transformation * Eigen::Vector4d(triangle_normal(0),
                                                 triangle_normal(1),
                                                 triangle_normal(2), 0.0);
        triangle_normal = new_normal.block<3, 1>(0, 0);
    }
    return *this;
}

TriangleMesh &TriangleMesh::Translate(const Eigen::Vector3d &translation) {
    for (auto &vertex : vertices_) {
        vertex += translation;
    }
    return *this;
}

TriangleMesh &TriangleMesh::Scale(const double scale) {
    for (auto &vertex : vertices_) {
        vertex *= scale;
    }
    return *this;
}

TriangleMesh &TriangleMesh::Rotate(const Eigen::Vector3d &rotation,
                                   RotationType type) {
    const Eigen::Matrix3d R = GetRotationMatrix(rotation, type);
    for (auto &vertex : vertices_) {
        vertex = R * vertex;
    }
    for (auto &normal : vertex_normals_) {
        normal = R * normal;
    }
    for (auto &normal : triangle_normals_) {
        normal = R * normal;
    }
    return *this;
}

TriangleMesh &TriangleMesh::operator+=(const TriangleMesh &mesh) {
    if (mesh.IsEmpty()) return (*this);
    size_t old_vert_num = vertices_.size();
    size_t add_vert_num = mesh.vertices_.size();
    size_t new_vert_num = old_vert_num + add_vert_num;
    size_t old_tri_num = triangles_.size();
    size_t add_tri_num = mesh.triangles_.size();
    size_t new_tri_num = old_tri_num + add_tri_num;
    if ((!HasVertices() || HasVertexNormals()) && mesh.HasVertexNormals()) {
        vertex_normals_.resize(new_vert_num);
        for (size_t i = 0; i < add_vert_num; i++)
            vertex_normals_[old_vert_num + i] = mesh.vertex_normals_[i];
    } else {
        vertex_normals_.clear();
    }
    if ((!HasVertices() || HasVertexColors()) && mesh.HasVertexColors()) {
        vertex_colors_.resize(new_vert_num);
        for (size_t i = 0; i < add_vert_num; i++)
            vertex_colors_[old_vert_num + i] = mesh.vertex_colors_[i];
    } else {
        vertex_colors_.clear();
    }
    vertices_.resize(new_vert_num);
    for (size_t i = 0; i < add_vert_num; i++)
        vertices_[old_vert_num + i] = mesh.vertices_[i];

    if ((!HasTriangles() || HasTriangleNormals()) &&
        mesh.HasTriangleNormals()) {
        triangle_normals_.resize(new_tri_num);
        for (size_t i = 0; i < add_tri_num; i++)
            triangle_normals_[old_tri_num + i] = mesh.triangle_normals_[i];
    } else {
        triangle_normals_.clear();
    }
    triangles_.resize(triangles_.size() + mesh.triangles_.size());
    Eigen::Vector3i index_shift((int)old_vert_num, (int)old_vert_num,
                                (int)old_vert_num);
    for (size_t i = 0; i < add_tri_num; i++) {
        triangles_[old_tri_num + i] = mesh.triangles_[i] + index_shift;
    }
    if (HasAdjacencyList()) {
        ComputeAdjacencyList();
    }
    return (*this);
}

TriangleMesh TriangleMesh::operator+(const TriangleMesh &mesh) const {
    return (TriangleMesh(*this) += mesh);
}

void TriangleMesh::ComputeTriangleNormals(bool normalized /* = true*/) {
    triangle_normals_.resize(triangles_.size());
    for (size_t i = 0; i < triangles_.size(); i++) {
        auto &triangle = triangles_[i];
        Eigen::Vector3d v01 = vertices_[triangle(1)] - vertices_[triangle(0)];
        Eigen::Vector3d v02 = vertices_[triangle(2)] - vertices_[triangle(0)];
        triangle_normals_[i] = v01.cross(v02);
    }
    if (normalized) {
        NormalizeNormals();
    }
}

void TriangleMesh::ComputeVertexNormals(bool normalized /* = true*/) {
    if (HasTriangleNormals() == false) {
        ComputeTriangleNormals(false);
    }
    vertex_normals_.resize(vertices_.size(), Eigen::Vector3d::Zero());
    for (size_t i = 0; i < triangles_.size(); i++) {
        auto &triangle = triangles_[i];
        vertex_normals_[triangle(0)] += triangle_normals_[i];
        vertex_normals_[triangle(1)] += triangle_normals_[i];
        vertex_normals_[triangle(2)] += triangle_normals_[i];
    }
    if (normalized) {
        NormalizeNormals();
    }
}

void TriangleMesh::ComputeAdjacencyList() {
    adjacency_list_.clear();
    adjacency_list_.resize(vertices_.size());
    for (const auto &triangle : triangles_) {
        adjacency_list_[triangle(0)].insert(triangle(1));
        adjacency_list_[triangle(0)].insert(triangle(2));
        adjacency_list_[triangle(1)].insert(triangle(0));
        adjacency_list_[triangle(1)].insert(triangle(2));
        adjacency_list_[triangle(2)].insert(triangle(0));
        adjacency_list_[triangle(2)].insert(triangle(1));
    }
}

void TriangleMesh::Purge() {
    RemoveDuplicatedVertices();
    RemoveDuplicatedTriangles();
    RemoveNonManifoldTriangles();
    RemoveNonManifoldVertices();
}

std::shared_ptr<PointCloud> SamplePointsUniformly(
        const TriangleMesh &input,
        size_t number_of_points,
        std::vector<double> &triangle_areas,
        double surface_area) {
    // triangle areas to cdf
    triangle_areas[0] /= surface_area;
    for (size_t tidx = 1; tidx < input.triangles_.size(); ++tidx) {
        triangle_areas[tidx] =
                triangle_areas[tidx] / surface_area + triangle_areas[tidx - 1];
    }

    // sample point cloud
    bool has_vert_normal = input.HasVertexNormals();
    bool has_vert_color = input.HasVertexColors();
    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    auto pcd = std::make_shared<PointCloud>();
    pcd->points_.resize(number_of_points);
    if (has_vert_normal) {
        pcd->normals_.resize(number_of_points);
    }
    if (has_vert_color) {
        pcd->colors_.resize(number_of_points);
    }
    size_t point_idx = 0;
    for (size_t tidx = 0; tidx < input.triangles_.size(); ++tidx) {
        size_t n = std::round(triangle_areas[tidx] * number_of_points);
        while (point_idx < n) {
            double r1 = dist(mt);
            double r2 = dist(mt);
            double a = (1 - std::sqrt(r1));
            double b = std::sqrt(r1) * (1 - r2);
            double c = std::sqrt(r1) * r2;

            const Eigen::Vector3i &triangle = input.triangles_[tidx];
            pcd->points_[point_idx] = a * input.vertices_[triangle(0)] +
                                      b * input.vertices_[triangle(1)] +
                                      c * input.vertices_[triangle(2)];
            if (has_vert_normal) {
                pcd->normals_[point_idx] =
                        a * input.vertex_normals_[triangle(0)] +
                        b * input.vertex_normals_[triangle(1)] +
                        c * input.vertex_normals_[triangle(2)];
            }
            if (has_vert_color) {
                pcd->colors_[point_idx] =
                        a * input.vertex_colors_[triangle(0)] +
                        b * input.vertex_colors_[triangle(1)] +
                        c * input.vertex_colors_[triangle(2)];
            }

            point_idx++;
        }
    }

    return pcd;
}

void TriangleMesh::FilterSharpen(int number_of_iterations,
                                 double strength,
                                 FilterScope scope) {
    if (!HasAdjacencyList()) {
        ComputeAdjacencyList();
    }

    bool filter_vertex =
            scope == FilterScope::All || scope == FilterScope::Vertex;
    bool filter_normal =
            (scope == FilterScope::All || scope == FilterScope::Normal) &&
            HasVertexNormals();
    bool filter_color =
            (scope == FilterScope::All || scope == FilterScope::Color) &&
            HasVertexColors();

    for (int iter = 0; iter < number_of_iterations; ++iter) {
        std::vector<Eigen::Vector3d> prev_vertices = vertices_.Read();
        std::vector<Eigen::Vector3d> prev_vertex_normals = vertex_normals_.Read();
        std::vector<Eigen::Vector3d> prev_vertex_colors = vertex_colors_.Read();

        for (size_t vidx = 0; vidx < vertices_.size(); ++vidx) {
            Eigen::Vector3d vertex_sum(0, 0, 0);
            Eigen::Vector3d normal_sum(0, 0, 0);
            Eigen::Vector3d color_sum(0, 0, 0);
            for (int nbidx : adjacency_list_[vidx]) {
                if (filter_vertex) {
                    vertex_sum += prev_vertices[nbidx];
                }
                if (filter_normal) {
                    normal_sum += prev_vertex_normals[nbidx];
                }
                if (filter_color) {
                    color_sum += prev_vertex_colors[nbidx];
                }
            }

            size_t nb_size = adjacency_list_[vidx].size();
            if (filter_vertex) {
                vertices_[vidx] =
                        prev_vertices[vidx] +
                        strength * (prev_vertices[vidx] * nb_size - vertex_sum);
            }
            if (filter_normal) {
                vertex_normals_[vidx] =
                        prev_vertex_normals[vidx] +
                        strength * (prev_vertex_normals[vidx] * nb_size -
                                    normal_sum);
            }
            if (filter_color) {
                vertex_colors_[vidx] =
                        prev_vertex_colors[vidx] +
                        strength * (prev_vertex_colors[vidx] * nb_size -
                                    color_sum);
            }
        }
    }
}

void TriangleMesh::FilterSmoothSimple(int number_of_iterations,
                                      FilterScope scope) {
    if (!HasAdjacencyList()) {
        ComputeAdjacencyList();
    }

    bool filter_vertex =
            scope == FilterScope::All || scope == FilterScope::Vertex;
    bool filter_normal =
            (scope == FilterScope::All || scope == FilterScope::Normal) &&
            HasVertexNormals();
    bool filter_color =
            (scope == FilterScope::All || scope == FilterScope::Color) &&
            HasVertexColors();

    for (int iter = 0; iter < number_of_iterations; ++iter) {
        std::vector<Eigen::Vector3d> prev_vertices = vertices_.Read();
        std::vector<Eigen::Vector3d> prev_vertex_normals = vertex_normals_.Read();
        std::vector<Eigen::Vector3d> prev_vertex_colors = vertex_colors_.Read();

        for (size_t vidx = 0; vidx < vertices_.size(); ++vidx) {
            Eigen::Vector3d vertex_sum(0, 0, 0);
            Eigen::Vector3d normal_sum(0, 0, 0);
            Eigen::Vector3d color_sum(0, 0, 0);
            for (int nbidx : adjacency_list_[vidx]) {
                if (filter_vertex) {
                    vertex_sum += prev_vertices[nbidx];
                }
                if (filter_normal) {
                    normal_sum += prev_vertex_normals[nbidx];
                }
                if (filter_color) {
                    color_sum += prev_vertex_colors[nbidx];
                }
            }

            size_t nb_size = adjacency_list_[vidx].size();
            if (filter_vertex) {
                vertices_[vidx] =
                        (prev_vertices[vidx] + vertex_sum) / (1 + nb_size);
            }
            if (filter_normal) {
                vertex_normals_[vidx] =
                        (prev_vertex_normals[vidx] + normal_sum) /
                        (1 + nb_size);
            }
            if (filter_color) {
                vertex_colors_[vidx] =
                        (prev_vertex_colors[vidx] + color_sum) / (1 + nb_size);
            }
        }
    }
}

void TriangleMesh::FilterSmoothLaplacian(int number_of_iterations,
                                         double lambda,
                                         FilterScope scope) {
    if (!HasAdjacencyList()) {
        ComputeAdjacencyList();
    }

    bool filter_vertex =
            scope == FilterScope::All || scope == FilterScope::Vertex;
    bool filter_normal =
            (scope == FilterScope::All || scope == FilterScope::Normal) &&
            HasVertexNormals();
    bool filter_color =
            (scope == FilterScope::All || scope == FilterScope::Color) &&
            HasVertexColors();

    for (int iter = 0; iter < number_of_iterations; ++iter) {
        std::vector<Eigen::Vector3d> prev_vertices = vertices_.Read();
        std::vector<Eigen::Vector3d> prev_vertex_normals = vertex_normals_.Read();
        std::vector<Eigen::Vector3d> prev_vertex_colors = vertex_colors_.Read();

        for (size_t vidx = 0; vidx < vertices_.size(); ++vidx) {
            Eigen::Vector3d vertex_sum(0, 0, 0);
            Eigen::Vector3d normal_sum(0, 0, 0);
            Eigen::Vector3d color_sum(0, 0, 0);
            double total_weight = 0;
            for (int nbidx : adjacency_list_[vidx]) {
                auto diff = prev_vertices[vidx] - prev_vertices[nbidx];
                double dist = diff.norm();
                double weight = 1. / (dist + 1e-12);
                total_weight += weight;

                if (filter_vertex) {
                    vertex_sum += weight * prev_vertices[nbidx];
                }
                if (filter_normal) {
                    normal_sum += weight * prev_vertex_normals[nbidx];
                }
                if (filter_color) {
                    color_sum += weight * prev_vertex_colors[nbidx];
                }
            }

            if (filter_vertex) {
                vertices_[vidx] = prev_vertices[vidx] +
                                  lambda * (vertex_sum / total_weight -
                                            prev_vertices[vidx]);
            }
            if (filter_normal) {
                vertex_normals_[vidx] = prev_vertex_normals[vidx] +
                                        lambda * (normal_sum / total_weight -
                                                  prev_vertex_normals[vidx]);
            }
            if (filter_color) {
                vertex_colors_[vidx] = prev_vertex_colors[vidx] +
                                       lambda * (color_sum / total_weight -
                                                 prev_vertex_colors[vidx]);
            }
        }
    }
}

void TriangleMesh::FilterSmoothTaubin(int number_of_iterations,
                                      double lambda,
                                      double mu,
                                      FilterScope scope) {
    for (int iter = 0; iter < number_of_iterations; ++iter) {
        FilterSmoothLaplacian(1, lambda, scope);
        FilterSmoothLaplacian(1, -mu, scope);
    }
}

std::shared_ptr<PointCloud> SamplePointsUniformly(const TriangleMesh &input,
                                                  size_t number_of_points) {
    if (number_of_points <= 0) {
        utility::PrintWarning("[SamplePointsUniformly] number_of_points <= 0");
        return std::make_shared<PointCloud>();
    }
    if (input.triangles_.size() == 0) {
        utility::PrintWarning(
                "[SamplePointsUniformly] input mesh has no triangles");
        return std::make_shared<PointCloud>();
    }

    // Compute area of each triangle and sum surface area
    std::vector<double> triangle_areas;
    double surface_area = input.GetSurfaceArea(triangle_areas);

    return SamplePointsUniformly(input, number_of_points, triangle_areas,
                                 surface_area);
}

std::shared_ptr<PointCloud> SamplePointsPoissonDisk(
        const TriangleMesh &input,
        size_t number_of_points,
        double init_factor /* = 5 */,
        const std::shared_ptr<PointCloud> pcl_init /* = nullptr */) {
    if (number_of_points <= 0) {
        utility::PrintWarning("[SamplePointsUniformly] number_of_points <= 0");
        return std::make_shared<PointCloud>();
    }
    if (input.triangles_.size() == 0) {
        utility::PrintWarning(
                "[SamplePointsUniformly] input mesh has no triangles");
        return std::make_shared<PointCloud>();
    }
    if (pcl_init == nullptr && init_factor < 1) {
        utility::PrintWarning(
                "[SamplePointsUniformly] either pass pcl_init with #points "
                "> "
                "number_of_points or init_factor > 1");
        return std::make_shared<PointCloud>();
    }
    if (pcl_init != nullptr && pcl_init->points_.size() < number_of_points) {
        utility::PrintWarning(
                "[SamplePointsUniformly] either pass pcl_init with #points "
                "> "
                "number_of_points, or init_factor > 1");
        return std::make_shared<PointCloud>();
    }

    // Compute area of each triangle and sum surface area
    std::vector<double> triangle_areas;
    double surface_area = input.GetSurfaceArea(triangle_areas);

    // Compute init points using uniform sampling
    std::shared_ptr<PointCloud> pcl;
    if (pcl_init == nullptr) {
        pcl = SamplePointsUniformly(input, init_factor * number_of_points,
                                    triangle_areas, surface_area);
    } else {
        pcl = std::make_shared<PointCloud>();
        pcl->points_ = pcl_init->points_;
        pcl->normals_ = pcl_init->normals_;
        pcl->colors_ = pcl_init->colors_;
    }

    // Set-up sample elimination
    double alpha = 8;    // constant defined in paper
    double beta = 0.5;   // constant defined in paper
    double gamma = 1.5;  // constant defined in paper
    double ratio = double(number_of_points) / double(pcl->points_.size());
    double r_max = 2 * std::sqrt((surface_area / number_of_points) /
                                 (2 * std::sqrt(3.)));
    double r_min = r_max * beta * (1 - std::pow(ratio, gamma));

    std::vector<double> weights(pcl->points_.size());
    std::vector<bool> deleted(pcl->points_.size(), false);
    KDTreeFlann kdtree(*pcl);

    auto WeightFcn = [&](double d2) {
        double d = std::sqrt(d2);
        if (d < r_min) {
            d = r_min;
        }
        return std::pow(1 - d / r_max, alpha);
    };

    auto ComputePointWeight = [&](int pidx0) {
        std::vector<int> nbs;
        std::vector<double> dists2;
        kdtree.SearchRadius(pcl->points_[pidx0], r_max, nbs, dists2);
        double weight = 0;
        for (size_t nbidx = 0; nbidx < nbs.size(); ++nbidx) {
            int pidx1 = nbs[nbidx];
            // only count weights if not the same point if not deleted
            if (pidx0 == pidx1 || deleted[pidx1]) {
                continue;
            }
            weight += WeightFcn(dists2[nbidx]);
        }

        weights[pidx0] = weight;
    };

    // init weights and priority queue
    typedef std::tuple<int, double> QueueEntry;
    auto WeightCmp = [](const QueueEntry &a, const QueueEntry &b) {
        return std::get<1>(a) < std::get<1>(b);
    };
    std::priority_queue<QueueEntry, std::vector<QueueEntry>,
                        decltype(WeightCmp)>
            queue(WeightCmp);
    for (size_t pidx0 = 0; pidx0 < pcl->points_.size(); ++pidx0) {
        ComputePointWeight(pidx0);
        queue.push(QueueEntry(pidx0, weights[pidx0]));
    };

    // sample elimination
    int current_number_of_points = pcl->points_.size();
    while (current_number_of_points > number_of_points) {
        int pidx;
        double weight;
        std::tie(pidx, weight) = queue.top();
        queue.pop();

        // test if the entry is up to date (because of reinsert)
        if (deleted[pidx] || weight != weights[pidx]) {
            continue;
        }

        // delete current sample
        deleted[pidx] = true;
        current_number_of_points--;

        // update weights
        std::vector<int> nbs;
        std::vector<double> dists2;
        kdtree.SearchRadius(pcl->points_[pidx], r_max, nbs, dists2);
        for (int nb : nbs) {
            ComputePointWeight(nb);
            queue.push(QueueEntry(nb, weights[nb]));
        }
    }

    // update pcl
    bool has_vert_normal = pcl->HasNormals();
    bool has_vert_color = pcl->HasColors();
    int next_free = 0;
    for (size_t idx = 0; idx < pcl->points_.size(); ++idx) {
        if (!deleted[idx]) {
            pcl->points_[next_free] = pcl->points_[idx];
            if (has_vert_normal) {
                pcl->normals_[next_free] = pcl->normals_[idx];
            }
            if (has_vert_color) {
                pcl->colors_[next_free] = pcl->colors_[idx];
            }
            next_free++;
        }
    }
    pcl->points_.resize(next_free);
    if (has_vert_normal) {
        pcl->normals_.resize(next_free);
    }
    if (has_vert_color) {
        pcl->colors_.resize(next_free);
    }

    return pcl;
}

void TriangleMesh::RemoveDuplicatedVertices() {
    typedef std::tuple<double, double, double> Coordinate3;
    std::unordered_map<Coordinate3, size_t,
                       utility::hash_tuple::hash<Coordinate3>>
            point_to_old_index;
    std::vector<int> index_old_to_new(vertices_.size());
    bool has_vert_normal = HasVertexNormals();
    bool has_vert_color = HasVertexColors();
    size_t old_vertex_num = vertices_.size();
    size_t k = 0;                                  // new index
    for (size_t i = 0; i < old_vertex_num; i++) {  // old index
        Coordinate3 coord = std::make_tuple(vertices_[i](0), vertices_[i](1),
                                            vertices_[i](2));
        if (point_to_old_index.find(coord) == point_to_old_index.end()) {
            point_to_old_index[coord] = i;
            vertices_[k] = vertices_[i];
            if (has_vert_normal) vertex_normals_[k] = vertex_normals_[i];
            if (has_vert_color) vertex_colors_[k] = vertex_colors_[i];
            index_old_to_new[i] = (int)k;
            k++;
        } else {
            index_old_to_new[i] = index_old_to_new[point_to_old_index[coord]];
        }
    }
    vertices_.resize(k);
    if (has_vert_normal) vertex_normals_.resize(k);
    if (has_vert_color) vertex_colors_.resize(k);
    if (k < old_vertex_num) {
        for (auto &triangle : triangles_) {
            triangle(0) = index_old_to_new[triangle(0)];
            triangle(1) = index_old_to_new[triangle(1)];
            triangle(2) = index_old_to_new[triangle(2)];
        }
        if (HasAdjacencyList()) {
            ComputeAdjacencyList();
        }
    }
    utility::PrintDebug(
            "[RemoveDuplicatedVertices] %d vertices have been removed.\n",
            (int)(old_vertex_num - k));
}

void TriangleMesh::RemoveDuplicatedTriangles() {
    typedef std::tuple<int, int, int> Index3;
    std::unordered_map<Index3, size_t, utility::hash_tuple::hash<Index3>>
            triangle_to_old_index;
    bool has_tri_normal = HasTriangleNormals();
    size_t old_triangle_num = triangles_.size();
    size_t k = 0;
    for (size_t i = 0; i < old_triangle_num; i++) {
        Index3 index;
        // We first need to find the minimum index. Because triangle (0-1-2)
        // and triangle (2-0-1) are the same.
        if (triangles_[i](0) <= triangles_[i](1)) {
            if (triangles_[i](0) <= triangles_[i](2)) {
                index = std::make_tuple(triangles_[i](0), triangles_[i](1),
                                        triangles_[i](2));
            } else {
                index = std::make_tuple(triangles_[i](2), triangles_[i](0),
                                        triangles_[i](1));
            }
        } else {
            if (triangles_[i](1) <= triangles_[i](2)) {
                index = std::make_tuple(triangles_[i](1), triangles_[i](2),
                                        triangles_[i](0));
            } else {
                index = std::make_tuple(triangles_[i](2), triangles_[i](0),
                                        triangles_[i](1));
            }
        }
        if (triangle_to_old_index.find(index) == triangle_to_old_index.end()) {
            triangle_to_old_index[index] = i;
            triangles_[k] = triangles_[i];
            if (has_tri_normal) triangle_normals_[k] = triangle_normals_[i];
            k++;
        }
    }
    triangles_.resize(k);
    if (has_tri_normal) triangle_normals_.resize(k);
    if (k < old_triangle_num && HasAdjacencyList()) {
        ComputeAdjacencyList();
    }
    utility::PrintDebug(
            "[RemoveDuplicatedTriangles] %d triangles have been removed.\n",
            (int)(old_triangle_num - k));
}

void TriangleMesh::RemoveNonManifoldVertices() {
    // Non-manifold vertices are vertices without a triangle reference. They
    // should not exist in a valid triangle mesh.
    std::vector<bool> vertex_has_reference(vertices_.size(), false);
    for (const auto &triangle : triangles_) {
        vertex_has_reference[triangle(0)] = true;
        vertex_has_reference[triangle(1)] = true;
        vertex_has_reference[triangle(2)] = true;
    }
    std::vector<int> index_old_to_new(vertices_.size());
    bool has_vert_normal = HasVertexNormals();
    bool has_vert_color = HasVertexColors();
    size_t old_vertex_num = vertices_.size();
    size_t k = 0;                                  // new index
    for (size_t i = 0; i < old_vertex_num; i++) {  // old index
        if (vertex_has_reference[i]) {
            vertices_[k] = vertices_[i];
            if (has_vert_normal) vertex_normals_[k] = vertex_normals_[i];
            if (has_vert_color) vertex_colors_[k] = vertex_colors_[i];
            index_old_to_new[i] = (int)k;
            k++;
        } else {
            index_old_to_new[i] = -1;
        }
    }
    vertices_.resize(k);
    if (has_vert_normal) vertex_normals_.resize(k);
    if (has_vert_color) vertex_colors_.resize(k);
    if (k < old_vertex_num) {
        for (auto &triangle : triangles_) {
            triangle(0) = index_old_to_new[triangle(0)];
            triangle(1) = index_old_to_new[triangle(1)];
            triangle(2) = index_old_to_new[triangle(2)];
        }
        if (HasAdjacencyList()) {
            ComputeAdjacencyList();
        }
    }
    utility::PrintDebug(
            "[RemoveNonManifoldVertices] %d vertices have been removed.\n",
            (int)(old_vertex_num - k));
}

void TriangleMesh::RemoveNonManifoldTriangles() {
    // Non-manifold triangles are degenerate triangles that have one vertex
    // as its multiple end-points. They are usually the product of removing
    // duplicated vertices.
    bool has_tri_normal = HasTriangleNormals();
    size_t old_triangle_num = triangles_.size();
    size_t k = 0;
    for (size_t i = 0; i < old_triangle_num; i++) {
        const auto &triangle = triangles_[i];
        if (triangle(0) != triangle(1) && triangle(1) != triangle(2) &&
            triangle(2) != triangle(0)) {
            triangles_[k] = triangles_[i];
            if (has_tri_normal) triangle_normals_[k] = triangle_normals_[i];
            k++;
        }
    }
    triangles_.resize(k);
    if (has_tri_normal) triangle_normals_.resize(k);
    if (k < old_triangle_num && HasAdjacencyList()) {
        ComputeAdjacencyList();
    }
    utility::PrintDebug(
            "[RemoveNonManifoldTriangles] %d triangles have been "
            "removed.\n",
            (int)(old_triangle_num - k));
}

std::unordered_map<Edge, int, utility::hash_tuple::hash<Edge>>
TriangleMesh::GetEdgeTriangleCount() const {
    std::unordered_map<Edge, int, utility::hash_tuple::hash<Edge>>
            trias_per_edge;
    auto AddEdge = [&](int vidx0, int vidx1) {
        int min0 = std::min(vidx0, vidx1);
        int max0 = std::max(vidx0, vidx1);
        Edge edge(min0, max0);
        if (trias_per_edge.count(edge) == 0) {
            trias_per_edge[edge] = 1;
        } else {
            trias_per_edge[edge] += 1;
        }
    };
    for (auto triangle : triangles_) {
        AddEdge(triangle(0), triangle(1));
        AddEdge(triangle(1), triangle(2));
        AddEdge(triangle(2), triangle(0));
    }
    return trias_per_edge;
}

double ComputeTriangleArea(const Eigen::Vector3d &p0,
                           const Eigen::Vector3d &p1,
                           const Eigen::Vector3d &p2) {
    const Eigen::Vector3d x = p0 - p1;
    const Eigen::Vector3d y = p0 - p2;
    double area = 0.5 * x.cross(y).norm();
    return area;
}

double TriangleMesh::GetTriangleArea(size_t triangle_idx) const {
    const Eigen::Vector3i &triangle = triangles_[triangle_idx];
    const Eigen::Vector3d &vertex0 = vertices_[triangle(0)];
    const Eigen::Vector3d &vertex1 = vertices_[triangle(1)];
    const Eigen::Vector3d &vertex2 = vertices_[triangle(2)];
    return ComputeTriangleArea(vertex0, vertex1, vertex2);
}

double TriangleMesh::GetSurfaceArea() const {
    double surface_area = 0;
    for (size_t tidx = 0; tidx < triangles_.size(); ++tidx) {
        double triangle_area = GetTriangleArea(tidx);
        surface_area += triangle_area;
    }
    return surface_area;
}

double TriangleMesh::GetSurfaceArea(std::vector<double> &triangle_areas) const {
    double surface_area = 0;
    triangle_areas.resize(triangles_.size());
    for (size_t tidx = 0; tidx < triangles_.size(); ++tidx) {
        double triangle_area = GetTriangleArea(tidx);
        triangle_areas[tidx] = triangle_area;
        surface_area += triangle_area;
    }
    return surface_area;
}

Eigen::Vector4d ComputeTrianglePlane(const Eigen::Vector3d &p0,
                                     const Eigen::Vector3d &p1,
                                     const Eigen::Vector3d &p2) {
    const Eigen::Vector3d e0 = p1 - p0;
    const Eigen::Vector3d e1 = p2 - p0;
    Eigen::Vector3d abc = e0.cross(e1);
    double norm = abc.norm();
    // if the three points are co-linear, return invalid plane
    if (norm == 0) {
        return Eigen::Vector4d(0, 0, 0, 0);
    }
    abc /= abc.norm();
    double d = -abc.dot(p0);
    return Eigen::Vector4d(abc(0), abc(1), abc(2), d);
}

Eigen::Vector4d TriangleMesh::GetTrianglePlane(size_t triangle_idx) const {
    const Eigen::Vector3i &triangle = triangles_[triangle_idx];
    const Eigen::Vector3d &vertex0 = vertices_[triangle(0)];
    const Eigen::Vector3d &vertex1 = vertices_[triangle(1)];
    const Eigen::Vector3d &vertex2 = vertices_[triangle(2)];
    return ComputeTrianglePlane(vertex0, vertex1, vertex2);
}

int TriangleMesh::EulerPoincareCharacteristic() const {
    std::unordered_set<Edge, utility::hash_tuple::hash<Edge>> edges;
    for (auto triangle : triangles_) {
        int min0 = std::min(triangle(0), triangle(1));
        int max0 = std::max(triangle(0), triangle(1));
        edges.emplace(Edge(min0, max0));

        int min1 = std::min(triangle(0), triangle(2));
        int max1 = std::max(triangle(0), triangle(2));
        edges.emplace(Edge(min1, max1));

        int min2 = std::min(triangle(1), triangle(2));
        int max2 = std::max(triangle(1), triangle(2));
        edges.emplace(Edge(min2, max2));
    }

    int E = edges.size();
    int V = vertices_.size();
    int F = triangles_.size();
    return V + F - E;
}

bool TriangleMesh::IsEdgeManifold(
        bool allow_boundary_edges /* = true */) const {
    std::unordered_map<Edge, int, utility::hash_tuple::hash<Edge>> edges;
    for (auto triangle : triangles_) {
        int min0 = std::min(triangle(0), triangle(1));
        int max0 = std::max(triangle(0), triangle(1));
        Edge e0(min0, max0);
        if (edges.count(e0) == 0) {
            edges[e0] = 1;
        } else {
            edges[e0] += 1;
        }

        int min1 = std::min(triangle(0), triangle(2));
        int max1 = std::max(triangle(0), triangle(2));
        Edge e1(min1, max1);
        if (edges.count(e1) == 0) {
            edges[e1] = 1;
        } else {
            edges[e1] += 1;
        }

        int min2 = std::min(triangle(1), triangle(2));
        int max2 = std::max(triangle(1), triangle(2));
        Edge e2(min2, max2);
        if (edges.count(e2) == 0) {
            edges[e2] = 1;
        } else {
            edges[e2] += 1;
        }
    }

    for (auto &kv : edges) {
        if ((allow_boundary_edges && (kv.second < 1 || kv.second > 2)) ||
            (!allow_boundary_edges && kv.second != 2)) {
            return false;
        }
    }

    return true;
}

bool TriangleMesh::IsVertexManifold() const {
    std::vector<std::unordered_set<int>> vert_to_triangles(vertices_.size());
    for (size_t tidx = 0; tidx < triangles_.size(); ++tidx) {
        const auto &tria = triangles_[tidx];
        vert_to_triangles[tria(0)].emplace(tidx);
        vert_to_triangles[tria(1)].emplace(tidx);
        vert_to_triangles[tria(2)].emplace(tidx);
    }

    for (size_t vidx = 0; vidx < vertices_.size(); ++vidx) {
        const auto &triangles = vert_to_triangles[vidx];
        if (triangles.size() == 0) {
            continue;
        }

        // collect edges and vertices
        std::unordered_map<int, std::unordered_set<int>> edges;
        for (int tidx : triangles) {
            const auto &triangle = triangles_[tidx];
            if (triangle(0) != vidx && triangle(1) != vidx) {
                edges[triangle(0)].emplace(triangle(1));
                edges[triangle(1)].emplace(triangle(0));
            } else if (triangle(0) != vidx && triangle(2) != vidx) {
                edges[triangle(0)].emplace(triangle(2));
                edges[triangle(2)].emplace(triangle(0));
            } else if (triangle(1) != vidx && triangle(2) != vidx) {
                edges[triangle(1)].emplace(triangle(2));
                edges[triangle(2)].emplace(triangle(1));
            }
        }

        // test if vertices are connected
        std::queue<int> next;
        std::unordered_set<int> visited;
        next.push(edges.begin()->first);
        visited.emplace(edges.begin()->first);
        while (!next.empty()) {
            int vert = next.front();
            next.pop();

            for (auto nb : edges[vert]) {
                if (visited.count(nb) == 0) {
                    visited.emplace(nb);
                    next.emplace(nb);
                }
            }
        }
        if (visited.size() != edges.size()) {
            return false;
        }
    }

    return true;
}

bool TriangleMesh::IsSelfIntersecting() const {
    for (size_t tidx0 = 0; tidx0 < triangles_.size() - 1; ++tidx0) {
        const Eigen::Vector3i &tria0 = triangles_[tidx0];
        const Eigen::Vector3d &p0 = vertices_[tria0(0)];
        const Eigen::Vector3d &p1 = vertices_[tria0(1)];
        const Eigen::Vector3d &p2 = vertices_[tria0(2)];
        for (size_t tidx1 = tidx0 + 1; tidx1 < triangles_.size(); ++tidx1) {
            const Eigen::Vector3i &tria1 = triangles_[tidx1];
            // check if neighbour triangle
            if (tria0(0) == tria1(0) || tria0(0) == tria1(1) ||
                tria0(0) == tria1(2) || tria0(1) == tria1(0) ||
                tria0(1) == tria1(1) || tria0(1) == tria1(2) ||
                tria0(2) == tria1(0) || tria0(2) == tria1(1) ||
                tria0(2) == tria1(2)) {
                continue;
            }

            // check for intersection
            const Eigen::Vector3d &q0 = vertices_[tria1(0)];
            const Eigen::Vector3d &q1 = vertices_[tria1(1)];
            const Eigen::Vector3d &q2 = vertices_[tria1(2)];
            if (IntersectingTriangleTriangle3d(p0, p1, p2, q0, q1, q2)) {
                return true;
            }
        }
    }
    return false;
}

}  // namespace geometry
}  // namespace open3d
