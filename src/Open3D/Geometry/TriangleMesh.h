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

#pragma once

#include <Eigen/Core>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "Open3D/Geometry/Geometry3D.h"
#include "Open3D/Types/Blob.h"
#include "Open3D/Utility/Helper.h"

namespace open3d {
namespace geometry {

class PointCloud;

typedef std::tuple<int, int> Edge;

class TriangleMesh : public Geometry3D {
public:
    enum class SimplificationContraction { Average, Quadric };
    enum class FilterScope { All, Color, Normal, Vertex };

    TriangleMesh() : Geometry3D(Geometry::GeometryType::TriangleMesh) {}
    ~TriangleMesh() override {}

public:
    void Clear() override;
    bool IsEmpty() const override;
    Eigen::Vector3d GetMinBound() const override;
    Eigen::Vector3d GetMaxBound() const override;
    TriangleMesh &Transform(const Eigen::Matrix4d &transformation) override;
    TriangleMesh &Translate(const Eigen::Vector3d &translation) override;
    TriangleMesh &Scale(const double scale) override;
    TriangleMesh &Rotate(const Eigen::Vector3d &rotation,
                         RotationType type = RotationType::XYZ) override;

public:
    TriangleMesh &operator+=(const TriangleMesh &mesh);
    TriangleMesh operator+(const TriangleMesh &mesh) const;

    /// Function to compute triangle normals, usually called before rendering
    void ComputeTriangleNormals(bool normalized = true);

    /// Function to compute vertex normals, usually called before rendering
    void ComputeVertexNormals(bool normalized = true);

    /// Function to compute adjacency list, call before adjacency list is needed
    void ComputeAdjacencyList();

    /// Function to remove duplicated and non-manifold vertices/triangles
    void Purge();

    /// Function to sharpen triangle mesh
    void FilterSharpen(int number_of_iterations,
                       double strength,
                       FilterScope scope = FilterScope::All);

    /// Function to smooth triangle mesh with simple neighbour average
    void FilterSmoothSimple(int number_of_iterations,
                            FilterScope scope = FilterScope::All);

    /// Function to smooth triangle mesh using Laplacian
    void FilterSmoothLaplacian(int number_of_iterations,
                               double lambda,
                               FilterScope scope = FilterScope::All);

    /// Function to smooth triangle mesh using method of Taubin
    void FilterSmoothTaubin(int number_of_iterations,
                            double lambda,
                            double mu,
                            FilterScope scope = FilterScope::All);

protected:
    // Forward child class type to avoid indirect nonvirtual base
    TriangleMesh(Geometry::GeometryType type) : Geometry3D(type) {}
    virtual void RemoveDuplicatedVertices();
    virtual void RemoveDuplicatedTriangles();
    virtual void RemoveNonManifoldVertices();
    virtual void RemoveNonManifoldTriangles();

public:
    bool HasVertices() const { return vertices_.size() > 0; }

    bool HasTriangles() const {
        return vertices_.size() > 0 && triangles_.size() > 0;
    }

    bool HasVertexNormals() const {
        return vertices_.size() > 0 &&
               vertex_normals_.size() == vertices_.size();
    }

    bool HasVertexColors() const {
        return vertices_.size() > 0 &&
               vertex_colors_.size() == vertices_.size();
    }

    bool HasTriangleNormals() const {
        return HasTriangles() && triangles_.size() == triangle_normals_.size();
    }

    bool HasAdjacencyList() const {
        return vertices_.size() > 0 &&
               adjacency_list_.size() == vertices_.size();
    }

    void NormalizeNormals() {
        for (size_t i = 0; i < vertex_normals_.size(); i++) {
            vertex_normals_[i].normalize();
            if (std::isnan(vertex_normals_[i](0))) {
                vertex_normals_[i] = Eigen::Vector3d(0.0, 0.0, 1.0);
            }
        }
        for (size_t i = 0; i < triangle_normals_.size(); i++) {
            triangle_normals_[i].normalize();
            if (std::isnan(triangle_normals_[i](0))) {
                triangle_normals_[i] = Eigen::Vector3d(0.0, 0.0, 1.0);
            }
        }
    }

    void PaintUniformColor(const Eigen::Vector3d &color) {
        vertex_colors_.resize(vertices_.size());
        for (size_t i = 0; i < vertices_.size(); i++) {
            vertex_colors_[i] = color;
        }
    }

    /// Function that computes the Euler-Poincaré characteristic V + F - E
    int EulerPoincareCharacteristic() const;

    /// Function that checks if the given triangle mesh is edge-manifold.
    /// A mesh is edge­manifold if each edge is bounding either one or two
    /// triangles. If allow_boundary_edges is set to false, than retuns false if
    /// there exists boundary edges.
    bool IsEdgeManifold(bool allow_boundary_edges = true) const;

    /// Function that checks if all vertices in the triangle mesh are manifold.
    /// A vertex is manifold if its star is edge‐manifold and edge‐connected.
    /// (Two or more faces connected only by a vertex and not by an edge.)
    bool IsVertexManifold() const;

    /// Function that tests if the triangle mesh is self-intersecting.
    /// Tests each triangle pair for intersection.
    bool IsSelfIntersecting() const;

    /// Function that counts the number of faces an edge belongs.
    /// Returns a map of Edge (vertex0, vertex1) to number of faces.
    std::unordered_map<Edge, int, utility::hash_tuple::hash<Edge>>
    GetEdgeTriangleCount() const;

    /// Function that computes the area of a mesh triangle identified by the
    /// triangle index
    double GetTriangleArea(size_t triangle_idx) const;

    /// Function that computes the surface area of the mesh, i.e. the sum of
    /// the individual triangle surfaces.
    double GetSurfaceArea() const;

    /// Function that computes the surface area of the mesh, i.e. the sum of
    /// the individual triangle surfaces.
    double GetSurfaceArea(std::vector<double> &triangle_areas) const;

    /// Function that computes the plane equation of a mesh triangle identified
    /// by the triangle index.
    Eigen::Vector4d GetTrianglePlane(size_t triangle_idx) const;

public:
    Vertices vertices_;
    Vertex_normals vertex_normals_;
    Vertex_colors vertex_colors_;
    Triangles triangles_;
    Triangle_normals triangle_normals_;
    std::vector<std::unordered_set<int>> adjacency_list_;
};

/// Function that computes the area of a mesh triangle
double ComputeTriangleArea(const Eigen::Vector3d &p0,
                           const Eigen::Vector3d &p1,
                           const Eigen::Vector3d &p2);

/// Function that computes the plane equation from the three points.
/// If the three points are co-linear, then this function returns the invalid
/// plane (0, 0, 0, 0).
Eigen::Vector4d ComputeTrianglePlane(const Eigen::Vector3d &p0,
                                     const Eigen::Vector3d &p1,
                                     const Eigen::Vector3d &p2);

/// Function to sample \param number_of_points points uniformly from the mesh
std::shared_ptr<PointCloud> SamplePointsUniformly(const TriangleMesh &input,
                                                  size_t number_of_points);

/// Function to sample \param number_of_points points (blue noise).
/// Based on the method presented in Yuksel, "Sample Elimination for Generating
/// Poisson Disk Sample Sets", EUROGRAPHICS, 2015
/// The PointCloud \param pcl_init is used for sample elimination if given,
/// otherwise a PointCloud is first uniformly sampled with
/// \param init_number_of_points x \param number_of_points number of points.
std::shared_ptr<PointCloud> SamplePointsPoissonDisk(
        const TriangleMesh &input,
        size_t number_of_points,
        double init_factor = 5,
        const std::shared_ptr<PointCloud> pcl_init = nullptr);

/// Function to subdivide triangle mesh using the simple midpoint algorithm.
/// Each triangle is subdivided into four triangles per iteration and the
/// new vertices lie on the midpoint of the triangle edges.
std::shared_ptr<TriangleMesh> SubdivideMidpoint(const TriangleMesh &input,
                                                int number_of_iterations);

/// Function to subdivide triangle mesh using Loop's scheme.
/// Cf. Charles T. Loop, "Smooth subdivision surfaces based on triangles", 1987.
/// Each triangle is subdivided into four triangles per iteration.
std::shared_ptr<TriangleMesh> SubdivideLoop(const TriangleMesh &input,
                                            int number_of_iterations);

/// Function to simplify mesh using Vertex Clustering.
/// The result can be a non-manifold mesh.
std::shared_ptr<TriangleMesh> SimplifyVertexClustering(
        const TriangleMesh &input,
        double voxel_size,
        TriangleMesh::SimplificationContraction contraction =
                TriangleMesh::SimplificationContraction::Average);

/// Function to simplify mesh using Quadric Error Metric Decimation by
/// Garland and Heckbert.
std::shared_ptr<TriangleMesh> SimplifyQuadricDecimation(
        const TriangleMesh &input, int target_number_of_triangles);

/// Function to select points from \param input TriangleMesh into
/// \return output TriangleMesh
/// Vertices with indices in \param indices are selected.
std::shared_ptr<TriangleMesh> SelectDownSample(
        const TriangleMesh &input, const std::vector<size_t> &indices);

/// Function to crop \param input tringlemesh into output tringlemesh
/// All points with coordinates less than \param min_bound or larger than
/// \param max_bound are clipped.
std::shared_ptr<TriangleMesh> CropTriangleMesh(
        const TriangleMesh &input,
        const Eigen::Vector3d &min_bound,
        const Eigen::Vector3d &max_bound);

/// Factory function to create a tetrahedron mesh (trianglemeshfactory.cpp).
/// the mesh centroid will be at (0,0,0) and \param radius defines the distance
/// from the center to the mesh vertices.
std::shared_ptr<TriangleMesh> CreateMeshTetrahedron(double radius = 1.0);

/// Factory function to create a octahedron mesh (trianglemeshfactory.cpp).
/// the mesh centroid will be at (0,0,0) and \param radius defines the distance
/// from the center to the mesh vertices.
std::shared_ptr<TriangleMesh> CreateMeshOctahedron(double radius = 1.0);

/// Factory function to create a icosahedron mesh (trianglemeshfactory.cpp).
/// the mesh centroid will be at (0,0,0) and \param radius defines the distance
/// from the center to the mesh vertices.
std::shared_ptr<TriangleMesh> CreateMeshIcosahedron(double radius = 1.0);

/// Factory function to create a box mesh (TriangleMeshFactory.cpp)
/// The left bottom corner on the front will be placed at (0, 0, 0).
/// The \param width is x-directional length, and \param height and \param depth
/// are y- and z-directional lengths respectively.
std::shared_ptr<TriangleMesh> CreateMeshBox(double width = 1.0,
                                            double height = 1.0,
                                            double depth = 1.0);

/// Factory function to create a sphere mesh (TriangleMeshFactory.cpp)
/// The sphere with \param radius will be centered at (0, 0, 0).
/// Its axis is aligned with z-axis.
/// The longitudes will be split into \param resolution segments.
/// The latitudes will be split into \param resolution * 2 segments.
std::shared_ptr<TriangleMesh> CreateMeshSphere(double radius = 1.0,
                                               int resolution = 20);

/// Factory function to create a cylinder mesh (TriangleMeshFactory.cpp)
/// The axis of the cylinder will be from (0, 0, -height/2) to (0, 0, height/2).
/// The circle with \param radius will be split into \param resolution segments.
/// The \param height will be split into \param split segments.
std::shared_ptr<TriangleMesh> CreateMeshCylinder(double radius = 1.0,
                                                 double height = 2.0,
                                                 int resolution = 20,
                                                 int split = 4);

/// Factory function to create a cone mesh (TriangleMeshFactory.cpp)
/// The axis of the cone will be from (0, 0, 0) to (0, 0, \param height).
/// The circle with \param radius will be split into \param resolution segments.
/// The height will be split into \param split segments.
std::shared_ptr<TriangleMesh> CreateMeshCone(double radius = 1.0,
                                             double height = 2.0,
                                             int resolution = 20,
                                             int split = 1);

/// Factory function to create a torus mesh (TriangleMeshFactory.cpp)
/// The torus will be centered at (0, 0, 0) and a radius of \param torus_radius.
/// The tube of the torus will have a radius of \param tube_radius.
/// The number of segments in radial and tubular direction are \param
/// radial_resolution and \param tubular_resolution respectively.
std::shared_ptr<TriangleMesh> CreateMeshTorus(double torus_radius = 1.0,
                                              double tube_radius = 0.5,
                                              int radial_resolution = 30,
                                              int tubular_resolution = 20);

/// Factory function to create an arrow mesh (TriangleMeshFactory.cpp)
/// The axis of the cone with \param cone_radius will be along the z-axis.
/// The cylinder with \param cylinder_radius is from
/// (0, 0, 0) to (0, 0, cylinder_height), and
/// the cone is from (0, 0, cylinder_height)
/// to (0, 0, cylinder_height + cone_height).
/// The cone will be split into \param resolution segments.
/// The \param cylinder_height will be split into \param cylinder_split
/// segments. The \param cone_height will be split into \param cone_split
/// segments.
std::shared_ptr<TriangleMesh> CreateMeshArrow(double cylinder_radius = 1.0,
                                              double cone_radius = 1.5,
                                              double cylinder_height = 5.0,
                                              double cone_height = 4.0,
                                              int resolution = 20,
                                              int cylinder_split = 4,
                                              int cone_split = 1);

/// Factory function to create a coordinate frame mesh (TriangleMeshFactory.cpp)
/// The coordinate frame will be centered at \param origin
/// The x, y, z axis will be rendered as red, green, and blue arrows
/// respectively. \param size is the length of the axes.
std::shared_ptr<TriangleMesh> CreateMeshCoordinateFrame(
        double size = 1.0,
        const Eigen::Vector3d &origin = Eigen::Vector3d(0.0, 0.0, 0.0));

}  // namespace geometry
}  // namespace open3d
