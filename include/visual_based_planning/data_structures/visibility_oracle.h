#pragma once
#include <vector>
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

typedef bg::model::point<double, 3, bg::cs::cartesian> Point3D;
typedef bg::model::box<Point3D> Box3D;
typedef bg::model::segment<Point3D> Segment3D;
typedef std::pair<Box3D, unsigned int> Value; // Box + ID

class VisibilityOracle {
    bgi::rtree< Value, bgi::quadratic<16> > rtree;

public:
    void addObstacle(double min_x, double min_y, double min_z, double max_x, double max_y, double max_z, int id) {
        Box3D b(Point3D(min_x, min_y, min_z), Point3D(max_x, max_y, max_z));
        rtree.insert(std::make_pair(b, id));
    }

    bool checkVisibility(const std::vector<double>& p1, const std::vector<double>& p2) {
        // Create segment and query intersection
        // Stub: return true if visible
        return true;
    }
};

























// #include <vector>
// #include <functional>
// #include <Eigen/Core>
// #include <vector>
// #include <stdio.h>
// #include <algorithm> 
// #include <cmath> 
// #include <Eigen/Geometry>


// typedef Eigen::AlignedBox<double,3> AABB;
// typedef Eigen::Vector3d Vector3d;


// class VisibilityOracle {

//     public:

//         // struct Ray {
//         //     Vector3d origin;
//         //     Vector3d dir;     // Must be normalized
//         //     float t_max;  // Max distance usually
//         // };

//         struct Segment {
//             Vector3d a;
//             Vector3d b;    
//         };

//         VisibilityOracle() = default;
//         virtual ~VisibilityOracle() = default;

//         VisibilityOracle(std::vector<AABB>& boxes) {
//             printf("built a visibility oracle for AABBs");
//         }

//         std::vector<AABB> boxes_;


//             // Public interface to check visibility based on the selected method
//         bool IsVisible(const Segment& segment, std::string method = "BF")
//         {
//             if (method == "BF") {
//                 return BruteForceVisibilityCheck(segment);
//             } else if (method == "GPUBF") {
//                 return GPUBFVisibilityCheck(segment);
//             } else if (method == "KDTree") {
//                 return KDTreeVisibilityCheck(segment);
//             } else {
//                 throw std::runtime_error("Unknown visibility check method: " + method);
//             }
//         }



//     private:

//         bool SegmentAABBIntersection(const Segment& segment, const AABB& box)
//         {
//             // Direction vector of the segment
//             Vector3d dir = segment.b - segment.a;

//             // The segment is defined parametrically as P(t) = a + t * dir
//             // Since it is a finite segment, valid intersection must occur within t = [0, 1]
//             double tmin = 0.0;
//             double tmax = 1.0;

//             // Iterate over the 3 axes (X, Y, Z)
//             for (int i = 0; i < 3; ++i) {
//                 // Calculate the inverse direction to replace division with multiplication
//                 // Note: If dir[i] is 0, this results in +/- infinity. 
//                 // IEEE 754 floating point logic handles comparisons with infinity correctly 
//                 // for the min/max logic below, assuming the segment isn't exactly on the slab boundary.
//                 double invDir = 1.0 / dir[i];

//                 // Calculate intersection t-values for the near and far planes of the slab
//                 double t1 = (box.min()[i] - segment.a[i]) * invDir;
//                 double t2 = (box.max()[i] - segment.a[i]) * invDir;

//                 // Ensure t1 is the entry point (smaller) and t2 is the exit point (larger)
//                 if (t1 > t2) {
//                     std::swap(t1, t2);
//                 }

//                 // Narrow the intersection interval based on the current slab
//                 tmin = std::max(tmin, t1);
//                 tmax = std::min(tmax, t2);

//                 if (tmin > tmax) {
//                     return false;
//                 }
//             }

//             return true;
//         }


//         // Returns true if the segment is NOT blocked by any AABB in the list
//         bool BruteForceVisibilityCheck(const Segment& segment)
//         {
//             // Precompute direction and inverse direction for the segment
//             // Optimization: Calculate this once instead of for every box
//             Vector3d dir = segment.b - segment.a;
//             Vector3d invDir;
//             for (int i = 0; i < 3; ++i) {
//                 invDir[i] = 1.0 / dir[i];
//             }

//             for (const auto& box : boxes_) {
//                 // Segment parameter range is always [0, 1]
//                 double tmin = 0.0;
//                 double tmax = 1.0;

//                 bool hit = true;
//                 // Check intersection with the current box using the Slab Method
//                 for (int i = 0; i < 3; ++i) {
//                     double t1 = (box.min()[i] - segment.a[i]) * invDir[i];
//                     double t2 = (box.max()[i] - segment.a[i]) * invDir[i];

//                     if (t1 > t2) {
//                         std::swap(t1, t2);
//                     }

//                     tmin = std::max(tmin, t1);
//                     tmax = std::min(tmax, t2);

//                     if (tmin > tmax) {
//                         hit = false;
//                         break;
//                     }
//                 }

//                 if (hit) {
//                     return false; // Blocked by this box
//                 }
//             }

//             return true; // Not blocked by any box (Visible)
//         }

//         // Throws an error as it is not implemented yet
//         bool GPUBFVisibilityCheck(const Segment& segment)
//         {
//             throw std::runtime_error("GPUBF unimplemented");
//         }

//         // Throws an error as it is not implemented yet
//         bool KDTreeVisibilityCheck(const Segment& segment)
//         {
//             throw std::runtime_error("KDTree unimplemented");
//         }

// };


// // #endif
