/**
 * @file Benchmark
 * @author Kevin Fedyna
 * @version 1.1.0
 * 
 * @brief Allows to benchmark different features for GeoPolyMesh.
 */


////////////////////////////////////////////////////////////////////////////////


// Basic includes

#include <vector>
#include <random>
#include <chrono>
#include <iostream>
#include <functional>

// CGAL includes

#include <CGAL/point_generators_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Triangulation_vertex_base_with_info_3.h>

// Geogram includes

#include <geogram/delaunay/periodic_delaunay_3d.h>


////////////////////////////////////////////////////////////////////////////////


/** @brief Predicates kernel for CGAL */
typedef CGAL::Exact_predicates_inexact_constructions_kernel EPICK;
/** @brief Triangulation vertex base */
typedef CGAL::Triangulation_vertex_base_with_info_3<int, EPICK> VertexBase;
/** @brief Triangulation data structure */
typedef CGAL::Triangulation_data_structure_3<VertexBase, CGAL::Triangulation_ds_cell_base_3<void>, CGAL::Parallel_tag> IndexedDataStructure;
/** @brief 3D Delaunay implemented in CGAL */
typedef CGAL::Delaunay_triangulation_3<EPICK, IndexedDataStructure> CGALDelaunay;
/** @brief CGAL class for Points */
typedef CGALDelaunay::Point Point;
/** @brief CGAL vertex informations */
typedef CGALDelaunay::Vertex::Info Info;
/** @brief CGAL class for Verteces pointers */
typedef CGALDelaunay::Vertex Vertex;
/** @brief CGAL class for Verteces pointers */
typedef CGALDelaunay::Vertex_handle VertexHandle;
/** @brief CGAL class for fancy Points */
typedef std::pair<Point, Info> IndexedPoint;

/** @brief Alias for PRG in sphere */
typedef CGAL::Random_points_in_sphere_3<Point> PRG;

/** @brief 3D Delaunay implemented in Geogram */
typedef GEO::PeriodicDelaunay3d GEODelaunay;


////////////////////////////////////////////////////////////////////////////////


namespace Benchmark {
    /**
     * @brief Compute in-function time for void functions
     * @param[in] callback The callback function
     * @param[in] repetitions The number of repetitions wanted
     * @returns The average in-function time
     * 
     * @example
     * time([]() {
     *     a = RandomMatrix();
     *     b = RandomMatrix();
     *     std::cout << (a * b);
     * }, 100)
     */
    long long time(std::function<void()> callback, int repetitions=1) {
        long long finalTime = 0;

        for (int i = 0; i < repetitions; ++i) {
            std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();

            callback();
            
            std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
            finalTime += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / repetitions;
        }

        return finalTime;
    }

    /**
     * @brief Create random points for CGAL representation
     * @param[in,out] prg The pseudo random generator
     * @param[in] count The number of points to generate
     * @param[out] CGALPoints The vector to store the points into
     */
    void createRandomPoints(PRG prg, const int count, std::vector<Point>& CGALPoints) {
        for (int i = 0; i < count; ++i) {
            Point p = *prg++;

            CGALPoints.push_back(p);
        }
    }

    /**
     * @brief Create random points for CGAL representation
     * @param[in,out] prg The pseudo random generator
     * @param[in] count The number of points to generate
     * @param[out] CGALPoints The vector to store the points into
     */
    void createRandomPoints(PRG prg, const int count, std::vector<IndexedPoint>& CGALPoints) {
        for (int i = 0; i < count; ++i) {
            Point p = *prg++;
            IndexedPoint point = IndexedPoint(p, i);

            CGALPoints.push_back(point);
        }
    }

    /**
     * @brief Create random points for Geogram representation
     * @param[in,out] prg The pseudo random generator
     * @param[in] count The number of points to generate
     * @param[out] GEOPoints The vector to store the points into
     */
    void createRandomPoints(PRG prg, const int count, std::vector<double>& GEOPoints) {
        for (int i = 0; i < count; ++i) {
            Point p = *prg++;

            GEOPoints.push_back(p.x());
            GEOPoints.push_back(p.y());
            GEOPoints.push_back(p.z());
        }
    }

    /**
     * @brief Create random points for CGAL and Geogram representations
     * @param[in,out] prg The pseudo random generator
     * @param[in] count The number of points to generate
     * @param[out] CGALPoints The vector to store the points into
     * @param[out] GEOPoints The vector to store the points into
     */
    void createRandomPoints(PRG prg, const int count, std::vector<Point>& CGALPoints, std::vector<double>& GEOPoints) {
        for (int i = 0; i < count; ++i) {
            Point p = *prg++;

            CGALPoints.push_back(p);

            GEOPoints.push_back(p.x());
            GEOPoints.push_back(p.y());
            GEOPoints.push_back(p.z());
        }
    }
}


////////////////////////////////////////////////////////////////////////////////


/**
 * @brief The main function
 * @param[in] argc The number of arguments given in command line.
 * @param[in] argv The arguments list separated by spaces.
 * @returns The program state.
 */
int main(int argc, char *argv[]) {
    GEO::initialize();
    PRG rnd;

    typedef std::vector<VertexHandle> PTVMap;
    typedef std::unordered_map<VertexHandle, std::vector<int>> VTPMap;
    
    for (double p = 1 ; p < 7 ; p += 0.1) {
        CGALDelaunay delaunay = new CGALDelaunay();
        CGALDelaunay delaunay_steps = new CGALDelaunay();

        unsigned int points_number = pow(10, p);
        
        std::vector<Point> CGALPoints;
        Benchmark::createRandomPoints(rnd, points_number, CGALPoints);

        std::vector<IndexedPoint> CGALIndexedPoints;
        Benchmark::createRandomPoints(rnd, points_number, CGALIndexedPoints);
        
        PTVMap ptv_delaunay = PTVMap();
        ptv_delaunay.reserve(points_number);

        PTVMap ptv_delaunay_steps = PTVMap();
        ptv_delaunay_steps.reserve(points_number);

        VTPMap vtp_delaunay = VTPMap();
        VTPMap vtp_delaunay_steps = VTPMap();
        
        // ------------------ CGAL One time
        std::cout << Benchmark::time([&]() {
            delaunay.insert(CGALIndexedPoints.begin(), CGALIndexedPoints.end());

            auto handle_range = delaunay.all_vertex_handles();

            for (auto iterator = handle_range.begin(); iterator != handle_range.end(); iterator++) {
                VertexHandle handle = *iterator;
                int bound_index = handle->info();

                // Throw out verteces null vertex that can pop with (negative|absurdly large) index
                if (bound_index < 0 || bound_index >= points_number) continue;

                ptv_delaunay[bound_index] = handle;

                if (vtp_delaunay.find(handle) != vtp_delaunay.end()) {
                    vtp_delaunay[handle].push_back(bound_index);
                } else {
                    vtp_delaunay[handle] = std::vector<int>(1, bound_index);
                }
            }
        }) << ";";

        // ------------------ CGAL One by one
        std::cout << Benchmark::time([&]() {
            for (int i = 0; i < CGALPoints.size(); ++i) {
                VertexHandle matching_vertex = delaunay_steps.insert(CGALPoints[i]);

                ptv_delaunay_steps[i] = matching_vertex;

                if (vtp_delaunay_steps.find(matching_vertex) != vtp_delaunay_steps.end()) {
                    vtp_delaunay_steps[matching_vertex].push_back(i);
                } else {
                    vtp_delaunay_steps[matching_vertex] = std::vector<int>(1, i);
                }
            }
        }) << std::endl;
    }
    
    return 0;
}
