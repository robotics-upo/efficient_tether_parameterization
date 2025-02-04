#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <dbscan_line/dbscan_lines.h>
#include "catenary_checker/obstacle_2d.hpp"
#include "catenary_checker/2d_projection.hpp"
#include "catenary_checker/scenario.hpp"
#include "parabola.hpp"

#ifndef __CATENARY_CHECKER_LIB__
#define __CATENARY_CHECKER_LIB__

typedef std::pair<Point2D, Point2D> TwoPoints;

//! @brief Checks for the existence of a Catenary between two 3D points with 2D obstacles
//! @param A First point
//! @param B Second point
//! @param scenario The 2D scenario
//! @return The length of the catenary, or -1 if not found
float checkCatenary(const pcl::PointXYZ &A, const pcl::PointXYZ &B, const Scenario &scenario);

//! @brief Checks for the existence of a Catenary between two 3D points in a PC
//! @param A First point
//! @param B Second point
//! @param pc Point cloud where the obstacles will be obtained
//! @param plane_dist Maximum distance from an obstacle to the plane to be included
//! @param dbscan_min_points Minimum points that clusters consist of
//! @param dbscan_epsilon Max. distance between points in a cluster
//! @return The length of the catenary, or -1 if not found
float checkCatenary(const pcl::PointXYZ &A, const pcl::PointXYZ &B, const pcl::PointCloud<pcl::PointXYZ> &pc, float plane_dist, int dbscan_min_points, float dbscan_epsilon);

//! @brief Gets the 3D points of a parabola
//! @param parabola The parabola to get the points
//! @param A First point of the parabola
//! @param B End point of the parabola
//! @param par_points Output: The 3D points of the parabola
//! @param delta_t: spacing in the 2D x coordinate
//! @return The parabola length
float getParabolaPoints(Parabola &parabola, const pcl::PointXYZ &A, const pcl::PointXYZ &B, pcl::PointCloud<pcl::PointXYZ> &par_points, float delta_t=0.05);

float getParabolaPoints(Parabola &parabola, const Point3D &A, const Point3D &B, 
                      std::vector<geometry_msgs::Point> &points, float delta_t = 0.05f);

//! @brief Deprecated: Makes a preprocess of several planes in a given (x,y) position (UGV fixed)
//! @param A First point (fixed)
//! @param pc Point cloud where the obstacles will be obtained
//! @param n_planes Number of planes to be calculated
//! @param plane_dist Maximum distance from an obstacle to the plane to be included
//! @param obstacles Obstacles in the environment
//! @return The length of the catenary, or -1 if not found
std::vector<Scenario> preprocessObstacle2D(const pcl::PointXYZ &A, const pcl::PointCloud<pcl::PointXYZ> &pc, int n_planes, float plane_dist, int dbscan_min_points, float dbscan_epsilon);

// Clustering and 2D Obstacles related
std::shared_ptr<Scenario> PC2Obstacles(const pcl::PointXYZ &A, const pcl::PointXYZ &B,const pcl::PointCloud<pcl::PointXYZ> &pc, float plane_dist, int dbscan_min_points, float dbscan_epsilon);

DBSCAN *clusterize(const pcl::PointCloud<pcl::PointXY> &pc_2d, int minPts, float epsilon);

DBSCAN *clusterize_lines(const pcl::PointCloud<pcl::PointXY> &cloud_2d_in, int minPts,
			 float epsilon, float gamma, float theta);

std::shared_ptr<Scenario> getObstacles(DBSCAN *dbscan, const PlaneParams &p);

Obstacle2D toObstacle(const std::vector<Point> &obs);

#endif
