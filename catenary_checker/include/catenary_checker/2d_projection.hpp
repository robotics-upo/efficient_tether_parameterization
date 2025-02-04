#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <dbscan_line/dbscan_lines.h>
#include "catenary_checker/obstacle_2d.hpp"
#include "parabola.hpp"
#include <geometry_msgs/Point.h>
#include "yaml-cpp/yaml.h"

#ifndef __2D_PROYECTION_LIB__
#define __2D_PROYECTION_LIB__


// ----------- Plane projection related classes / functions -----------

class PlaneParams {
public:
  float a, b, c, d;

  PlaneParams() {
    a = b = c = d = 0.0;
  }

  PlaneParams(const YAML::Node &e) {
    a = e[0].as<float>();
    b = e[1].as<float>();
    c = e[2].as<float>();
    d = e[3].as<float>();
  }

  inline float getSignedDistance(const pcl::PointXYZ &p) const {
    return a*p.x + b*p.y + c*p.z + d;
  }

  inline std::string toString() const {
    std::ostringstream os;

    os << "Plane: " << a << "·x + " << b << "·y + " << c << "·z + " << d << "= 0";

    return os.str();
  }

  inline geometry_msgs::Point project3D_p(const Point2D &p) {
    geometry_msgs::Point ret;
    ret.x = -p.x * b - a * d;
    ret.y = p.x * a - b * d;
    ret.z = p.y;

    return ret;
  }

  inline pcl::PointXYZ project3D(const Point2D &p) const {
    pcl::PointXYZ p_3d(-p.x * b - a * d,
                       p.x * a - b * d,
                       p.y);
    return p_3d;
  }

  inline Point2D project2D(const pcl::PointXYZ &p) const {
    return Point2D(p.y * a - p.x * b, p.z);
  }

  inline Point2D project2D(const Point3D &p) const {
    return Point2D(p.y * a - p.x * b, p.z);
  }

};

inline YAML::Emitter& operator << (YAML::Emitter &out, const PlaneParams &p) {
  out << YAML::Flow;
  out << YAML::BeginSeq << p.a << p.b << p.c << p.d << YAML::EndSeq;

  return out;
}

pcl::PointCloud<pcl::PointXY> project2D(const pcl::PointCloud<pcl::PointXYZ> &cloud_in,
					const pcl::PointXYZ &p1, const pcl::PointXYZ &p2, 
					const float max_dist, const float min_z = 0.1);

std::vector<pcl::PointCloud<pcl::PointXY> > project2D_theta(const float theta, const pcl::PointCloud<pcl::PointXYZ> &cloud_in,
					const pcl::PointXYZ &min, const pcl::PointXYZ &max, PlaneParams &p, const float dist_planes, const float min_z = 0.1f);


PlaneParams getVerticalPlane(const pcl::PointXYZ &p1, const pcl::PointXYZ &p2);

inline PlaneParams getVerticalPlane(double theta) {
  PlaneParams p;
  p.c = p.d = 0.0;
  p.a = -sin(theta);
  p.b = cos(theta);

  return p;
}

// 3D Reprojection

pcl::PointCloud<pcl::PointXYZ> reproject3D(const pcl::PointCloud<pcl::PointXY> &cloud_2d_in,
                                           const pcl::PointXYZ &p1, const pcl::PointXYZ &p2);

#endif
