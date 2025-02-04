#include <catenary_checker/preprocessed_scenario.hpp>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <filesystem>
#include <pcl_conversions/pcl_conversions.h>
#include <chrono>

using namespace pcl;
using namespace std;
using namespace std::chrono;

PreprocessedScenario::PreprocessedScenario() {
  ros::NodeHandle nh, pnh("~");
  
  _pub_marker = nh.advertise<visualization_msgs::MarkerArray>("problems", 2, true);

  pnh.param("ws_x_min", _min.x, 0.0f);
  pnh.param("ws_y_min", _min.y, 0.0f);
  pnh.param("ws_x_max", _max.x, 10.0f);
  pnh.param("ws_y_max", _max.y, 10.0f);
  pnh.param("ws_z_max", _max_z, 10.0f);
  pnh.param("n_theta", _n_theta, 30);
  pnh.param("plane_dist", _plane_dist, 0.2f);

  pnh.param("db_min_points", _db_min_points, 20);
  pnh.param("db_epsilon", _db_epsilon, 0.05f);

  ROS_INFO("Started Preprocessed Scenario from ROS. ");
  ROS_INFO("min = %s \t max = %s", _min.toString().c_str(),_max.toString().c_str());
  ROS_INFO("n_theta = %d\tplane_dist = %f", _n_theta, _plane_dist);
}

PreprocessedScenario::PreprocessedScenario(const std::string &filename) {
  ros::NodeHandle nh, pnh("~");

  _pub = nh.advertise<sensor_msgs::PointCloud2>("slices", 1, true);
  _pub_marker = nh.advertise<visualization_msgs::MarkerArray>("scenarios", 2, true);
  _sub = nh.subscribe("point_cloud", 1, &PreprocessedScenario::PC_Callback, this);

  _filename = filename;


  if (!loadScenario(filename)) {
    pnh.param("ws_x_min", _min.x, 0.0f);
    pnh.param("ws_y_min", _min.y, 0.0f);
    pnh.param("ws_x_max", _max.x, 10.0f);
    pnh.param("ws_y_max", _max.y, 10.0f);
    pnh.param("ws_z_max", _max_z, 10.0f);
    pnh.param("n_theta", _n_theta, 60);
    pnh.param("plane_dist", _plane_dist, 0.1f);

    pnh.param("db_min_points", _db_min_points, 20);
    pnh.param("db_epsilon", _db_epsilon, 0.05f);

    ROS_INFO("Started Preprocessed Scenario from ROS. ");
    ROS_INFO("min = %s \t max = %s", _min.toString().c_str(),_max.toString().c_str());
    ROS_INFO("n_theta = %d\tplane_dist = %f", _n_theta, _plane_dist);
  } else {
    ROS_INFO("PreprocessedScenario::Got the scenarios from file %s. Publishing.", filename.c_str());
    sleep(2);
  }
}

void PreprocessedScenario::precompute(const PointCloud<PointXYZ> &pc) {
  // We have to sample only Pi (one plane goes to a direction and its opposite)
  auto st = chrono::system_clock::now();
  
  const float increment = M_PI / static_cast<float>(_n_theta + 1);

  _scenarios.clear();
  // First we sample the angle

  auto pc_filtered = filterHeight(pc, 0.2);

  pcl::PointXYZ min, max;
  min.x = _min.x; min.y = _min.y; min.z = 0.0;
  max.x = _max.x; max.y = _max.y; max.z = _max_z;

  _scenarios.resize(_n_theta);
  

  #pragma omp parallel for num_threads(24) shared(_scenarios) 
  for (int i = 0; i < _n_theta; i++) {
    PlaneParams p;
    float angle = i * increment - M_PI * 0.5;
    std::vector <std::shared_ptr<Scenario> > planes;
    auto vec_points_2d = project2D_theta(angle, pc, min, max, p, _plane_dist);

    planes.resize(vec_points_2d.size());
    size_t s = vec_points_2d.size();
    double min_d = p.d;
    for (int i = 0; i < s ; i++) {
      PlaneParams curr_plane = p;
      curr_plane.d = min_d - i * _plane_dist;
      simplifyCloud(vec_points_2d[i]);
      auto dbscan = clusterize(vec_points_2d[i], _db_min_points, _db_epsilon);
      planes[i] = getObstacles(dbscan, curr_plane);
      
    }
    _scenarios[i] = planes;
    
  }
  #pragma omp barrier

  // #pragma omp barrier  
  
  
  auto end = chrono::system_clock::now();
  duration<float, std::milli> duration = end - st;
  ROS_INFO("Precomputed scenarios. Expended time: %f s", duration.count() * milliseconds::period::num / milliseconds::period::den);
  ROS_INFO("Scenarios statistics: %s", getStats().c_str() );
}

void PreprocessedScenario::simplifyCloud(pcl::PointCloud<pcl::PointXY> &c, double min_dist) {
  float min_x = 1e20;
  float min_y = 1e20;
  float max_x = -1e20;
  float max_y = -1e20;
  size_t s = c.points.size();
  if (s == 0) {
    return;
  }
  for (auto &x:c) {
    min_x = std::min(min_x, x.x);
    max_x = std::max(max_x, x.x);
    min_y = std::min(min_y, x.y);
    max_y = std::max(max_y, x.y);
  }

  float d_1 = 1.0 / min_dist;

  int rows = (max_y - min_y) * d_1 + 1; 
  int cols = (max_x - min_x) * d_1 + 1;

  std::vector<std::vector <bool> > occupied(rows);

  for (auto &x:occupied) {
    x.resize(cols);
    for (int i = 0; i < cols; i++) {
      x[i] = false;
    }
  }

  std::vector<int> to_be_cleared;
  to_be_cleared.reserve(s);
  for (int curr = 0; curr < s; curr++) {
    int j = (c[curr].x - min_x) * d_1;
    int i = (c[curr].y - min_y) * d_1;

    if (!occupied[i][j])
      occupied[i][j] = true;
    else
      to_be_cleared.push_back(curr);
  }

  for (int i = to_be_cleared.size() - 1; i >= 0; i--) {
    auto x = c.points.begin();
    x += to_be_cleared[i];
    
    c.points.erase(x);
  }
}

//! @brief Gets the problems to sample the workspace with vertical planes in the theta direction
//! @param theta The direction (x axis)
//! @param min Minimum x and y
//! @param max Maximum x and y
//! @param dist Distance between planes
vector<TwoPoints> PreprocessedScenario::getProblemsTheta(double theta) const {
  vector<TwoPoints> ret;

  double delta_x = _max.x - _min.x;
  double delta_y = _max.y - _min.y;
  double diag = sqrt(delta_x*delta_x + delta_y*delta_y);

  double s_theta = sin(theta);
  double c_theta = cos(theta);

  Point2D S, E, A, B; // Starting point
  S = _min;
  E = _max;
  if (theta >= 0.0) {
    S.y = _max.y;
    E.y = _min.y;
  }
  double gamma = atan2(delta_x, delta_y); // We want the complementary angle (hence x,y)
  int n_planes = round(diag * cos(gamma - fabs(theta)) / _plane_dist);
  double inc_c = _plane_dist;
  if (!signbit(theta) || theta == 0.0) {
    // In these cases we should go the other way round
    inc_c *= -1.0;
  }
  double curr_c = -s_theta * S.x + c_theta * S.y + 0.5 *inc_c;
  
  for (int i = 0; i < n_planes; i++, curr_c += inc_c) {
    // Get the y_c for x_min
    A.x = S.x;
    A.y = (curr_c + s_theta * S.x) / c_theta;
    if (A.y < _min.y || A.y > _max.y || isnan(A.y)) {
      A.y = E.y;
      A.x = (c_theta * E.y - curr_c) / s_theta;
    }

    // Get the B.x for E.y
    B.y = S.y;
    B.x = (c_theta * B.y - curr_c) / s_theta;
    if (B.x < _min.x || B.x > _max.x || isnan(B.x)) {
      B.x = E.x;
      B.y = (curr_c + s_theta * E.x) / c_theta;
    }
    ret.push_back(TwoPoints(A,B));
  }

  return ret;
}

float PreprocessedScenario::checkCatenary(const pcl::PointXYZ &A, const pcl::PointXYZ &B,  std::vector<geometry_msgs::Point> &pts_c_, bool debug)
{
  double inc_x = B.x - A.x;
  double inc_y = B.y - A.y;

  if (fabs(inc_x) < 1e-4 && fabs(inc_y) < 1e-4) {
    inc_x = 1.0; // Avoid degenerate cases (same 2D point )
  }

  double yaw = atan(inc_y/inc_x);
  float ret_val = -1.0;

  // Get the theta
  int n = round((yaw + M_PI * 0.5) / M_PI * (_n_theta));
  n = n % _n_theta;

  // Then get the plane
  const auto &scenes = _scenarios[n];

  if (scenes.size() > 0) {
    const Scenario &s_ = *scenes[0];
    float min_dist = fabs(s_.plane.getSignedDistance(A));
    int i = 0;
    int j = round(fabs(min_dist) / _plane_dist);
    
    const Scenario &scen = *scenes[j];
    _last_plane = scen.plane;
    _pa = scen.to2D(A);
    _pb = scen.to2D(B);
    if (fabs(_pa.x - _pb.x) < 1e-3) {
      ROS_ERROR("PreprocessedScenario::checkCatenary: Warning. Points too close. _pa: %s. pb: %s. yaw = %f, n = %d",
                _pa.toString().c_str(), _pb.toString().c_str(), yaw, n);
    }
    _parabola.reset();
    if (debug) {
      float intensity = 1.0f;
      _pub.publish(scen.toPC("map", j, intensity));
      _pub_marker.publish(scen.toMarkerArray("map", j));

      _pub_marker.publish(getMarkerProblem(A, B));
      



      ROS_INFO("PreprocessedScenario::checkCatenary. N theta = %d, Yaw = %f. n = %d, j = %d. Scenario size: %lu", _n_theta, yaw, n, j, scen.size());
      ROS_INFO("PreprocessedScenario::checkCatenary. Projected points: A = %s. B = %s", _pa.toString().c_str(), _pb.toString().c_str());
      ROS_INFO("Scenario: %s", scen.toString().c_str());
    }

    if (_parabola.approximateParabola(scen, _pa, _pb)) {
      auto points_2d = _parabola.getPoints(_pa.x, _pb.x, 0.1);

      pts_c_.clear();
      pcl::PointXYZ p;
      geometry_msgs::Point gp;
      for (auto &x: points_2d) {
        
        p = scen.to3D(x);
        gp.x = p.x; gp.y = p.y; gp.z = p.z;

        if (debug) {
          ROS_INFO("Adding parabola point. %f %f %f", p.x, p.y, p.z);
        }

        pts_c_.push_back(gp);
      }
      ret_val = _parabola.getLength(_pa.x, _pb.x);
      if (debug) 
        ROS_INFO("PreprocessedScenario::checkCatenary --> could get the parabola %s. Length = %f. Parabola points: %lu", 
                 _parabola.toString().c_str(), ret_val, points_2d.size());

    } else {
      if (debug)
        ROS_INFO("PreprocessedScenario::checkCatenary --> could NOT get the parabola.");
    }
  }

  return ret_val;
}

namespace fs = std::filesystem;

// Function that exports scenarios --> They are split into several files, one for each plane, not just one, and then compress it!
bool PreprocessedScenario::exportScenario() const {
  bool ret_val = true;

  // Ensure that we have a proper scenario
  if (_scenarios.size() != _n_theta) {
    return false;
  }


  fs::path original_path = (fs::current_path());
  fs::current_path(fs::temp_directory_path());

  string name = getFilename(_filename);

  if (fs::is_directory(name)) {
    fs::remove_all(name);
  }
  if (fs::create_directories(name)) {
    fs::current_path(name);
     int i = 0;
    // Export the metadata
    ofstream metadata(string(name + ".yaml").c_str());
    YAML::Emitter e;
    e << YAML::BeginMap;
    e << YAML::Key << "min" << YAML::Value << _min;
    e << YAML::Key << "max" << YAML::Value << _max;
    e << YAML::Key << "max_z" << YAML::Value << _max_z;
    e << YAML::Key << "n_theta" << YAML::Value << _n_theta;
    e << YAML::Key << "plane_dist" << YAML::Value << _plane_dist;
    e << YAML::Key << "db_min_points" << YAML::Value << _db_min_points;
    e << YAML::Key << "db_epsilon" << YAML::Value << _db_epsilon;
    metadata << e.c_str() << endl;
    metadata.close();

    try {
      // Save the scenarios
      for (int i = 0; i < _n_theta; i++) {
        string curr_dir = to_string(i);
        fs::create_directory(curr_dir);
        fs::current_path(curr_dir);
        auto &curr_vec = _scenarios[i];
        int j = 0;

        for (auto &x:curr_vec) {
          string curr_file = to_string(j++);
          ofstream ofs(curr_file);
          YAML::Emitter e;
          e << *x;
          ofs << e.c_str()<<endl;
        }
        fs::current_path("..");  
      }
    } catch(ios_base::failure &e) {
      cerr << "Could not create: " << _filename << "\t Failure:" << e.what() << endl;
    }
  }
  fs::current_path(fs::temp_directory_path());


  ostringstream oss;
  oss << "tar czf " << _filename << " " << name;
  ROS_INFO("Executing: %s ", oss.str().c_str());

  if (system(oss.str().c_str()) == 0) {
    ROS_INFO("File %s created successfully", _filename.c_str());
  }

  fs::current_path(original_path);

  return ret_val;
}

bool PreprocessedScenario::loadScenario(const std::string &file) {
  _scenarios.clear();

  bool ret_val = true;

  fs::path original_path = fs::current_path();
  fs::current_path(fs::temp_directory_path());

  ostringstream oss;
  oss << "tar xf " << file;
  cout << "Executing command: \"" << oss.str() << "\"" << endl;

  if (system(oss.str().c_str()) == 0) {
    ROS_INFO("File %s decompressed OK. ", file.c_str());
  } else {
    ROS_INFO("Could not decompress %s. Calculating scenarios.", file.c_str());
    return false;
  }

  string f = getFilename(file); // gets the filename without .tar.gz and without /
  ROS_INFO("Filename: %s", f.c_str());

  fs::current_path(f);

  if (getMetadata(f+".yaml")) {
    for (int i = 0; i < _n_theta; i++) {
      vector<std::shared_ptr<Scenario> >  curr_vec;
      fs::current_path(std::to_string(i));
      
      // Get number of scenarios
      int n_scen = 0;
      for (;fs::is_regular_file(std::to_string(n_scen)); n_scen++) {

      }
      curr_vec.resize(n_scen);

      int j = 0;

      #pragma omp parallel for num_threads(24) shared(curr_vec) 
      for (j = 0; j < n_scen; j++) {
        std::shared_ptr<Scenario> s(new Scenario);
        if (s->loadScenario(std::to_string(j))) {
          curr_vec[j] = s;
        }
      }
      #pragma omp barrier
      
      ROS_INFO("Loaded angle %d. Number of scenarios %d", i, static_cast<int>(curr_vec.size()));
      _scenarios.push_back(curr_vec);
      fs::current_path("..");
    }
    ROS_INFO("Loaded scenario. Number of angles: %d", _n_theta);
  } else {
    ret_val = false;
  }

  fs::current_path(original_path);

  return ret_val;
}

bool PreprocessedScenario::getMetadata(const std::string &f) {
  bool ret_val = true;

  try {
    ifstream ifs(f.c_str());

    YAML::Node y = YAML::Load(ifs);

    _min = Point2D(y["min"]);
    _max = Point2D(y["max"]);
    _max_z = y["max_z"].as<float>();
    _plane_dist = y["plane_dist"].as<float>();
    _n_theta = y["n_theta"].as<int>();
    _db_min_points = y["db_min_points"].as<int>();
    _db_epsilon = y["db_epsilon"].as<float>();
  } catch (exception &e) {
    cerr << "PreprocessedScenario::getMetadata --> exception when loading metadata.";
    cerr << "Content: " << e.what() << endl;
    ret_val = false;
  }

  return ret_val;
}

string PreprocessedScenario::getFilename(const string &f) const {
  string ret;

  auto start_ = f.rfind('/');
  auto end_ = f.rfind(".tar.gz");

  if (start_ == std::string::npos) {
    start_ = 0;
  } else {
    start_++;
  }

  if (end_ == std::string::npos) {
    end_ = f.size();
  }

  return f.substr(start_, end_ - start_);
}

void PreprocessedScenario::PC_Callback(const sensor_msgs::PointCloud2::ConstPtr &pc) {
  ROS_INFO("Received PointCloud. Height, Width: %d, %d", pc->height, pc->width);
  _pc = pc;  
  if (_scenarios.size() > 0) {
    return; // ALready computed
  }

  pcl::fromROSMsg(*_pc, _pcl_pc);
  precompute(_pcl_pc);

  ROS_INFO("Exporting scenario. Filename %s", _filename.c_str());
  exportScenario();
}

void PreprocessedScenario::publishScenarios(unsigned int scen) {
  if (_scenarios.size() > scen) {
    ROS_INFO("Publishing scenario set %u.", scen);

    int i = 0;
    int total = _scenarios[scen].size();
    for (auto &x:_scenarios[scen]) {
      float intensity = 0.5f + 0.5f/(static_cast<float>(i) / static_cast<float>(total));
      _pub.publish(x->toPC("map", i, intensity));
      _pub_marker.publish(x->toMarkerArray("map", i++));
    }
  }
}

visualization_msgs::MarkerArray PreprocessedScenario::getMarkerProblem(const pcl::PointXYZ &A, const pcl::PointXYZ &B) {
  visualization_msgs::MarkerArray marks;
  visualization_msgs::Marker marker;

  static int seq = 0;

  //Modifier to describe what the fields are.
  marker.header = std_msgs::Header();
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = "map";
  marker.header.seq = seq;

  marker.id = 0;
  marker.ns = "decision problem";
  marker.action = visualization_msgs::Marker::ADD;

  // First the point in the origin
  marker.type = visualization_msgs::Marker::SPHERE_LIST;
  marker.scale.x = marker.scale.y = marker.scale.z = 0.2;
  marker.pose.orientation.w = 1; 
  marker.color = getColor(seq);
  
  geometry_msgs::Point g_p;
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.w = 1.0;
  g_p.x = A.x;
  g_p.y = A.y;
  g_p.z = A.z;
  marker.points.push_back(g_p);
  marks.markers.push_back(marker);

  marker.points.clear();
  marker.header.seq++;
  marker.id = 1;
  g_p.x = B.x;
  g_p.y = B.y;
  g_p.z = B.z;
  marker.color = getColor(seq+1);
  marker.points.push_back(g_p);

  marks.markers.push_back(marker);

  return marks;
}

visualization_msgs::MarkerArray problemsToMarkerArray(const std::vector<TwoPoints> v, std::string frame_id = "map", int skip = 9) {
  visualization_msgs::MarkerArray msg;
  visualization_msgs::Marker marker;

  static int seq = 0;

  //Modifier to describe what the fields are.
  marker.header = std_msgs::Header();
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = frame_id;
  marker.header.seq = 0;

  marker.id = 0;
  marker.ns = "problems";
  marker.action = visualization_msgs::Marker::ADD;

  // First the point in the origin
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.scale.x = marker.scale.y = 1.0; 
  marker.scale.z = 1.0;
  marker.pose.orientation.z = 0;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.color = getColor(seq++);
  msg.markers.push_back(marker);

  geometry_msgs::Point g_p;
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;

  g_p.z = 1;
  
  // Add the lines
  marker.points.resize(v.size() * 2);
  int i = 0;
  int n = 0;
  for (const TwoPoints &o:v) {

    if (n++ < skip) 
      continue;

    n = 0;
    g_p.x = o.first.x;
    g_p.y = o.first.y;
    
    marker.points[i++] = g_p;
    g_p.x = o.second.x;
    g_p.y = o.second.y;
    marker.points[i++] = g_p;
  }
  msg.markers.push_back(marker);

  return msg;
}


void PreprocessedScenario::publishProblems(unsigned int scen) {
  scen = scen % _n_theta;
  float theta = M_PI / static_cast<float>(_n_theta + 1) * scen - M_PI * 0.5;
  
  ROS_INFO("Publishing scenario set %u.", scen);

  auto problems = getProblemsTheta(theta);

  _pub_marker.publish(problemsToMarkerArray(problems));
}

PointCloud<PointXYZ> PreprocessedScenario::filterHeight(const PointCloud<PointXYZ> &pc, const float min_height) {
  PointCloud<PointXYZ> ret;
  ret.reserve(pc.size());
  for (const auto &x:pc) {
    if (x.z > min_height) {
      ret.push_back(x);
    }

  }

  return ret;
}
