#ifndef _RANDOM_UAV_PLANNER_HPP__
#define _RANDOM_UAV_PLANNER_HPP__

#include <math.h>
#include <random>

#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <octomap_msgs/Octomap.h> //Octomap Binary
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/filters/random_sample.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#include <pcl/filters/passthrough.h>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <Eigen/StdVector>

// #include "rrt_star_planners/near_neighbor.hpp"
#include "rrt_planners/RRTNode.h"
#include "rrt_planners/kdtree.hpp"
#include "rrt_planners/random_graph_markers.h"

#include "misc/catenary_solver_ceres.hpp"

#include "catenary_checker/near_neighbor.hpp"
#include "catenary_checker/grid3d.hpp"
#include "catenary_checker/bisection_catenary_3D.h"
#include "catenary_checker/catenary_checker_manager.h"

#define PRINTF_REGULAR "\x1B[0m"
#define PRINTF_RED "\x1B[31m"
#define PRINTF_GREEN "\x1B[32m"
#define PRINTF_YELLOW "\x1B[33m"
#define PRINTF_BLUE "\x1B[34m"
#define PRINTF_MAGENTA "\x1B[35m"
#define PRINTF_CYAN "\x1B[36m"
#define PRINTF_WHITE "\x1B[37m"
#define PRINTF_ORANGE  "\x1B[38;2;255;128;0m"
#define PRINTF_ROSE    "\x1B[38;2;255;151;203m"
#define PRINTF_LBLUE   "\x1B[38;2;53;149;240m"
#define PRINTF_LGREEN  "\x1B[38;2;17;245;120m"
#define PRINTF_GRAY    "\x1B[38;2;176;174;174m"

// Uncomment to set length catenary in nodes

namespace PathPlanners
{
typedef geometry_msgs::Vector3 Vector3;
typedef geometry_msgs::Quaternion Quaternion;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef trajectory_msgs::MultiDOFJointTrajectory Trajectory;
typedef visualization_msgs::Marker RVizMarker;

//*****************************************************************
// 				RandomPlanner Algoritm Class Declaration
//*****************************************************************

class RandomUAVPlanner
{
public:
	RandomUAVPlanner();

	/**
		  Initialization
		   @param planner name for topic names 
		   @param frame_id for debug markers 
		   @param simetric or asimetric workspace centered at (0,0,0) [meters]
		   @param NodeHandle 
		**/
	void init(std::string plannerName, std::string frame_id_, float ws_x_max_, float ws_y_max_, float ws_z_max_,
            float ws_x_min_, float ws_y_min_, float ws_z_min_, float step_, float h_inflation_, float v_inflation_,
            ros::NodeHandlePtr nh_, double goal_gap_m_, bool debug_rrt_, double distance_obstacle_uav_,
            double distance_catenary_obstacle_, Grid3d *grid3D_, bool nodes_marker_debug_,
            bool use_distance_function_, std::string map_file_, bool get_catenary_data_,
            std::string catenary_file_, bool use_parabola_, CatenaryCheckerManager *catenary_manager);

  ~RandomUAVPlanner();

  virtual int computeTree();      
  
  float getYawFromQuaternion(RRTNode n_);

  std::list<RRTNode*> nodes_tree; // TODO: single tree planners

  virtual void clearStatus();
	bool getGlobalPath(Trajectory &trajectory);

	/**
		  Set initial position of the path only check if 
		  it's inside WS but not if it's occupied
			@param 3D position data (Discrete or Continuous)
			@return false if is outside the workspace
		**/
	bool setInitialPositionIndependent(RRTNode n_);
	bool setInitialPosition(Vector3 p);

  inline bool setValidInitialPositionMarsupial(RRTNode n_)
	{
    delete disc_initial;
    disc_initial = new RRTNode(n_);
    disc_initial->parentNode = NULL; // Make sure this node has no parents
    if(!isInitialPositionUAVOccupied()){
      initial_position_ugv.x = disc_initial->point.x * step;
      initial_position_ugv.y = disc_initial->point.y * step;
      initial_position_ugv.z = disc_initial->point.z * step;
      initial_position_uav.x = disc_initial->point_uav.x * step;
      initial_position_uav.y = disc_initial->point_uav.y * step;
      initial_position_uav.z = disc_initial->point_uav.z * step;

      auto p_reel_ = getReelNode(*disc_initial);
      std::vector <geometry_msgs::Point> pts_cat;
      geometry_msgs::Point p_i;
      p_i.x = initial_position_uav.x; p_i.y = initial_position_uav.y; p_i.z = initial_position_uav.z;
      bool ret_val = ccm->searchCatenary(p_reel_, p_i, pts_cat);
      if (ret_val)
        disc_initial->length_cat = ccm->length_cat_final;


      ROS_INFO("RandomUAVPlanner: Initial Marsupial discrete position UGV [%d, %d, %d] and UAV [%d, %d, %d] OK", 
               n_.point.x, n_.point.y, n_.point.z, n_.point_uav.x, n_.point_uav.y, n_.point_uav.z);
      

      return ret_val;
    }
    else{
      ROS_WARN("RandomUAVPlanner: Cannot set initial position UAV Marsupial out of bounds!!");
    }
           
    return false;
  }


  inline bool setValidInitialPositionMarsupial(Vector3 p1,Vector3 p2, Quaternion q1, Quaternion q2)
	{
		RRTNode n_ ;
		DiscretePosition pp1 = discretizePosition(p1);
		DiscretePosition pp2 = discretizePosition(p2);
		
		n_.point.x = pp1.x;
		n_.point.y = pp1.y;
		n_.point.z = pp1.z;
		n_.rot_ugv.w = q1.w;
		n_.rot_ugv.x = q1.x;
		n_.rot_ugv.y = q1.y;
		n_.rot_ugv.z = q1.z;

		n_.point_uav.x = pp2.x;
		n_.point_uav.y = pp2.y;
		n_.point_uav.z = pp2.z;
		n_.rot_uav.w = q2.w;
		n_.rot_uav.x = q2.x;
		n_.rot_uav.y = q2.y;
		n_.rot_uav.z = q2.z;

		return setValidInitialPositionMarsupial(n_);
	}


	/**
		  Set final position of the path only check if 
		  it's inside WS but not if it's occupied
			@param 3D position data (Discrete or Continuous)
			@return false if is outside the workspace
		**/
  bool setFinalPosition(DiscretePosition p);
	bool setFinalPosition(Vector3 p);
  	/**
		  Set initial/final position of the path checking if 
		  it's inside WS and if it'ts occupied (--> Valid)
		   @param [x,y,z] discrete or continuous position
		   @return true if is a valid initial/final position and has been set correctly
		**/

	inline bool setValidFinalPosition(DiscretePosition p)
	{
		if (setFinalPosition(p))
		{
			if (!isFinalPositionOccupied())
			{
				ROS_INFO("RandomUAVPlanner: Final discrete position [%d, %d, %d] set correctly", p.x, p.y, p.z);
				return true;
			}
		}
		else
		{
			ROS_WARN("RandomUAVPlanner: Final position outside the workspace attempt!! [%d, %d, %d]", p.x, p.y, p.z);
		}

		return false;
	}
	
	inline bool setValidFinalPosition(Vector3 p)
	{
		DiscretePosition pp = discretizePosition(p);

		return setValidFinalPosition(pp);
	}
	
  inline bool isInside(Vector3 &v)
	{
		int x_ = v.x * step_inv;
		int y_ = v.y * step_inv;
		int z_ = v.z * step_inv;
		return isInside(x_, y_, z_);
	}

  /**
     Override actual occupancy matrix
     @param octomap msg
  **/
	void updateMap(octomap_msgs::OctomapConstPtr message);

	/**
		  Add a cloud to the actual occupancy matrix
		   @param pcl pointCloud 
		**/
	void updateMap(PointCloud cloud);
	/**
		 * 
		 * 
		**/
	void updateMap(const PointCloud::ConstPtr &map);

	/** 
		   Clear occupancy discrete matrix
		**/
	void clearMap();
	/** 
		   Clear occupancy discrete matrix reduced
		**/
	void clearMapReduced(size_t _size);
	/**
		  Publish via topic the discrete map constructed
		**/

	void configRRTParameters(double _l_m, geometry_msgs::Vector3 _p_reel,
                           geometry_msgs::Vector3 _p_ugv, 
                           int n_iter_, int n_loop_, double r_nn_,
                           double s_s_, int s_g_r_);

	/**
	   Receive whole PointCloud2 from map to get UAV obstacles 
	**/
	void readPointCloudMapForUAV(const sensor_msgs::PointCloud2::ConstPtr& msg);

	void setInitialCostGoal(RRTNode* n_);

	void clearNodesMarker();
	void clearCatenaryGPMarker();
	void clearLinesGPMarker();
	double getPointDistanceFullMap(bool use_distance_function, geometry_msgs::Vector3 p_);

	CatenaryCheckerManager *ccm;

	/** Variables **/
	float step; // Resolution of the Matrix and its inverse
	float step_inv;

	//Shearching Pyramid parameters
	geometry_msgs::Vector3 pos_reel_ugv, pos_tf_ugv;
	geometry_msgs::Vector3 new_start, new_goal;
	Eigen::Matrix3f base_sp;
	double angle_square_pyramid, max_theta_axe_reduced, sweep_range;
	double phi_min, phi_max, theta_min, theta_max ;

	pointVec v_nodes_kdtree, v_uav_nodes_kdtree;

	RRTNode *disc_initial, *disc_final; // Discretes
	RRTNode *disc_goal; // That node is fill it by the node that approach the goal in Independent configuration

  int n_intermediate = 6; // Number of steps to check in checkBetweenNodes

	std::vector<double> length_catenary;

	bool debug_rrt;
	bool use_distance_function; //Only related with tether and UAV distance

	std::string node_name, path_name;

	NearNeighbor nn_obs_uav;
	PlannerGraphMarkers rrtgm;
	Grid3d *grid_3D;
		
	std::ifstream file_time1, file_time2;
  std::ofstream ofs_time1, ofs_time2;
	std::string output_file_time_methods, output_file_time_solutions, map_file;
	std::string path;
	bool new_solution;
	std::vector<double> v_length_cat, v_min_dist_obs_cat, v_time_cat;
	bool get_catenary_data, use_parabola;
	std::string catenary_file;


protected:
	
	RRTNode getRandomNode(); 
	bool extendGraph(const RRTNode q_rand_);
	RRTNode* getNearestNode(const RRTNode q_rand_);
  bool steering(const RRTNode &q_nearest_, const RRTNode &q_rand_, float factor_steer_, RRTNode &q_new_);
	bool obstacleFreeBetweenNodes(const RRTNode q_nearest, const RRTNode q_new);
	std::vector<int> getNearNodes(const RRTNode &q_new_, double radius_);
	std::vector<float> getNearestUAVNode(const RRTNode &q_new_);
	void getOrientation(RRTNode &n_ , RRTNode p_);
	bool checkNodeFeasibility(const RRTNode pf_ );
	bool checkPointsCatenaryFeasibility(const RRTNode pf_);
	bool checkCatenary(RRTNode &q_init_, vector<geometry_msgs::Point> &points_catenary_);
	geometry_msgs::Point getReelNode(const RRTNode node_);
	geometry_msgs::Vector3 getReelTfInNode(const RRTNode &q_init_);
	void updateKdtreeNode(const RRTNode ukT_);
	void updateKdtreeUAV(const RRTNode ukT_);
	void getParamsNode(RRTNode &node_, bool is_init_= false);
	void updateParamsNode(RRTNode &node_);
	bool saveNode(RRTNode* sn_, bool is_init_=false);
	double costNode(const RRTNode q_new_);
	double costBetweenNodes(const RRTNode q_near_, const RRTNode q_new_);
  	
	bool isGoal(const RRTNode st_);
  	std::list<RRTNode*> getPath();

	/**
		 Get discrete occupancy matrix index for this discrete (x,y,z) position
		   @param x, y, z discrete position values
		   @return the index in the occupancy matrix
		**/
	inline unsigned int getWorldIndex(int &x, int &y, int &z)
	{
		return (unsigned int)((x - ws_x_min_inflated) + (Lx) * ((y - ws_y_min_inflated) + (Ly) * (z - ws_z_min_inflated)));
	}
  /**
	 Check if a Node/continuousPosition/discretePosition is inside the worksspace
	   @param ThetaStar Node / Discrete position
	   @return true if is inside the ws
	**/
	inline bool isInside(RRTNode n_)
	{
		return isInside(n_.point_uav.x, n_.point_uav.y, n_.point_uav.z);
	}
	inline bool isInside(int x, int y, int z)
	{
		return (x < (ws_x_max - 1) && x > (ws_x_min + 1)) &&
			   (y < (ws_y_max - 1) && y > (ws_y_min + 1)) &&
			   (z < (ws_z_max - 1) && z > (ws_z_min + 1));
	}

	DiscretePosition discretizePosition(Vector3 p);

	bool isInitialPositionUAVOccupied();
	bool isFinalPositionOccupied();

		/** 
		 Check if a node is occupied
		   @param A RRTNode
		   @return true if is occupied in the occupancy matrix
		**/
	bool isOccupied(RRTNode n_);

	/**
		  Inflate a occupied cells filling all cells around in the occupancy matrix
		  Improvement: fast version using memset() to set to zero (.notOccupied) all 
p		  occupation matrix nodes that have to be inflated
			@param discrete position of the cell to inflate		
		**/
	inline void inflateNodeAsCube(int &x_, int &y_, int &z_)
	{
		// Inflation limits around the node
		int x_inflated_max = (x_ + h_inflation) + 1;
		int x_inflated_min = (x_ - h_inflation) - 1;
		int y_inflated_max = (y_ + h_inflation) + 1;
		int y_inflated_min = (y_ - h_inflation) - 1;
		int z_inflated_max = (z_ + v_inflation) + 1;
		int z_inflated_min = (z_ - v_inflation) - 1;

		// Loop 'x axis' by 'x axis' for all discrete occupancy matrix cube around the node that must be inflated

		// Due to the inflated occupancy matrix size increment, the inside checking is not neccesary
		for (int i = x_inflated_min; i < x_inflated_max; i++)
			for (int j = y_inflated_min; j <= y_inflated_max; j++)
				for (int k = z_inflated_min; k <= z_inflated_max; k++)
				{
					unsigned int world_index_ = getWorldIndex(i, j, k);
					discrete_world[world_index_].notOccupied = false;
				}
	}

	/**
		  Inflate a occupied cells filling all cells inside the around cylinder in 
		  the occupancy matrix. 
			@param discrete position of the cell to inflate
		**/
	inline void inflateNodeAsCylinder(int &x_, int &y_, int &z_)
	{
		// Get discretized radius of the inflation cylinder
		int R = h_inflation;

		// Inflation limits around the node
		int x_inflated_max = (x_ + h_inflation) + 1;
		int x_inflated_min = (x_ - h_inflation) - 1;
		int y_inflated_max = (y_ + h_inflation) + 1;
		int y_inflated_min = (y_ - h_inflation) - 1;
		int z_inflated_max = (z_ + v_inflation) + 1;
		int z_inflated_min = (z_ - v_inflation) - 1;

		// Loop throug inflation limits checking if it is inside the cylinder
		// Due to the inflated occupancy matrix size increment, the inside checking is not neccesary
		for (int i = x_inflated_min; i <= x_inflated_max; i++)
			for (int j = y_inflated_min; j <= y_inflated_max; j++)
				for (int k = z_inflated_min; k <= z_inflated_max; k++)
				{
					if (isInsideTheCylinder(i, j, x_, y_, R))
					{
						unsigned int world_index_ = getWorldIndex(i, j, k);
						discrete_world[world_index_].notOccupied = false;
					}
				}
	}

	/**
		  Inflate a occupied cells filling all cells around in the occupancy matrix ONLY HORIZONTALLY
			@param discrete position of the cell to inflate
		**/
	inline void inflateNodeAsXyRectangle(int &x_, int &y_, int &z_)
	{
		// Inflation limits
		int x_inflated_max = (x_ + h_inflation) + 1;
		int x_inflated_min = (x_ - h_inflation) - 1;
		int y_inflated_max = (y_ + h_inflation) + 1;
		int y_inflated_min = (y_ - h_inflation) - 1;

		// Due to the inflated occupancy matrix size increment, the inside checking is not neccesary
		for (int i = x_inflated_min; i <= x_inflated_max; i++)
			for (int j = y_inflated_min; j <= y_inflated_max; j++)
			{
				unsigned int world_index_ = getWorldIndex(i, j, z_);
				discrete_world[world_index_].notOccupied = false;
			}
	}

	/**
		  Check if the [x,y] position is inside the cylinder [xo,yo,R] 
		  (really is circle, yes...)
			@return true if it is inside
		**/
	inline bool isInsideTheCylinder(int &x, int &y, int &xo, int &yo, int &R)
	{
		int R_ = sqrt((x - xo) * (x - xo) + (y - yo) * (y - yo));

		if (R_ <= R)
			return true;
		else
			return false;
	}

	/** Variables **/

	ros::NodeHandlePtr nh; // Pointer to the process NodeHandle to publish topics

	PointCloud occupancy_marker; // Occupancy Map as PointCloud markers
	ros::Publisher occupancy_marker_pub_;

	// bool got_to_goal = false;
	int got_to_goal = 1;
	int num_goal_found;
	double minR;

	octomap::OcTree *map;
	std::vector<geometry_msgs::Point> points_catenary_new_node;
  ros::Publisher tree_rrt_star_uav_pub_, lines_uav_marker_pub_, catenary_marker_pub_, all_catenary_marker_pub_;
  ros::Publisher goal_point_pub_, rand_point_pub_, one_catenary_marker_pub_ , points_marker_pub_, new_point_pub_, nearest_point_pub_, reel1_point_pub_, reel2_point_pub_;
	ros::Publisher new_catenary_marker_pub_, nearest_catenary_marker_pub_, reducedMapPublisher;

	Vector3 initial_position_ugv, initial_position_uav, final_position;   // Continuous
	double goal_gap_m;

	std::list<RRTNode*> rrt_path;

	// Max time to get path
	int timeout;

	std::vector<RRTNodeLink3D> discrete_world; // Occupancy Matrix and its size

	int count_qnew_fail, count_fail_connect_goal, count_loop; // count times that fail get a new q_new;
	int matrix_size;
	int K, n_iter, n_loop, count_graph;
	int ws_x_max, ws_y_max, ws_z_max; // WorkSpace lenghts from origin (0,0,0)
	int ws_x_min, ws_y_min, ws_z_min;
	int h_inflation; // Inflation (Real and Safe distances from the MAV CoG)
	int v_inflation;
	int ws_x_max_inflated, ws_y_max_inflated, ws_z_max_inflated; // Inflated WorkSpace, the real size of the Occupancy Matrix
	int ws_x_min_inflated, ws_y_min_inflated, ws_z_min_inflated; // ... less isInside() checking
	int Lx, Ly, Lz;												 // Inflated WorkSpace lenghts and theirs pre-computed inverses
	float Lx_inv, Ly_inv, Lz_inv;
	std::string frame_id;
	bool markers_debug, nodes_marker_debug;
	double length_tether_max, radius_near_nodes, step_steer;
	double min_dist_for_steer_ugv; // min distance UGV-UAV to steer a new position of UGV. Oblide to steer wheen legth cable is longer thant this value
	int samp_goal_rate;

  double distance_obstacle_uav, distance_catenary_obstacle; //Safe distance to obstacle to accept a point valid for UGV and UAV
	int id_ugv_init, id_uav_init;

	visualization_msgs::MarkerArray catenary_marker;
	
};

}
#endif
