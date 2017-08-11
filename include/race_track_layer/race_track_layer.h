#ifndef RACE_TRACK_LAYER_H_
#define RACE_TRACK_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point.h>
#include <boost/algorithm/string.hpp>
#include <race_track_layer/robot_type.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <cmath>

namespace costmap_2d{
	class RaceTrackLayer : public costmap_2d::Layer, public costmap_2d::Costmap2D{
		public:
		  RaceTrackLayer();
		  virtual void onInitialize();
		  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
				                     double* max_y); 
		  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
		  bool isDiscretized();

		private:
	      bool isInCriticalRegion(costmap_2d::Costmap2D& master_grid, const MapLocation& filling);
	      bool weightSet;
	      float weight;
		  std::string nmspc;
		  double robotx, roboty;
		  unsigned int robox, roboy;
		  bool robotsSet, initSet, is;
		  std::vector<RobotType*> robots;
		  std::vector<std::vector<MapLocation> > polys, polysPrev;
		  std::vector<std::vector<MapLocation> > rt, rtPrev;
		  bool rolling_window_;
		  void findPolys(const costmap_2d::Costmap2D& master_grid);
		  void findRaceTracks(const costmap_2d::Costmap2D&);
		  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
		  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
	};
}
#endif
