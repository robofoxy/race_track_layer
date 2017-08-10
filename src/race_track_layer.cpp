#include <race_track_layer/race_track_layer.h>
#include <pluginlib/class_list_macros.h>


PLUGINLIB_EXPORT_CLASS(costmap_2d::RaceTrackLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;
using costmap_2d::NO_INFORMATION;

namespace costmap_2d{
	RaceTrackLayer::RaceTrackLayer() {}

	void RaceTrackLayer::onInitialize(){
	
		ros::NodeHandle nh("~/" + Layer::name_);
		initSet = false;
		nmspc = nh.getNamespace();
		is = false;
		weightSet = false;
		std::vector<std::string> lv_elems;
		char lc_delim[2];
		lc_delim[0] = '/';
		lc_delim[1] = '\0';
		
		boost::algorithm::split( lv_elems, nh.getNamespace(), boost::algorithm::is_any_of( lc_delim ) );
		nmspc = lv_elems[1];

		robotsSet = false;
		rolling_window_ = Layer::layered_costmap_->isRolling();
		current_ = true;
		default_value_ = NO_INFORMATION;
		dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
		dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
									&RaceTrackLayer::reconfigureCB, this, _1, _2);
		dsrv_->setCallback(cb);
	}


	void RaceTrackLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level){
	
		enabled_ = config.enabled;
	}
	
	

	void RaceTrackLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
		                                   double* min_y, double* max_x, double* max_y){
		if (rolling_window_ && is)
			updateOrigin(  robot_x - getSizeInMetersX() / 2 ,  robot_y - getSizeInMetersY() / 2);
		
		robotx = robot_x;
		roboty = robot_y;
		initSet = true;
		
		if(!weightSet){
			std::vector<geometry_msgs::Point> fp = Layer::getFootprint();
			int size = fp.size();
			float max = -1;
			for(int i = 0; i < size; i++){
				float firstX = fp[i].x, firstY = fp[i].y;
				for(int j = i+1; j < size; j++){
					float secondX = fp[j].x, secondY = fp[j].y;
					float res = sqrt(pow(secondX - firstX, 2) + pow(secondY - firstY, 2));
					if(max < res) max = res;
				}
			}
			
			weight = max;
			
			weightSet = true;
		}
		
		resetMap(0, 0, getSizeInCellsX(), getSizeInCellsY());
		
		if (!enabled_) return;

		double mark_x = robot_x + cos(robot_yaw), mark_y = robot_y + sin(robot_yaw);
		
		*min_x = std::min(*min_x, mark_x);
		*min_y = std::min(*min_y, mark_y);
		*max_x = std::max(*max_x, mark_x);
		*max_y = std::max(*max_y, mark_y);
	}
	
	
	
	bool RaceTrackLayer::isDiscretized(){
	
		return true;
	}
	
	
	void RaceTrackLayer::findRaceTracks(const costmap_2d::Costmap2D& master_grid){
		if(!robotsSet) return;
		
		rt.clear();
		int size = robots.size();
		
		for(int i = 0; i < size; i++){
			double angle = robots[i]->getYaw();
			double speed = robots[i]->getSpeed();
			geometry_msgs::Point mid = robots[i]->getMidPoint();
			
			if(speed < 0.01) continue;
			
			std::vector<MapLocation> raceTrack;
			geometry_msgs::Point a, b, c;
			MapLocation q, p, r;
			
			a.x = mid.x + cos(angle + 0.5) ;
			a.y = mid.y + sin(angle + 0.5) ;
			
			b.x = mid.x + cos(angle - 0.5) ;
			b.y = mid.y + sin(angle - 0.5) ;
			
			if(speed >= 1){
				c.x = mid.x + cos(angle) * speed * speed * 5;
				c.y = mid.y + sin(angle) * speed * speed * 5;
			}
			else{
				c.x = mid.x + cos(angle) * 5 ;
				c.y = mid.y + sin(angle) * 5 ;
			}
			
			master_grid.worldToMap (a.x, a.y, q.x, q.y);
			master_grid.worldToMap (b.x, b.y, p.x, p.y);
			master_grid.worldToMap (c.x, c.y, r.x, r.y);
			
			raceTrack.push_back(q);
			raceTrack.push_back(p);
			raceTrack.push_back(r);
			
			rt.push_back(raceTrack);
		}
	}
	
	
	void RaceTrackLayer::findPolys(){
	
		if(!robotsSet) return;
		polys.clear();	
		int size = robots.size();
		
		for(int i = 0; i < size; i++){
			int numOfPoints = robots[i]->getFootprint().polygon.points.size();
			std::vector<geometry_msgs::Point> poly;
			for(int j = 0; j < numOfPoints; j++){
				geometry_msgs::Point p;
				
				p.x = robots[i]->getFootprint().polygon.points[j].x;
				p.y = robots[i]->getFootprint().polygon.points[j].y;
			
				poly.push_back(p);
			}
			
			polys.push_back(poly);
		}
	}
	
	
	float findDistance(MapLocation mid, MapLocation fill){
		float a = (mid.x - fill.x) * (mid.x - fill.x), b = (mid.y - fill.y) * (mid.y - fill.y);	
		return sqrt(a + b);
	}
	
	void sortCells(std::vector<MapLocation>& filling, MapLocation a, MapLocation b){
		
		MapLocation temp, mid;
		
		mid.x = (a.x + b.x) / 2;
		mid.y = (a.y + b.y) / 2;
		
		unsigned int i = 0;
		int size = filling.size();
		
		while(i < size - 1){
			MapLocation qq, rr;
			qq.x = 0;
			qq.y = 0;
			rr.x = 5;
			rr.y = 12;
			
			if(findDistance(mid, filling[i]) > findDistance(mid, filling[i+1])){
				temp = filling[i];
				filling[i] = filling[i+1];
				filling[i+1] = temp;
				if(i>0) i--;
			}
			else i++; 
		}
		
	}
	

	bool RaceTrackLayer::isInCriticalRegion(costmap_2d::Costmap2D& master_grid, const MapLocation& filling){
		MapLocation coordMax, coordMin;
		
		if(!rolling_window_)
			master_grid.mapToWorld(robox, roboy, robotx, roboty);
		
		float dist = (weight / 2 + 0.5);
		
		master_grid.worldToMap(robotx - dist, roboty - dist, coordMin.x, coordMin.y);
		master_grid.worldToMap(robotx + dist, roboty + dist, coordMax.x, coordMax.y);
		
		if(filling.x > coordMin.x && filling.x < coordMax.x && filling.y < coordMax.y && filling.y > coordMin.y)
			return true;
		else
			return false;
	}




	void RaceTrackLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
		                                      int max_j){
		if (!enabled_) return;
		std::clock_t start;
	    double duration;
	    
	    if(!rolling_window_){
	    	robox = (min_i + max_i) / 2;
	    	roboy = (min_j + max_j) / 2;
	    }

		if(initSet){
		
			if(!robotsSet && robots.size()==0){
				robotsSet = true;
				ros::Duration(0.5).sleep();
				std::vector<std::string> lv_elems;
				char lc_delim[2];
				lc_delim[0] = '/';
				lc_delim[1] = '\0';

				ros::master::V_TopicInfo topic_infos;
				ros::master::getTopics(topic_infos);
				for(int k = 0; k< topic_infos.size(); k++){
					boost::algorithm::split( lv_elems, topic_infos[k].name, boost::algorithm::is_any_of( lc_delim ) );
					
					if(lv_elems[1] != nmspc && lv_elems.size() == 5 && lv_elems[3]=="global_costmap" && lv_elems[4]=="footprint"){
						RobotType *robot = new RobotType(lv_elems[1]);
						robots.push_back(robot);
					}
				}
			
			}
			
			findPolys();
		
			int numOfPrevRobots = polysPrev.size();
			
		
			for(int i = 0; i < numOfPrevRobots; i++){
				if(polysPrev.size() > i && polysPrev[i].size() > 0 && polys[i][0].x == polysPrev[i][0].x) continue;
					master_grid.setConvexPolygonCost(polysPrev[i], FREE_SPACE);
			}
		
			findRaceTracks(master_grid);
		
			int numOfPrevTracks = rtPrev.size();
		
			for(int i = 0; i < numOfPrevTracks; i++){
				std::vector<MapLocation> filling;
				master_grid.convexFillCells(rtPrev[i], filling);
				int fs = filling.size();
				for(int j = 0; j< fs; j++){
					if(master_grid.getCost(filling[j].x, filling[j].y) < LETHAL_OBSTACLE)
						master_grid.setCost(filling[j].x, filling[j].y, 0);
				}
			}
			
			int numOfTracks = rt.size();
			
			for(int i = 0; i <numOfTracks; i++){
				std::vector<MapLocation> filling;
				master_grid.convexFillCells(rt[i], filling);
				sortCells(filling, rt[i][0], rt[i][1]);
				int fs = filling.size(), c = 0;
				char cost = 240;
				int q = fs / 200;
				
				for(int j = 0; j< fs; j++){
					if(isInCriticalRegion(master_grid, filling[j]))
						break;
					if(master_grid.getCost(filling[j].x, filling[j].y) == LETHAL_OBSTACLE)
						break;
					
					master_grid.setCost(filling[j].x, filling[j].y, cost);
					
					if(++c > q){
						c = 0;
						cost --;
					}
						
				}
			}
			
			rtPrev = rt;
			
			int numOfOtherRobots = polys.size();
			int prev = polysPrev.size();

			for(int i = 0; i <numOfOtherRobots; i++)
				master_grid.setConvexPolygonCost(polys[i], LETHAL_OBSTACLE);
					
			polysPrev = polys;
		}
	
	
	}
} // end namespace
