//TODO: initialize navigator w/ non-default constructor! 
// NavfnROS(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
// set up variables for getPlanFromPotential(const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan)
// Pick goal poses for each spot & set those for their respective puck location instead of trying to sit on the puck!



#include <ros/ros.h>
#include <navfn_ros.h> //or is it navfn/navfn_ros.h... or...
#include <trajectory_planner_ros.h>
#include<config.h>


void localPlan(const std::vector<geometry_msgs::poseStamped> & globalPlan,const costmap_2d::Costmap2DROS & theCostMap){
  ros::NodeHandle n2;
  tf::TransformListener transforms(n2, ros::Duration(DEFAULT_CACHE_TIME), true);
  
  base_local_planner::TrajectoryPlannerROS skipper("skipper",transforms,theCostMap);
  skipper.setPlan(globalPlan);
  while(!skipper.isGoalReached()){
    //TODO
  }

}

void spinThread() {
  ros::spin(); //spins in bg when function is called
}


int main(int argc, char* argv[]) {

  //set up topic
  ros::init(argc, argv, "getThere");	 //initialize topic
  boost::thread spin_thread(&spinThread);//instantiation allows multiple topics 
                                            //if deemed necessary in the future
  

  //get puck location indexes
  ros::NodeHandle n; 
  motionPlanning::getLocations::Request req;
  motionPlanning::getLocations::Response res;

  std::vector<geometry_msgs::Point> pucks(6);
  
  req.s = "";

  ros::ServiceClient pcksrv = n.serviceClient<motionPlanning::getLocations>("PuckQuadrantsPlease");
  pcksrv.call(req,res);

  for (i = 0, i<res.s.size(), i++) { 
	if((int)((res.s[i]-1)/4.0) == 0) { //first row
	  pucks.at(i).x = 0.1524; //meters
    } else if((int)((res.s[i]-1)/4.0) == 1) { //second row
      pucks.at(i).x = 0.4572; //meters   
    } else if((int)((res.s[i]-1)/4.0) == 2) { //third row
	  pucks.at(i).x = 0.7620; //meters
    } else if((int)((res.s[i]-1)/4.0) == 3) { //fourth row
      pucks.at(i).x = 1.0668; //meters
	}
	if((int)((res.s[i])%4.0) == 1) { //first row
	  pucks.at(i).y = 0.1524; //meters
	} else if((int)((res.s[i])%4.0) == 2) { //second row
	  pucks.at(i).y = 0.4572; //meters
	} else if((int)((res.s[i])%4.0) == 3) { //third row
	  pucks.at(i).y = 0.7620; //meters
	} else if((int)((res.s[i])%4.0) == 0) { //fourth row
	  pucks.at(i).y = 1.0668; //meters
    }
  } 



  double holder, goal;
  geometry_msgs::Point currentLocation, goalLocation;
  NavfnROS navigator;
  
  currentLocation.x = 0.4;
  currentLocation.y = 0.4;
  currentLocation.z = 0.3;
   
  while(n.ok() && pucks.size()>0) {
    if(navigator.computePotential(currentLocation)){
	  goal = navigator.getPointPotential(pucks.at(0));
	  goalLocation = pucks.at(0);
	  for(i=1,i<pucks.size(),i++){
	    holder = navigator.getPointPotential(pucks.at(i));
	    if (holder>goal) {
		  goal = holder;
		  goalLocation = pucks.at(i);
		}
	  }
	  //if(
//TODO  navigator.getPlanFromPotential(/*const geometry_msgs::PoseStamped &goal*/,/*std::vector<geometry_msgs::PoseStamped> &plan);
     // )
	  {
	    //we now have the global plan. time for local planning
		localPlan(globalPlan);
	  }
	}
  }
