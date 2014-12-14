//adds interactive (clickable) markers and set those locations as navigation goals

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/Marker.h>
#include <urdf/model.h>

//when you release the mouse on a marker this gets called
//eventually this will set the location of that marker as a nav goal for Carl
void onClick(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &f){
  if (f->event_type == visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP){
    ROS_INFO("marker clicked!");
    float x = f->pose.position.x;
    float y = f->pose.position.y;
    ROS_INFO("at location %f,%f",x,y);
  }   
}

//returns true only if the string ends in "nav_goal_link"
bool isNavGoal(std::string link_name){
  std::string search_param = "nav_goal_link";
  return link_name.find(search_param,link_name.length()-search_param.length()) != std::string::npos;
}

//creates a clickable marker at the origin of the given frame id
//this frame id is a string of the name of a link
visualization_msgs::InteractiveMarker createParkingSpot(std::string frame_id){
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = frame_id;
  int_marker.scale = 1;
  int_marker.name = frame_id+"_parking_spot";
  
  visualization_msgs::InteractiveMarkerControl control;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  control.name = "button";
  
  visualization_msgs::Marker box;
  box.type = visualization_msgs::Marker::CUBE;
  box.scale.x=0.15;
  box.scale.y=0.15;
  box.scale.z=0.05;
  box.color.r=0;
  box.color.g=0.5;
  box.color.b=0.25;
  box.color.a=1;
 
  control.markers.push_back(box);
  control.always_visible = true;
  int_marker.controls.push_back(control);

  return int_marker;
} 

int main(int argc, char** argv){

  ros::init(argc,argv,"create_parking_spots");
  ros::NodeHandle node;
  ros::Rate rate(10.0);

  interactive_markers::InteractiveMarkerServer server("parking_markers");

  urdf::Model ilab;

  //load the urdf with all the furniture off the param server
  if (!ilab.initParam("/ilab_description")){
    ROS_INFO("couldn't find /ilab_description on param server.");
    return 1;
  }

  std::map<std::string, boost::shared_ptr<urdf::Link> > links = ilab.links_;
  std::map<std::string, boost::shared_ptr<urdf::Link> >::iterator itr;
  
  //go through all links and filter out the ones that end in "nav_goal_link"
  for(itr = links.begin(); itr != links.end(); itr++) {
    std::string link_name = itr->first;
    if (isNavGoal(link_name)){
      server.insert(createParkingSpot(link_name), &onClick);
    }
  }

  ROS_INFO("creating clickable nav goals...");

  //when these are called the markers will actually appear
  server.applyChanges();
  ros::spin();


  return 0;
}
