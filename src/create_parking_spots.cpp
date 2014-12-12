//adds interactive (clickable) markers to in the middle of all edges of all parts of the furniture

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>

using namespace visualization_msgs;

void onClick(const InteractiveMarkerFeedbackConstPtr &f){
  if (f->event_type == InteractiveMarkerFeedback::MOUSE_UP){
    ROS_INFO_STREAM("marker clicked");
  }   
}

int main(int argc, char** argv){

  ros::init(argc,argv,"create_parking_spots");
  interactive_markers::InteractiveMarkerServer server("parking_markers");
  
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "ilab";
  int_marker.scale = 1;
  int_marker.name = "parking_spot";
  
  InteractiveMarkerControl control;
  control.interaction_mode = InteractiveMarkerControl::BUTTON;
  control.name = "button_control";
  
  Marker box;
  box.type = Marker::CUBE;
  box.scale.x=1;
  box.scale.y=1;
  box.scale.z=0.01;
  box.color.r=.5;
  box.color.g=.5;
  box.color.b=.5;
  box.color.a=1;
 
  control.markers.push_back(box);
  control.always_visible = true;
  int_marker.controls.push_back(control);
  
  server.insert(int_marker, &onClick);

  server.applyChanges();

  ros::spin();

  //read in robot_pose_publisher (furniture are robot parts)
  //calculate mid-points on edges of bounding box
  //get transform of furniture (tfListener.lookupTransform)
  //add edge-mid-point to transform
  //add interactive marker at that point (InteractiveMarker)
  //make interactive marker clickable
  //print location on click

}
