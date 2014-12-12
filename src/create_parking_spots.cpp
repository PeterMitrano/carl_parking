//adds interactive (clickable) markers to in the middle of all edges of all parts of the furniture

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>

//just for conveneince, so I don't have to write it out a lot
using namespace visualization_msgs;

//when you release the mouse on a marker this gets called
//eventually this will set the location of that marker as a nav goal for Carl
void onClick(const InteractiveMarkerFeedbackConstPtr &f){
  if (f->event_type == InteractiveMarkerFeedback::MOUSE_UP){
    ROS_INFO("marker clicked!");
    float x = f->pose.position.x;
    float y = f->pose.position.y;
    ROS_INFO("at location %f,%f",x,y);
  }   
}

int main(int argc, char** argv){

  ros::init(argc,argv,"create_parking_spots");
  ros::NodeHandle node;
  ros::Rate rate(10.0);

  tf::TransformListener listener;

  //read through all the transforms
  while (node.ok()){

    //create the transform that you're going to read in
    tf::StampedTransform transform;

    try{

      listener.lookupTransform("/ilab","/kitchen_counter_link",ros::Time(0),transform);   
      float x = transform.getOrigin().x();
      float y = transform.getOrigin().y();
      ROS_INFO("origin = (%f,%f)",x,y);

    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep(); 
    }
    rate.sleep();
  }

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
  box.color.r=0;
  box.color.g=0;
  box.color.b=1;
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
  return 0;
}
