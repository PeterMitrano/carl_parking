#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <ros/console.h>

int main(int argc, char** argv){

	ros::init(argc,argv,"read_parking_spots");
	ros::NodeHandle node;
	ros::Rate rate(10.0);

	tf::TransformListener listener;
	tf::Transformer transformer;

	std::vector<std::string> ids;

	//get all the links that I might want transforms of
	transformer.getFrameStrings(ids);

	ROS_INFO("frame 1 %s\n",ids.at(0).c_str());

	while (node.ok()){
		tf::StampedTransform transform;
		try{
			listener.lookupTransform("/ilab","/kitchen_counter_link",ros::Time(0),transform);		
		}
		catch (tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
		}

		float x = transform.getOrigin().x();
		float y = transform.getOrigin().y();

		ROS_INFO("origin = (%f,%f)",x,y);

		rate.sleep();
	}

	return 0;
}