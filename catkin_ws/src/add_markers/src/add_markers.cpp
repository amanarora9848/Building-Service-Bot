 #include <ros/ros.h>
 #include <visualization_msgs/Marker.h>
 #include <std_msgs/UInt8.h>

 double pickup[4] = {4.17677974701, -1.02883398, 0.15230178833, 1.0};
 double dropoff[4] = {4.00920200348, -5.74286270142, 0.15299790581, 1.0};

 uint8_t whereami = 0;

 void trackCallback (const std_msgs::UInt8::ConstPtr& msg)
 {
 	whereami = msg->data;
	return;
 }

 int main( int argc, char** argv )
 {
   ros::init(argc, argv, "add_markers");
   ros::NodeHandle n;
   ros::Rate r(1);
   ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
   
   // The subscriber to our node "target"
   ros::Subscriber goal_sub = n.subscribe("target", 100, trackCallback);
 
   // Set our initial shape type to be a cube
   uint32_t shape = visualization_msgs::Marker::CUBE;
 
   while (ros::ok())
   {
     visualization_msgs::Marker marker;
     // Set the frame ID and timestamp.
     marker.header.frame_id = "odom";
     marker.header.stamp = ros::Time::now();
 
     // Set the namespace and id for this marker.  This serves to create a unique ID
     // Any marker sent with the same namespace and id will overwrite the old one
     marker.ns = "basic_shapes";
     marker.id = 0;
 
     // Set the marker type.
     marker.type = shape;
 
     // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
     // marker.action = visualization_msgs::Marker::ADD;
 
     // Set the scale of the marker -- 1x1x1 here means 1m on a side
     marker.scale.x = 0.3;
     marker.scale.y = 0.3;
     marker.scale.z = 0.3;
 
     // Set the color -- be sure to set alpha to something non-zero!
     marker.color.r = 0.0f;
     marker.color.g = 1.0f;
     marker.color.b = 0.0f;
     marker.color.a = 1.0;
 
     marker.lifetime = ros::Duration();
     
     if(whereami == 0)
	 {
	    marker.action = visualization_msgs::Marker::ADD;
	    
	 	marker.pose.position.x = pickup[0];
     	marker.pose.position.y = pickup[1];
     	marker.pose.position.z = pickup[2];
     	marker.pose.orientation.x = 0.0;
     	marker.pose.orientation.y = 0.0;
    	marker.pose.orientation.z = 0.0;
     	marker.pose.orientation.w = pickup[3];
	 }
	 else if(whereami == 1)
	 {
	 	marker.action = visualization_msgs::Marker::DELETE;
	 	ROS_INFO("Service Ferrari has reached pick up point.");
	 }
	 else if(whereami == 2)
	 {
	 	marker.action = visualization_msgs::Marker::ADD;
	    
	 	marker.pose.position.x = dropoff[0];
     	marker.pose.position.y = dropoff[1];
     	marker.pose.position.z = dropoff[2];
     	marker.pose.orientation.x = 0.0;
     	marker.pose.orientation.y = 0.0;
    	marker.pose.orientation.z = 0.0;
     	marker.pose.orientation.w = dropoff[3];
     	ROS_INFO("Service Ferrari has reached drop off point.");
	 }
 
     // Publish the marker
     while (marker_pub.getNumSubscribers() < 1)
     {
       if (!ros::ok())
       {
         return 0;
       }
       ROS_WARN_ONCE("Please create a subscriber to the marker");
       sleep(1);
     }
     marker_pub.publish(marker);

     r.sleep();
     ros::spinOnce();
     
   }
 }
