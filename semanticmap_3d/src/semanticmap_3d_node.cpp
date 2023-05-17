#include <ros/ros.h>
#include <semanticmap_3d/semanticmap_3d.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "semanticmap_node");
 
  Semanticmap3D map(0,0,0,0,0,0,0);

  ros::spin();

  return(0);
}
