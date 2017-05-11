#include "node_pid.h"
#include "ins_positioning/InsPos.h"

#define SUBSCRIBER_BUFFER_SIZE 1  // Size of buffer for subscriber.
#define PUBLISHER_BUFFER_SIZE 1000  // Size of buffer for publisher.
#define TOLERANCE 0.01  // Distance from the target distance, at which the distance will be considered as achieved.
#define TOLERANCE_ANGLE 0.02  // Differenc from target angle, which will be tolerated.
#define MAX_SPEED 0.5    // Maximum speed of robot.
#define MAX_A_SPEED 2.0    // Maximum angular speed of robot.
// #define PUBLISHER_TOPIC "/syros/base_cmd_vel"
#define PUBLISHER_TOPIC "/yocs_cmd_vel_mux/fine_pos/cmd_vel"
// #define SUBSCRIBER_TOPIC "/syros/global_odom"
#define SUBSCRIBER_TOPIC "odom"
#define PI 3.141592

double targetDistance = 0;
double tolerance = TOLERANCE;
double toleranceAngle = TOLERANCE_ANGLE;
double targetAngle = 0;
double maxSpeed = MAX_SPEED;
double maxASpeed = MAX_A_SPEED; 
bool Start = false;
bool finished = false;

bool startpos(ins_positioning::InsPos::Request  &req, ins_positioning::InsPos::Response &res)
{
   if (req.command == 0){
           targetDistance = req.distance;
           maxSpeed = req.LinVel;
           targetAngle = 0;
           maxASpeed = 0;
           Start = true;
	   finished = false;
           res.Done = finished;
  } else if (req.command == 1){
           targetAngle = req.angle;
           maxASpeed = req.AngVel;
           targetDistance = 0;
           maxSpeed = 0;
           Start = true;
	   finished = false;
           res.Done = finished;
  } else {
  	   ROS_INFO("command not found");
  }

   return true;
}

int main(int argc, char **argv)
{
  //Initialization of node
  ros::init(argc, argv, "pid");
  ros::NodeHandle n;

  ros::ServiceServer servicePos = n.advertiseService("ins_pos", startpos);

  //Creating publisher
  ros::Publisher pubMessage = n.advertise<geometry_msgs::Twist>(PUBLISHER_TOPIC, PUBLISHER_BUFFER_SIZE);

  //Creating object, which stores data from sensors and has methods for
  //publishing and subscribing
  NodePID *nodePID = new NodePID(pubMessage, TOLERANCE, TOLERANCE_ANGLE, targetDistance, targetAngle, maxSpeed, maxASpeed);

  //Creating subscriber and publisher
  ros::Subscriber sub = n.subscribe(SUBSCRIBER_TOPIC, SUBSCRIBER_BUFFER_SIZE, &NodePID::messageCallback, nodePID);

  ros::spin();

  return 0;
}
