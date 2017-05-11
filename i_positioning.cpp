#include "node_pid.h"
#include "ins_positioning/InsPos.h"
#include "ins_positioning/InsPosS.h"

#define SUBSCRIBER_BUFFER_SIZE 1  // Size of buffer for subscriber.
#define PUBLISHER_BUFFER_SIZE 1000  // Size of buffer for publisher.

#define TOLERANCE 0.02  // Distance from the target distance, at which the distance will be considered as achieved.
#define TOLERANCE_ANGLE 0.01  // Differenc from target angle, which will be tolerated.

#define MAX_SPEED 0.2    // Maximum speed of robot.
#define MAX_A_SPEED 1.0    // Maximum angular speed of robot.

// #define PUBLISHER_TOPIC "/syros/base_cmd_vel"
#define PUBLISHER_TOPIC "/yocs_cmd_vel_mux/fine_pos/cmd_vel"

// #define SUBSCRIBER_TOPIC "/syros/global_odom"
#define SUBSCRIBER_TOPIC "odom"

#define PI 3.141592

#define F_KP 2.58  // P constant for PSD translation controller
#define F_KD 0.047  // D constant for PSD translation controller
#define F_KI 0.0  // S constant for PSD translation controller

#define R_KP 2.0  // P constant for PSD rotation controller
#define R_KD 0.1  // D constant for PSD rotation controller
#define R_KI 0.0  // S constant for PSD rotation controller

MyPoint* actual;
double targetDistance = 0;
double tolerance = TOLERANCE;
double toleranceAngle = TOLERANCE_ANGLE;
double targetAngle = 0;
double maxSpeed = 0; //MAX_SPEED;
double maxASpeed = 0; //MAX_A_SPEED;  
int iterations = 0;
double sumDistance = 0;
double sumAngle = 0;
MyPoint *start; 
MyPoint *last;
ros::Publisher pubMessage;
bool Start = false;
bool finished = false;
bool stop_p = false;

bool closeEnough(MyPoint* actual)
{
  double distance;
  distance = start->getDistance(actual)*copysign(1.0, targetDistance);
  ROS_INFO("Distance - targetDistance = %f",distance-targetDistance);
  if ((fabs(distance-targetDistance) > tolerance))
  {
    if (targetDistance > 0) {
        if ((distance-targetDistance) > 0){
              return true;
        }
    }
    
    else if (targetDistance < 0) {
        if ((distance-targetDistance) < 0){
              return true;
        }
    }   
 
    return false;
  }
  if (fabs(targetAngle - (actual->angle - start->angle)) > toleranceAngle &
    fabs(targetAngle - (actual->angle - start->angle) + 2*PI) > toleranceAngle &
    fabs(targetAngle - (actual->angle - start->angle) - 2*PI) > toleranceAngle)
  {
    return false;
  }
  
  return true;
}

double calculatePSD(MyPoint* actual, double actualValue, double lastValue, double reference, double kP, double kD, double kS, double *sum)
{
  double speed = 0;
  double error = reference - actualValue;
  double previousError = reference - lastValue;
  double dt = actual->time.toSec() - last->time.toSec();
  double derivative = (error - previousError)/dt;
  *sum = *sum + error*dt;
  speed = kP*error + kD*derivative + kS*(*sum);
  return speed;
}

//Publisher
void publishMessage(double angleCommand, double speedCommand)
{
  //preparing message
  geometry_msgs::Twist msg;

  msg.linear.x = speedCommand;
  msg.angular.z = angleCommand;

  //publishing message
  pubMessage.publish(msg);
}


//Subscriber
void messageCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  actual = new MyPoint(msg->pose.pose.position.x, msg->pose.pose.position.y, 2.0*asin(msg->pose.pose.orientation.z), msg->header.stamp);
  ROS_INFO("Actual x = %f",msg->pose.pose.position.x);
  if (Start)
    {
	ROS_INFO("Positioning running");
       double angleCommand = 0;
       double speedCommand = 0;

       if (closeEnough(actual) == true)
       {
          ROS_INFO("POSITION ACHIEVED");
          publishMessage(0.0,0.0);
          iterations = 0;
          targetAngle = 0;
          targetDistance = 0;
          Start = false;
          finished = true;
       } else {
    if (iterations == 0)
    {
        start->x = actual->x;
        start->y = actual->y;
        start->time = actual->time;
        start->angle = actual->angle;
        last->x = start->x;
        last->y = start->y;
        last->time = start->time;
        last->angle = start->angle;
     }
     iterations++;

      //Calculation of action intervention.
      if (fabs(targetDistance) > tolerance)
      {
        speedCommand = calculatePSD(actual,start->getDistance(actual)*copysign(1.0, targetDistance),start->getDistance(last)*copysign(1.0, targetDistance),targetDistance,F_KP,F_KD,F_KI,&sumDistance);
      }

      if (actual->angle-last->angle < -PI)
      {
       actual->angle += 2*PI;
      } 
      else if (actual->angle-last->angle > PI)
      {
        actual->angle -= 2*PI;
      }

      angleCommand = calculatePSD(actual,actual->angle-start->angle, last->angle-start->angle,targetAngle,R_KP,R_KD,R_KI,&sumAngle);

      //Saving position to last
      last->x = actual->x;
      last->y = actual->y;
      last->time = actual->time;
      last->angle = actual->angle;

      //Invoking method for publishing message
      publishMessage(fmin(maxASpeed,angleCommand), fmin(maxSpeed,speedCommand));
}
    }
}

bool stoppos(ins_positioning::InsPosS::Request  &req, ins_positioning::InsPosS::Response &res)
{
	if (req.stop == true){
		if (Start == true){
			stop_p = true;
		}else{
			stop_p = false;	
		}	
	}
	res.stopped = true;
	return true;
}

bool startpos(ins_positioning::InsPos::Request  &req, ins_positioning::InsPos::Response &res)
{
   if (req.command == 0){

           targetDistance = req.distance;
           maxSpeed = req.LinVel;
           targetAngle = 0;
           maxASpeed = 0;

           start->x = actual->x;
           start->y = actual->y;
           start->time = actual->time;
           start->angle = actual->angle;

           Start = true;
	   finished = false;

           ros::Rate r(10);
           while ((!finished) && (!stop_p)){
                 ros::spinOnce();
                 r.sleep();
           }
	   Start = false;
	   stop_p = false;
           res.Done = finished;

  } else if (req.command == 1){

           targetAngle = req.angle;
           maxASpeed = req.AngVel;
           targetDistance = 0;
           maxSpeed = 0;

           start->x = actual->x;
           start->y = actual->y;
           start->time = actual->time;
           start->angle = actual->angle;

           Start = true;
	   finished = false;

           ros::Rate r(10);
           while ((!finished) && (!stop_p)){
                 ros::spinOnce();
                 r.sleep();
           }
	   Start=false;
	   stop_p = false;
           res.Done = finished;

  } else {
  	   ROS_INFO("command not found");
  }

   return true;
}

int main(int argc, char **argv)
{
  //Initialization of node
  ros::init(argc, argv, "i_positioning");
  ros::NodeHandle n;
  
  start = new MyPoint(0.0, 0.0, 0.0, ros::Time::now());
  last = new MyPoint(0.0, 0.0, 0.0, ros::Time::now());

  ros::ServiceServer servicePos = n.advertiseService("ins_pos", startpos);
  ros::ServiceServer servicePosS = n.advertiseService("ins_pos_stop", stoppos);
  //Creating publisher
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>(PUBLISHER_TOPIC,1);
  
  //Creating subscriber and publisher
  
  pubMessage = pub;
  ros::Subscriber sub = n.subscribe(SUBSCRIBER_TOPIC, 1000, messageCallback);

  ros::spin();

  return 0;
}
