#include <ros/ros.h>

//ROS libraries
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

//ROS messages
#include <std_msgs/UInt8.h>
#include <sensor_msgs/Range.h>

using namespace std;

//Globals
double collisionDistance = 0.6; //meters the ultrasonic detectors will flag obstacles
double collistion_dis_cube_approach = 0.9;//sammi for cube detection
string publishedName;
char host[128];

//Publishers
ros::Publisher obstaclePublish;

//helper function to get the direction of the closest obstable
int mode(double left, double center, double right);

//Callback handlers
void sonarHandler(const sensor_msgs::Range::ConstPtr& sonarLeft, const sensor_msgs::Range::ConstPtr& sonarCenter, const sensor_msgs::Range::ConstPtr& sonarRight);


int main(int argc, char** argv) {
    gethostname(host, sizeof (host));
    string hostname(host);
    
    if (argc >= 2) {
        publishedName = argv[1];
        cout << "Welcome to the world of tomorrow " << publishedName << "! Obstacle module started." << endl;
    } else {
        publishedName = hostname;
        cout << "No name selected. Default is: " << publishedName << endl;
    }

    ros::init(argc, argv, (publishedName + "_OBSTACLE"));
    ros::NodeHandle oNH;
    
    obstaclePublish = oNH.advertise<std_msgs::UInt8>((publishedName + "/obstacle"), 10);
    
    message_filters::Subscriber<sensor_msgs::Range> sonarLeftSubscriber(oNH, (publishedName + "/sonarLeft"), 10);
    message_filters::Subscriber<sensor_msgs::Range> sonarCenterSubscriber(oNH, (publishedName + "/sonarCenter"), 10);
    message_filters::Subscriber<sensor_msgs::Range> sonarRightSubscriber(oNH, (publishedName + "/sonarRight"), 10);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Range, sensor_msgs::Range, sensor_msgs::Range> sonarSyncPolicy;

    message_filters::Synchronizer<sonarSyncPolicy> sonarSync(sonarSyncPolicy(10), sonarLeftSubscriber, sonarCenterSubscriber, sonarRightSubscriber);
    sonarSync.registerCallback(boost::bind(&sonarHandler, _1, _2, _3));

    ros::spin();

    return EXIT_SUCCESS;
}

void sonarHandler(const sensor_msgs::Range::ConstPtr& sonarLeft, const sensor_msgs::Range::ConstPtr& sonarCenter, const sensor_msgs::Range::ConstPtr& sonarRight) {
	std_msgs::UInt8 obstacleMode;
	
	if ((sonarLeft->range > collisionDistance) && (sonarCenter->range > collisionDistance) && (sonarRight->range > collisionDistance)) {
		obstacleMode.data = 0; //no collision
        //change if less than sonar detection len
        if ((sonarLeft->range < collistion_dis_cube_approach) || (sonarCenter->range < collistion_dis_cube_approach) || (sonarRight->range < collistion_dis_cube_approach)) {
            obstacleMode.data = mode(sonarLeft->range, sonarCenter->range, sonarRight->range);
        }
	}
	else if ((sonarLeft->range > collisionDistance) && (sonarRight->range < collisionDistance)) {
		obstacleMode.data = 1; //collision on right side
	}
	else {
		obstacleMode.data = 2; //collision in front or on left side
    }



    if (sonarCenter->range < 0.12) //block in front of center unltrasound.
    {
        //        obstacleMode.data += 4;
        obstacleMode.data = 4;
	}
	
        obstaclePublish.publish(obstacleMode);
}

int mode(double left, double center, double right){
    int mode = 11;
    if(left > center){
        mode = 22;
    }else if(left > right){
        mode = 33;
    }
    return mode;
}

