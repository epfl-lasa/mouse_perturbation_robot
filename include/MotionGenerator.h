#ifndef __MOTION_GENERATOR_H__
#define __MOTION_GENERATOR_H__

#include <fstream>
#include <mutex>
#include <pthread.h>
#include <vector>
#include "Eigen/Eigen"

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include <dynamic_reconfigure/server.h>
#include "geometry_msgs/PointStamped.h"
#include <rosserial_mbed/Adc.h>
#include "MouseInterface.h"

#define MAX_XY_REL 350
#define MIN_XY_REL 200
#define PERTURBATION_VELOCITY 15.0f
#define MAX_PERTURBATION_OFFSET 0.1f
#define MIN_PERTURBATION_OFFSET 0.05f
#define TARGET_TOLERANCE 0.05f
#define NB_TARGETS 4
// #define 

class MotionGenerator 
{
	private:


    enum State {INIT = 0, CLEAN_MOTION = 1, PAUSE = 2 , JERKY_MOTION = 3}; 
    enum Target {A = 0, B = 1, C = 2, D = 3};

		// ROS variables
		ros::NodeHandle _n;
		ros::Rate _loopRate;
		float _dt;

		// Subscribers and publishers declaration
		ros::Subscriber _subRealPose;						// Subscribe to robot current pose
    ros::Subscriber _subRealTwist;          // Subscribe to robot current pose
		ros::Subscriber _subMouse;              // Subscribe to foot mouse data
    ros::Publisher _pubDesiredOrientation;  // Publish desired orientation
    ros::Publisher _pubDesiredTwist;        // Publish desired twist

    // Messages declaration
    geometry_msgs::Pose _msgRealPose;
    geometry_msgs::Quaternion _msgDesiredOrientation;
    geometry_msgs::Twist _msgDesiredTwist;
		mouse_perturbation_robot::MouseMsg _msgMouse;

    // End effector state variables
    Eigen::Vector3f _x;      // Current position [m] (3x1)
    Eigen::Vector3f _x0;     // Initial end effector postion (3x1)
    Eigen::Vector3f _v;      // Current end effector velocity [m/s] (3x1)
    Eigen::Matrix3f _wRb;    // Current rotation matrix (3x3)
    Eigen::Vector4f _q;      // Current end effector quaternion (4x1)

    // End effector desired variables
    Eigen::Vector3f _xd;        // Desired position [m] (3x1)
    Eigen::Vector3f _vd;        // Desired velocity [m/s] (3x1)
    Eigen::Vector4f _qd;        // Desired end effector quaternion (4x1)
    Eigen::Vector3f _omegad;    // Desired angular velocity [rad/s] (3x1)

    // Motion variables
    Eigen::Vector2f _mouseVelocity;
    Eigen::Matrix<float,3,NB_TARGETS> _targetOffset;
    Eigen::Vector3f _perturbationOffset;
    Eigen::Vector3f _perturbationDirection;
    Eigen::Vector3f _motionDirection;
    float _mouseOffset;
    double _initTime;
    double _reachedTime;
    double _pauseDuration;
    double _motionDuration;
    double _minMotionDuration;
    double _maxCleanMotionDuration;
    double _maxJerkyMotionDuration;
    double _initDuration;
    uint32_t _trialCount;
    uint32_t _perturbationCount;
    uint8_t _lastMouseEvent;           // Last foot mouse event

    //Booleans
    bool _firstRealPoseReceived;  // Monitor the first robot pose update
    bool _firstMouseEventReceived;
    bool _stop;
    bool _perturbation;
    bool _useMouse;
    bool _mouseInUse;

    // Arduino related variables
    int farduino;
    bool trigger_raised;
    ros::Time trigger_begin;
    
    // Other variables
    static MotionGenerator* me;   // Pointer on the instance
    std::mutex _mutex;
    std::ofstream _outputFile;
    State _state;
    Target _currentTarget;
    Target _previousTarget;
    Eigen::Vector3f _xi;

  public:
    // Class constructor
    MotionGenerator(ros::NodeHandle &n, double frequency);

		// Initialize node
		bool init();

		// Run node main loop
		void run();

	private:
    // Stop node callback 
		static void stopNodeCallback(int sig);

    // Compute command to be sent to passive ds controller
    void computeCommand();

    void backAndForthMotion();

    void multipleTargetsMotion();

    // Process mouse events
    void processMouseEvents();

    void processCursorEvent(float relX, float relY, bool newEvent);
    
    // Publish data to topics
    void publishData();

    // Publish data to topics
    void logData();

    // Callback to update real robot pose
    void updateRealTwist(const geometry_msgs::Twist::ConstPtr& msg);
    
    void updateRealPose(const geometry_msgs::Pose::ConstPtr& msg);

    void updateMouseData(const mouse_perturbation_robot::MouseMsg::ConstPtr& msg);

    // Convert quaternion to rotation matrix
    Eigen::Matrix3f quaternionToRotationMatrix(Eigen::Vector4f q);
    
    void initArduino();

    void closeArduino();

    void sendValueArduino(uint8_t value);
		// Dynamic reconfigure callback
		// void dynamicReconfigureCallback(foot_surgical_robot::footIsometricController_paramsConfig &config, uint32_t level);
};


#endif
