#include "MotionGenerator.h"
#include <signal.h>
#include <fcntl.h>
#include <termios.h>

MotionGenerator* MotionGenerator::me = NULL;

MotionGenerator::MotionGenerator(ros::NodeHandle &n, double frequency): _n(n), _loopRate(frequency), _dt(1 / frequency)
{
	me = this;
	ROS_INFO_STREAM("The motion generator node is created at: " << _n.getNamespace() << " with freq: " << frequency << "Hz");

	//obstacle definition
	_obs._a << 0.5f,0.1f,0.12f;
	_obs._p.setConstant(1.0f);
	_obs._safetyFactor = 1.0f;
	_obs._tailEffect = false;
	_obs._bContour = false;
	_obs._rho = 1.0f;
}


bool MotionGenerator::init() 
{
	// Variable initialization
  _wRb.setConstant(0.0f);
  _x.setConstant(0.0f);
  _q.setConstant(0.0f);
  _x0.setConstant(0.0f);

  _qd.setConstant(0.0f);
  _omegad.setConstant(0.0f);
  _xd.setConstant(0.0f);
  _vd.setConstant(0.0f);

  _xp.setConstant(0.0f);
  _mouseVelocity.setConstant(0.0f);
  _targetOffset.col(Target::A) << 0.0f, 0.0f, 0.0f;
  _targetOffset.col(Target::B) << 0.0f, 0.85f, 0.0f;
  _targetOffset.col(Target::C) << -0.16f,0.25f,0.0f;
  _targetOffset.col(Target::D) << -0.16f,-0.25f,0.0f;
  _perturbationOffset.setConstant(0.0f);
  _phaseDuration = 0.0f;
  _minCleanMotionDuration = 5.0f;
  _maxCleanMotionDuration = 12.0f;
  _jerkyMotionDuration = 0.4f;
  _initDuration = 10.0f;
  _pauseDuration = 0.4f;
  _reachedTime = 0.0f;
  _trialCount = 0;
  _perturbationCount = 0;
  _lastMouseEvent = mouse_perturbation_robot::MouseMsg::M_NONE;
  _errorButtonCounter = 0;

  _firstRealPoseReceived = false;
  _firstMouseEventReceived = false;
  _stop = false;
  _perturbation = false;
  _mouseControlledMotion = false;
  _mouseInUse = false;
  _useArduino = false;
  _perturbationFlag = false;
  _switchingTrajectories = false;
  _errorButtonPressed = false;

	_state = State::INIT;
	_previousTarget = Target::A;
	_currentTarget = Target::B;

	Eigen::Vector3f temp;
	temp << 0.0f,0.0f,1.0f;
	_motionDirection = _targetOffset.col(_currentTarget)-_targetOffset.col(_previousTarget);
	_motionDirection.normalize();
	_perturbationDirection = temp.cross(_motionDirection);
	_perturbationDirection.normalize();

	// Subscriber definitions
	_subMouse= _n.subscribe("/mouse", 1, &MotionGenerator::updateMouseData, this, ros::TransportHints().reliable().tcpNoDelay());
	_subRealPose = _n.subscribe("/lwr/ee_pose", 1, &MotionGenerator::updateRealPose, this, ros::TransportHints().reliable().tcpNoDelay());
	_subRealTwist = _n.subscribe("/lwr/ee_vel", 1, &MotionGenerator::updateRealTwist, this, ros::TransportHints().reliable().tcpNoDelay());

	// Publisher definitions
	_pubDesiredTwist = _n.advertise<geometry_msgs::Twist>("/lwr/joint_controllers/passive_ds_command_vel", 1);
	_pubDesiredOrientation = _n.advertise<geometry_msgs::Quaternion>("/lwr/joint_controllers/passive_ds_command_orient", 1);
	
	// Dynamic reconfigure definition
	_dynRecCallback = boost::bind(&MotionGenerator::dynamicReconfigureCallback, this, _1, _2);
	_dynRecServer.setCallback(_dynRecCallback);

	// Open file to save data
	_outputFile.open ("src/mouse_perturbation_robot/informationKUKA.txt");
	_outputFile << "NEW EXPERIMENT\n";
	
	// Catch CTRL+C event with the callback provided
	signal(SIGINT,MotionGenerator::stopNodeCallback);

	// Initialize arduino
	if(_useArduino)
	{
		initArduino();
	}

	// Check if node OK
	if (_n.ok()) 
	{ 
		// Wait for poses being published
		ros::spinOnce();
		ROS_INFO("The motion generator is ready.");
		return true;
	}
	else 
	{
		ROS_ERROR("The ros node has a problem.");
		return false;
	}
}


int MotionGenerator::getch()
{
	static struct termios oldt, newt;
	tcgetattr( STDIN_FILENO, &oldt);           // save old settings
	newt = oldt;
	newt.c_lflag &= ~(ICANON);                 // disable buffering   
	newt.c_cc[VMIN] = 0;
	newt.c_cc[VTIME] = 0;   
	tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

	int c = getchar();  // read character (non-blocking)

	tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings

	return c;
}


void MotionGenerator::run()
{
	srand (time(NULL));
	
	// Initialize motion duration and initial time reference
	_phaseDuration = _minCleanMotionDuration+(_maxCleanMotionDuration-_minCleanMotionDuration)*((float)std::rand()/RAND_MAX);
	_initTime = ros::Time::now().toSec();

	while (!_stop) 
	{
		// Check if we received the robot pose and foot data
		if(_firstRealPoseReceived)
		{
			// Compute control command
			computeCommand();

			// Start monitoring the keyboard
			if (getch() == ' ')
			{
				_errorButtonPressed = true;
				_errorButtonCounter = 0;
				ROS_INFO_STREAM("Received key press");
			}

			// Log data
			logData();

			// Publish data to topics
			publishData();
		}
		else
		{
			_initTime = ros::Time::now().toSec();
		}

		ros::spinOnce();

		_loopRate.sleep();
	}

	// Close arduino communication
	if(_useArduino)
	{
		closeArduino();
	}

	// Send zero linear and angular velocity to stop the robot
	_vd.setConstant(0.0f);
	_omegad.setConstant(0.0f);
	_qd = _q;

	publishData();
	ros::spinOnce();
	_loopRate.sleep();

  // Close file
	_outputFile.close();

  // Close ros
	ros::shutdown();
}


void MotionGenerator::stopNodeCallback(int sig)
{
	me->_stop = true;
}


void  MotionGenerator::computeCommand()
{
  if(!_mouseControlledMotion)
  {
  	// Back and forth motion
    backAndForthMotion();
  }
  else
  {
  	// Mouse controlled motion
    processMouseEvents();
    mouseControlledMotion(); 
  }
}


void MotionGenerator::backAndForthMotion()
{
	// Update current time
	double currentTime = ros::Time::now().toSec();

	Eigen::Vector3f gains, error;
	gains.setConstant(0.0f);
	error.setConstant(0.0f);

	switch (_state)
	{
		case State::INIT:
		{
			_phaseDuration = _initDuration;
		}
		case State::CLEAN_MOTION:
		{
			// Compute desired target position
			_xd = _x0+_targetOffset.col(_currentTarget);

			// Compute distance to target
			float distance = (_xd-_x).norm();
			if(distance < TARGET_TOLERANCE)
			{
				// Target is reached 
				_trialCount++;
				_previousTarget = _currentTarget;

				// Update target
				if (_currentTarget == Target::A)
				{	
					_currentTarget = Target::B;
					if(_useArduino)
					{
						sendValueArduino(4);
					}
				}
				else
				{
					_currentTarget = Target::A;
					if(_useArduino)
					{
						sendValueArduino(2);
					}
				}
				_obs._x0 = _x0 + (_targetOffset.col(_currentTarget)+_targetOffset.col(_previousTarget))/2;
				_obs._x0(2) -= 0.05f;

				if (_switchingTrajectories and (float)std::rand()/RAND_MAX>0.5)
				{
					_obs._safetyFactor = 1.0f + 0.5f*(float)std::rand()/RAND_MAX;
					_obs._rho = 1.0f + 7*(float)std::rand()/RAND_MAX;
					ROS_INFO_STREAM("Switching Trajectory parameters. Safety Factor: " << _obs._safetyFactor << "Rho: " << _obs._rho);	
				}

				obsModulator.setObstacle(_obs);
				// Update motion and perturbation direction
				Eigen::Vector3f temp;
				temp << 0.0f,0.0f,1.0f;
				_motionDirection = _targetOffset.col(_currentTarget)-_targetOffset.col(_previousTarget);
				_motionDirection.normalize();
				_perturbationDirection = temp.cross(_motionDirection);
				_perturbationDirection.normalize();

				// Go in pause state
				_reachedTime = ros::Time::now().toSec();
				_state = State::PAUSE;
			}

			// Compute the gain matrix M = B*L*B'
			// B is an orthogonal matrix containing the directions corrected
			// L is a diagonal matrix defining the correction gains along the directions
			Eigen::Matrix3f B,L;
			B.col(0) = _motionDirection;
			B.col(1) = _perturbationDirection;
			B.col(2) << 0.0f,0.0f,1.0f;
			gains << 10.0f, 10.0f, 30.0f;

			// Compute error and desired velocity
			error = _xd-_x;
			L = gains.asDiagonal();
			_vd = B*L*B.transpose()*error;

			_vd = obsModulator.obsModulationEllipsoid(_x, _vd, false);
			// Check for end of clean motion phase
			if(currentTime-_initTime > _phaseDuration and _perturbationFlag)
			{
				// Go to jerky motion phase
				_perturbation = true;
				_state = State::JERKY_MOTION;
				_initTime = ros::Time::now().toSec();
				_phaseDuration = _jerkyMotionDuration;
				if(_useArduino)
				{
					sendValueArduino(1);
				}
			}
			break;
		}
		case State::PAUSE:
		{
			// Send zero velocity
			_vd.setConstant(0.0f);

			// Check for end of pause
			if(currentTime-_reachedTime>_pauseDuration)
			{
				// Go back to clean motion phase
				_state = State::CLEAN_MOTION;
			}
			break;
		}
		case State::JERKY_MOTION:
		{
			_perturbationDirection << 0.0f,0.0f,1.0f;
			// Update perturbation offset based on perturbation velocity + apply saturation
			_perturbationOffset += PERTURBATION_VELOCITY*(-1+2*(float)std::rand()/RAND_MAX)*_dt*_perturbationDirection;
			if(_perturbationOffset.norm()>MAX_PERTURBATION_OFFSET)
			{
				_perturbationOffset *= MAX_PERTURBATION_OFFSET/_perturbationOffset.norm();
			}
			while(_perturbationOffset.norm()< MIN_PERTURBATION_OFFSET)
			{
				_perturbationOffset = MAX_PERTURBATION_OFFSET*(-1+2*(float)std::rand()/RAND_MAX)*_perturbationDirection;
			}

			// Compute desired position by considering perturbation offset
			_xd = _x+_perturbationOffset;

			// Compute the gain matrix M = B*L*B'
			// The gain along the motion direction is set to zero to stay in place 
			// The gain along the z axis is kept to keep the height
			Eigen::Matrix3f B,L;
			B.col(0) = _motionDirection;
			B.col(1) = _perturbationDirection;
			B.col(2) << 1.0f,0.0f,0.0f;
			gains << 0, 30.0f, 10.0f;

			error = _xd-_x;
			L = gains.asDiagonal();
			_vd = B*L*B.transpose()*error;

			// Check for end of jerky motion phase
			if(currentTime-_initTime > _phaseDuration)
			{
				// Update perturbation count + go to clean motion phase
				_perturbationCount++;
				_perturbationOffset.setConstant(0.0f);
				_perturbation = false;
				_state = State::CLEAN_MOTION;
				_initTime = ros::Time::now().toSec();
				_phaseDuration = _minCleanMotionDuration+(_maxCleanMotionDuration-_minCleanMotionDuration)*((float)std::rand()/RAND_MAX);
				if(_useArduino)
				{
					sendValueArduino(0);
				}
			}
			break;
		}
		default:
		{
			break;
		}
	}

	// ROS_INFO_STREAM("Input " << _vd);
	// ROS_INFO_STREAM("Output " << obsModulator.obsModulationEllipsoid(_x, _vd, false));

	// Bound desired velocity
	if (_vd.norm()>0.3f)
	{
		_vd = _vd*0.3f/_vd.norm();
	}

	// Desired quaternion to have the end effector looking down
	_qd << 0.0f, 0.0f, 1.0f, 0.0f;

	// std::cerr << "trialCount: " << _trialCount << " target: " << (int) (_currentTarget) << " perturbation: " << (int) (_perturbation) << " perturbationCount: " << _perturbationCount << std::endl;
	// std::cerr << "target: " << _xd << " position: " << _x  << std::endl;
	// std::cerr << "obstacle: " << _obs._x0  << std::endl;
}


void MotionGenerator::mouseControlledMotion()
{
	// Update current time
	double currentTime = ros::Time::now().toSec();

	Eigen::Vector3f gains, error;
	gains.setConstant(0.0f);
	error.setConstant(0.0f);

	Target temporaryTarget;

	_obs._x0 = _x0 + (_targetOffset.col(Target::A)+_targetOffset.col(Target::B))/2;
	_obs._x0(2) -= 0.05f;

	switch (_state)
	{
		case State::INIT:
		{
			_phaseDuration = _initDuration;
		}
		case State::CLEAN_MOTION:
		{
			// Check if mouse is in use
			if(_mouseInUse)
			{
				// Save current target
				temporaryTarget = _currentTarget;

				// Update target from mouse input
				if(fabs(_mouseVelocity(0))>fabs(_mouseVelocity(1)))
				{
					if(_mouseVelocity(0)>0.0f)
					{
						_currentTarget = Target::A;
					}
					else
					{
						_currentTarget = Target::B;
					}
					_obs._a(0) = 0.5f;
					_obs._a(1) = 0.1f;
					obsModulator.setObstacle(_obs);
				}
				else
				{
					if(_mouseVelocity(1)>0.0f)
					{
						_currentTarget = Target::D;
					}
					else
					{
						_currentTarget = Target::C;
					}
					_obs._a(0) = 0.1f;
					_obs._a(1) = 0.5f;
					obsModulator.setObstacle(_obs);
				}

				// If new target, updates previous one and compte new motion and perturbation direction
				if(_currentTarget != temporaryTarget)
				{
					_previousTarget = temporaryTarget;
					
					// Update motion and perturbation direction
					Eigen::Vector3f temp;
					temp << 0.0f,0.0f,1.0f;
					_motionDirection = _targetOffset.col(_currentTarget)-_targetOffset.col(_previousTarget);
					_motionDirection.normalize();
					_perturbationDirection = temp.cross(_motionDirection);
					_perturbationDirection.normalize();
				}

				// Compute desired target position
				_xd = _x0+_targetOffset.col(_currentTarget);

				// Compute distance to target
				float distance = (_xd-_x).norm();
				if(distance < TARGET_TOLERANCE)
				{
					// Target is reached 
					_trialCount++;
					_previousTarget = _currentTarget;

					// Update target
					_reachedTime = ros::Time::now().toSec();
					_state = State::PAUSE;
				}

				// Compute the gain matrix M = B*L*B'
				// B is an orthogonal matrix containing the directions corrected
				// L is a diagonal matrix defining the correction gains along the directions
				Eigen::Matrix3f B,L;
				B.col(0) = _motionDirection;
				B.col(1) = _perturbationDirection;
				B.col(2) << 0.0f,0.0f,1.0f;
				gains << 3, 10.0f, 30.0f;

				// Compute error and desired velocity
				error = _xd-_x;
				L = gains.asDiagonal();
				_vd = B*L*B.transpose()*error;
				_vd = obsModulator.obsModulationEllipsoid(_x, _vd, false);
				_xp = _x;
			}
			else
			{
				// Track the last position where the mouse was in use
				Eigen::Matrix3f L;
				gains << 3, 3, 3;
				error = _xp-_x;
				L = gains.asDiagonal();
				_vd = L*error;

			}

			// Check for end of clean motion phase
			if(currentTime-_initTime > _phaseDuration && _v.norm()> 1.0e-2f)
			{
				// Go to jerky motion phase
        		_perturbation = true;
				_state = State::JERKY_MOTION;
				_initTime = ros::Time::now().toSec();
				_phaseDuration = _jerkyMotionDuration;			
				if(_useArduino)
				{
					sendValueArduino(1);
				}
			}
			else if(currentTime-_initTime > _phaseDuration)
			{
				_initTime = ros::Time::now().toSec();
			}
			break;
		}
		case State::PAUSE:
		{
			_vd.setConstant(0.0f);

			// Check for end of pause
			if(currentTime-_reachedTime>_pauseDuration)
			{
				// Go back to clean motion phase
				_state = State::CLEAN_MOTION;
			}
			break;
		}
		case State::JERKY_MOTION:
    	{
	    	_perturbationDirection << 0.0f,0.0f,1.0f;
				// Update perturbation offset based on perturbation velocity + apply saturation
	      _perturbationOffset += PERTURBATION_VELOCITY*(-1+2*(float)std::rand()/RAND_MAX)*_dt*_perturbationDirection;
	      if(_perturbationOffset.norm()>MAX_PERTURBATION_OFFSET)
	      {
	        _perturbationOffset *= MAX_PERTURBATION_OFFSET/_perturbationOffset.norm();
	      }
	      while(_perturbationOffset.norm()< MIN_PERTURBATION_OFFSET)
	      {
	        _perturbationOffset = MAX_PERTURBATION_OFFSET*(-1+2*(float)std::rand()/RAND_MAX)*_perturbationDirection;
	      }

			// Compute desired position by considering perturbation offset
			_xd = _x+_perturbationOffset;

			// Compute the gain matrix M = B*L*B'
			// The gain along the motion direction is set to zero to stay in place 
			// The gain along the z axis is kept to keep the height
			Eigen::Matrix3f B,L;
			B.col(0) = _motionDirection;
			B.col(1) = _perturbationDirection;
			B.col(2) << 1.0f,0.0f,0.0f;
			gains << 0.0f, 10.0f, 30.0f;

			error = _xd-_x;
			L = gains.asDiagonal();
			_vd = B*L*B.transpose()*error;

			// Check for end of jerky motion phase
			if(currentTime-_initTime > _phaseDuration)
			{
				// Update perturbation count + go to clean motion phase
				_perturbationCount++;
        		_perturbation = false;
				_perturbationOffset.setConstant(0.0f);
				_state = State::CLEAN_MOTION;
				_initTime = ros::Time::now().toSec();
				_phaseDuration = 10+(20-10)*((float)std::rand()/RAND_MAX);
				if(_useArduino)
				{
					sendValueArduino(0);
				}
			}
			break;
		}
		default:
		{
			break;
		}
	}

	// Bound desired velocity
	if (_vd.norm()>0.3f)
	{
		_vd = _vd*0.3f/_vd.norm();
	}

	// Desired quaternion to have the end effector looking down
	_qd << 0.0f, 0.0f, 1.0f, 0.0f;

  std::cerr << "Current target: " << (int) (_currentTarget) << " Previous target: " << (int) (_previousTarget) << " perturbation: " << (int) (_perturbation) << " mouse in use: "<< _mouseInUse << std::endl;
  std::cerr << " perturbationDirection: " << _perturbationDirection.transpose() << " speed: " << _v.norm() <<std::endl;
}


void MotionGenerator::processMouseEvents()
{
  uint8_t event;
  int buttonState, relX, relY, relWheel;
  float filteredRelX = 0.0f, filteredRelY = 0.0f;
  bool newEvent = false;

  // If new event received update last event
  // Otherwhise keep the last one
  if(_msgMouse.event > 0)
  {
    _lastMouseEvent = _msgMouse.event;
    buttonState = _msgMouse.buttonState;
    relX = _msgMouse.relX;
    relY = _msgMouse.relY;
    relWheel = _msgMouse.relWheel;
    filteredRelX = _msgMouse.filteredRelX;
    filteredRelY = _msgMouse.filteredRelY;
    newEvent = true;
  }
  else
  {
    buttonState = 0;
    relX = 0;
    relY = 0;
    relWheel = 0;
    filteredRelX = 0;
    filteredRelY = 0;
    newEvent = false;
  }

  event = _lastMouseEvent;

  // Process corresponding event
  switch(event)
  {
    case mouse_perturbation_robot::MouseMsg::M_CURSOR:
    {
      processCursorEvent(filteredRelX,filteredRelY,newEvent);
      break;
    }
    default:
    {
      break;
    }
  }
}


void MotionGenerator::processCursorEvent(float relX, float relY, bool newEvent)
{
  if(!newEvent) // No new event received
  {
    _mouseVelocity.setConstant(0.0f);
  }
  else
  {
  	_mouseInUse = false;
  	// If absolute value higher than min mouse velocity threshold, the mouse is in use,
  	// otherwise mouse velocity is set to zero
    if(fabs(relX)>MIN_XY_REL)
    {
      _mouseVelocity(0) = relX;
      _mouseInUse = true;
    }
    else
    {
    	_mouseVelocity(0) = 0.0f;
    }

    if(fabs(relY)>MIN_XY_REL)
    {
      _mouseVelocity(1) = relY;
      _mouseInUse = true;
    }
    else
    {
    	_mouseVelocity(1) = 0.0f;
    } 
  }
}


void MotionGenerator::publishData()
{
	_mutex.lock();

	// Publish desired twist (passive ds controller)
	_msgDesiredTwist.linear.x  = _vd(0);
	_msgDesiredTwist.linear.y  = _vd(1);
	_msgDesiredTwist.linear.z  = _vd(2);
	_msgDesiredTwist.angular.x = _omegad(0);
	_msgDesiredTwist.angular.y = _omegad(1);
	_msgDesiredTwist.angular.z = _omegad(2);

	_pubDesiredTwist.publish(_msgDesiredTwist);

	// Publish desired orientation
	_msgDesiredOrientation.w = _qd(0);
	_msgDesiredOrientation.x = _qd(1);
	_msgDesiredOrientation.y = _qd(2);
	_msgDesiredOrientation.z = _qd(3);

	_pubDesiredOrientation.publish(_msgDesiredOrientation);

	_mutex.unlock();
}


void MotionGenerator::logData()
{
	_outputFile << ros::Time::now() << " " << _x(0) << " " << _x(1) << " " << _x(2) << " " << (int)(_perturbationFlag) << " " << (int)(_switchingTrajectories) << " " << _obs._p(0) << " " << _obs._safetyFactor << " " << _obs._rho << " " << (int)(_errorButtonPressed) << std::endl;
	if (_errorButtonPressed and _errorButtonCounter > 4)
	{
		_errorButtonPressed = false;
	}
	else
		_errorButtonCounter++;
}


void MotionGenerator::updateRealPose(const geometry_msgs::Pose::ConstPtr& msg)
{
	_msgRealPose = *msg;

	// Update end effecotr pose (position+orientation)
	_x << _msgRealPose.position.x, _msgRealPose.position.y, _msgRealPose.position.z;
	_q << _msgRealPose.orientation.w, _msgRealPose.orientation.x, _msgRealPose.orientation.y, _msgRealPose.orientation.z;
  _wRb = quaternionToRotationMatrix(_q);

	if(!_firstRealPoseReceived)
	{
		_firstRealPoseReceived = true;
		_xd = _x;
		_qd = _q;
		_x0 = _xd;
		_xp = _x;
		_vd.setConstant(0.0f);
		_obs._x0 = _x0 + (_targetOffset.col(_currentTarget)+_targetOffset.col(_previousTarget))/2;
		_obs._x0(2) -= 0.05f;
		obsModulator.setObstacle(_obs);
	}
}


void MotionGenerator::updateRealTwist(const geometry_msgs::Twist::ConstPtr& msg)
{
	_v << msg->linear.x, msg->linear.y, msg->linear.z;
}


void MotionGenerator::updateMouseData(const mouse_perturbation_robot::MouseMsg::ConstPtr& msg)
{
  _msgMouse = *msg;

  if(!_firstMouseEventReceived && _msgMouse.event > 0)
  {
    _firstMouseEventReceived = true;
  }
}


Eigen::Matrix3f MotionGenerator::quaternionToRotationMatrix(Eigen::Vector4f q)
{
  Eigen::Matrix3f R;

  float q0 = q(0);
  float q1 = q(1);
  float q2 = q(2);
  float q3 = q(3);

  R(0,0) = q0*q0+q1*q1-q2*q2-q3*q3;
  R(1,0) = 2.0f*(q1*q2+q0*q3);
  R(2,0) = 2.0f*(q1*q3-q0*q2);

  R(0,1) = 2.0f*(q1*q2-q0*q3);
  R(1,1) = q0*q0-q1*q1+q2*q2-q3*q3;
  R(2,1) = 2.0f*(q2*q3+q0*q1);

  R(0,2) = 2.0f*(q1*q3+q0*q2);
  R(1,2) = 2.0f*(q2*q3-q0*q1);
  R(2,2) = q0*q0-q1*q1-q2*q2+q3*q3;  

  return R;
}


void MotionGenerator::dynamicReconfigureCallback(mouse_perturbation_robot::obstacleAvoidance_paramsConfig &config, uint32_t level)
{
	// ROS_INFO("Reconfigure Request: %d %s %s %f %f", 
 //            config.obstacle_shape_param,
 //            config.perturbation_flag?"True":"False",
 //            config.random_trajectory_switching?"True":"False",
 //            config.obstacle_safety_factor,
 //            config.obstacle_rho);

	_obs._p.setConstant(config.obstacle_shape_param);
	_perturbationFlag = config.perturbation_flag;
	_switchingTrajectories = config.random_trajectory_switching;

	if (_switchingTrajectories)
		ROS_WARN("Cannot change safety factor or rho if random switching is on");
	else
	{
		_obs._safetyFactor = config.obstacle_safety_factor;
		_obs._rho = config.obstacle_rho;
	}
	obsModulator.setObstacle(_obs);

	_jerkyMotionDuration = config.jerky_motion_duration;
	_pauseDuration = config.pause_duration;
}


void MotionGenerator::closeArduino()
{
  close(farduino);
}


void MotionGenerator::initArduino()
{
  struct termios toptions;

  farduino = open("//dev//ttyACM0", O_RDWR | O_NONBLOCK );

  if (farduino == -1)
  {
    perror("serialport_init: Unable to open port ");
  }

  if (tcgetattr(farduino, &toptions) < 0)
  {
    perror("serialport_init: Couldn't get term attributes");
  }
     
  cfsetispeed(&toptions, B115200);
  cfsetospeed(&toptions, B115200);

  // 8N1
  toptions.c_cflag &= ~PARENB;
  toptions.c_cflag &= ~CSTOPB;
  toptions.c_cflag &= ~CSIZE;
  toptions.c_cflag |= CS8;
  // No flow control
  toptions.c_cflag &= ~CRTSCTS;

  toptions.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
  toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl

  toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
  toptions.c_oflag &= ~OPOST; // make raw

  toptions.c_cc[VMIN]  = 0;
  toptions.c_cc[VTIME] = 0;
  //toptions.c_cc[VTIME] = 20;

  tcsetattr(farduino, TCSANOW, &toptions);
  if( tcsetattr(farduino, TCSAFLUSH, &toptions) < 0)
  {
    perror("init_serialport: Couldn't set term attributes");
  }
}


void MotionGenerator::sendValueArduino(uint8_t value)
{
  write(farduino,&value,1);
  std::cout << "Arduino message " << (int)value << std::endl;
  if (value>0)
  {
    trigger_begin = ros::Time::now();
    trigger_raised = true;
  }
  else
  {
    trigger_raised = false;
  }
}

