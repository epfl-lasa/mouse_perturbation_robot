#include "DSObstacleAvoidance.h"

DSObstacleAvoidance::DSObstacleAvoidance(Obstacle obs): _obs(obs)
{	
	ROS_INFO_STREAM("Obstacle parameters loaded");
}

void DSObstacleAvoidance::obsModulationEllipsoid(Eigen::Vector3f x, Eigen::Vector3f xd, bool bContour)
{
	_modulationMatrix.setConstant(0.0f);
	_rotationMatrix.setConstant(0.0f);
	for (int i = 0; i < 3; i++)
	{
		_modulationMatrix(i,i) = 1.0;
		_rotationMatrix(i,i) = 1.0;
	}

	x = x - _x0;

}

void DSObstacleAvoidance::computeBasisMatrix(Eigen::Vector3f x)
{
	_gamma = x(0)/_obs._a(0) + x(1)/_obs._a(1) + x(2)/_obs._a(2);
	_basisMatrix.setConstant(0.0f);
	
}