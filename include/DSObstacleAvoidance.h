#ifndef __DS_OBSTACLE_AVOIDANCE_H__
#define __DS_OBSTACLE_AVOIDANCE_H__

#include <vector>
#include "Eigen/Eigen"
#include "ros/ros.h"

struct Obstacle
{
    Eigen::Vector3f _a, _p, _x0;
    double _safetyFactor, _rho, _thR;
    bool _tailEffect, _bContour;
};

class DSObstacleAvoidance
{
	private:

		Obstacle _obs;
		Eigen::Vector3f _modulatedVel;
    	Eigen::Matrix3f _modulationMatrix, _rotationMatrix, _basisMatrix;
    	double _gamma;

	public:

		DSObstacleAvoidance(Obstacle obs);

		void obsModulationEllipsoid(Eigen::Vector3f x, Eigen::Vector3f xd, bool bContour);

		void computeBasisMatrix(Eigen::Vector3f x);

};