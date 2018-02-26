#include "DSObstacleAvoidance.h"

DSObstacleAvoidance::DSObstacleAvoidance(Obstacle obs): _obs(obs)
{	
	ROS_INFO_STREAM("Obstacle parameters loaded");
}

void DSObstacleAvoidance::obsModulationEllipsoid(Eigen::Vector3f x, Eigen::Vector3f xd, bool bContour)
{
	_modulationMatrix.setIdentity();

	_rotationMatrix.setIdentity();

	x = _rotationMatrix*(x - _obs._x0);
	computeBasisMatrix(x);

	Eigen::Vector3f eig_vals, d0;
	d0.setConstant(1.0f);
	d0(1) = -1.0;
	eig_vals = d0/(_gamma.pow(1/_obs._rho));
	if(_obs._tailEffect and xd.dot(_rotationMatrix*_basisMatrix.col(0))>=0)
		eig_vals(0) = 0.0;

	if(eig_vals(0)<-1.0)
	{
		eig_vals.tail(2).setConstant(1.0f);
		if(xd.dot(_rotationMatrix*_basisMatrix.col(0))<0)
			eig_vals(0) = -1.0;
	}

	eig_vals = eig_vals.array() + 1;
	_modulationMatrix = _rotationMatrix*_basisMatrix*(eig_vals.asDiagonal())*(_basisMatrix.inverse())*_rotationMatrix.transpose();

	_basisMatrix = _rotationMatrix*_basisMatrix;

	if(not bContour and eig_vals(0)<-0.98 and xd.dot(_basisMatrix.col(0))<0 and (_modulationMatrix*xd).norm()<0.02)
		bContour = true;

	if(bContour)
	{
		
	}
}

void DSObstacleAvoidance::computeBasisMatrix(Eigen::Vector3f x)
{
	Eigen::Vector3f nv;
	nv = x.array()/_obs.a.array();
	_gamma = nv.array().pow(2*_obs._p.array()).sum();
	_basisMatrix.setConstant(0.0f);
	nv = (2*_obs._p.array()/_obs.a.array())*(nv.array().pow(2*_obs._p.array()-1));

	_basisMatrix.col(0) = nv;
	_basisMatrix.block(0,1,1,2) = nv.tail(2).transpose();
	_basisMatrix.block(1,1,2,2).setIdentity();
	_basisMatrix.block(1,1,2,2)*= nv(1);
}