#pragma once

#include <ros/ros.h>
#include <sstream>

#include <map>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <numeric>
#include <iostream>




class RigidAlignment
{
public:
	RigidAlignment();
	Eigen::Matrix3d projectToSO3(const Eigen::Matrix3d& M);
	double frobeniusNorm(const Eigen::Matrix3d& A, const Eigen::Matrix3d& B);
	void computeResiduals(
	    const std::vector<Eigen::Matrix3d>& Rs,
	    const std::vector<Eigen::Vector3d>& ts,
	    const Eigen::Matrix3d& R_avg,
	    const Eigen::Vector3d& t_avg,
	    std::vector<double>& r_e,
	    std::vector<double>& t_e);
	double initMu(const std::vector<double>& errors, double noise_sq);
	std::vector<double> tlsWeights(const std::vector<double>& err, double mu, double noise_sq);
	std::tuple<Eigen::Matrix3d, Eigen::Vector3d, std::vector<bool>>
	gncTlsAverageRotations(
	    const std::vector<Eigen::Matrix3d>& Rs,
	    const std::vector<Eigen::Vector3d>& ts,
	    double R_noise_bound_deg = 10.0,
	    double t_noise_bound = 1.0,
	    int max_outer = 30,
	    double eta = 0.5,
	    double mu_stop_factor = 1.5
	);
	void addRelativeMeasure(const Eigen::Matrix4d& T, int i, int j);
	void replaceRelativeMeasure(const Eigen::Matrix4d& T, int i, int j);
	void getAlignent(std::vector<int> values);

	void printMatrix3d(const Eigen::MatrixXd& mat, const std::string& label = "Matrix");
	void printVector3d(const Eigen::Vector3d& vec, const std::string& label = "Vector");
	void stopUsing(int i, int j);

	
public:
	Eigen::Matrix3d R_est = Eigen::Matrix3d::Identity();
	Eigen::Vector3d t_est = Eigen::Vector3d::Zero();
	std::vector<Eigen::Matrix3d> R;
	std::vector<Eigen::Vector3d> t;
	bool estimateExists = false;
	std::map<std::pair<int,int>,int> index;
	std::vector<bool> shouldUse;
};


