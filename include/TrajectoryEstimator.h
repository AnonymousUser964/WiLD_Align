#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <sstream>
#include <vector>
#include <chrono>




class TrajectoryEstimator
{
public:
    TrajectoryEstimator();
    Eigen::Matrix3d skew(const Eigen::Vector3d& v);
    Eigen::Matrix3d exp_so3(const Eigen::Vector3d& omega);
    //void arun_procrustes(const Eigen::MatrixXd& P, const Eigen::MatrixXd& Q,
    //                 Eigen::Matrix3d& R, Eigen::Vector3d& t);
    std::tuple<Eigen::Matrix3d, Eigen::Vector3d, std::vector<bool>>
     align_gnc_lm(const Eigen::MatrixXd& p,
          const Eigen::MatrixXd& q,
          const Eigen::VectorXd& r,
          const Eigen::VectorXd& sigma, 
          const Eigen::Matrix3d* R_in = nullptr,       
          const Eigen::Vector3d* t_in = nullptr,
          float noise_bound = 10.0,
          int max_outer = 100,
          int max_inner = 1000,
          double eta = 1.4,
          double lm_lambda_init = 1e-4,
          float lambda_up = 10,
          float lambda_down = 0.3,
          double tol = 1e-6
    );

    std::tuple<Eigen::Matrix3d, Eigen::Vector3d, std::vector<bool>>
    align_gnc_lm_2d(
        const Eigen::MatrixXd& p,
        const Eigen::MatrixXd& q,
        const Eigen::VectorXd& r,
        const Eigen::VectorXd& sigma,
        const Eigen::Matrix3d* R_in = nullptr,       
        const Eigen::Vector3d* t_in = nullptr,
        double noise_bound = 10.0,
        int flatten = 2,
        int max_outer = 100,
        int max_inner = 1000,
        double eta = 1.1,
        double lm_lambda_init = 1e-4,
        double lambda_up = 10.0,
        double lambda_down = 0.3,
        double tol = 1e-6
        );
    
    std::pair<Eigen::Matrix3d,Eigen::Vector3d> arun_procrustes(const Eigen::MatrixXd& P, const Eigen::MatrixXd& Q,
                     const Eigen::VectorXd& W_in);
    void addMeasurementConstraint(
        const Eigen::Vector3d& p, const Eigen::Vector3d& q, float r,
        double noise_bound, bool is_2d
    );
    
    Eigen::MatrixXd vector3dListToMatrix(const std::vector<Eigen::Vector3d>& vecs);
    Eigen::VectorXd floatVectorToEigen(const std::vector<float>& vec);
    void printTransform(const Eigen::Matrix3d& R, const Eigen::Vector3d& t);

    public:
    std::vector<Eigen::Vector3d> allP;
    std::vector<Eigen::Vector3d> allQ;
    std::vector<float> allR;
    std::vector<float> allSigma;

    std::vector<bool> inliers;
    std::vector<bool> cliqueInliers;

    bool first_time;

    std::pair<Eigen::Matrix3d, Eigen::Vector3d> currentEstimate = {Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero()};
};