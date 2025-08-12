#include "RigidAlignment.h"


Eigen::Matrix3d RigidAlignment::projectToSO3(const Eigen::Matrix3d& M) {
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d Vt = svd.matrixV().transpose();
    
    Eigen::Matrix3d R = U * Vt;
    
    if (R.determinant() < 0) {
        U.col(2) *= -1;  // Flip the sign of the last column
        R = U * Vt;
    }

    return R;
}


double RigidAlignment::frobeniusNorm(const Eigen::Matrix3d& A, const Eigen::Matrix3d& B) {
    return (A - B).norm();
}

void RigidAlignment::computeResiduals(
    const std::vector<Eigen::Matrix3d>& Rs,
    const std::vector<Eigen::Vector3d>& ts,
    const Eigen::Matrix3d& R_avg,
    const Eigen::Vector3d& t_avg,
    std::vector<double>& r_e,
    std::vector<double>& t_e
) {
    size_t N = Rs.size();
    r_e.resize(N);
    t_e.resize(N);
    for (size_t i = 0; i < N; ++i) {
        r_e[i] = frobeniusNorm(R_avg, Rs[i]);
        t_e[i] = (ts[i] - t_avg).norm();
    }
}


double RigidAlignment::initMu(const std::vector<double>& errors, double noise_sq) {
    double max_err_sq = 0.0;
    for (double e : errors) {
        max_err_sq = std::max(max_err_sq, e * e);
    }
    if (max_err_sq == 0.0) return 1.0;
    return std::max(1.0, 1.0 / (2.0 * max_err_sq / noise_sq - 1.0));
}

std::vector<double> RigidAlignment::tlsWeights(const std::vector<double>& err, double mu, double noise_sq) {
    std::vector<double> w(err.size(), 0.0);
    double th1 = (mu + 1.0) / mu * noise_sq;
    double th2 = mu / (mu + 1.0) * noise_sq;
    for (size_t i = 0; i < err.size(); ++i) {
        double err_sq = err[i] * err[i];
        if (err_sq >= th1) {
            w[i] = 0.0;
        } else if (err_sq <= th2) {
            w[i] = 1.0;
        } else {
            w[i] = std::sqrt(noise_sq * mu * (mu + 1.0) / err_sq) - mu;
        }
    }
    return w;
}


std::tuple<Eigen::Matrix3d, Eigen::Vector3d, std::vector<bool>>
RigidAlignment::gncTlsAverageRotations(
    const std::vector<Eigen::Matrix3d>& Rs,
    const std::vector<Eigen::Vector3d>& ts,
    double R_noise_bound_deg,
    double t_noise_bound,
    int max_outer,
    double eta,
    double mu_stop_factor) {
    size_t N = Rs.size();
    assert(Rs.size() == ts.size());

    double R_noise_bound = 2.0 * std::sqrt(2.0) * std::sin(M_PI * R_noise_bound_deg / 360.0);
    double R_sq = R_noise_bound * R_noise_bound;
    double T_sq = t_noise_bound * t_noise_bound;

    // Compute initial averages
    Eigen::Matrix3d R_sum = Eigen::Matrix3d::Zero();
    Eigen::Vector3d t_sum = Eigen::Vector3d::Zero();
    for (size_t i = 0; i < N; ++i) {
        R_sum += Rs[i];
        t_sum += ts[i];
    }
    Eigen::Matrix3d R_avg = projectToSO3(R_sum / N);
    Eigen::Vector3d t_avg = t_sum / N;

    std::vector<double> r_e, t_e;
    computeResiduals(Rs, ts, R_avg, t_avg, r_e, t_e);

    double mu_r = initMu(r_e, R_sq);
    double mu_t = initMu(t_e, T_sq);
    double mu_r_min = mu_stop_factor;
    double mu_t_min = mu_stop_factor;

    for (int outer = 0; outer < max_outer; ++outer) {
        computeResiduals(Rs, ts, R_avg, t_avg, r_e, t_e);

        std::vector<double> w_r = tlsWeights(r_e, mu_r, R_sq);
        std::vector<double> w_t = tlsWeights(t_e, mu_t, T_sq);

        std::vector<double> w(N, 0.0);
        double w_sum = 0.0;
        for (size_t i = 0; i < N; ++i) {
            w[i] = w_r[i] * w_t[i];
            w_sum += w[i];
        }

        if (w_sum == 0.0) break;

        // Weighted rotation average
        Eigen::Matrix3d R_weighted = Eigen::Matrix3d::Zero();
        for (size_t i = 0; i < N; ++i) {
            R_weighted += w[i] * Rs[i];
        }
        R_avg = projectToSO3(R_weighted / w_sum);

        // Weighted translation average
        t_avg = Eigen::Vector3d::Zero();
        for (size_t i = 0; i < N; ++i) {
            t_avg += w[i] * ts[i];
        }
        t_avg /= w_sum;

        mu_r = std::max(mu_r * eta, mu_r_min);
        mu_t = std::max(mu_t * eta, mu_t_min);
    }

    computeResiduals(Rs, ts, R_avg, t_avg, r_e, t_e);

    std::vector<bool> inliers(N, false);
    for (size_t i = 0; i < N; ++i) {
        inliers[i] = (r_e[i] < R_noise_bound) && (t_e[i] < t_noise_bound);
    }

    return {R_avg, t_avg, inliers};
}

void RigidAlignment::addRelativeMeasure(const Eigen::Matrix4d& T, int i, int j){
	index[{i,j}] = R.size();
	R.push_back(T.block<3,3>(0, 0));
	t.push_back(T.block<3,1>(0, 3));
    shouldUse.push_back(true);
}

void RigidAlignment::replaceRelativeMeasure(const Eigen::Matrix4d& T, int i, int j){
	if(index.find({i,j}) != index.end()){
        int ind = index[{i,j}];
        R[ind] = T.block<3,3>(0, 0);
        t[ind] = T.block<3,1>(0, 3);
        shouldUse[ind] = true;
    }
}

void RigidAlignment::getAlignent(std::vector<int> values){
	std::vector<Eigen::Matrix3d> Rs;
	std::vector<Eigen::Vector3d> ts;
	
	for(int val : values){
        if(!shouldUse[val]) continue;
		Rs.push_back(R[val]);
		ts.push_back(t[val]);
	}
	auto res = gncTlsAverageRotations(Rs,ts);
	R_est = std::get<0>(res);
	t_est = std::get<1>(res);
	estimateExists = true;
}

void RigidAlignment::stopUsing(int i, int j){
    if(index.find({i,j}) != index.end()){
        int ind = index[{i,j}];
        shouldUse[ind] = false;
    }
}


RigidAlignment::RigidAlignment(){}

void RigidAlignment::printMatrix3d(const Eigen::MatrixXd& mat, const std::string& label) {
    std::stringstream ss;
    ss << "\n" << label << ":\n";
    ss << mat;
    ROS_INFO_STREAM(ss.str());
}



void RigidAlignment::printVector3d(const Eigen::Vector3d& vec, const std::string& label) {
    ROS_INFO_STREAM(label << ": [" << vec(0) << ", " << vec(1) << ", " << vec(2) << "]");
}
