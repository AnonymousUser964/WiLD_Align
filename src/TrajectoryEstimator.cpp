#include "TrajectoryEstimator.h"
#include <pmc/pmc_graph.h>
#include <pmc/pmc_maxclique.h>
#include <pmc/pmc_input.h>
#include <pmc/pmc_heu.h>
#include <ros/ros.h>

struct ConsistencyGraph {
  std::vector<std::vector<int>> adj;   // adjacency lists (sorted)
  Eigen::MatrixXd adj_matrix;          // optional dense view (N×N), 0/1
};

std::vector<int> find_pmc_clique(const Eigen::MatrixXd& adj, int workers, double timelimit=60, int lb=0, bool use_heu=true) {
    const size_t nnodes = adj.rows();

    std::vector<int> edges;
    std::vector<long long> vertices;

    vertices.push_back(0);
    size_t total_num_edges = 0;
    for (size_t i=0; i<nnodes; i++) {
        for (size_t j=0; j<nnodes; j++) {
            if (adj(i, j) == 1) {
                edges.push_back(j);
                total_num_edges++;
            }
        }
        vertices.push_back(total_num_edges);
    }
    pmc::pmc_graph G(vertices, edges);
    pmc::input in;

    in.algorithm = 0;
    in.threads = workers;
    in.experiment = 0;
    in.lb = 0;
    in.ub = 0;
    in.param_ub = 0;
    in.adj_limit = 20000;
    in.time_limit = 60 * timelimit;
    in.remove_time = 4;
    in.graph_stats = false;
    in.verbose = false;
    in.help = false;
    in.MCE = false;
    in.decreasing_order = false;
    in.heu_strat = "kcore";
    in.vertex_search_order = "deg";

    std::vector<int> C;
    G.compute_cores();
    auto max_core = G.get_max_core();
    in.ub = max_core + 1;
    in.lb = 0;

    if(in.lb == 0) {
        pmc::pmc_heu maxclique(G, in);
        in.lb = maxclique.search(G, C);
    }

    if(use_heu) {
        return C;
    }

    if(in.lb == in.ub) {
        return C;
    }

    in.lb = std::max(in.lb, lb);

    if (G.num_vertices() < in.adj_limit) {
        G.create_adj();
        pmc::pmc_maxclique finder(G, in);
        finder.search(G, C);
    } else {
        pmc::pmc_maxclique finder(G, in);
        finder.search(G, C);
    }

    return C;
}

ConsistencyGraph buildConsistencyGraph(const Eigen::MatrixXd& P,
                                              const Eigen::MatrixXd& Q,
                                              const Eigen::VectorXd& r,
                                              double eps)
{
  const int N = static_cast<int>(P.rows());
  ConsistencyGraph G;
  G.adj.resize(N);
  G.adj_matrix = Eigen::MatrixXd::Zero(N, N);

  for (int i = 0; i < N; ++i) {
    for (int j = i + 1; j < N; ++j) {
      const double dP = (P.row(i) - P.row(j)).norm();
      const double dQ = (Q.row(i) - Q.row(j)).norm();
      const double L  = std::abs(dP - dQ);
      const double U  = dP + dQ;

      const double low  = std::abs(r[i] - r[j]) - eps;
      const double high = (r[i] + r[j]) + eps;

      if (L <= high && U >= low) {
        G.adj[i].push_back(j);
        G.adj[j].push_back(i);
        G.adj_matrix(i, j) = 1.0;
        G.adj_matrix(j, i) = 1.0;
      }
    }
    std::sort(G.adj[i].begin(), G.adj[i].end()); // keep lists sorted
  }
  return G;
}



TrajectoryEstimator::TrajectoryEstimator(){
    first_time = true;
    currentEstimate = {Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero()};
}

Eigen::Matrix3d TrajectoryEstimator::skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d K;
    K <<     0, -v(2),  v(1),
          v(2),     0, -v(0),
         -v(1),  v(0),     0;
    return K;
}

Eigen::Matrix3d TrajectoryEstimator::exp_so3(const Eigen::Vector3d& omega) {
    double theta = omega.norm();
    if (theta < 1e-12)
        return Eigen::Matrix3d::Identity();

    Eigen::Vector3d axis = omega / theta;
    Eigen::Matrix3d K = skew(axis);
    double s = std::sin(theta);
    double c = std::cos(theta);

    return Eigen::Matrix3d::Identity() + (s * K) + ((1 - c) * (K * K));
}

std::tuple<Eigen::Matrix3d, Eigen::Vector3d, std::vector<bool>>
TrajectoryEstimator::align_gnc_lm(const Eigen::MatrixXd& p,
          const Eigen::MatrixXd& q,
          const Eigen::VectorXd& r,
          const Eigen::VectorXd& sigma, 
        const Eigen::Matrix3d* R_in,       //Retoation matrix (Optional)
          const Eigen::Vector3d* t_in,
        float noise_bound,         //Acceptable proximity (Optional)  
          int max_outer,            //Number of iterations for gnc (Optional)
          int max_inner,            //Numebr of iterations for LM (Optional)
          double eta,               //mu update factor, how fast transition from least square to non-convex (Optional)
          double lm_lambda_init,    //Initial reg factor of LM (Optional)
          float lambda_up,          //When unsuccessful, how much to increase lm_lambda (Optional)
          float lambda_down,        //When successful, how much to decrease lm_lambda (Optional)
          double tol               //Convergence (Optional)
    ) {     //Translation vector (Optional)

    
    const int N = r.size();
    if((q.rows() != p.rows())||(q.cols() != p.cols())||(q.rows() != N)||(sigma.size() != N))
        throw std::invalid_argument("Size mismatch");
    
    
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    std::vector<bool> inliers;
    if(!R_in || !t_in){
        auto transform = arun_procrustes(p, q, 1 / sigma.array().square());
        R = transform.first;
        t = transform.second;
    }else{
        R = *R_in;
        t = *t_in;
    }

    auto residuals_jac = [&](const Eigen::Matrix3d& R_, const Eigen::Vector3d& t_, Eigen::VectorXd& e, Eigen::MatrixXd& J){
        Eigen::MatrixXd Rq = (R_ * q.transpose()).transpose();
        Eigen::MatrixXd x = Rq.rowwise() + t_.transpose() - p;
        Eigen::VectorXd d = x.rowwise().norm().cwiseMax(1e-6);
        e = (d - r).cwiseQuotient(sigma);

        J.resize(N, 6);
        for (int i = 0; i < N; ++i) {
            Eigen::Vector3d u = x.row(i).transpose() / d(i);  
            Eigen::Vector3d Rqi = Rq.row(i).transpose();
            Eigen::Vector3d J_rot = Rqi.cross(u) / sigma(i); 
            Eigen::Vector3d J_trans = u / sigma(i);  

            J.row(i).head<3>() = J_rot.transpose();
            J.row(i).tail<3>() = J_trans.transpose();
        }
    };

    Eigen::VectorXd e;
    Eigen::MatrixXd J;

    residuals_jac(R, t, e, J);
    Eigen::VectorXd error = e.cwiseProduct(sigma);
    double noise_bound_sq = noise_bound * noise_bound;
    double max_residual = error.array().square().maxCoeff();
    double mu = 1.0 / (2 * max_residual / noise_bound_sq - 1.0);

    if (mu <= 0) mu = 1.0;

    Eigen::VectorXd errors;
    Eigen::VectorXd w = Eigen::VectorXd::Ones(N);
    double lm_lambda = lm_lambda_init;

    for(int outer = 0; outer < max_outer; outer++){
        int inner;
        for(inner = 0; inner < max_inner; inner++){
            residuals_jac(R,t,e,J);
            errors = e.cwiseProduct(sigma);

            double th1 = (mu + 1) / mu * noise_bound_sq;
            double th2 = mu / (mu + 1) * noise_bound_sq;

            w = (noise_bound_sq * mu * (mu + 1) / errors.array().square()).sqrt() - mu;
            for (int i = 0; i < N; ++i) {
                if (errors(i)*errors(i) >= th1)
                    w(i) = 0.0;
                else if (errors(i)*errors(i) <= th2)
                    w(i) = 1.0;
            }
            Eigen::MatrixXd A = J.transpose() * w.asDiagonal() * J; //Verify
            Eigen::VectorXd g = J.transpose() * (w.array() * e.array()).matrix();

            Eigen::MatrixXd A_lm = A + lm_lambda * A.diagonal().asDiagonal().toDenseMatrix();

            Eigen::VectorXd delta;
            Eigen::LDLT<Eigen::MatrixXd> solver(A_lm);
            if (solver.info() != Eigen::Success) {
                std::cerr << "Singular normal matrix." << std::endl;
                return std::make_tuple(Eigen::Matrix3d::Identity(),Eigen::Vector3d::Zero(),inliers);
            }
            delta = -solver.solve(g);

            if (delta.norm() < tol) break;

            Eigen::Matrix3d R_new = exp_so3(delta.head<3>()) * R;
            Eigen::Vector3d t_new = t + delta.tail<3>();

            Eigen::VectorXd e_new;
            Eigen::MatrixXd J_dummy;
            residuals_jac(R_new,t_new,e_new,J_dummy);

            double rho_num = (w.array() * (e.array().square() - e_new.array().square())).sum();
            double rho_den = delta.dot(lm_lambda * A.diagonal() - g);
            double rho = rho_num / (rho_den + 1e-12);

            if (rho > 0) {
                R = R_new;
                t = t_new;
                lm_lambda = std::max(lm_lambda * lambda_down, 1e-7);
            } else {
                lm_lambda *= lambda_up;
            }

        }
        mu *= eta;

    }
    inliers.clear();
    for(int i = 0; i < N; i++){
        inliers.push_back(w(i) > 0.0);
    }
    return std::make_tuple(R,t,inliers);

}





std::pair<Eigen::Matrix3d,Eigen::Vector3d> TrajectoryEstimator::arun_procrustes(const Eigen::MatrixXd& P, const Eigen::MatrixXd& Q,
                     const Eigen::VectorXd& W_in) {
    
    int N = P.rows();
    Eigen::VectorXd W = W_in;

    // Default weights to all 1s if not provided
    if (W.size() == 0) {
        W = Eigen::VectorXd::Ones(N);
    }

    Eigen::MatrixXd W_diag = W.asDiagonal();
    Eigen::RowVector3d P_mean = P.colwise().mean();
    Eigen::RowVector3d Q_mean = Q.colwise().mean();

    Eigen::MatrixXd Pc = P.rowwise() - P_mean;
    Eigen::MatrixXd Qc = Q.rowwise() - Q_mean;
    Eigen::Matrix3d H = Qc.transpose() * (W_diag * Pc);

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d Vt = svd.matrixV().transpose();

    Eigen::Matrix3d R = Vt.transpose() * U.transpose();

    if (R.determinant() < 0) {
        Vt.row(2) *= -1;
        R = Vt.transpose() * U.transpose();
    }

    Eigen::Vector3d t = P_mean.transpose() - R * Q_mean.transpose();

    return {R,t};
}

void TrajectoryEstimator::addMeasurementConstraint(
    const Eigen::Vector3d& p, 
    const Eigen::Vector3d& q, 
    float r,
    double noise_bound,
    bool use_2d
){
    allP.push_back(p);
    allQ.push_back(q);
    allR.push_back(r); // Assuming r is the distance between p and q
    allSigma.push_back(r * std::log(r + 1)); // Assuming sigma is the standard deviation of the measurement noise

    inliers.push_back(false); // Initially, all measurements are considered inliers
    cliqueInliers.push_back(false); // Initially, all measurements are considered inliers for the clique

    Eigen::MatrixXd P = vector3dListToMatrix(allP);
    Eigen::MatrixXd Q = vector3dListToMatrix(allQ);
    Eigen::VectorXd r_vec = floatVectorToEigen(allR);
    Eigen::VectorXd sigma_vec = floatVectorToEigen(allSigma);


    if(allP.size() < 7) {
        currentEstimate = {Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero()};
        return; // Not enough points to estimate a transformation
    }

    
    auto start = std::chrono::high_resolution_clock::now();
    ConsistencyGraph cg = buildConsistencyGraph(P, Q, r_vec, noise_bound);    

    // count cliuqe inliers so far
    size_t clique_inliers_count = std::count(cliqueInliers.begin(), cliqueInliers.end(), true);
    auto C = find_pmc_clique(cg.adj_matrix, 10, 60, clique_inliers_count, true);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;

    //RCLCPP_INFO(rclcpp::get_logger("TrajectoryEstimator"), "Clique of size %zu/%zu found in %.3f seconds", C.size(), cg.adj_matrix.rows(), elapsed.count());

    Eigen::VectorXi clique_indices(C.size());
    for (size_t k = 0; k < C.size(); ++k)
        clique_indices(k) = C[k];

    std::fill(cliqueInliers.begin(), cliqueInliers.end(), false);
    for (int idx : C) {
        cliqueInliers[idx] = true;
    }

    if (clique_indices.size() > 7) {
    std::vector<bool> inliers_(clique_indices.size(), false);

    // 1) build compact P and Q (rows in clique_indices, all columns)
    Eigen::MatrixXd Pc(clique_indices.size(), P.cols());
    Eigen::MatrixXd Qc(clique_indices.size(), Q.cols());
    for (int i = 0; i < int(clique_indices.size()); ++i) {
        int idx = clique_indices[i];
        Pc.row(i) = P.row(idx);
        Qc.row(i) = Q.row(idx);
    }

    // 2) (optional) if r_vec and sigma_vec also need manual indexing:
    Eigen::VectorXd rc(clique_indices.size()), sigmac(clique_indices.size());
    for (int i = 0; i < int(clique_indices.size()); ++i) {
        int idx = clique_indices[i];
        rc(i)    = r_vec(idx);
        sigmac(i)= sigma_vec(idx);
    }

    // 3) call align without Eigen::all
    if (use_2d) {
        std::tie(currentEstimate.first,
                 currentEstimate.second,
                 inliers_)
            = align_gnc_lm_2d(
                  Pc,
                  Qc,
                  rc,
                  sigmac,
                  first_time ? nullptr : &currentEstimate.first,
                  first_time ? nullptr : &currentEstimate.second,
                  noise_bound
              );
    } else {
        std::tie(currentEstimate.first,
                 currentEstimate.second,
                 inliers_)
            = align_gnc_lm(
                  Pc,
                  Qc,
                  rc,
                  sigmac,
                  first_time ? nullptr : &currentEstimate.first,
                  first_time ? nullptr : &currentEstimate.second,
                  noise_bound
              );
    }

        std::fill(inliers.begin(), inliers.end(), false);
        for(size_t i = 0; i < inliers_.size(); ++i) {
            if(inliers_[i]) {
                inliers[clique_indices[i]] = true;
            }
        }

        first_time = false;
    }
}

Eigen::MatrixXd TrajectoryEstimator::vector3dListToMatrix(const std::vector<Eigen::Vector3d>& vecs) {
    Eigen::MatrixXd mat(vecs.size(), 3);  // Each row = one vector
    for (size_t i = 0; i < vecs.size(); ++i) {
        mat.row(i) = vecs[i];
    }
    return mat;
}

Eigen::VectorXd TrajectoryEstimator::floatVectorToEigen(const std::vector<float>& vec) {
    Eigen::VectorXd result(vec.size());
    for (size_t i = 0; i < vec.size(); ++i) {
        result(i) = static_cast<double>(vec[i]);
    }
    return result;
}


Eigen::Matrix2d rot2d(double theta)
{
    double c = std::cos(theta);
    double s = std::sin(theta);
    Eigen::Matrix2d R;
    R << c, -s,
         s,  c;
    return R;
}

std::pair<double, Eigen::Vector2d> procrustes2d(const Eigen::MatrixXd& P,
                                                const Eigen::MatrixXd& Q,
                                                const Eigen::VectorXd& weights)
{
    // Normalize weights
    Eigen::VectorXd w = weights;
    if (w.size() == 0) w = Eigen::VectorXd::Ones(P.rows());
    double wsum = w.sum();
    Eigen::Vector2d pc = (P.array().colwise() * w.array()).colwise().sum() / wsum;
    Eigen::Vector2d qc = (Q.array().colwise() * w.array()).colwise().sum() / wsum;

    Eigen::MatrixXd X = P.rowwise() - pc.transpose();
    Eigen::MatrixXd Y = Q.rowwise() - qc.transpose();

    X = X.array().colwise() * w.array();
    Y = Y.array().colwise() * w.array();

    Eigen::Matrix2d S = Y.transpose() * X;
    Eigen::JacobiSVD<Eigen::Matrix2d> svd(S, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix2d R2 = svd.matrixU() * svd.matrixV().transpose();
    if (R2.determinant() < 0.0) {
        Eigen::Matrix2d U = svd.matrixU();
        U.col(1) *= -1.0;
        R2 = U * svd.matrixV().transpose();

        //RCLCPP_WARN(rclcpp::get_logger("TrajectoryEstimator"), "Negative determinant in 2D Procrustes, flipping second column of U.");
    }
    double theta = std::atan2(R2(1,0), R2(0,0));
    Eigen::Vector2d t = pc - R2 * qc;
    return {theta, t};
}

std::tuple<Eigen::Matrix3d, Eigen::Vector3d, std::vector<bool>>
TrajectoryEstimator::align_gnc_lm_2d(
        const Eigen::MatrixXd& p,
        const Eigen::MatrixXd& q,
        const Eigen::VectorXd& r,
        const Eigen::VectorXd& sigma,
        const Eigen::Matrix3d* R_in,
        const Eigen::Vector3d* t_in,
        double noise_bound,
        int flatten,
        int max_outer,
        int max_inner,
        double eta,
        double lm_lambda_init,
        double lambda_up,
        double lambda_down,
        double tol
    )
{
    if(!(p.rows() == q.rows() && p.cols()==3 && q.cols()==3 &&
         p.rows()==r.size() && r.size()==sigma.size()))
        throw std::invalid_argument("Size mismatch");

    const int N = static_cast<int>(r.size());

    if(flatten < 0 || flatten > 2) throw std::invalid_argument("flatten must be 0,1,2");
    int ax1 = (flatten+1)%3;
    int ax2 = (flatten+2)%3;

    Eigen::MatrixXd p2(N,2), q2(N,2);
    p2.col(0) = p.col(ax1);
    p2.col(1) = p.col(ax2);
    q2.col(0) = q.col(ax1);
    q2.col(1) = q.col(ax2);

    double theta; Eigen::Vector2d t2;
    if(R_in && t_in) {
        theta = std::atan2((*R_in)(ax2,ax1), (*R_in)(ax1,ax1));
        t2 << (*t_in)(ax1), (*t_in)(ax2);
    } else {
        std::tie(theta, t2) = procrustes2d(p2, q2, 1 / sigma.array().square()); 
    }
    auto residuals_jac = [&](double th,
                             const Eigen::Vector2d& t,
                             Eigen::VectorXd& e,
                             Eigen::MatrixXd& J)
    {
        Eigen::Matrix2d R2 = rot2d(th);
        Eigen::MatrixXd Rq = (R2 * q2.transpose()).transpose(); 
        Eigen::MatrixXd x = Rq.rowwise() + t.transpose() - p2;
        Eigen::VectorXd d = x.rowwise().norm();
        d = d.array().max(1e-6);
        e = (d - r).cwiseQuotient(sigma);

        Eigen::MatrixXd u = x.array().colwise() / d.array();

        Eigen::VectorXd dRq = (-Rq.col(1).array()*u.col(0).array() +
                                Rq.col(0).array()*u.col(1).array()) / sigma.array();

        J.resize(N,3);
        J.col(0) = dRq;
        J.col(1) = u.col(0).cwiseQuotient(sigma);
        J.col(2) = u.col(1).cwiseQuotient(sigma);
    };



    Eigen::VectorXd e0; Eigen::MatrixXd Jtmp;
    residuals_jac(theta, t2, e0, Jtmp);
    Eigen::VectorXd err0 = e0.cwiseProduct(sigma);
    double noise_sq = noise_bound*noise_bound;
    double max_residual = err0.array().square().maxCoeff();
    double mu = 1.0 / (2.0*max_residual/noise_sq - 1.0);
    if(mu <= 0.0) mu = 1.0;

    double lm_lambda = lm_lambda_init;
    std::vector<bool> inliers(N,true);
    Eigen::VectorXd w(N);

    for(int outer=0; outer<max_outer; ++outer)
    {
        for(int inner=0; inner<max_inner; ++inner)
        {
            Eigen::VectorXd e;
            Eigen::MatrixXd J;
            residuals_jac(theta, t2, e, J);
            Eigen::VectorXd errs = e.cwiseProduct(sigma);

            double th1 = (mu+1.0)/mu * noise_sq;
            double th2 = mu/(mu+1.0) * noise_sq;

            Eigen::ArrayXd errs_sq = errs.array().square();
            w = (noise_bound*noise_bound * mu*(mu+1.0) / errs_sq).sqrt() - mu;
            for(int i=0;i<N;++i)
            {
                if(errs_sq(i) >= th1) w(i)=0.0;
                else if(errs_sq(i) <= th2) w(i)=1.0;
            }

            Eigen::MatrixXd A = J.transpose() * w.asDiagonal() * J;
            Eigen::VectorXd g = J.transpose() * (w.array()*e.array()).matrix();
            Eigen::MatrixXd A_lm = A + lm_lambda * A.diagonal().asDiagonal().toDenseMatrix();

            Eigen::VectorXd delta;
            Eigen::LDLT<Eigen::MatrixXd> solver(A_lm);
            if(solver.info()!=Eigen::Success) break;
            delta = -solver.solve(g);
            if(delta.norm() < tol) break;

            double th_new = theta + delta(0);
            Eigen::Vector2d t2_new = t2 + delta.segment<2>(1);

            Eigen::VectorXd e_new;
            residuals_jac(th_new, t2_new, e_new, Jtmp);

            double rho_num = (w.array() * (e.array().square() - e_new.array().square())).sum();
            double rho_den = delta.dot(lm_lambda*A.diagonal() - g);
            double rho = rho_num / (rho_den + 1e-12);

            if(rho > 0) {
                theta = th_new;
                t2 = t2_new;
                lm_lambda = std::max(lm_lambda*lambda_down, 1e-7);
            }
            else {
                lm_lambda *= lambda_up;
            }
        }
        mu *= eta;
    }

    Eigen::VectorXd e_final;
    residuals_jac(theta, t2, e_final, Jtmp);
    inliers.resize(N);
    for(int i=0;i<N;++i) inliers[i] = std::abs(e_final(i))*sigma(i) < noise_bound;

    Eigen::Matrix3d R_full = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t_full = Eigen::Vector3d::Zero();
    Eigen::Matrix2d R2 = rot2d(theta);
    R_full(ax1,ax1) = R2(0,0);  R_full(ax1,ax2) = R2(0,1);
    R_full(ax2,ax1) = R2(1,0);  R_full(ax2,ax2) = R2(1,1);
    t_full(ax1) = t2(0);
    t_full(ax2) = t2(1);

    return std::make_tuple(R_full, t_full, inliers);
}
