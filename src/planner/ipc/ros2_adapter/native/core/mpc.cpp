#include "mpc.h"
#include <iomanip>

bool MPCPlannerClass::Run(void)
{
    double time_0 = ipc_native::now();

    // update reference
    LinearTerm(mpc_, X_0_, X_r_);

    // update system status and their constraints
    UpdateBound(mpc_, X_0_);
    
    // update safe flight corridor constraints
    corridor_rows_before=corridor_rows_after=0;
    bool add_sfc_flag = true;
    if (mpc_.T.rows() == 0) add_sfc_flag = false;
    if (add_sfc_flag) {
        mpc_.A_sfc.resize(mpc_.T.rows(), mpc_.A_p.cols());
        mpc_.A_sfc_upp.resize(mpc_.D_T.rows(), 1);
        mpc_.A_sfc = mpc_.T * mpc_.A_p;
        mpc_.A_sfc_upp = -mpc_.D_T - mpc_.T * mpc_.B_p;
        corridor_rows_before=mpc_.A_sfc.rows();
        if(prune_redundant_corridors){
            std::vector<int> keep;
            for(int r=0;r<mpc_.A_sfc.rows();++r){
                double minimum=0,maximum=0;
                for(int c=0;c<mpc_.A_sfc.cols();++c){
                    const double v=mpc_.A_sfc(r,c);
                    minimum+=v*(v>=0 ? mpc_.u_low[c] : mpc_.u_upp[c]);
                    maximum+=v*(v>=0 ? mpc_.u_upp[c] : mpc_.u_low[c]);
                }
                // Remove only rows implied for every input in the existing hard box.
                if(!(maximum<=mpc_.A_sfc_upp[r]-1e-9 && minimum>=mpc_.A_sfc_low[r]+1e-9))keep.push_back(r);
            }
            Eigen::MatrixXd constraints(keep.size(),mpc_.A_sfc.cols());
            Eigen::VectorXd lower(keep.size()),upper(keep.size());
            for(size_t i=0;i<keep.size();++i){constraints.row(i)=mpc_.A_sfc.row(keep[i]);lower[i]=mpc_.A_sfc_low[keep[i]];upper[i]=mpc_.A_sfc_upp[keep[i]];}
            mpc_.A_sfc=std::move(constraints);mpc_.A_sfc_low=std::move(lower);mpc_.A_sfc_upp=std::move(upper);
        }
        corridor_rows_after=mpc_.A_sfc.rows();
        add_sfc_flag=corridor_rows_after>0;
    }

    // generate all inequality constraints
    if (add_sfc_flag) {
        mpc_.A.resize(mpc_.A_sys.rows() + mpc_.A_sfc.rows(), mpc_.A_sys.cols());
        mpc_.A.block(0, 0, mpc_.A_sys.rows(), mpc_.A_sys.cols()) = mpc_.A_sys;
        mpc_.A.block(mpc_.A_sys.rows(), 0, mpc_.A_sfc.rows(), mpc_.A_sfc.cols()) = mpc_.A_sfc;
        mpc_.Alow.resize(mpc_.A_sys_low.rows() + mpc_.A_sfc_low.rows(), 1);
        mpc_.Alow.block(0, 0, mpc_.A_sys_low.rows(), 1) = mpc_.A_sys_low;
        mpc_.Alow.block(mpc_.A_sys_low.rows(), 0, mpc_.A_sfc_low.rows(), 1) = mpc_.A_sfc_low;
        mpc_.Aupp.resize(mpc_.A_sys_upp.rows() + mpc_.A_sfc_upp.rows(), 1);
        mpc_.Aupp.block(0, 0, mpc_.A_sys_upp.rows(), 1) = mpc_.A_sys_upp;
        mpc_.Aupp.block(mpc_.A_sys_upp.rows(), 0, mpc_.A_sfc_upp.rows(), 1) = mpc_.A_sfc_upp;
    } else {
        mpc_.A.resize(mpc_.A_sys.rows(), mpc_.A_sys.cols());
        mpc_.A.block(0, 0, mpc_.A_sys.rows(), mpc_.A_sys.cols()) = mpc_.A_sys;
        mpc_.Alow.resize(mpc_.A_sys_low.rows(), 1);
        mpc_.Alow.block(0, 0, mpc_.A_sys_low.rows(), 1) = mpc_.A_sys_low;
        mpc_.Aupp.resize(mpc_.A_sys_upp.rows(), 1);
        mpc_.Aupp.block(0, 0, mpc_.A_sys_upp.rows(), 1) = mpc_.A_sys_upp;
    }

    mpc_.H_sparse = mpc_.H.sparseView();
    mpc_.A_sparse = mpc_.A.sparseView();

    // data reset
    mpc_.T.resize(0, 0);
    mpc_.A_sfc.resize(0, 0);
    mpc_.A_sfc_low.resize(0, 1);
    mpc_.A_sfc_upp.resize(0, 1);
    mpc_.D_T.resize(0, 1);

    // use osqp-eigen to solve MPC problem
    OsqpEigen::Solver solver;
    // solver.settings()->setTimeLimit(0.008);
    solver.settings()->setVerbosity(0); // osqp stop print
    solver.settings()->setWarmStart(true);
    if(warm_available_)solver.settings()->setRho(last_rho_);
    // solver.setWarmStart()
    solver.data()->setNumberOfConstraints(mpc_.A_sparse.rows());
    solver.data()->setNumberOfVariables(mpc_.f.rows());
    solver.data()->setHessianMatrix(mpc_.H_sparse);
    solver.data()->setGradient(mpc_.f);
    solver.data()->setLinearConstraintsMatrix(mpc_.A_sparse);
    solver.data()->setLowerBound(mpc_.Alow);
    solver.data()->setUpperBound(mpc_.Aupp);

    bool init_flag = solver.initSolver();
    bool solve_flag = true;
    if (init_flag) {
        if(warm_available_ && warm_primal_.size()==mpc_.f.size() && warm_primal_.allFinite()) {
            const bool warm_ok=last_dual_.size()==mpc_.Alow.size() && last_dual_.allFinite()
                ? solver.setWarmStart(warm_primal_,last_dual_)
                : solver.setPrimalVariable(warm_primal_);
            if(!warm_ok) { have_solution_=false;warm_available_=false; return false; }
        }
        solve_flag = solver.solve();
    } else {
        IPC_LOG("[MPC]: Can't set mpc problem!");
        solve_flag = false;
    }
#ifdef PROFILING
    if(init_flag){last_setup_ms=1000*solver.workspace()->info->setup_time;last_optimize_ms=1000*solver.workspace()->info->solve_time;}
#endif
    last_iterations=init_flag ? solver.workspace()->info->iter : 0;
    fps_++;
    if (solve_flag && init_flag && solver.workspace()->info->status_val == OSQP_SOLVED) {
        mpc_.u_optimal = solver.getSolution();
        warm_primal_=mpc_.u_optimal;warm_available_=true;
        last_dual_=solver.getDualSolution();
        double rho=solver.workspace()->info->rho_estimate;
        if(std::isfinite(rho) && rho>=1e-6 && rho<=1e6)last_rho_=rho;
        have_solution_=true;
        static double vel_max = 0.0;
        if (vel_max < X_0_.block(3,0,3,1).norm()) vel_max = X_0_.block(3,0,3,1).norm();
        if ((ipc_native::now()-print_time_) > 2.0) {
            print_time_ = ipc_native::now();
            std::cout << "mpc fps: " << fps_/2 << ", this time is: " << (ipc_native::now()-time_0)*1000 << " ms. " 
                "Velocity now is: " << vel_max << "m/s. " << std::endl;
            fps_ = 0;
        }
        return true;
    } else {
        have_solution_=false;
        warm_available_=false;
        if(init_flag && solver.workspace()->info->status_val==OSQP_MAX_ITER_REACHED){
            auto primal=solver.getSolution();auto dual=solver.getDualSolution();
            double rho=solver.workspace()->info->rho_estimate;
            if(primal.allFinite() && dual.allFinite() && primal.cwiseAbs().maxCoeff()<1e4 && std::isfinite(rho) && rho>=1e-6 && rho<=1e6){
                warm_primal_=primal;last_dual_=dual;last_rho_=rho;warm_available_=true;
            }
        }
        bool flag = IsInFSC(X_0_.block(0,0,3,1), planes_[0]);
        // std::cout << "planes: " << std::endl << planes_ << std::endl;
        std::cout << "pos: " << X_0_.block(0,0,3,1).transpose() << "  fsc: " << std::boolalpha << flag << std::endl;
        if (init_flag) IPC_LOG("[MPC] solve rejected: status=%d primal=%g dual=%g rho=%g", int(solver.workspace()->info->status_val),solver.workspace()->info->pri_res,solver.workspace()->info->dua_res,solver.workspace()->info->rho_estimate);
        return false;
    }
}

void MPCPlannerClass::SetFSC(Eigen::Matrix<double, Eigen::Dynamic, 4>& planes, int step)
{
    if (step >= MPC_HORIZON || step < 0) {
        throw std::runtime_error("invalid SFC step");
    }
    if(!planes.allFinite())throw std::runtime_error("nonfinite SFC planes");
    planes_[step] = planes;
    mpc_.T.conservativeResize(mpc_.T.rows() + planes.rows(), mpc_.A_p.rows());
    mpc_.A_sfc_low.conservativeResize(mpc_.A_sfc_low.rows() + planes.rows(), 1);
    mpc_.D_T.conservativeResize(mpc_.D_T.rows() + planes.rows(), 1);
    for (int i = 0; i < planes.rows(); i++) {
        mpc_.T.row(mpc_.T.rows()-1 - i).setZero();
        mpc_.T.block(mpc_.T.rows()-1 - i, step*3, 1, 3) = Eigen::Vector3d(planes(i, 0), planes(i, 1), planes(i, 2)).transpose(); 
        mpc_.A_sfc_low(mpc_.A_sfc_low.rows()-1 - i, 0) = -OSQP_INFTY;
        mpc_.D_T(mpc_.D_T.rows()-1 - i, 0) = planes(i, 3);
    }
}

void MPCPlannerClass::ProblemFormation(void)
{
    /* system status: {p1, v1, a1, p2, v2, a2, ... , pN, vN, aN} 
       input: {u0, u1, u2, ... , u(N-1)}
    */

    // system model
    SystemModel(mpc_.Ax, mpc_.Bx, MPC_STEP);
    MPCModel(mpc_.Ax, mpc_.Bx, mpc_.M, mpc_.C);

    // cost function (Quadratic term and Linear term)
    Eigen::MatrixXd Q = Eigen::MatrixXd(9, 9).setZero();
    Q.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity() * R_p_;
    Q.block(3, 3, 3, 3) = Eigen::Matrix3d::Identity() * R_v_;
    Q.block(6, 6, 3, 3) = Eigen::Matrix3d::Identity() * R_a_;
    Eigen::MatrixXd R = Eigen::MatrixXd(3, 3).setIdentity() * R_u_;
    Eigen::MatrixXd R_con = Eigen::MatrixXd(3, 3).setIdentity() * R_u_con_;
    Eigen::MatrixXd F = Eigen::MatrixXd(9, 9).setZero();
    F.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity() * R_pN_;
    F.block(3, 3, 3, 3) = Eigen::Matrix3d::Identity() * R_vN_;
    F.block(6, 6, 3, 3) = Eigen::Matrix3d::Identity() * R_aN_;
    QuadraticTerm(mpc_, Q, R, R_con, F);
    LinearTerm(mpc_, Eigen::VectorXd(9, 1).setZero(), Eigen::VectorXd(9*MPC_HORIZON, 1).setZero());

    // system status and input constrains
    ALLConstraint(mpc_);

    // Inequality constrains (none)

    mpc_.u_optimal.resize(mpc_.f.rows(), 1);

    IPC_LOG("\033[1;32mMPC problem formation done!\033[0m");
}

void MPCPlannerClass::SystemModel(Eigen::MatrixXd& A, Eigen::MatrixXd& B, double t)
{
    A.resize(9, 9);
    A.setZero();
    A.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();
    A.block(0, 3, 3, 3) = Eigen::Matrix3d::Identity() * t;
    A.block(0, 6, 3, 3) = Eigen::Matrix3d::Identity() * t * t * 0.5;
    A.block(3, 3, 3, 3) = Eigen::Matrix3d::Identity() * (Eigen::Matrix3d::Identity() - Drag_ * t);
    A.block(3, 6, 3, 3) = Eigen::Matrix3d::Identity() * t;
    A.block(6, 6, 3, 3) = Eigen::Matrix3d::Identity();

    B.resize(9, 3);
    B.setZero();
    B.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity() * 1.0/6.0 * std::pow(t, 3);
    B.block(3, 0, 3, 3) = Eigen::Matrix3d::Identity() * 1.0/2.0 * std::pow(t, 2);
    B.block(6, 0, 3, 3) = Eigen::Matrix3d::Identity() * t;
}

void MPCPlannerClass::MPCModel(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, 
                               Eigen::MatrixXd& M, Eigen::MatrixXd& C)
{
    M.resize(MPC_HORIZON * A.rows(), A.cols());
    M.setZero();
    C.resize(MPC_HORIZON * B.rows(), MPC_HORIZON * B.cols());
    C.setZero();

    Eigen::MatrixXd temp = Eigen::MatrixXd(A.rows(), A.cols()).setIdentity();
    for (int i = 0; i < MPC_HORIZON; i++) {
        if (i == 0) {
            C.block(0, 0, B.rows(), B.cols()) = B;
        } else {
            Eigen::MatrixXd temp_c = Eigen::MatrixXd(B.rows(), C.cols());
            temp_c << temp * B, C.block((i-1) * B.rows(), 0, B.rows(), B.cols() * (MPC_HORIZON-1));
            C.block(B.rows() * i, 0, B.rows(), C.cols()) = temp_c;
        }

        temp = temp * A;
        M.block(A.rows() * i, 0, A.rows(), A.cols()) = temp;
    }
}

void MPCPlannerClass::QuadraticTerm(mpc_osqp_t& mpc, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R, 
                                    const Eigen::MatrixXd& R_con, const Eigen::MatrixXd& F)
{
    mpc.Q_bar.resize(Q.rows() * MPC_HORIZON, Q.cols() * MPC_HORIZON);
    mpc.Q_bar.setZero();
    mpc.R_bar.resize(R.rows() * MPC_HORIZON, R.cols() * MPC_HORIZON);
    mpc.R_bar.setZero();
    mpc.R_con_bar.resize(R_con.rows() * MPC_HORIZON, R_con.cols() * MPC_HORIZON);
    mpc.R_con_bar.setZero();
    for (int i = 0; i < MPC_HORIZON; i++) {
        mpc.Q_bar.block(i * Q.rows(), i * Q.cols(), Q.rows(), Q.cols()) = Q;
        mpc.R_bar.block(i * R.rows(), i * R.cols(), R.rows(), R.cols()) = R;
        if (i == 0) mpc.R_con_bar.block(0, 0, R_con.rows(), R_con.cols()) = R_con;
        else if (i == MPC_HORIZON - 1) {
            mpc.R_con_bar.block(i * R_con.rows(), i * R_con.cols(), R_con.rows(), R_con.cols()) = R_con;
            mpc.R_con_bar.block(i * R_con.rows(), (i-1) * R_con.cols(), R_con.rows(), R_con.cols()) = -2 * R_con;
        } else {
            mpc.R_con_bar.block(i * R_con.rows(), i * R_con.cols(), R_con.rows(), R_con.cols()) = 2 * R_con;
            mpc.R_con_bar.block(i * R_con.rows(), (i-1) * R_con.cols(), R_con.rows(), R_con.cols()) = -2 * R_con;
        }
    }
    mpc.Q_bar.block((MPC_HORIZON-1) * Q.rows(), (MPC_HORIZON-1) * Q.cols(), F.rows(), F.cols()) = F;

    mpc.H.resize(mpc.C.rows(), mpc.C.cols());
    mpc.H = mpc.C.transpose() * mpc.Q_bar * mpc.C + mpc.R_bar + mpc.R_con_bar;
}

void MPCPlannerClass::LinearTerm(mpc_osqp_t& mpc, const Eigen::VectorXd& x_0, const Eigen::VectorXd& x_r)
{
    if (x_r.rows() != mpc.M.rows()) {
        IPC_LOG("[MPC]: MPC linear term set goal error!");
        return;
    }
    mpc.f.resize(mpc.C.rows(), 1);
    mpc.f = ((x_0.transpose()*mpc.M.transpose() - x_r.transpose()) * mpc.Q_bar * mpc.C).transpose();
}

void MPCPlannerClass::ALLConstraint(mpc_osqp_t& mpc)
{
    mpc.A_sys.resize(3 * MPC_HORIZON * 3, 3 * MPC_HORIZON);
    mpc.A_sys_low.resize(3 * MPC_HORIZON * 3, 1);
    mpc.A_sys_upp.resize(3 * MPC_HORIZON * 3, 1);

    /* --system constraint: input, acceleration, velocity limit-- */
    mpc.u_low.resize(3 * MPC_HORIZON, 1);
    mpc.u_upp.resize(3 * MPC_HORIZON, 1);
    mpc.a_low.resize(3 * MPC_HORIZON, 1);
    mpc.a_upp.resize(3 * MPC_HORIZON, 1);
    mpc.v_low.resize(3 * MPC_HORIZON, 1);
    mpc.v_upp.resize(3 * MPC_HORIZON, 1);
    for (int i = 0; i < MPC_HORIZON; i++) {
        mpc.u_low.block(i * u_min_.rows(), 0, u_min_.rows(), 1) = u_min_;
        mpc.u_upp.block(i * u_max_.rows(), 0, u_max_.rows(), 1) = u_max_;
        mpc.a_low.block(i * a_min_.rows(), 0, a_min_.rows(), 1) = a_min_;
        mpc.a_upp.block(i * a_max_.rows(), 0, a_max_.rows(), 1) = a_max_;
        mpc.v_low.block(i * v_min_.rows(), 0, v_min_.rows(), 1) = v_min_;
        mpc.v_upp.block(i * v_max_.rows(), 0, v_max_.rows(), 1) = v_max_;
    }
    
    // input constraint
    Eigen::MatrixXd A_u;
    A_u.resize(3 * MPC_HORIZON, 3 * MPC_HORIZON);
    A_u.setIdentity();

    // calculate A_p, A_v, A_a: system state position p, v, a transform to input u
    mpc.A_p.resize(3 * MPC_HORIZON, mpc.C.cols());
    mpc.A_v.resize(3 * MPC_HORIZON, mpc.C.cols());
    mpc.A_a.resize(3 * MPC_HORIZON, mpc.C.cols());
    mpc.M_p.resize(3 * MPC_HORIZON, mpc.M.cols());
    mpc.M_v.resize(3 * MPC_HORIZON, mpc.M.cols());
    mpc.M_a.resize(3 * MPC_HORIZON, mpc.M.cols());
    mpc.B_a.resize(mpc.M_p.rows(), 1);
    mpc.B_v.resize(mpc.M_v.rows(), 1);
    mpc.B_p.resize(mpc.M_a.rows(), 1);

    for (int i = 0; i < MPC_HORIZON; i++) {
        mpc.A_p.block(3 * i, 0, 3, mpc.A_p.cols()) = mpc.C.block(9 * i + 0, 0, 3, mpc.C.cols());
        mpc.A_v.block(3 * i, 0, 3, mpc.A_v.cols()) = mpc.C.block(9 * i + 3, 0, 3, mpc.C.cols());
        mpc.A_a.block(3 * i, 0, 3, mpc.A_a.cols()) = mpc.C.block(9 * i + 6, 0, 3, mpc.C.cols());
        mpc.M_p.block(3 * i, 0, 3, mpc.M_p.cols()) = mpc.M.block(9 * i + 0, 0, 3, mpc.M.cols());
        mpc.M_v.block(3 * i, 0, 3, mpc.M_v.cols()) = mpc.M.block(9 * i + 3, 0, 3, mpc.M.cols());
        mpc.M_a.block(3 * i, 0, 3, mpc.M_a.cols()) = mpc.M.block(9 * i + 6, 0, 3, mpc.M.cols());
    }

    mpc.A_sys.block(0, 0, A_u.rows(), A_u.cols()) = A_u;
    mpc.A_sys.block(A_u.rows(), 0, mpc.A_a.rows(), mpc.A_a.cols()) = mpc.A_a;
    mpc.A_sys.block(A_u.rows() + mpc.A_a.rows(), 0, mpc.A_v.rows(), mpc.A_v.cols()) = mpc.A_v;

    // std::cout << "mpc.A_a: " << std::endl << mpc.A_a << std::endl;
    // std::cout << "mpc.A_v: " << std::endl << mpc.A_v << std::endl;
    // std::cout << "mpc.A_p: " << std::endl << mpc.A_p << std::endl;

    Eigen::VectorXd x_0;
    x_0.resize(9, 1);
    x_0.setZero();
    UpdateBound(mpc, x_0);
    // x_0 << 1.0,0.5,0, 2,1,0.5, 0.1,0,0;
    // UpdateBound(mpc, x_0);
}

void MPCPlannerClass::UpdateBound(mpc_osqp_t& mpc, const Eigen::VectorXd& x_0)
{
    if (x_0.rows() != mpc.M_p.cols() || x_0.rows() != mpc.M_v.cols() || x_0.rows() != mpc.M_a.cols()) {
        IPC_LOG("[MPC]: Update bound error!");
        return;
    }
    mpc.B_p = mpc.M_p * x_0;
    mpc.B_v = mpc.M_v * x_0;
    mpc.B_a = mpc.M_a * x_0;

    mpc.A_sys_low.block(0, 0, mpc.u_low.rows(), 1) = mpc.u_low;
    mpc.A_sys_upp.block(0, 0, mpc.u_upp.rows(), 1) = mpc.u_upp;
    mpc.A_sys_low.block(mpc.u_low.rows(), 0, mpc.a_low.rows(), 1) = mpc.a_low - mpc.B_a;
    mpc.A_sys_upp.block(mpc.u_upp.rows(), 0, mpc.a_upp.rows(), 1) = mpc.a_upp - mpc.B_a;
    mpc.A_sys_low.block(mpc.u_low.rows() + mpc.a_low.rows(), 0, mpc.v_low.rows(), 1) = mpc.v_low - mpc.B_v;
    mpc.A_sys_upp.block(mpc.u_upp.rows() + mpc.a_upp.rows(), 0, mpc.v_upp.rows(), 1) = mpc.v_upp - mpc.B_v;

    // std::cout << "mpc.B_a: " << mpc.B_a.transpose() << std::endl;
    // std::cout << "mpc.B_v: " << mpc.B_v.transpose() << std::endl;
    // std::cout << "mpc.B_p: " << mpc.B_p.transpose() << std::endl;
}


void MPCPlannerClass::dumpProblem(std::ostream& out) const {
    out<<std::setprecision(17)<<"{";
    auto matrix=[&out](const char* name,const auto& m){
        out<<"\""<<name<<"\":[";
        for(int r=0;r<m.rows();++r){if(r)out<<",";out<<"[";for(int c=0;c<m.cols();++c){if(c)out<<",";out<<m(r,c);}out<<"]";}out<<"]";
    };
    matrix("H",mpc_.H);out<<",";matrix("q",mpc_.f);out<<",";matrix("A",mpc_.A);out<<",";
    matrix("lower",mpc_.Alow);out<<",";matrix("upper",mpc_.Aupp);out<<",";matrix("state",X_0_);out<<"}\n";
}
