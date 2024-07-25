#pragma once

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/src/Core/Matrix.h>

#include <OsqpEigen/OsqpEigen.h>

namespace rmcs_core::controller::mpc {
template <int nx, int nu, int mpcWindow>
class MPCCalculator {
public:
    void Update(
        const Eigen::Matrix<double, nx, nx>& a, const Eigen::Matrix<double, nx, nu>& b,
        const double Q, const Eigen::DiagonalMatrix<double, nu * mpcWindow>& R,
        const Eigen::SparseMatrix<double>& linearMatrix, Eigen::VectorXd& lowerBound,
        Eigen::VectorXd& upperBound, const Eigen::Matrix<double, nx, 1>& x0,
        const Eigen::Matrix<double, nx, 1>& xRef, Eigen::Matrix<double, nu, 1>& output) {
        // output << Eigen ::MatrixXd::Zero(nu, 1);
        // allocate the initial and the reference state space
        // allocate QP problem matrices and vectors
        Eigen::Matrix<double, nx + nu, nx + nu> A;
        Eigen::Matrix<double, nx + nu, nu> B;
        Eigen::Matrix<double, nx + nu, nx> tmp;
        B << b, Eigen::MatrixXd::Identity(nu, nu);
        tmp << a, Eigen::MatrixXd::Zero(nu, nx);
        A << tmp, B;
        Eigen::SparseMatrix<double> hessian;
        Eigen::MatrixXd phi;
        Eigen::MatrixXd theta;
        Eigen::VectorXd gradient;

        set_theta_phi_matrices(A, B, theta, phi);
        // cast the MPC problem as QP problem
        cast_mpc_to_qp_hessian(Q, R, theta, hessian);
        cast_mpc_to_qp_gradient(Q, x0, xRef, d_u_hat_pre, theta, phi, gradient);

        solver.clearSolver();
        solver.data()->clearHessianMatrix();
        solver.data()->clearLinearConstraintsMatrix();
        // settings
        // solver.settings()->setVerbosity(false);
        solver.settings()->setWarmStart(false);
        // set the initial data of the QP solver
        solver.data()->setNumberOfVariables(nu * mpcWindow + 1);
        solver.data()->setNumberOfConstraints(int(linearMatrix.rows()));
        if (!solver.data()->setHessianMatrix(hessian)) {
            return;
        }
        if (!solver.data()->setGradient(gradient)) {
            return;
        }
        if (!solver.data()->setLinearConstraintsMatrix(linearMatrix))
            return;
        if (!solver.data()->setLowerBound(lowerBound))
            return;
        if (!solver.data()->setUpperBound(upperBound))
            return;

        // instantiate the solver
        if (!solver.initSolver())
            return;
        // controller input and QPSolution vector
        Eigen::VectorXd QPSolution;

        // number of iteration steps
        // solve the QP problem
        if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
            return;
        // get the controller input
        QPSolution = solver.getSolution();

        d_u_hat_pre << QPSolution.block(0, 0, 8, 1);
        u_hat_pre << d_u_hat_pre + u_hat_pre;
        u_pre = u_hat_pre + u_pre;
        output << u_pre;
    }
    void reset() {
        d_u_hat_pre = Eigen::MatrixXd::Zero(nu, 1);
        u_hat_pre   = Eigen::MatrixXd::Zero(nu, 1);
        u_pre       = Eigen::MatrixXd::Zero(nu, 1);
    }
    Eigen::Matrix<double, nu, 1> u_hat_pre = Eigen::MatrixXd::Zero(nu, 1);
    Eigen::Matrix<double, nu, 1> u_pre     = Eigen::MatrixXd::Zero(nu, 1);

private:
    void set_theta_phi_matrices(
        const Eigen::Matrix<double, nx + nu, nx + nu>& A,
        const Eigen::Matrix<double, nx + nu, nu>& B, Eigen::MatrixXd& theta,
        Eigen::MatrixXd& phi) const {
        theta.resize(mpcWindow * nx, mpcWindow * nu);
        phi.resize(mpcWindow * nx, nx + nu);
        Eigen::MatrixXd C;
        C        = Eigen::MatrixXd::Identity(nx, nx + nu);
        auto tmp = C;

        for (int i = 1; i <= mpcWindow; i++) {
            phi.block(nx * (i - 1), 0, nx, nx + nu) << tmp * A;
            Eigen::MatrixXd tmp_c     = Eigen::MatrixXd::Zero(nx, mpcWindow * nu);
            tmp_c.block(0, 0, nx, nu) = tmp * B;
            if (i > 1)
                tmp_c.block(0, nu, nx, mpcWindow * nu - nu)
                    << theta.block((i - 2) * nx, 0, nx, mpcWindow * nu - nu);
            theta.block(nx * (i - 1), 0, nx, mpcWindow * nu) = tmp_c;

            tmp = tmp * A;
        }
    }

    void cast_mpc_to_qp_hessian(
        const double Q, const Eigen::Matrix<double, nu * mpcWindow, nu * mpcWindow>& R,
        Eigen::MatrixXd& theta, Eigen::SparseMatrix<double>& hessianMatrix) const {
        Eigen::MatrixXd tmp = (theta.transpose() * Q * theta + R);
        hessianMatrix.resize(nu * mpcWindow + 1, nu * mpcWindow + 1);

        for (int i = 0; i < nu * mpcWindow; i++)
            for (int j = 0; j < nu * mpcWindow; j++)
                if (tmp(i, j) != 0)
                    hessianMatrix.insert(i, j) = 2 * tmp(i, j);
        hessianMatrix.insert(nu * mpcWindow, nu * mpcWindow) = 20;
    }

    void cast_mpc_to_qp_gradient(
        const double Q, const Eigen::Matrix<double, nx, 1>& x,
        const Eigen::Matrix<double, nx, 1>& xRef, const Eigen::Matrix<double, nu, 1>& uPre,
        Eigen::MatrixXd& theta, Eigen::MatrixXd& phi, Eigen::VectorXd& gradient) const {
        Eigen::Matrix<double, nu + nx, 1> kesi;
        kesi.block(0, 0, nx, 1) << (x - xRef);
        kesi.block(nx, 0, nu, 1) << uPre;
        auto E = phi * kesi;
        gradient.resize(mpcWindow * nu + 1);
        auto tmp = 2 * E.transpose() * Q * theta;
        for (int i = 0; i < mpcWindow * nu; i++)
            gradient(i) = tmp(i);
        gradient(mpcWindow * nu) = 0;
    }
    Eigen::Matrix<double, nu, 1> d_u_hat_pre = Eigen::MatrixXd::Zero(nu, 1);
    OsqpEigen::Solver solver                 = OsqpEigen::Solver();
};
} // namespace rmcs_core::controller::mpc