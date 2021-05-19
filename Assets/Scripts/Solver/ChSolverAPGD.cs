using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;

namespace chrono
{

    /// An iterative solver based on Nesterov's Projected Gradient Descent.\n
    /// See ChSystemDescriptor for more information about the problem formulation and the data structures
    /// passed to the solver.

    class ChSolverAPGD : ChIterativeSolver
    {


        protected double residual = 0;
        protected int nc = 0;
        protected ChMatrixDynamic<double> gamma_hat = new ChMatrixDynamic<double>();
        protected ChMatrixDynamic<double> gammaNew = new ChMatrixDynamic<double>();
        protected ChMatrixDynamic<double> g = new ChMatrixDynamic<double>();
        protected ChMatrixDynamic<double> y = new ChMatrixDynamic<double>();
        protected ChMatrixDynamic<double> gamma = new ChMatrixDynamic<double>();
        protected ChMatrixDynamic<double> yNew = new ChMatrixDynamic<double>();
        protected ChMatrixDynamic<double> r = new ChMatrixDynamic<double>();
        protected ChMatrixDynamic<double> tmp = new ChMatrixDynamic<double>();


        public ChSolverAPGD(int mmax_iters = 1000,     //< max.number of iterations
                 bool mwarm_start = false,  //< uses warm start?
                 double mtolerance = 0.0    //< tolerance for termination criterion
                 )
        : base(mmax_iters, mwarm_start, mtolerance, 0.0001) { }


        public override Type GetType() { return Type.APGD; }

        /// Performs the solution of the problem.
        public override double Solve(ref ChSystemDescriptor sysd)
        {
            bool verbose = false;
            List<ChConstraint> mconstraints = sysd.GetConstraintsList();
           // List<ChVariables> mvariables;// = sysd.GetVariablesList();
          //  if (verbose)                
           //     Debug.Log("Number of constraints: " + mconstraints.Count + "nNumber of variables :" + mvariables.Count);

            // Update auxiliary data in all constraints before starting,
            // that is: g_i=[Cq_i]*[invM_i]*[Cq_i]' and  [Eq_i]=[invM_i]*[Cq_i]'
            for (int ic = 0; ic < mconstraints.Count; ic++)
                mconstraints[ic].Update_auxiliary();

            double L, t;
            double theta;
            double thetaNew;
            double Beta;
            double obj1, obj2;

            nc = sysd.CountActiveConstraints();
            gamma_hat.Resize(nc, 1);
            gammaNew.Resize(nc, 1);
            g.Resize(nc, 1);
            y.Resize(nc, 1);
            gamma.Resize(nc, 1);
            yNew.Resize(nc, 1);
            r.Resize(nc, 1);
            tmp.Resize(nc, 1);

            residual = 10e30;

            Beta = 0.0;
            obj1 = 0.0;
            obj2 = 0.0;

            // Compute the b_shur vector in the Shur complement equation N*l = b_shur
            ShurBvectorCompute(ref sysd);

            // Optimization: backup the  q  sparse data computed above,
            // because   (M^-1)*k   will be needed at the end when computing primals.
            ChMatrixDynamic<double> Minvk = new ChMatrixDynamic<double>();
            sysd.FromVariablesToVector(Minvk.matrix, true);

            // (1) gamma_0 = zeros(nc,1)
            if (warm_start)
            {
                for (int ic = 0; ic < mconstraints.Count; ic++)
                    if (mconstraints[ic].IsActive())
                        mconstraints[ic].Increment_q(mconstraints[ic].Get_l_i());
            }
            else
            {
                for (int ic = 0; ic < mconstraints.Count; ic++)
                    mconstraints[ic].Set_l_i(0.0);
            }
            sysd.FromConstraintsToVector(gamma.matrix);

            // (2) gamma_hat_0 = ones(nc,1)
            gamma_hat.matrix.FillElem(1);

            // (3) y_0 = gamma_0
            y.matrix.CopyFromMatrix(gamma.matrix);

            // (4) theta_0 = 1
            theta = 1.0;

            // (5) L_k = norm(N * (gamma_0 - gamma_hat_0)) / norm(gamma_0 - gamma_hat_0)
            tmp.matrix.MatrSub(gamma.matrix, gamma_hat.matrix);
            L = tmp.matrix.NormTwo();
            sysd.ShurComplementProduct(yNew.matrix, tmp.matrix);
            L = yNew.matrix.NormTwo() / L;
            yNew.matrix.FillElem(0);  // reset yNew to be all zeros

            // (6) t_k = 1 / L_k
            t = 1.0 / L;

            // (7) for k := 0 to N_max
            for (tot_iterations = 0; tot_iterations < max_iterations; tot_iterations++)
            {
                // (8) g = N * y_k - r
                sysd.ShurComplementProduct(g.matrix, y.matrix);
                g.matrix.MatrInc(r.matrix);

                // (9) gamma_(k+1) = ProjectionOperator(y_k - t_k * g)
                gammaNew.matrix.CopyFromMatrix(g.matrix);
                gammaNew.matrix.MatrScale(-t);
                gammaNew.matrix.MatrInc(y.matrix);
                sysd.ConstraintsProject(gammaNew.matrix);

                // (10) while 0.5 * gamma_(k+1)' * N * gamma_(k+1) - gamma_(k+1)' * r >= 0.5 * y_k' * N * y_k - y_k' * r + g' *
                // (gamma_(k+1) - y_k) + 0.5 * L_k * norm(gamma_(k+1) - y_k)^2
                sysd.ShurComplementProduct(tmp.matrix, gammaNew.matrix);  // Here tmp is equal to N*gammaNew;
                tmp.matrix.MatrScale(0.5);
                tmp.matrix.MatrInc(r.matrix);
                obj1 = tmp.matrix.MatrDot(gammaNew.matrix, tmp.matrix);

                sysd.ShurComplementProduct(tmp.matrix, y.matrix);  // Here tmp is equal to N*y;
                tmp.matrix.MatrScale(0.5);
                tmp.matrix.MatrInc(r.matrix);
                obj2 = tmp.matrix.MatrDot(y.matrix, tmp.matrix);
                tmp.matrix.MatrSub(gammaNew.matrix, y.matrix);  // Here tmp is equal to gammaNew - y
                obj2 = obj2 + tmp.matrix.MatrDot(tmp.matrix, g.matrix) + 0.5 * L * tmp.matrix.MatrDot(tmp.matrix, tmp.matrix);

                while (obj1 >= obj2)
                {
                    // (11) L_k = 2 * L_k
                    L = 2.0 * L;

                    // (12) t_k = 1 / L_k
                    t = 1.0 / L;

                    // (13) gamma_(k+1) = ProjectionOperator(y_k - t_k * g)
                    gammaNew.matrix.CopyFromMatrix(g.matrix);
                    gammaNew.matrix.MatrScale(-t);
                    gammaNew.matrix.MatrInc(y.matrix);
                    sysd.ConstraintsProject(gammaNew.matrix);

                    // Update the components of the while condition
                    sysd.ShurComplementProduct(tmp.matrix, gammaNew.matrix);  // Here tmp is equal to N*gammaNew;
                    tmp.matrix.MatrScale(0.5);
                    tmp.matrix.MatrInc(r.matrix);
                    obj1 = tmp.matrix.MatrDot(gammaNew.matrix, tmp.matrix);

                    sysd.ShurComplementProduct(tmp.matrix, y.matrix);  // Here tmp is equal to N*y;
                    tmp.matrix.MatrScale(0.5);
                    tmp.matrix.MatrInc(r.matrix);
                    obj2 = tmp.matrix.MatrDot(y.matrix, tmp.matrix);
                    tmp.matrix.MatrSub(gammaNew.matrix, y.matrix);  // Here tmp is equal to gammaNew - y
                    obj2 = obj2 + tmp.matrix.MatrDot(tmp.matrix, g.matrix) + 0.5 * L * tmp.matrix.MatrDot(tmp.matrix, tmp.matrix);

                    // (14) endwhile
                }

                // (15) theta_(k+1) = (-theta_k^2 + theta_k * sqrt(theta_k^2 + 4)) / 2
                thetaNew = (-Math.Pow(theta, 2.0) + theta * Math.Sqrt(Math.Pow(theta, 2.0) + 4.0)) / 2.0;

                // (16) Beta_(k+1) = theta_k * (1 - theta_k) / (theta_k^2 + theta_(k+1))
                Beta = theta * (1.0 - theta) / (Math.Pow(theta, 2) + thetaNew);

                // (17) y_(k+1) = gamma_(k+1) + Beta_(k+1) * (gamma_(k+1) - gamma_k)
                tmp.matrix.MatrSub(gammaNew.matrix, gamma.matrix);  // Here tmp is equal to gammaNew - gamma;
                tmp.matrix.MatrScale(Beta);
                yNew.matrix.MatrAdd(gammaNew.matrix, tmp.matrix);

                // (18) r = r(gamma_(k+1))
                double res = Res4(ref sysd);

                // (19) if r < epsilon_min
                if (res < residual)
                {
                    // (20) r_min = r
                    residual = res;

                    // (21) gamma_hat = gamma_(k+1)
                    gamma_hat.matrix.CopyFromMatrix(gammaNew.matrix);

                    // (22) endif
                }

                // (23) if r < Tau
                if (residual < this.tolerance)
                {
                    // (24) break
                    break;

                    // (25) endif
                }

                // (26) if g' * (gamma_(k+1) - gamma_k) > 0
                tmp.matrix.MatrSub(gammaNew.matrix, gamma.matrix);
                if (tmp.matrix.MatrDot(tmp.matrix, g.matrix) > 0)
                {
                    // (27) y_(k+1) = gamma_(k+1)
                    yNew.matrix.CopyFromMatrix(gammaNew.matrix);

                    // (28) theta_(k+1) = 1
                    thetaNew = 1.0;

                    // (29) endif
                }

                // (30) L_k = 0.9 * L_k
                L = 0.9 * L;

                // (31) t_k = 1 / L_k
                t = 1.0 / L;

                // perform some tasks at the end of the iteration
               /* if (this.record_violation_history)
                {
                    tmp.MatrSub(gammaNew, gamma);
                    AtIterationEnd(residual, tmp.NormInf(), tot_iterations);
                }*/

                // Update iterates
                theta = thetaNew;
                gamma.matrix.CopyFromMatrix(gammaNew.matrix);
                y.matrix.CopyFromMatrix(yNew.matrix);

                // (32) endfor
            }

         //  if (verbose)
           //     Debug.Log("Residual: " + residual + ", Iter: " + tot_iterations);

            // (33) return Value at time step t_(l+1), gamma_(l+1) := gamma_hat
            sysd.FromVectorToConstraints(gamma_hat.matrix);

            // Resulting PRIMAL variables:
            // compute the primal variables as   v = (M^-1)(k + D*l)
            // v = (M^-1)*k  ...    (by rewinding to the backup vector computed at the beginning)
            sysd.FromVectorToVariables(Minvk.matrix); // PROBLEM slow!

            // ... + (M^-1)*D*l     (this increment and also stores 'qb' in the ChVariable items)
            for (int ic = 0; ic < mconstraints.Count; ic++)
            {
                if (mconstraints[ic].IsActive())
                    mconstraints[ic].Increment_q(mconstraints[ic].Get_l_i());
            }

            return residual;
        }

        public void ShurBvectorCompute(ref ChSystemDescriptor sysd)
        {
            // ***TO DO*** move the following thirty lines in a short function ChSystemDescriptor::ShurBvectorCompute() ?

            // Compute the b_shur vector in the Shur complement equation N*l = b_shur
            // with
            //   N_shur  = D'* (M^-1) * D
            //   b_shur  = - c + D'*(M^-1)*k = b_i + D'*(M^-1)*k
            // but flipping the sign of lambdas,  b_shur = - b_i - D'*(M^-1)*k
            // Do this in three steps:

            // Put (M^-1)*k    in  q  sparse vector of each variable..
            for (int iv = 0; iv < sysd.GetVariablesList().Count; iv++)
                if (sysd.GetVariablesList()[iv].IsActive())
                    sysd.GetVariablesList()[iv].Compute_invMb_v(sysd.GetVariablesList()[iv].Get_qb().matrix,
                                                                 sysd.GetVariablesList()[iv].Get_fb().matrix);  // q = [M]'*fb

            // ...and now do  b_shur = - D'*q = - D'*(M^-1)*k ..
            int s_i = 0;
            for (int ic = 0; ic < sysd.GetConstraintsList().Count; ic++)
                if (sysd.GetConstraintsList()[ic].IsActive())
                {
                    r.matrix[s_i, 0] = sysd.GetConstraintsList()[ic].Compute_Cq_q();
                    ++s_i;
                }

            // ..and finally do   b_shur = b_shur - c
            sysd.BuildBiVector(tmp.matrix);  // b_i   =   -c   = phi/h
            r.matrix.MatrInc(tmp.matrix);
        }
        public double Res4(ref ChSystemDescriptor sysd)
        {
            // Project the gradient (for rollback strategy)
            // g_proj = (l-project_orthogonal(l - gdiff*g, fric))/gdiff;
            double gdiff = 1.0 / Math.Pow(nc, 2.0);
            sysd.ShurComplementProduct(tmp.matrix, gammaNew.matrix);
            tmp.matrix.MatrInc(r.matrix);
            tmp.matrix.MatrScale(-gdiff);
            tmp.matrix.MatrInc(gammaNew.matrix);
            sysd.ConstraintsProject(tmp.matrix);
            tmp.matrix.MatrSub(gammaNew.matrix, tmp.matrix);
            tmp.matrix.MatrScale(1.0 / gdiff);

            return tmp.matrix.NormTwo();
        }

        public double GetResidual() { return residual; }

        public void Dump_Rhs(ref List<double> temp)
        {
            for (int i = 0; i < r.matrix.GetRows(); i++)
            {
                temp.Add(r.matrix[i, 0]);
            }
        }

        public void Dump_Lambda(ref List<double> temp)
        {
            for (int i = 0; i < gamma_hat.matrix.GetRows(); i++)
            {
                temp.Add(gamma_hat.matrix[i, 0]);
            }
        }
    };

} // end namespace chrono
