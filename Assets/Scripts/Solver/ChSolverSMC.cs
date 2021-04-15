using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;

namespace chrono
{

    /// A solver for problems arising in SMmooth Contact (SMC) i.e. \e penalty formulations.\n
    /// See ChSystemDescriptor for more information about the problem formulation and the data structures
    /// passed to the solver.
    public class ChSolverSMC : ChIterativeSolver
    {


        public ChSolverSMC(int mmax_iters = 50,       //< max.number of iterations
                bool mwarm_start = false,             //< uses warm start?
                double mtolerance = 0.0,              //< tolerance for termination criterion
                double momega = 1.0)                  //< overrelaxation criterion
        : base(mmax_iters, mwarm_start, mtolerance, momega)
        {

        }
        public override Type GetType() { return Type.SOLVER_SMC; }

        /// Performs the solution of the problem.
        /// \return  the maximum constraint violation after termination.
        public override double Solve(ref ChSystemDescriptor sysd  //< system description with constraints and variables
                         )
        {
            List<ChConstraint> mconstraints = sysd.GetConstraintsList();
            List<ChVariables> mvariables = sysd.GetVariablesList();

            // 1)  Update auxiliary data in all constraints before starting,
            //     that is: g_i=[Cq_i]*[invM_i]*[Cq_i]' and  [Eq_i]=[invM_i]*[Cq_i]'
            for (int ic = 0; ic < mconstraints.Count; ic++)
                mconstraints[ic].Update_auxiliary();

            // 2)  Compute, for all items with variables, the initial guess for
            //     still unconstrained system:
            for (int iv = 0; iv < mvariables.Count; iv++)
            {
                if (mvariables[iv].IsActive())
                    mvariables[iv].Compute_invMb_v(mvariables[iv].Get_qb(), mvariables[iv].Get_fb());  // q = [M]'*fb
            }

            // 3)  For all items with variables, add the effect of initial (guessed)
            //     lagrangian reactions of constraints, if a warm start is desired.
            //     Otherwise, if no warm start, simply resets initial lagrangians to zero.
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

            // 4)  Perform the iteration loops (if there are any constraints)
            double maxviolation = 0.0;
            double maxdeltalambda = 0.0;

            if (mconstraints.Count == 0)
                return maxviolation;

            for (int iter = 0; iter < max_iterations; iter++)
            {
                maxviolation = 0;
                maxdeltalambda = 0;

                // The iteration on all constraints
                for (int ic = 0; ic < mconstraints.Count; ic++)
                {
                    // skip computations if constraint not active.
                    if (mconstraints[ic].IsActive())
                    {
                        // compute residual  c_i = [Cq_i]*q + b_i
                        double mresidual = mconstraints[ic].Compute_Cq_q() + mconstraints[ic].Get_b_i();

                        // true constraint violation may be different from 'mresidual' (ex:clamped if unilateral)
                        double candidate_violation = Math.Abs(mconstraints[ic].Violation(mresidual));

                        // compute:  delta_lambda = -(omega/g_i) * ([Cq_i]*q + b_i )
                        double deltal = (omega / mconstraints[ic].Get_g_i()) * (-mresidual);

                        // update:   lambda += delta_lambda;
                        double old_lambda = mconstraints[ic].Get_l_i();
                        mconstraints[ic].Set_l_i(old_lambda + deltal);

                        // If new lagrangian multiplier does not satisfy inequalities, project
                        // it into an admissible orthant (or, in general, onto an admissible set)
                        mconstraints[ic].Project();

                        // After projection, the lambda may have changed a bit..
                        double new_lambda = mconstraints[ic].Get_l_i();

                        // Apply the smoothing: lambda= sharpness*lambda_new_projected + (1-sharpness)*lambda_old
                        if (this.shlambda != 1.0)
                        {
                            new_lambda = shlambda * new_lambda + (1.0 - shlambda) * old_lambda;
                            mconstraints[ic].Set_l_i(new_lambda);
                        }

                        double true_delta = new_lambda - old_lambda;

                        // For all items with variables, add the effect of incremented
                        // (and projected) lagrangian reactions:
                        mconstraints[ic].Increment_q(true_delta);

                        if (this.record_violation_history)
                            maxdeltalambda = ChMaths.ChMax(maxdeltalambda, Math.Abs(true_delta));

                        maxviolation = ChMaths.ChMax(maxviolation, Math.Abs(candidate_violation));

                    }  // end IsActive()

                }  // end loop on constraints

                // For recording into violation history, if debugging
                if (this.record_violation_history)
                    AtIterationEnd(maxviolation, maxdeltalambda, iter);

                // Terminate the loop if violation in constraints has been successfully limited.
                if (maxviolation < tolerance)
                    break;

            }  // end iteration loop

            return maxviolation;
        }

    }
}
