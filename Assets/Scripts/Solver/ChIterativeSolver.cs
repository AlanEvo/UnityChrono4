using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono
{
    /// Base class for ITERATIVE solvers aimed at solving complementarity problems arising
    /// from QP optimization problems.
    /// This class does nothing: it is up to derived classes to implement specific solution
    /// methods.
    /// The problem is described by a variational inequality VI(Z*x-d,K):\n
    /// 
    ///  | M -Cq'|*|q|- | f|= |0| , l \f$\in\f$ Y, C \f$\in\f$ Ny, normal cone to Y\n
    ///  | Cq -E | |l|  |-b|  |c|
    /// 
    /// Also Z symmetric by flipping sign of l_i: |M  Cq'|*| q|-| f|=|0|\n
    ///                                           |Cq  E | |-l| |-b| |c|
    /// * case linear problem:  all Y_i = R, Ny=0, ex. all bilaterals
    /// * case LCP: all Y_i = R+:  c>=0, l>=0, l*c=0
    /// * case CCP: Y_i are friction cones
    public class ChIterativeSolver : ChSolver
    {

        protected int max_iterations;  //< maximum allowed iterations
        protected int tot_iterations;  //< total number of iterations performed by the solver
        protected bool warm_start;     //< indicate whether or not to use warm starting
        protected double tolerance;    //< tolerance for termination criteria
        protected double omega;        //< over-relaxation factor
        protected double shlambda;     //< sharpness factor

        protected bool record_violation_history;
        protected List<double> violation_history;
        protected List<double> dlambda_history;

        public ChIterativeSolver(int mmax_iters = 50,       //< max.number of iterations
                      bool mwarm_start = false,  //< uses warm start?
                      double mtolerance = 0.0,   //< tolerance for termination criterion
                      double momega = 1.0,       //< overrelaxation, if any
                      double mshlambda = 1.0     //< sharpness, if any
                      )
        {
            max_iterations = mmax_iters;
            tot_iterations = 0;
            warm_start = mwarm_start;
            tolerance = mtolerance;
            omega = momega;
            shlambda = mshlambda;
            record_violation_history = false;
        }

        /// Indicate whether ot not the Solve() phase requires an up-to-date problem matrix.
        /// Typically, this is the case for iterative solvers (as the matrix is needed for
        /// the matrix-vector operations).
        public override bool SolveRequiresMatrix() { return true; }

        public override double Solve(ref ChSystemDescriptor sysd  //< system description with constraints and variables
                 )
        { return 0; }

        /// Set the maximum number of iterations.
        /// If the solver exceed this limit, it should stop even if the required tolerance
        /// isn't yet reached. Default limit: 50 iterations.
        public void SetMaxIterations(int mval) { max_iterations = mval; }

        /// Get the current value for the maximum allowed number of iterations.
        public int GetMaxIterations() { return max_iterations; }

        /// Get the number of total iterations taken by the solver.
        public int GetTotalIterations() { return tot_iterations; }

        /// Set the overrelaxation factor.
        /// This factor may be used by SOR-like methods. Default: 1.
        public void SetOmega(double mval)
        {
            if (mval > 0.0)
                omega = mval;
        }

        /// Return the current value of the overrelaxation factor.
        public double GetOmega() { return omega; }

        /// Set the sharpness factor.
        /// This factor may be used by SOR-like methods with projection (see Mangasarian LCP method).
        /// Usually in the range [0,1]. Default: 1. 
        public virtual void SetSharpnessLambda(double mval)
        {
            if (mval > 0.0)
                shlambda = mval;
        }

        /// Return the current value of the sharpness factor.
        public virtual double GetSharpnessLambda() { return shlambda; }

        /// Enable/disable 'warm start'.
        /// If enabled, the initial guess is set to the current values of the unknowns.
        ///	Useful if the variables are already near to the solution. In most iterative schemes,
        /// this is often a good option.
        public void SetWarmStart(bool mval) { warm_start = mval; }

        /// Return a flag indicating whether or not warm start is enabled.
        public bool GetWarmStart() { return warm_start; }

        /// Set the tolerance for stopping criterion.
        /// The iteration is stopped when the constraint feasibility error is below this value.
        /// Default: 0.0
        public void SetTolerance(double mval) { tolerance = mval; }

        /// Return the current value of the solver tolerance.
        public double GetTolerance() { return tolerance; }

        /// Enable/disable recording of the constraint violation history.
        /// If enabled, the maximum constraint violation at the end of each iteration is
        /// stored in a vector (see GetViolationHistory).
        public void SetRecordViolation(bool mval) { record_violation_history = mval; }

        /// Return a flag indicating whether or not constraint violations are recorded.
        public bool GetRecordViolation() { return record_violation_history; }

        /// Access the vector of constraint violation history.
        /// Note that collection of constraint violations must be enabled through SetRecordViolation.
        public List<double> GetViolationHistory() { return violation_history; }

        /// Access the vector with history of maximum change in Lagrange multipliers
        /// Note that collection of constraint violations must be enabled through SetRecordViolation.
        public List<double> GetDeltalambdaHistory() { return dlambda_history; }

        /// This method MUST be called by all iterative methods INSIDE their iteration loops
        /// (at the end). If history recording is enabled, this function will store the
        /// current values as passed as arguments.
        /// Note: 'iternum' starts at 0 for the first iteration.
        protected void AtIterationEnd(double mmaxviolation, double mdeltalambda, int iternum)
        {
            if (!record_violation_history)
                return;
            if (iternum != violation_history.Count)
            {
                violation_history.Clear();
                //violation_history.AddRange(iternum);
            }
            if (iternum != dlambda_history.Count)
            {
                dlambda_history.Clear();
                //dlambda_history.resize(iternum);
            }
            violation_history.Add(mmaxviolation);
            dlambda_history.Add(mdeltalambda);
        }

     

    }
}
