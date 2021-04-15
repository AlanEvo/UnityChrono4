using System.Collections;
using System.Collections.Generic;
using UnityEngine;


namespace chrono
{
    // TO DO, Looks well complicated

    /// An iterative solver based on projective fixed point method, with overrelaxation
    /// and immediate variable update as in SOR methods. Multi-threaded.\n
    /// See ChSystemDescriptor for more information about the problem formulation and the data structures
    /// passed to the solver.
    /// 
    public class ChSolverSORmultithread : ChIterativeSolver
    {

        protected ChThreads solver_threads;

        public ChSolverSORmultithread(string uniquename = "solver",  //< this name must be unique.
                           int nthreads = 2,                   //< number of threads
                           int mmax_iters = 50,                //< max.number of iterations
                           bool mwarm_start = false,           //< uses warm start?
                           double mtolerance = 0.0,            //< tolerance for termination criterion
                           double momega = 1.0                 //< overrelaxation criterion
                           ) : base(mmax_iters, mwarm_start, mtolerance, momega)
        {
          //  ChThreadConstructionInfo create_args = new ChThreadConstructionInfo(uniquename, SolverThreadFunc, SolverMemoryFunc, nthreads);

          //  solver_threads = new ChThreads(create_args);
        }

        /// Return type of the solver.
        public override Type GetType() { return Type.SOR_MULTITHREAD; }

        /// Performs the solution of the problem.
        /// \return  the maximum constraint violation after termination.
      /*  public override double Solve(ref ChSystemDescriptor sysd  //< system description with constraints and variables
                         )
        {

        }*/

        /// Changes the number of threads which run in parallel (should be > 1 )
        public void ChangeNumberOfThreads(int mthreads = 2) { }
    }


}
