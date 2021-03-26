using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono
{
    /// Base class for solvers aimed at solving complementarity problems arising from QP optimization
    /// problems. This is an abstract class and specific solution methods are implemented in derived
    /// classes (e.g., SOR, APGD, etc.)\n
    /// See ChSystemDescriptor for more information about the problem formulation and the data structures
    /// passed to the solver.
    public abstract class ChSolver
    {
        /// Available types of solvers.
        public enum Type
        {
            SOR = 0,
            SYMMSOR,
            JACOBI,
            SOR_MULTITHREAD,
            PMINRES,
            BARZILAIBORWEIN,
            PCG,
            APGD,
            MINRES,
            SOLVER_SMC,
            CUSTOM,
        };

        public ChSolver()
        {
            verbose = false;
        }

        /// Return type of the solver.
        /// Default is CUSTOM. Derived classes should override this function.
        public virtual Type GetType() { return Type.CUSTOM; }

        /// Indicate whether or not the Solve() phase requires an up-to-date problem matrix.
        /// Typically, direct solvers only need the matrix for the Setup() phase. However,
        /// iterative solvers likely require the matrix to perform the necessary matrix-vector
        /// operations.
        public abstract bool SolveRequiresMatrix();

        /// Performs the solution of the problem.
        /// This function MUST be implemented in children classes, with specialized
        /// methods such as iterative or direct solvers.
        /// Returns true if it successfully solves the problem and false otherwise.
        public abstract double Solve(ref ChSystemDescriptor sysd  //< system description with constraints and variables
                         );

        /// This function does the setup operations for the solver.
        /// The purpose of this function is to prepare the solver for subsequent calls to the
        /// solve function. This function is called only as frequently it is determined that
        /// it is appropriate to perform the setup phase.
        public virtual bool Setup(ref ChSystemDescriptor sysd  //< system description with constraints and variables
                           )
        {
            return true;
        }

        /// Set verbose output from solver.
        public void SetVerbose(bool mv) { verbose = mv; }

        // Return whether or not verbose output is enabled.
        public bool GetVerbose() { return verbose; }


        protected bool verbose;
    }

}
