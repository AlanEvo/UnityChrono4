using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono
{

    /// This class is inherited by the base ChConstraint(),
    /// which does almost nothing. So here this class implements
    /// the functionality for a constraint between a THREE
    /// objects of type ChVariables(), and defines three constraint
    /// matrices, whose column number automatically matches the number
    /// of elements in variables vectors.
    ///  Before starting the solver one must provide the proper
    /// values in constraints (and update them if necessary), i.e.
    /// must set at least the c_i and b_i values, and jacobians.

    public abstract class ChConstraintThree : ChConstraint
    {

        protected ChVariables variables_a;  //< The first  constrained object
        protected ChVariables variables_b;  //< The second constrained object
        protected ChVariables variables_c;  //< The third constrained object


        /// Default constructor
        public ChConstraintThree()
        {
            variables_a = null;
            variables_b = null;
            variables_c = null;
        }

        /// Copy constructor
        public ChConstraintThree(ChConstraintThree other)
        {
            variables_a = other.variables_a;
            variables_b = other.variables_b;
            variables_c = other.variables_c;
        }

        /// Assignment operator: copy from other object
       // ChConstraintThree& operator=(const ChConstraintThree& other);

        /// Access jacobian matrix
        public abstract ChMatrix Get_Cq_a();
        /// Access jacobian matrix
        public abstract ChMatrix Get_Cq_b();
        /// Access jacobian matrix
        public abstract ChMatrix Get_Cq_c();

        /// Access auxiliary matrix (ex: used by iterative solvers)
        public abstract ChMatrix Get_Eq_a();
        /// Access auxiliary matrix (ex: used by iterative solvers)
        public abstract ChMatrix Get_Eq_b();
        /// Access auxiliary matrix (ex: used by iterative solvers)
        public abstract ChMatrix Get_Eq_c();

        /// Access the first variable object
        public ChVariables GetVariables_a() { return variables_a; }
        /// Access the second variable object
        public ChVariables GetVariables_b() { return variables_b; }
        /// Access the second variable object
        public ChVariables GetVariables_c() { return variables_c; }

        /// Set references to the constrained objects, each of ChVariables type,
        /// automatically creating/resizing jacobians if needed.
        public abstract void SetVariables(ChVariables mvariables_a, ChVariables mvariables_b, ChVariables mvariables_c);

    }
}
