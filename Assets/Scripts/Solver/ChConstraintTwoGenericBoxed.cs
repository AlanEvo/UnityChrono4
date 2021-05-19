using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono
{

    /// This class is inherited from the base ChConstraintTwoGeneric,
    /// which can be used for most pairwise constraints, and adds the
    /// feature that the multiplier must be
    ///        l_min < l < l_max
    /// that is, the multiplier is 'boxed'.
    /// Note that, if l_min = 0 and l_max = infinite, this can work
    /// also as an unilateral constraint..
    ///  Before starting the solver one must provide the proper
    /// values in constraints (and update them if necessary), i.e.
    /// must set at least the c_i and b_i values, and jacobians.

    public class ChConstraintTwoGenericBoxed : ChConstraintTwoGeneric
    {

        protected double l_min;
        protected double l_max;


        /// Default constructor
        public ChConstraintTwoGenericBoxed()
        {
            l_min = -1;
            l_max = 1;
        }

        /// Construct and immediately set references to variables
        public ChConstraintTwoGenericBoxed(ChVariables mvariables_a, ChVariables mvariables_b) : base(mvariables_a, mvariables_b)
        {
            l_min = -1;
            l_max = 1;
        }

        /// Copy constructor
        public ChConstraintTwoGenericBoxed(ChConstraintTwoGenericBoxed other) : base(other)
        {
            l_min = other.l_min;
            l_max = other.l_max;
        }

        /// "Virtual" copy constructor (covariant return type).
        public override ChConstraint Clone()
        {
            return new ChConstraintTwoGenericBoxed(this);
        }

        /// Assignment operator: copy from other object
        //ChConstraintTwoGenericBoxed& operator=(const ChConstraintTwoGenericBoxed& other);

        /// Set lower/upper limit for the multiplier.
        public void SetBoxedMinMax(double mmin, double mmax)
        {
            //assert(mmin <= mmax);
            l_min = mmin;
            l_max = mmax;
        }

        /// Get the lower limit for the multiplier
        public double GetBoxedMin() { return l_min; }
        /// Get the upper limit for the multiplier
        public double GetBoxedMax() { return l_max; }

        /// For iterative solvers: project the value of a possible
        /// 'l_i' value of constraint reaction onto admissible orthant/set.
        /// This 'boxed implementation overrides the default do-nothing case.
        public override void Project()
        {
            if (l_i < l_min)
                l_i = l_min;
            if (l_i > l_max)
                l_i = l_max;
        }

        /// Given the residual of the constraint computed as the
        /// linear map  mc_i =  [Cq]*q + b_i + cfm*l_i , returns the
        /// violation of the constraint, considering inequalities, etc.
        public override double Violation(double mc_i)
        {
            if ((l_i - 10e-5 < l_min) || (l_i + 10e-5 > l_max))
                return 0;
            return mc_i;
        }
    }
}
