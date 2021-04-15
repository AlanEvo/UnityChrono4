using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

namespace chrono
{

    /// This is enough to use dynamic_casting<> to detect all template types
    /// from ChConstraintTwoTuplesRollingN
    public class ChConstraintTwoTuplesRollingNall { };

    /// This class is inherited from ChConstraintTwoTuples,
    /// It is used to represent the normal reaction between two objects,
    /// each represented by a tuple of ChVariables objects,
    /// ONLY when also two ChConstraintTwoTuplesFrictionT objects are
    /// used to represent friction. (If these two tangent constraint
    /// are not used, for frictionless case, please use a simple ChConstraintTwo
    /// with the CONSTRAINT_UNILATERAL mode.)
    /// Differently from an unilateral constraint, this does not enforce
    /// projection on positive constraint, since it will be up to the 'companion'
    /// ChConstraintTwoTuplesFrictionT objects to call a projection on the cone, by
    /// modifying all the three components (normal, u, v) at once.
    /// Templates Ta and Tb are of ChVariableTupleCarrier_Nvars classes

    public class ChConstraintTwoTuplesRollingN<Ta, Tb> : ChConstraintTwoTuples<Ta, Tb> where Ta : notnull where Tb : notnull
    {
        protected ChConstraintTwoTuplesRollingNall constraintTwoTuplesRollingNall;

        protected float rollingfriction;   //< the rolling friction coefficient
        protected float spinningfriction;  //< the spinning friction coefficient

        protected ChConstraintTwoTuplesRollingT<Ta, Tb> constraint_U;  //< the pointer to U tangential component
        protected ChConstraintTwoTuplesRollingT<Ta, Tb> constraint_V;  //< the pointer to V tangential component
        protected ChConstraintTwoTuplesContactN<Ta, Tb> constraint_N;  //< the pointer to N  component


        /// Default constructor
        public ChConstraintTwoTuplesRollingN()
        {
            rollingfriction = 0;
            spinningfriction = 0;
            constraint_U = null;
            constraint_V = null;
            constraint_N = null;

            this.mode = eChConstraintMode.CONSTRAINT_FRIC;
        }

        /// Copy constructor
        public ChConstraintTwoTuplesRollingN(ChConstraintTwoTuplesRollingN<Ta, Tb> other) : base(other)
        {
            rollingfriction = other.rollingfriction;
            spinningfriction = other.spinningfriction;
            constraint_U = other.constraint_U;
            constraint_V = other.constraint_V;
            constraint_N = other.constraint_N;
        }

        /// "Virtual" copy constructor (covariant return type).
        public override ChConstraint Clone() { return new ChConstraintTwoTuplesRollingN<Ta, Tb>(this); }

        /// Assignment operator: copy from other object
        /*  ChConstraintTwoTuplesRollingN& operator=(const ChConstraintTwoTuplesRollingN& other)
      {
          if (&other == this)
              return *this;
          // copy parent class data
          ChConstraintTwoTuples < Ta, Tb >::operator= (other);

          rollingfriction = other.rollingfriction;
          spinningfriction = other.spinningfriction;
          constraint_U = other.constraint_U;
          constraint_V = other.constraint_V;
          constraint_N = other.constraint_N;
          return *this;
      }*/

        /// Get the rolling friction coefficient
        public float GetRollingFrictionCoefficient() { return rollingfriction; }
        /// Set the rolling friction coefficient
        public void SetRollingFrictionCoefficient(float mcoeff) { rollingfriction = mcoeff; }

        /// Get the spinning friction coefficient
        public float GetSpinningFrictionCoefficient() { return spinningfriction; }
        /// Set the spinning friction coefficient
        public void SetSpinningFrictionCoefficient(float mcoeff) { spinningfriction = mcoeff; }

        /// Get pointer to U tangential component
        public ChConstraintTwoTuplesRollingT<Ta, Tb> GetRollingConstraintU() { return constraint_U; }
        /// Get pointer to V tangential component
        public ChConstraintTwoTuplesRollingT<Ta, Tb> GetRollingConstraintV() { return constraint_V; }
        /// Get pointer to normal contact component
        public ChConstraintTwoTuplesContactN<Ta, Tb> GetNormalConstraint() { return constraint_N; }

        /// Set pointer to U tangential component
        public void SetRollingConstraintU(ChConstraintTwoTuplesRollingT<Ta, Tb> mconstr) { constraint_U = mconstr; }
        /// Set pointer to V tangential component
        public void SetRollingConstraintV(ChConstraintTwoTuplesRollingT<Ta, Tb> mconstr) { constraint_V = mconstr; }
        /// Set pointer to normal contact component
        public void SetNormalConstraint(ChConstraintTwoTuplesContactN<Ta, Tb> mconstr) 
        { 
            constraint_N = mconstr; 
        }

        /// For iterative solvers: project the value of a possible
        /// 'l_i' value of constraint reaction onto admissible set.
        /// This projection will also modify the l_i values of the two
        /// tangential friction constraints (projection onto the friction cone,
        /// as by Anitescu-Tasora theory).
        public override void Project()
        {
            if (constraint_U == null)
                return;

            if (constraint_V == null)
                return;

            if (constraint_N == null)
                return;

            // METHOD
            // Anitescu-Tasora projection on rolling-friction cone generator and polar cone
            // (contractive, but performs correction on three components: normal,u,v)

            double f_n = constraint_N.Get_l_i();
            double t_n = this.Get_l_i();
            double t_u = constraint_U.Get_l_i();
            double t_v = constraint_V.Get_l_i();
            double t_tang = Math.Sqrt(t_v * t_v + t_u * t_u);
            double t_sptang = Math.Abs(t_n);  // = sqrt(t_n*t_n);

            // A. Project the spinning friction (approximate - should do cone
            //   projection stuff as in B, but spinning friction is usually very low...)

            if (spinningfriction != 0)
            {
                if (t_sptang < spinningfriction * f_n)
                {
                    // inside upper cone? keep untouched!
                }
                else
                {
                    // inside lower cone? reset  normal,u,v to zero!
                    if ((t_sptang < -(1.0 / spinningfriction) * f_n) || (Math.Abs(f_n) < 10e-15))
                    {
                        constraint_N.Set_l_i(0);
                        this.Set_l_i(0);
                    }
                    else
                    {
                        // remaining case: project orthogonally to generator segment of upper cone (CAN BE simplified)
                        double f_n_proj2 = (t_sptang * spinningfriction + f_n) / (spinningfriction * spinningfriction + 1);
                        double t_tang_proj2 = f_n_proj2 * spinningfriction;
                        double tproj_div_t2 = t_tang_proj2 / t_sptang;
                        double t_n_proj = tproj_div_t2 * t_n;

                        constraint_N.Set_l_i(f_n_proj2);
                        this.Set_l_i(t_n_proj);
                    }
                }
            }

            // B. Project the rolling friction

            // shortcut
            if (rollingfriction == 0)
            {
                constraint_U.Set_l_i(0);
                constraint_V.Set_l_i(0);
                if (f_n < 0)
                    constraint_N.Set_l_i(0);
                return;
            }

            // inside upper cone? keep untouched!
            if (t_tang < rollingfriction * f_n)
                return;

            // inside lower cone? reset  normal,u,v to zero!
            if ((t_tang < -(1.0 / rollingfriction) * f_n) || (Math.Abs(f_n) < 10e-15))
            {
                double f_n_proj2 = 0;
                double t_u_proj2 = 0;
                double t_v_proj2 = 0;

                constraint_N.Set_l_i(f_n_proj2);
                constraint_U.Set_l_i(t_u_proj2);
                constraint_V.Set_l_i(t_v_proj2);

                return;
            }

            // remaining case: project orthogonally to generator segment of upper cone
            double f_n_proj = (t_tang * rollingfriction + f_n) / (rollingfriction * rollingfriction + 1);
            double t_tang_proj = f_n_proj * rollingfriction;
            double tproj_div_t = t_tang_proj / t_tang;
            double t_u_proj = tproj_div_t * t_u;
            double t_v_proj = tproj_div_t * t_v;

            constraint_N.Set_l_i(f_n_proj);
            constraint_U.Set_l_i(t_u_proj);
            constraint_V.Set_l_i(t_v_proj);
        }
    };

}  // end namespace chrono
