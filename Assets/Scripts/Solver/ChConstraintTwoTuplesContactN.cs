using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

namespace chrono
{
    /// This is enough to use dynamic_casting<> to detect all template types
    /// from ChConstraintTwoTuplesContactN

    public class ChConstraintTwoTuplesContactNall
    {

        /// Get the friction coefficient
        public double GetFrictionCoefficient() { return friction; }
        /// Set the friction coefficient
        public void SetFrictionCoefficient(double mcoeff) { friction = mcoeff; }

        /// Get the cohesion
        public double GetCohesion() { return cohesion; }
        /// Set the cohesion
        public void SetCohesion(double mcoh) { cohesion = mcoh; }


        /// the friction coefficient 'f', for  sqrt(Tx^2+Ty^2)<f*Nz
        protected double friction;
        /// the cohesion 'c', positive, if any, for  sqrt(Tx^2+Ty^2)<f*(Nz+c)
        protected double cohesion;
    };

    /// This class is inherited from the ChConstraintTwoTuples,
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
    public class ChConstraintTwoTuplesContactN<Ta, Tb> : ChConstraintTwoTuples<Ta, Tb> where Ta : notnull where Tb : notnull
    {
        public ChConstraintTwoTuplesContactNall Constraint2TuplesNall = new ChConstraintTwoTuplesContactNall();

        /// the pointer to U tangential component
        protected ChConstraintTwoTuplesFrictionT<Ta, Tb> constraint_U = new ChConstraintTwoTuplesFrictionT<Ta, Tb>();
        /// the pointer to V tangential component
        protected ChConstraintTwoTuplesFrictionT<Ta, Tb> constraint_V = new ChConstraintTwoTuplesFrictionT<Ta, Tb>();

        /// Default constructor
        public ChConstraintTwoTuplesContactN()
        {
            this.mode = eChConstraintMode.CONSTRAINT_FRIC;
            Constraint2TuplesNall.SetFrictionCoefficient(0.0);
            Constraint2TuplesNall.SetCohesion(0.0);
            constraint_U = constraint_V = null;
        }



        /// Copy constructor
        public ChConstraintTwoTuplesContactN(ChConstraintTwoTuplesContactN<Ta, Tb> other) : base(other)
        {
            Constraint2TuplesNall.SetFrictionCoefficient(other.Constraint2TuplesNall.GetFrictionCoefficient());
            Constraint2TuplesNall.SetCohesion(other.Constraint2TuplesNall.GetCohesion());
            constraint_U = other.constraint_U;
            constraint_V = other.constraint_V;
        }

        /// "Virtual" copy constructor (covariant return type).
        public override ChConstraint Clone()
        {
            return new ChConstraintTwoTuplesContactN<Ta, Tb>(this);
        }

        /// Get pointer to U tangential component
        public ChConstraintTwoTuplesFrictionT<Ta, Tb> GetTangentialConstraintU() { return constraint_U; }
        /// Get pointer to V tangential component
        public ChConstraintTwoTuplesFrictionT<Ta, Tb> GetTangentialConstraintV() { return constraint_V; }

        /// Set pointer to U tangential component
        public void SetTangentialConstraintU(ChConstraintTwoTuplesFrictionT<Ta, Tb> mconstr) { constraint_U = mconstr; }
        /// Set pointer to V tangential component
        public void SetTangentialConstraintV(ChConstraintTwoTuplesFrictionT<Ta, Tb> mconstr) { constraint_V = mconstr; }

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

            // METHOD
            // Anitescu-Tasora projection on cone generator and polar cone
            // (contractive, but performs correction on three components: normal,u,v)

            double f_n = this.l_i + this.Constraint2TuplesNall.GetCohesion();
            double f_u = constraint_U.Get_l_i();
            double f_v = constraint_V.Get_l_i();

            double f_tang = Math.Sqrt(f_v * f_v + f_u * f_u);

            // shortcut
            if (Constraint2TuplesNall.GetFrictionCoefficient() == 0)
            {
                constraint_U.Set_l_i(0);
                constraint_V.Set_l_i(0);
                if (f_n < 0)
                    this.Set_l_i(0);
                return;
            }

            // inside upper cone? keep untouched!

            double fcoif = Constraint2TuplesNall.GetFrictionCoefficient();
            if (f_tang < Constraint2TuplesNall.GetFrictionCoefficient() * f_n)
                return;

            // inside lower cone? reset  normal,u,v to zero!
            if ((f_tang < -(1.0 / Constraint2TuplesNall.GetFrictionCoefficient()) * f_n) || (Math.Abs(f_n) < 10e-15))
            {
                double f_n_proj2 = 0;
                double f_u_proj2 = 0;
                double f_v_proj2 = 0;

                this.Set_l_i(f_n_proj2);
                constraint_U.Set_l_i(f_u_proj2);
                constraint_V.Set_l_i(f_v_proj2);

                return;
            }

            // remaining case: project orthogonally to generator segment of upper cone
            double f_n_proj = (f_tang * Constraint2TuplesNall.GetFrictionCoefficient() + f_n) / (Constraint2TuplesNall.GetFrictionCoefficient() * Constraint2TuplesNall.GetFrictionCoefficient() + 1);
            double f_tang_proj = f_n_proj * Constraint2TuplesNall.GetFrictionCoefficient();
            double tproj_div_t = f_tang_proj / f_tang;
            double f_u_proj = tproj_div_t * f_u;
            double f_v_proj = tproj_div_t * f_v;

            this.Set_l_i(f_n_proj - this.Constraint2TuplesNall.GetCohesion());
            constraint_U.Set_l_i(f_u_proj);
            constraint_V.Set_l_i(f_v_proj);
        }

    }

}
