using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono
{
    /// Modes for raint
    public enum eChConstraintMode
    {
        CONSTRAINT_FREE = 0,        //< the raint does not enforce anything
        CONSTRAINT_LOCK = 1,        //< the raint enforces c_i=0;
        CONSTRAINT_UNILATERAL = 2,  //< the raint enforces linear complementarity
        /// c_i>=0, l_i>=0, l_1*c_i=0;
        CONSTRAINT_FRIC = 3,        //< the raint is one of three reactions in friction
                                    //(cone complementarity problem)
    };

    /// Base class for representing raints to be used
    /// with variational inequality solvers, used with Linear/CCP/LCP
    /// problems including inequalities, equalities, nonlinearities, etc.
    ///
    /// See ChSystemDescriptor for more information about the overall
    /// problem and data representation.
    ///
    /// The jacobian matrix [Cq] is built row by row by jacobians
    /// [Cq_i] of the raints.\n
    /// [E] optionally includes 'cfm_i' terms on the diagonal.
    ///
    /// In general, typical bilateral raints must be solved
    /// to have residual c_i = 0 and unilaterals: c_i>0
    /// where the following linearization is introduced:
    ///      c_i= [Cq_i]*q + b_i
    ///
    /// The base class introduces just the minimum requirements
    /// for the solver, that is the basic methods that will be
    /// called by the solver. It is up to the derived classes
    /// to implement these methods, and to add further features..
    public abstract class ChConstraint
    {

        protected double c_i;  ///< The 'c_i' residual of the raint (if satisfied, c must be
                 ///=0)
        protected double l_i;  ///< The 'l_i' lagrangian multiplier (reaction)
        protected double b_i;  ///< The 'b_i' right term in [Cq_i]*q+b_i=0 , note: c_i= [Cq_i]*q
                               ///+ b_i

                               /// The raint force mixing, if needed (usually is zero) to add some
                               /// numerical 'compliance' in the raint, that is the equation becomes:
                               /// c_i= [Cq_i]*q + b_i + cfm*l_i =0;
                               /// Example, it could be   cfm = [k * h^2](^-1)   where k is stiffness
        protected double cfm_i;

        private bool valid;      //< the link has no formal problems (references restored
        /// correctly, etc)
        private bool disabled;   //< the user can turn on/off the link easily
        private bool redundant;  //< the raint is redundant or singular
        private bool broken;     //< the raint is broken (someone pulled too much..)
        private bool _active;    //< Cached active state depending on previous flags. Internal
        /// update.

        protected eChConstraintMode mode;  //< mode of the raint: free / lock / complementar
        protected double g_i;              //<  'g_i' product [Cq_i]*[invM_i]*[Cq_i]' (+cfm)
        protected int offset;              //< offset in global "l" state vector (needed by some solvers)

                                           /// Default ructor
        public ChConstraint()
        {
            c_i = 0;
            g_i = 0;
            b_i = 0;
            l_i = 0;
            cfm_i = 0;
            valid = false;
            disabled = false;
            redundant = false;
            broken = false;
            _active = true;
            mode = eChConstraintMode.CONSTRAINT_LOCK;
        }

        /// Copy ructor
        public ChConstraint(ChConstraint other) {
            c_i = other.c_i;
            g_i = other.g_i;
            b_i = other.b_i;
            l_i = other.l_i;
            cfm_i = other.cfm_i;
            valid = other.valid;
            disabled = other.disabled;
            redundant = other.redundant;
            broken = other.broken;
            mode = other.mode;
        }

        /// "Virtual" copy ructor.
        public abstract ChConstraint Clone();

        /// Assignment operator: copy from other object
        //  public ChConstraint operator=( ChConstraint& other);

        /// Comparison (compares only flags, not the jacobians etc.)
        //bool operator ==( ChConstraint& other) ;

        /// Tells if the raint data is currently valid.
        public virtual bool IsValid() { return valid; }
        /// Use this function to set the valid state (child class
        /// Children classes must use this function depending on
        /// the result of their implementations of RestoreReference();
        public virtual void SetValid(bool mon)
        {
            valid = mon;
            UpdateActiveFlag();
        }

        /// Tells if the raint is currently turned on or off by the user.
        public virtual bool IsDisabled() { return disabled; }
        /// User can use this to enable/disable the raint as desired
        public virtual void SetDisabled(bool mon)
        {
            disabled = mon;
            UpdateActiveFlag();
        }

        /// Tells if the raint is redundant or singular.
        public virtual bool IsRedundant() { return redundant; }
        /// Solvers may use the following to mark a raint as redundant
        public virtual void SetRedundant(bool mon)
        {
            redundant = mon;
            UpdateActiveFlag();
        }

        /// Tells if the raint is broken, for excess of pulling/pushing.
        public virtual bool IsBroken() { return broken; }
        /// 3rd party software can set the 'broken' status via this method
        /// (by default, raints never break);
        public virtual void SetBroken(bool mon)
        {
            broken = mon;
            UpdateActiveFlag();
        }

        /// Tells if the raint is unilateral (typical complementarity
        /// raint).
        public virtual bool IsUnilateral() { return mode == eChConstraintMode.CONSTRAINT_UNILATERAL; }

        /// Tells if the raint is linear (if non linear, returns false).
        public virtual bool IsLinear() { return true; }

        /// Gets the mode of the raint: free / lock / complementary
        /// A typical raint has 'lock = true' by default.
        public eChConstraintMode GetMode() { return mode; }

        /// Sets the mode of the raint: free / lock / complementary
        public void SetMode(eChConstraintMode mmode)
        {
            mode = mmode;
            UpdateActiveFlag();
        }

        /// A VERY IMPORTANT function!
        /// Tells if the raint is currently active, in general,
        /// that is tells if it must be included into the system solver or not.
        /// This method cumulates the effect of all flags (so a raint may
        /// be not active either because 'disabled', or 'broken', o 'redundant', or
        /// not 'valid'.)
        public virtual bool IsActive() {
            /*
                            return ( valid &&
                                    !disabled &&
                                    !redundant &&
                                    !broken &&
                                    mode!=(CONSTRAINT_FREE));
                                    */  // Optimized:
                                        // booleans
                                        // precomputed
                                        // and cached
                                        // in
                                        // _active.
            return _active;
        }

        /// Set the status of the raint to active
        public virtual void SetActive(bool isactive) { _active = isactive; }

        /// Compute the residual of the raint using the LINEAR
        /// expression   c_i= [Cq_i]*q + cfm_i*l_i + b_i . For a satisfied bilateral
        /// raint, this residual must be near zero.
        public virtual double Compute_c_i()
        {
            c_i = Compute_Cq_q() + cfm_i * l_i + b_i;
            return c_i;
        }

        /// Return the residual 'c_i' of this raint. // CURRENTLY NOT USED
        public double Get_c_i() { return c_i; }

        /// Sets the known term b_i in [Cq_i]*q + b_i =0,
        /// where: c_i = [Cq_i]*q + b_i = 0
        public void Set_b_i(double mb) { b_i = mb; }

        /// Return the known term b_i in [Cq_i]*q + b_i =0,
        /// where: c_i= [Cq_i]*q + b_i = 0
        public double Get_b_i() { return b_i; }

        /// Sets the raint force mixing term (default=0).
        /// Adds artificial 'elasticity' to the raint,
        /// as:   c_i= [Cq_i]*q + b_i + cfm*l_i =0;
        public void Set_cfm_i(double mcfm) { cfm_i = mcfm; }

        /// Returns the raint force mixing term.
        public double Get_cfm_i() { return cfm_i; }

        /// Sets the 'l_i' value (raint reaction, see 'l' vector)
        public virtual void Set_l_i(double ml_i) { l_i = ml_i; }

        /// Return the 'l_i' value (raint reaction, see 'l' vector)
        public virtual double Get_l_i() { return l_i; }

        // -----  Functions often used by iterative solvers:

        ///  This function must update jacobians and auxiliary
        /// data such as the 'g_i' product. This function is
        /// often called by solvers at the beginning of the
        /// solution process.
        /// *** This function MUST BE OVERRIDDEN by specialized
        /// inherited classes, which have some jacobians!
        public virtual void Update_auxiliary() { }

        /// Return the 'g_i' product , that is [Cq_i]*[invM_i]*[Cq_i]' (+cfm)
        public double Get_g_i() { return g_i; }

        /// Usually you should not use the Set_g_i function, because g_i
        /// should be automatically computed during the Update_auxiliary() .
        public void Set_g_i(double m_g_i) { g_i = m_g_i; }

        ///  This function must computes the product between
        /// the row-jacobian of this raint '[Cq_i]' and the
        /// vector of variables, 'q', that is, Cq_q=[Cq_i]*q.
        ///  This is used for some iterative solvers.
        /// *** This function MUST BE OVERRIDDEN by specialized
        /// inherited classes! (since it will be called frequently,
        /// when iterative solvers are used, the implementation of
        /// the [Cq_i]*q product must be AS FAST AS POSSIBLE!).
        ///  It returns the result of the computation.
        public abstract double Compute_Cq_q();

        ///  This function must increment the vector of variables
        /// 'q' with the quantity [invM]*[Cq_i]'*deltal,that is
        ///   q+=[invM]*[Cq_i]'*deltal
        ///  This is used for some iterative solvers.
        /// *** This function MUST BE OVERRIDDEN by specialized
        /// inherited classes!
        public abstract void Increment_q(double deltal);

        /// Computes the product of the corresponding block in the
        /// system matrix by 'vect', and add to 'result'.
        /// NOTE: the 'vect' vector must already have
        /// the size of the total variables&raints in the system; the procedure
        /// will use the ChVariable offsets (that must be already updated) to know the
        /// indexes in result and vect;
        public abstract void MultiplyAndAdd(ref double result, ChMatrix vect);

        /// Computes the product of the corresponding transposed block in the
        /// system matrix (ie. the TRANSPOSED jacobian matrix C_q') by 'l', and add to
        /// 'result'.
        /// NOTE: the 'result' vectors must already have
        /// the size of the total variables&raints in the system; the procedure
        /// will use the ChVariable offsets (that must be already updated) to know the
        /// indexes in result and vect;
        public abstract void MultiplyTandAdd(ChMatrix result, double l);

        /// For iterative solvers: project the value of a possible
        /// 'l_i' value of raint reaction onto admissible orthant/set.
        /// Default behavior: if raint is unilateral and l_i<0, reset l_i=0
        /// *** This function MAY BE OVERRIDDEN by specialized
        /// inherited classes! For example, a bilateral raint
        /// can do nothing, a monolateral: l_i= ChMax(0., l_i);
        /// a 'boxed raint': l_i= ChMin(ChMax(min., l_i), max); etc. etc.
        public virtual void Project() {
            if (mode == eChConstraintMode.CONSTRAINT_UNILATERAL)
            {
                if (l_i < 0.0)
                    l_i = 0.0;
            }
        }

        /// Given the residual of the raint computed as the
        /// linear map  mc_i =  [Cq]*q + b_i + cfm*l_i , returns the
        /// violation of the raint, considering inequalities, etc.
        ///   For bilateral raint,  violation = mc_i.
        ///   For unilateral raint, violation = min(mc_i, 0),
        ///   For boxed raints or such, inherited class MAY OVERRIDE THIS!
        public virtual double Violation(double mc_i) {
            if (mode == eChConstraintMode.CONSTRAINT_UNILATERAL)
            {
                if (mc_i > 0.0)
                    return 0.0;
            }

            return mc_i;
        }

        /// Puts the jacobian portions into the 'insrow' row of a sparse matrix,
        /// where each portion of jacobian is shifted in order to match the
        /// offset of the corresponding ChVariable.
        public abstract void Build_Cq(ref ChSparseMatrix storage, int insrow);

        /// Same as Build_Cq, but puts the _transposed_ jacobian row as a column.
        public abstract void Build_CqT(ref ChSparseMatrix storage, int inscol);

        /// Set offset in global q vector (set automatically by ChSystemDescriptor)
        public void SetOffset(int moff) { offset = moff; }

        /// Get offset in global q vector
        public int GetOffset() { return offset; }


        private void UpdateActiveFlag()
        {
            this._active = (valid && !disabled && !redundant && !broken && mode != (eChConstraintMode.CONSTRAINT_FREE));
        }

    }

}
