using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono
{

    /// Base class for all link joints which use a 'six degrees of freedom' mask
    /// and auxiliary matrices to store jacobians. The management of jacobian matrices,
    /// as well as other auxiliary matrices, is automatic as soon as the mask is
    /// changed.
    /// Also, links inherited from this class inherit six ChLinkForce objects, to
    /// set inner spring-damper features for each degree of freedom.

    public class ChLinkMasked : ChLinkMarkers
    {

        // The mask of the locked coords, with the status of the
        // scalar constraints. This encapsulated object also
        // contains the jacobians and residuals for the solver.
        protected ChLinkMask mask = new ChLinkMask();  //< scalar constraints

        // the following counters are cached here for optimization purposes
        protected int ndoc;    //< number of DOC, degrees of constraint
        protected int ndoc_c;  //< number of DOC, degrees of constraint (only bilaterals)
        protected int ndoc_d;  //< number of DOC, degrees of constraint (only unilaterals)

        // internal forces
        protected ChLinkForce force_D;// = new ChLinkForce();   //< the force acting on the straight line m1-m2 (distance)
        protected ChLinkForce force_R;// = new ChLinkForce();   //< the torque acting about rotation axis
        protected ChLinkForce force_X;// = new ChLinkForce();   //< the force acting along X dof
        protected ChLinkForce force_Y;// = new ChLinkForce();   //< the force acting along Y dof
        protected ChLinkForce force_Z;// = new ChLinkForce();   //< the force acting along Z dof
        protected ChLinkForce force_Rx;// = new ChLinkForce();  //< the torque acting about Rx dof
        protected ChLinkForce force_Ry;// = new ChLinkForce();  //< the torque acting about Ry dof
        protected ChLinkForce force_Rz;// = new ChLinkForce();  //< the torque acting about Rz dof
        protected double d_restlength;                       //< the rest length of the "d_spring" spring

        protected ChMatrixDynamic<double> C = new ChMatrixDynamic<double>();       //< {C(q,q_dt,t)}, <<<<!!!   that is
        protected ChMatrixDynamic<double> C_dt = new ChMatrixDynamic<double>();    //< the violation= relC = C(q,qdt,t)
        protected ChMatrixDynamic<double> C_dtdt = new ChMatrixDynamic<double>();  //< also speed violations. and acc violations

        protected ChMatrixDynamic<double> Cq1 = new ChMatrixDynamic<double>();  //< [Cq1], the jacobian of the constraint, for coords1, [ndoc,7]
        protected ChMatrixDynamic<double> Cq2 = new ChMatrixDynamic<double>();  //< [Cq2], the jacobian of the constraint, for coords2. [ndoc,7]



        protected ChMatrixDynamic<double> Cqw1 = new ChMatrixDynamic<double>();  //< [Cqw1], the jacobian [ndoc,6] for 3 Wl rot.coordinates instead of quaternions
        protected ChMatrixDynamic<double> Cqw2 = new ChMatrixDynamic<double>();  //< [Cqw2], the jacobian [ndoc,6] for 3 Wl rot.coordinates instead of quaternions

        protected ChMatrixDynamic<double> Qc = new ChMatrixDynamic<double>();  //< {Qc}, the known part, {Qc}=-{C_dtdt}-([Cq]{q_dt})q-2[Cq_dt]{q_dt}

        protected ChMatrixDynamic<double> Ct = new ChMatrixDynamic<double>();  //< partial derivative of the link kin. equation wrt to time

        public ChMatrixDynamic<double> react = new ChMatrixDynamic<double>();  //< {l}, the lagrangians forces in the constraints

        // TEST
        ChMatrixNM<IntInterface.Three, IntInterface.Four> mGl = new ChMatrixNM<IntInterface.Three, IntInterface.Four>(0);


        public ChLinkMasked()
        {
            mask = new ChLinkMask(1);                    // create the mask;
            mask.Constr_N(0).SetMode(eChConstraintMode.CONSTRAINT_FREE);  // default: one constr.eq. but not working

            BuildLink();  // setup all matrices - if any (i.e. none in this base link)-
                          // setting automatically  n. of DOC and DOF,
        }

        public ChLinkMasked(ChLinkMasked other) : base(other) {
            mask = other.mask.Clone();

            // setup -alloc all needed matrices!!
            ChangedLinkMask();

            force_D = other.force_D.Clone();
            force_R = other.force_R.Clone();
            force_X = other.force_X.Clone();
            force_Y = other.force_Y.Clone();
            force_Z = other.force_Z.Clone();
            force_Rx = other.force_Rx.Clone();
            force_Ry = other.force_Ry.Clone();
            force_Rz = other.force_Rz.Clone();

            d_restlength = other.d_restlength;
        }

        /// "Virtual" copy constructor (covariant return type).
        public override ChObj Clone() { return new ChLinkMasked(this); }


        // [mostly internal], allocates matrices and, in general, initializes all stuff
        // which is mask-dependant.  Sets n. DOF and n .DOC.
        // Copies the mask from new_mask.
        protected void BuildLink(ChLinkMask new_mask)
        {
            // set mask
            mask = null;
            mask = new_mask.Clone();

            // setup matrices;
            BuildLink();
        }

        // [mostly internal]. as before, but uses the current (just changed) mask
        protected void BuildLink()
        {
            // set ndoc by counting non-dofs
            ndoc = mask.GetMaskDoc();
            ndoc_c = mask.GetMaskDoc_c();
            ndoc_d = mask.GetMaskDoc_d();

            // create matrices
            if (ndoc > 0)
            {
                C = new ChMatrixDynamic<double>(ndoc, 1);
                C_dt = new ChMatrixDynamic<double>(ndoc, 1);
                C_dtdt = new ChMatrixDynamic<double>(ndoc, 1);
                react = new ChMatrixDynamic<double>(ndoc, 1);
                Qc = new ChMatrixDynamic<double>(ndoc, 1);
                Ct = new ChMatrixDynamic<double>(ndoc, 1);
                Cq1 = new ChMatrixDynamic<double>(ndoc, ChBody.BODY_QDOF);
                Cq2 = new ChMatrixDynamic<double>(ndoc, ChBody.BODY_QDOF);
                Cqw1 = new ChMatrixDynamic<double>(ndoc, ChBody.BODY_DOF);
                Cqw2 = new ChMatrixDynamic<double>(ndoc, ChBody.BODY_DOF);
            }
            else
            {
               /* C = null;
                C_dt = null;
                C_dtdt = null;
                react = null;
                Qc = null;
                Ct = null;
                Cq1 = null;
                Cq2 = null;
                Cqw1 = null;
                Cqw2 = null;*/
            }
        }

        // [mostly internal], frees matrices allocated by BuildLink
        protected void DestroyLink()
        {
            if (ndoc > 0)
            {
                /*if (C != null)
                {
                    C = null;
                }
                if (C_dt != null)
                {
                    C_dt = null;
                }
                if (C_dtdt != null)
                {
                    C_dtdt = null;
                }
                if (react != null)
                {
                    react = null;
                }
                if (Qc != null)
                {
                    Qc = null;
                }
                if (Ct != null)
                {
                    Ct = null;
                }
                if (Cq1 != null)
                {
                    Cq1 = null;
                }
                if (Cq2 != null)
                {
                    Cq2 = null;
                }
                if (Cqw1 != null)
                {
                    Cqw1 = null;
                }
                if (Cqw2 != null)
                {
                    Cqw2 = null;
                }
                if (react != null)
                {
                    react = null;
                }*/
            }

            ndoc = 0;
        }


        /// Must be called after whatever change the mask of the link,
        /// in order to update auxiliary matrices sizes...
        public void ChangeLinkMask(ChLinkMask new_mask)
        {
            DestroyLink();
            BuildLink(new_mask);
        }

        /// Must be called after whatever change the mask of the link,
        /// in order to update auxiliary matrices sizes...
        public void ChangedLinkMask()
        {
            DestroyLink();
            BuildLink();
        }

        /// If some constraint is redundant, return to normal state
        public override int RestoreRedundant()  ///< \return number of changed states
        {
            int mchanges = mask.RestoreRedundant();
            if (mchanges != 0)
                ChangedLinkMask();
            return mchanges;
        }

        /// User can use this to enable/disable all the constraint of
        /// the link as desired.
        public override void SetDisabled(bool mdis)
        {
            base.SetDisabled(mdis);

            if (mask.SetAllDisabled(mdis) > 0)
                ChangedLinkMask();
        }

        /// Ex:3rd party software can set the 'broken' status via this method
        public override void SetBroken(bool mbro)
        {
            base.SetBroken(mbro);

            if (mask.SetAllBroken(mbro) > 0)
                ChangedLinkMask();
        }

        /// Get the pointer to the link mask, ie. a ChLinkMask (sort of
        /// array containing a set of ChConstraint items).
        public ChLinkMask GetMask() { return mask; }

        /// overwrites inherited implementation of this method
        public override void SetUpMarkers(ChMarker mark1, ChMarker mark2)
        {
            base.SetUpMarkers(mark1, mark2);

            // could the line below be:     assert(this.Body1 && this.Body2); ?
            if (this.Body1 != null && this.Body2 != null)
            {
                ((ChLinkMaskLF)this.mask).SetTwoBodiesVariables(Body1.Variables(), Body2.Variables());
                // This is needed because only if all constraints in mask are now active, and C,Ct,etc.
                // matrices must be allocated accordingly, otherwise are null.
                DestroyLink();
                BuildLink();
            }
        }

        //
        // STATE FUNCTIONS
        //

        public override int GetDOC() { return ndoc; }
        public override int GetDOC_c() { return ndoc_c; }
        public override int GetDOC_d() { return ndoc_d; }

        // (override/implement interfaces for global state vectors, see ChPhysicsItem for comments.)
        public override void IntStateGatherReactions(int off_L, ref ChVectorDynamic<double> L)
        {
           // if (react != null)
                L.matrix.PasteMatrix(react.matrix, off_L, 0);
        }

        public override void IntStateScatterReactions(int off_L, ChVectorDynamic<double> L)
        {
            react_force = new ChVector(0, 0, 0);   // Do not update 'intuitive' react force and torque here: just set as 0.
            react_torque = new ChVector(0, 0, 0);  // Child classes implementations should compute them.

           // if (react != null)
                react.matrix.PasteClippedMatrix(L.matrix, off_L, 0, react.matrix.GetRows(), 1, 0, 0);
        }

        public override void IntLoadResidual_CqL(int off_L,
                                     ref ChVectorDynamic<double> R,
                                     ChVectorDynamic<double> L,
                                     double c)
        {
            int cnt = 0;
            for (int i = 0; i < mask.nconstr; i++)
            {
                if (mask.Constr_N(i).IsActive())
                {
                    mask.Constr_N(i).MultiplyTandAdd(R.matrix, L.matrix[off_L + cnt] * c);
                    cnt++;
                }
            }
        }

        public override void IntLoadConstraint_C(int off_L,
                                     ref ChVectorDynamic<double> Qc,
                                     double c,
                                     bool do_clamp,
                                     double recovery_clamp)
        {
            int cnt = 0;
            for (int i = 0; i < mask.nconstr; i++) {
                if (mask.Constr_N(i).IsActive()) {
                    if (do_clamp){
                        if (mask.Constr_N(i).IsUnilateral()){
                            Qc.matrix[off_L + cnt] += ChMaths.ChMax(c * C.matrix.ElementN(cnt), -recovery_clamp);
                        } else {
                            Qc.matrix[off_L + cnt] += ChMaths.ChMin(ChMaths.ChMax(c * C.matrix.ElementN(cnt), -recovery_clamp), recovery_clamp);
                        }  
                        
                    } else 
                        Qc.matrix[off_L + cnt] += c * C.matrix.ElementN(cnt);
                    cnt++;     // The Gremlin in the works was here!   I had accidently locked this variable in the above else, causing joint jittering          
                }
            }
        }

        public override void IntLoadConstraint_Ct(int off_L, ref ChVectorDynamic<double> Qc, double c)
        {
            int cnt = 0;
            for (int i = 0; i < mask.nconstr; i++) {
                if (mask.Constr_N(i).IsActive()) {
                    Qc.matrix[off_L + cnt] += c * Ct.matrix.ElementN(cnt);
                    cnt++;
                }
            }
        }

        public override void IntToDescriptor(int off_v,
                                 ChStateDelta v,
                                 ChVectorDynamic<double> R,
                                 int off_L,
                                 ChVectorDynamic<double> L,
                                 ChVectorDynamic<double> Qc)
        {
            int cnt = 0;
            for (int i = 0; i < mask.nconstr; i++) {
                if (mask.Constr_N(i).IsActive()) {
                    mask.Constr_N(i).Set_l_i(L.matrix[off_L + cnt]); // This isn't the l_i problem!
                    mask.Constr_N(i).Set_b_i(Qc.matrix[off_L + cnt]);
                    cnt++;
                }
            }
        }
        public override void IntFromDescriptor(int off_v,
                                   ref ChStateDelta v,
                                   int off_L,
                                   ref ChVectorDynamic<double> L)
        {
            int cnt = 0;
            for (int i = 0; i < mask.nconstr; i++) {
                if (mask.Constr_N(i).IsActive()) {
                    L.matrix[off_L + cnt] = mask.Constr_N(i).Get_l_i(); // PROBLEM not return correct l_i values
                    cnt++;
                }
            }
        }

        //
        // SOLVER INTERFACE
        //

        public override void InjectConstraints(ref ChSystemDescriptor mdescriptor)
        {
            if (!this.IsActive())
                return;

            for (int i = 0; i < mask.nconstr; i++) {
                if (mask.Constr_N(i).IsActive())
                    mdescriptor.InsertConstraint(mask.Constr_N(i));
            }
        }
        public override void ConstraintsBiReset()
        {
            for (int i = 0; i < mask.nconstr; i++) {
                mask.Constr_N(i).Set_b_i(0.0);
            }
        }
        public override void ConstraintsBiLoad_C(double factor = 1, double recovery_clamp = 0.1, bool do_clamp = false)
        {
            int cnt = 0;
            for (int i = 0; i < mask.nconstr; i++) {
                if (mask.Constr_N(i).IsActive()) {
                    if (do_clamp) {
                        if (mask.Constr_N(i).IsUnilateral())
                            mask.Constr_N(i).Set_b_i(mask.Constr_N(i).Get_b_i() +
                                                     ChMaths.ChMax(factor * C.matrix.ElementN(cnt), -recovery_clamp));
                        else
                            mask.Constr_N(i).Set_b_i(mask.Constr_N(i).Get_b_i() +
                                                      ChMaths.ChMin(ChMaths.ChMax(factor * C.matrix.ElementN(cnt), -recovery_clamp), recovery_clamp));
                    }
                    else
                        mask.Constr_N(i).Set_b_i(mask.Constr_N(i).Get_b_i() + factor * C.matrix.ElementN(cnt));

                    cnt++;
                }
            }
        }
        public override void ConstraintsBiLoad_Ct(double factor = 1)
        {
            int cnt = 0;
            for (int i = 0; i < mask.nconstr; i++)
            {
                if (mask.Constr_N(i).IsActive())
                {
                    mask.Constr_N(i).Set_b_i(mask.Constr_N(i).Get_b_i() + factor * Ct.matrix.ElementN(cnt));
                    cnt++;
                }
            }
        }
        public override void ConstraintsBiLoad_Qc(double factor = 1)
        {
            int cnt = 0;
            for (int i = 0; i < mask.nconstr; i++)
            {
                if (mask.Constr_N(i).IsActive())
                {
                    mask.Constr_N(i).Set_b_i(mask.Constr_N(i).Get_b_i() + factor * Qc.matrix.ElementN(cnt));
                    cnt++;
                }
            }
        }
        public override void ConstraintsLoadJacobians()
        {
            if (this.ndoc == 0)
                return;

            int cnt = 0;
            for (int i = 0; i < mask.nconstr; i++)
            {
                if (mask.Constr_N(i).IsActive())
                {
                    mask.Constr_N(i).Get_Cq_a().PasteClippedMatrix(Cqw1.matrix, cnt, 0, 1, this.Cqw1.matrix.GetColumns(), 0, 0);
                    mask.Constr_N(i).Get_Cq_b().PasteClippedMatrix(Cqw2.matrix, cnt, 0, 1, this.Cqw2.matrix.GetColumns(), 0, 0);
                    cnt++;

                     // sets also the CFM term
                    // mask.Constr_N(i).Set_cfm_i(this.attractor);
                }
            }
        }
        public override void ConstraintsFetch_react(double factor = 1)
        {
            react_force = new ChVector(0, 0, 0);   // Do not update 'intuitive' react force and torque here: just set as 0.
            react_torque = new ChVector(0, 0, 0);  // Child classes implementations should compute them.

            // From constraints to react vector:
            int cnt = 0;
            for (int i = 0; i < mask.nconstr; i++)
            {
                if (mask.Constr_N(i).IsActive())
                {
                    react.matrix.ElementN(cnt) = mask.Constr_N(i).Get_l_i() * factor;
                    cnt++;
                }
            }
        }

        ////////////////////////////////////
        ///
        ///    UPDATING PROCEDURES

        /////////   4.5- UPDATE Cqw1 and Cqw2
        /////////
        public void Transform_Cq_to_Cqw(ChMatrix mCq, ChMatrix mCqw, ChBodyFrame mbody)
        {
           // if (mCq == null)
            //    return;

            // translational part - not changed
            mCqw.PasteClippedMatrix(mCq, 0, 0, mCq.GetRows(), 3, 0, 0);

            // rotational part [Cq_w] = [Cq_q]*[Gl]'*1/4
            int col, row, colres;
            double sum;

           // ChMatrixNM<IntInterface.Three, IntInterface.Four> mGl = new ChMatrixNM<IntInterface.Three, IntInterface.Four>(0);
            ChFrame<double>.SetMatrix_Gl(ref mGl, mbody.GetCoord().rot);

            for (colres = 0; colres < 3; colres++)
            {
                for (row = 0; row < (mCq.GetRows()); row++)
                {
                    sum = 0;
                    for (col = 0; col < 4; col++)
                    {
                        sum += ((mCq.GetElement(row, col + 3)) * (mGl.matrix.GetElement(colres, col)));
                    }
                    mCqw.SetElement(row, colres + 3, sum * 0.25);
                }
            }
        }

        //
        // UPDATING FUNCTIONS
        //

        /// This is expected to update the values in C, C_dt, C_dtdt, in
        /// jacobians Cq1 and Cq2, in Qc and in Ct.
        /// By default, this does NOTHING, so it is up to the inherited
        /// classes to fill these vectors/matrices depending on how they
        /// describe the constraint equations.
        public virtual void UpdateState() { }

        /// Updates Cqw1 and Cqw2  given updated  Cq1 and Cq2, i.e. computes the
        /// jacobians with 'Wl' rotational coordinates knowing the jacobians
        /// for body rotations in quaternion coordinates.
        public virtual void UpdateCqw()
        {
            //if (Cq1 == null || Cq2 == null)
            //    return;

           // ChLinkMasked tran = new ChLinkMasked();
            this.Transform_Cq_to_Cqw(Cq1.matrix, Cqw1.matrix, Body1);
            this.Transform_Cq_to_Cqw(Cq2.matrix, Cqw2.matrix, Body2);
        }

        /// Inherits, and updates the C_force and C_torque 'intuitive' forces,
        /// adding the effects of the contained ChLinkForce objects.
        /// (Default: inherits parent UpdateForces(), then C_force and C_torque are
        /// incremented with the Link::ChLinkForces objects)
        public override void UpdateForces(double mytime)
        {
            base.UpdateForces(mytime);

            // ** Child class can inherit this method. The parent implementation must
            //    be called _before_ adding further custom forces.

            ChVector m_force;// = new ChVector(0, 0, 0);   // initialize to zero the m1-m2 force/torque
            ChVector m_torque;// = new ChVector(0, 0, 0);  // 'intuitive' vectors (can be transformed&added later into Qf)

            // COMPUTE THE FORCES IN THE LINK, FOR EXAMPLE
            // CAUSED BY SPRINGS
            // NOTE!!!!!   C_force and C_torque   are considered in the reference coordsystem
            // of marker2  (the MAIN marker), and their application point is considered the
            // origin of marker1 (the SLAVE marker)

            // 1)========== the generic spring-damper

            if (force_D != null && force_D.Get_active())
            {
                double dfor;
                dfor = force_D.Get_Force((dist - d_restlength), dist_dt, ChTime);
                m_force = ChVector.Vmul(ChVector.Vnorm(relM.pos), dfor);

                C_force = ChVector.Vadd(C_force, m_force);
            }

            // 2)========== the generic torsional spring / torsional damper

            if (force_R != null && force_R.Get_active())
            {
                double tor;
                // 1) the tors. spring
                tor = force_R.Get_Force(relAngle, 0, ChTime);
                m_torque = ChVector.Vmul(relAxis, tor);
                C_torque = ChVector.Vadd(C_torque, m_torque);
                // 2) the tors. damper
                double angle_dt = ChVector.Vlength(relWvel);
                tor = force_R.Get_Force(0, angle_dt, ChTime);
                m_torque = ChVector.Vmul(ChVector.Vnorm(relWvel), tor);
                C_torque = ChVector.Vadd(C_torque, m_torque);
            }

            // 3)========== the XYZ forces

            m_force = ChVector.VNULL;

            if (force_X != null && force_X.Get_active())
            {
                m_force.x = force_X.Get_Force(relM.pos.x, relM_dt.pos.x, ChTime);
            }

            if (force_Y != null && force_Y.Get_active())
            {
                m_force.y = force_Y.Get_Force(relM.pos.y, relM_dt.pos.y, ChTime);
            }

            if (force_Z != null && force_Z.Get_active())
            {
                m_force.z = force_Z.Get_Force(relM.pos.z, relM_dt.pos.z, ChTime);
            }

            C_force = ChVector.Vadd(C_force, m_force);

            // 4)========== the RxRyRz forces (torques)

            m_torque = new ChVector(0, 0, 0);

            if (force_Rx != null && force_Rx.Get_active())
            {
                m_torque.x = force_Rx.Get_Force(relRotaxis.x, relWvel.x, ChTime);
            }

            if (force_Ry != null && force_Ry.Get_active())
            {
                m_torque.y = force_Ry.Get_Force(relRotaxis.y, relWvel.y, ChTime);
            }

            if (force_Rz != null && force_Rz.Get_active())
            {
                m_torque.z = force_Rz.Get_Force(relRotaxis.z, relWvel.z, ChTime);
            }

            C_torque = ChVector.Vadd(C_torque, m_torque);
        }

        // -----------COMPLETE UPDATE.
        // sequence:
        //			UpdateTime;
        //          UpdateRelMarkerCoords;
        //			UpdateState;
        //          UpdateCqw
        //			UpdateForces;

        /// This following "complete" update functions actually fill all the
        /// matrices of the link, and does all calculus, by
        /// calling all the previous Update functions in sequence.
        public override void update(double time, bool update_assets = true)
        {
            // 1 -
            UpdateTime(time);

            // 2 -
            UpdateRelMarkerCoords();

            // 3 -
            UpdateState();

            // 3b-
            UpdateCqw();

            // 4 -
            UpdateForces(time);

            // Inherit time changes of parent class (ChLinkMarkers)
            base.update(time, update_assets);
        }

        // LINK VIOLATIONS
        //
        // to get the constraint violations,
        // i.e. the residual of the constraint equations and their time derivatives

        /// Link violation (residuals of the link constraint equations)
        public ChMatrix GetC() { return C.matrix; }
        /// Time derivatives of link violations
        public ChMatrix GetC_dt() { return C_dt.matrix; }
        /// Double time derivatives of link violations
        public ChMatrix GetC_dtdt() { return C_dtdt.matrix; }

        // LINK STATE MATRICES
        //
        // Here follow the functions used by simulation engines to
        // fetch the system state matrices (the jacobians, the Q vector, etc.)
        // for building the state system matrices
        // Note that these function does not compute/update such matrices,
        // because all the updating/computing must be implemented in the Update...
        // functions above.

        /// The jacobian (body n.1 part, i.e. columns= 7 ,  rows= ndoc)
        public ChMatrix GetCq1() { return Cq1.matrix; }
        /// The jacobian (body n.2 part, i.e. columns= 7 ,  rows= ndoc)
        public ChMatrix GetCq2() { return Cq2.matrix; }

        /// The jacobian for Wl (col 6, rows= ndoc), as [Cqw1_rot]=[Cq_rot]*[Gl_1]'
        public ChMatrix GetCqw1() { return Cqw1.matrix; }
        /// The jacobian for Wl (col 6, rows= ndoc)	as [Cqw2_rot]=[Cq_rot]*[Gl_2]'
        public ChMatrix GetCqw2() { return Cqw2.matrix; }

        /// The gamma vector used in dynamics,  [Cq]x''=Qc
        public ChMatrix GetQc() { return Qc.matrix; }

        /// The Ct vector used in kinematics,  [Cq]x'=Ct
        public ChMatrix GetCt() { return Ct.matrix; }

        /// Access the reaction vector, after dynamics computations
        public ChMatrix GetReact() { return react.matrix; }

        //
        // OTHER DATA
        //

        // for the internal forces
        public ChLinkForce GetForce_D() { return force_D; }
        public ChLinkForce GetForce_R() { return force_R; }
        public ChLinkForce GetForce_X() { return force_X; }
        public ChLinkForce GetForce_Y() { return force_Y; }
        public ChLinkForce GetForce_Z() { return force_Z; }
        public ChLinkForce GetForce_Rx() { return force_Rx; }
        public ChLinkForce GetForce_Ry() { return force_Ry; }
        public ChLinkForce GetForce_Rz() { return force_Rz; }

        public void SetForce_D(ChLinkForce m_for)
        {
            if (force_D == null)
                force_D = null;
            force_D = m_for;
        }

        public void SetForce_R(ChLinkForce m_for)
        {
            if (force_R == null)
                force_R = null;
            force_R = m_for;
        }

        public void SetForce_X(ChLinkForce m_for)
        {
            if (force_X == null)
                force_X = null;
            force_X = m_for;
        }

        public void SetForce_Y(ChLinkForce m_for)
        {
            if (force_Y == null)
                force_Y = null;
            force_Y = m_for;
        }

        public void SetForce_Z(ChLinkForce m_for)
        {
            if (force_Z == null)
                force_Z = null;
            force_Z = m_for;
        }

        public void SetForce_Rx(ChLinkForce m_for)
        {
            if (force_Rx == null)
                force_Rx = null;
            force_Rx = m_for;
        }

        public void SetForce_Ry(ChLinkForce m_for)
        {
            if (force_Ry == null)
                force_Ry = null;
            force_Ry = m_for;
        }

        public void SetForce_Rz(ChLinkForce m_for)
        {
            if (force_Rz == null)
                force_Rz = null;
            force_Rz = m_for;
        }

    }

}


