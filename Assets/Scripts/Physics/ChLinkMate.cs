using System.Collections;
using System.Collections.Generic;
using UnityEngine;


namespace chrono
{

    /// Base class for all 'simple' constraints between
    /// two frames attached to two bodies. These constraints
    /// can correspond to the typical 'mating' conditions that
    /// are created in assemblies of 3D CAD tools (parallel
    /// axis, or face-to-face, etc.).
    /// Note that most of the ChLinkMate constraints can be
    /// done also with the constraints inherited from ChLinkLock...
    /// but in case of links of the ChLinkLock class they
    /// reference two ChMarker objects, tht can also move, but
    /// this is could be an unnecessary complication in most cases.

    public class ChLinkMate : ChLink
    {

        public ChLinkMate() { }
        public ChLinkMate(ChLinkMate other) : base(other) { }

        /// "Virtual" copy constructor (covariant return type).
        public override ChObj Clone()
        {
            return new ChLinkMate(this);
        }
    }

    // -----------------------------------------------------------------------------

    /// Generic mate constraint, where one can select which DOFs must be constrained
    /// between two frames attached to the two bodies.
    public class ChLinkMateGeneric : ChLinkMate
    {

        protected ChFrame<double> frame1 = new ChFrameMoving<double>();
        protected ChFrame<double> frame2 = new ChFrameMoving<double>();

        protected bool c_x;
        protected bool c_y;
        protected bool c_z;
        protected bool c_rx;
        protected bool c_ry;
        protected bool c_rz;

        protected int ndoc;    //< number of DOC, degrees of constraint
        protected int ndoc_c;  //< number of DOC, degrees of constraint (only bilaterals)
        protected int ndoc_d;  //< number of DOC, degrees of constraint (only unilaterals)

        protected ChLinkMask mask;

        protected ChMatrixDynamic<double> C;  ///< residuals


        public ChLinkMateGeneric(bool mc_x = true,
                      bool mc_y = true,
                      bool mc_z = true,
                      bool mc_rx = true,
                      bool mc_ry = true,
                      bool mc_rz = true)
        {
            c_x = mc_x;
            c_y = mc_y;
            c_z = mc_z;
            c_rx = mc_rx;
            c_ry = mc_ry;
            c_rz = mc_rz;

           // C = null;

            mask = new ChLinkMask();

            SetupLinkMask();
        }
        public ChLinkMateGeneric(ChLinkMateGeneric other)
        {
            c_x = other.c_x;
            c_y = other.c_y;
            c_z = other.c_z;
            c_rx = other.c_rx;
            c_ry = other.c_ry;
            c_rz = other.c_rz;

            SetupLinkMask();
        }


        /// "Virtual" copy constructor (covariant return type).
        public override ChObj Clone()
        {
            return new ChLinkMateGeneric(this);
        }

        /// Get the link coordinate system, expressed relative to Body2 (the 'master'
        /// body). This represents the 'main' reference of the link: reaction forces
        /// are expressed in this coordinate system.
        /// (It is the coordinate system of the contact plane relative to Body2)
        public override ChCoordsys GetLinkRelativeCoords() { return frame2.GetCoord(); }

        /// Get the master coordinate system for the assets (this will return the
        /// absolute coordinate system of the 'master' marker2)
        //public ChFrame<> GetAssetsFrame(unsigned int nclone = 0) override { return frame2 >> * GetBody2(); }

        /// Access the coordinate system considered attached to body1.
        /// Its position is expressed in the coordinate system of body1.
        public ChFrame<double> GetFrame1() { return frame1; }

        /// Access the coordinate system considered attached to body2.
        /// Its position is expressed in the coordinate system of body2.
        public ChFrame<double> GetFrame2() { return frame2; }

        public bool IsConstrainedY() { return c_y; }
        public bool IsConstrainedZ() { return c_z; }
        public bool IsConstrainedRx() { return c_rx; }
        public bool IsConstrainedRy() { return c_ry; }
        public bool IsConstrainedRz() { return c_rz; }

        /// Sets which movements (of frame 1 respect to frame 2) are constrained
        public void SetConstrainedCoords(bool mc_x, bool mc_y, bool mc_z, bool mc_rx, bool mc_ry, bool mc_rz)
        {
            c_x = mc_x;
            c_y = mc_y;
            c_z = mc_z;
            c_rx = mc_rx;
            c_ry = mc_ry;
            c_rz = mc_rz;

            SetupLinkMask();
        }

        /// Specialized initialization for generic mate, given the two bodies to be connected, the
        /// positions of the two frames to connect on the bodies (each expressed
        /// in body or abs. coordinates).
        public virtual void Initialize(ChBodyFrame mbody1,  //< first body to link
                            ChBodyFrame mbody2,  /// second body to link
                            bool pos_are_relative,  //< true: following pos. are relative to bodies.
                            ChFrame<double> mpos1,      //< mate frame (slave), for 1st body (rel. or abs., see flag above)
                            ChFrame<double> mpos2       //< mate frame (master), for 2nd body (rel. or abs., see flag above)
                            )
        {
            Debug.Assert(mbody1 != mbody2);

            this.Body1 = mbody1;
            this.Body2 = mbody2;
            // this.SetSystem(mbody1.GetSystem());

            this.mask.SetTwoBodiesVariables(Body1.Variables(), Body2.Variables());

            if (pos_are_relative)
            {
                this.frame1 = mpos1;
                this.frame2 = mpos2;
            }
            else
            {
                // from abs to body-rel
                (this.Body1).TransformParentToLocal(mpos1, this.frame1);
                this.Body2.TransformParentToLocal(mpos2, this.frame2);
            }
        }

        /// Specialized initialization for generic mate, given the two bodies to be connected, and
        /// the absolute position of the mate (the two frames to connect on the bodies will be initially cohincindent to that frame)
        public virtual void Initialize(ChBodyFrame mbody1,  //< first body to link
                            ChBodyFrame mbody2,  //< second body to link
                            ChFrame<double> mabsframe                   //< mate frame (both for slave and master), in abs. coordinate
                            )
        {
            this.Initialize(mbody1, mbody2, false, mabsframe, mabsframe);
        }

        /// Initialization based on passing two vectors (point + dir) on the
        /// two bodies, they will represent the X axes of the two frames (Y and Z will
        /// be built from the X vector via Gram Schmidt orthonormalization).
        /// Use the other ChLinkMateGeneric::Initialize() if you want to set the two frames directly.
        public virtual void Initialize(ChBodyFrame mbody1,  //< first body to link
                            ChBodyFrame mbody2,  //< second body to link
                            bool pos_are_relative,  //< true: following pos. are relative to bodies.
                            ChVector mpt1,    //< origin of slave frame 1, for 1st body (rel. or abs., see flag above)
                            ChVector mpt2,    //< origin of master frame 2, for 2nd body (rel. or abs., see flag above)
                            ChVector mnorm1,  //< X axis of slave plane, for 1st body (rel. or abs., see flag above)
                            ChVector mnorm2   //< X axis of master plane, for 2nd body (rel. or abs., see flag above)
                            )
        {
            Debug.Assert(mbody1!= mbody2);

            this.Body1 = mbody1;
            this.Body2 = mbody2;
            // this.SetSystem(mbody1.GetSystem());

            this.mask.SetTwoBodiesVariables(Body1.Variables(), Body2.Variables());

            ChVector mx = new ChVector(0, 0, 0);
            ChVector my = new ChVector(0, 0, 0);
            ChVector mz = new ChVector(0, 0, 0);
            ChVector mN = new ChVector(0, 0, 0);
            ChMatrix33<double> mrot = new ChMatrix33<double>();

            ChFrame<double> mfr1 = new ChFrame<double>();
            ChFrame<double> mfr2 = new ChFrame<double>();

            if (pos_are_relative)
            {
                mN = mnorm1;
                mN.DirToDxDyDz(ref mx, ref my, ref mz, new ChVector(0, 1, 0));
                mrot.Set_A_axis(mx, my, mz);
                mfr1.SetRot(mrot);
                mfr1.SetPos(mpt1);

                mN = mnorm2;
                mN.DirToDxDyDz(ref mx, ref my, ref mz, new ChVector(0, 1, 0));
                mrot.Set_A_axis(mx, my, mz);
                mfr2.SetRot(mrot);
                mfr2.SetPos(mpt2);
            }
            else
            {
                ChVector temp = ChVector.VECT_Z;
                // from abs to body-rel
                mN = this.Body1.TransformDirectionParentToLocal(mnorm1);
                mN.DirToDxDyDz(ref mx, ref my, ref mz, temp);
                mrot.Set_A_axis(mx, my, mz);
                mfr1.SetRot(mrot);
                mfr1.SetPos(this.Body1.TransformPointParentToLocal(mpt1));

                mN = this.Body2.TransformDirectionParentToLocal(mnorm2);
                mN.DirToDxDyDz(ref mx, ref my, ref mz, temp);
                mrot.Set_A_axis(mx, my, mz);
                mfr2.SetRot(mrot);
                mfr2.SetPos(this.Body2.TransformPointParentToLocal(mpt2));
            }

            this.frame1 = mfr1;
            this.frame2 = mfr2;
        }

        //
        // UPDATING FUNCTIONS
        //

        /// Override _all_ time, jacobian etc. updating.
        public override void update(double mytime, bool update_assets = true)
        {
            // Inherit time changes of parent class (ChLink), basically doing nothing :)
            base.update(mytime, update_assets);

            if (this.Body1 != null && this.Body2 != null)
            {
                this.mask.SetTwoBodiesVariables(Body1.Variables(), Body2.Variables());

                ChFrame<double> aframe = ChFrame<double>.BitShiftRight(this.frame1 , (this.Body1));
                ChVector p1_abs = aframe.GetPos();
                ChFrame<double> aframe2 = ChFrame<double>.BitShiftRight(this.frame2 , (this.Body2));
                ChVector p2_abs = aframe2.GetPos();
                ChFrame<double> bframe = new ChFrame<double>();
                (this.Body2).TransformParentToLocal(aframe, bframe);
                this.frame2.TransformParentToLocal(bframe, aframe);
                // Now 'aframe' contains the position/rotation of frame 1 respect to frame 2, in frame 2 coords. 
                //***TODO*** check if it is faster to do   aframe2.TransformParentToLocal(aframe, bframe); instead of two transforms above

                ChMatrix33<double> Jx1 = new ChMatrix33<double>(0);
                ChMatrix33<double> Jx2 = new ChMatrix33<double>(0);
                ChMatrix33<double> Jr1 = new ChMatrix33<double>(0);
                ChMatrix33<double> Jr2 = new ChMatrix33<double>(0);
                ChMatrix33<double> Jw1 = new ChMatrix33<double>(0);
                ChMatrix33<double> Jw2 = new ChMatrix33<double>(0);
                ChMatrix33<double> mtempM = new ChMatrix33<double>(0);
                ChMatrix33<double> mtempQ = new ChMatrix33<double>(0);

                ChMatrix33<double> abs_plane = new ChMatrix33<double>(0);
                abs_plane.nm.matrix.MatrMultiply(Body2.GetA().nm.matrix, frame2.GetA().nm.matrix);

                Jx1.nm.matrix.CopyFromMatrixT(abs_plane.nm.matrix);
                Jx2.nm.matrix.CopyFromMatrixT(abs_plane.nm.matrix);
                Jx2.nm.matrix.MatrNeg();

                Jw1.nm.matrix.MatrTMultiply(abs_plane.nm.matrix, Body1.GetA().nm.matrix);
                Jw2.nm.matrix.MatrTMultiply(abs_plane.nm.matrix, Body2.GetA().nm.matrix);

                mtempM.Set_X_matrix(frame1.GetPos());
                Jr1.nm.matrix.MatrMultiply(Jw1.nm.matrix, mtempM.nm.matrix);
                Jr1.nm.matrix.MatrNeg();

                mtempM.Set_X_matrix(frame2.GetPos());
                Jr2.nm.matrix.MatrMultiply(Jw2.nm.matrix, mtempM.nm.matrix);

                ChVector p2p1_base2 = (Body2.GetA()).MatrT_x_Vect(ChVector.Vsub(p1_abs, p2_abs));
                mtempM.Set_X_matrix(p2p1_base2);
                mtempQ.nm.matrix.MatrTMultiply(frame2.GetA().nm.matrix, mtempM.nm.matrix);
                Jr2.nm.matrix.MatrInc(mtempQ.nm.matrix);

                Jw2.nm.matrix.MatrNeg();

                // Premultiply by Jw1 and Jw2 by  0.5*[Fp(q_resid)]' to get residual as imaginary part of a quaternion.
                // For small misalignment this effect is almost insignificant cause [Fp(q_resid)]=[I],
                // but otherwise it is needed (if you want to use the stabilization term - if not, you can live without).
                mtempM.Set_X_matrix((aframe.GetRot().GetVector()) * 0.5);
                mtempM.nm.matrix[0, 0] = 0.5 * aframe.GetRot().e0;
                mtempM.nm.matrix[1, 1] = 0.5 * aframe.GetRot().e0;
                mtempM.nm.matrix[2, 2] = 0.5 * aframe.GetRot().e0;
                mtempQ.nm.matrix.MatrTMultiply(mtempM.nm.matrix, Jw1.nm.matrix);
                Jw1 = mtempQ;
                mtempQ.nm.matrix.MatrTMultiply(mtempM.nm.matrix, Jw2.nm.matrix);
                Jw2 = mtempQ;

                int nc = 0;

                if (c_x)
                {
                    this.C.matrix.ElementN(nc) = aframe.GetPos().x;
                    this.mask.Constr_N(nc).Get_Cq_a().PasteClippedMatrix(Jx1.nm.matrix, 0, 0, 1, 3, 0, 0);
                    this.mask.Constr_N(nc).Get_Cq_a().PasteClippedMatrix(Jr1.nm.matrix, 0, 0, 1, 3, 0, 3);
                    this.mask.Constr_N(nc).Get_Cq_b().PasteClippedMatrix(Jx2.nm.matrix, 0, 0, 1, 3, 0, 0);
                    this.mask.Constr_N(nc).Get_Cq_b().PasteClippedMatrix(Jr2.nm.matrix, 0, 0, 1, 3, 0, 3);
                    nc++;
                }
                if (c_y)
                {
                    this.C.matrix.ElementN(nc) = aframe.GetPos().y;
                    this.mask.Constr_N(nc).Get_Cq_a().PasteClippedMatrix(Jx1.nm.matrix, 1, 0, 1, 3, 0, 0);
                    this.mask.Constr_N(nc).Get_Cq_a().PasteClippedMatrix(Jr1.nm.matrix, 1, 0, 1, 3, 0, 3);
                    this.mask.Constr_N(nc).Get_Cq_b().PasteClippedMatrix(Jx2.nm.matrix, 1, 0, 1, 3, 0, 0);
                    this.mask.Constr_N(nc).Get_Cq_b().PasteClippedMatrix(Jr2.nm.matrix, 1, 0, 1, 3, 0, 3);
                    nc++;
                }
                if (c_z)
                {
                    this.C.matrix.ElementN(nc) = aframe.GetPos().z;
                    this.mask.Constr_N(nc).Get_Cq_a().PasteClippedMatrix(Jx1.nm.matrix, 2, 0, 1, 3, 0, 0);
                    this.mask.Constr_N(nc).Get_Cq_a().PasteClippedMatrix(Jr1.nm.matrix, 2, 0, 1, 3, 0, 3);
                    this.mask.Constr_N(nc).Get_Cq_b().PasteClippedMatrix(Jx2.nm.matrix, 2, 0, 1, 3, 0, 0);
                    this.mask.Constr_N(nc).Get_Cq_b().PasteClippedMatrix(Jr2.nm.matrix, 2, 0, 1, 3, 0, 3);
                    nc++;
                }
                if (c_rx)
                {
                    this.C.matrix.ElementN(nc) = aframe.GetRot().e1;
                    this.mask.Constr_N(nc).Get_Cq_a().FillElem(0);
                    this.mask.Constr_N(nc).Get_Cq_b().FillElem(0);
                    this.mask.Constr_N(nc).Get_Cq_a().PasteClippedMatrix(Jw1.nm.matrix, 0, 0, 1, 3, 0, 3);
                    this.mask.Constr_N(nc).Get_Cq_b().PasteClippedMatrix(Jw2.nm.matrix, 0, 0, 1, 3, 0, 3);
                    nc++;
                }
                if (c_ry)
                {
                    this.C.matrix.ElementN(nc) = aframe.GetRot().e2;                    
                    this.mask.Constr_N(nc).Get_Cq_a().FillElem(0);
                    this.mask.Constr_N(nc).Get_Cq_b().FillElem(0);
                    this.mask.Constr_N(nc).Get_Cq_a().PasteClippedMatrix(Jw1.nm.matrix, 1, 0, 1, 3, 0, 3);
                    this.mask.Constr_N(nc).Get_Cq_b().PasteClippedMatrix(Jw2.nm.matrix, 1, 0, 1, 3, 0, 3);
                    nc++;
                }
                if (c_rz)
                {
                    this.C.matrix.ElementN(nc) = aframe.GetRot().e3;
                    this.mask.Constr_N(nc).Get_Cq_a().FillElem(0);
                    this.mask.Constr_N(nc).Get_Cq_b().FillElem(0);
                    this.mask.Constr_N(nc).Get_Cq_a().PasteClippedMatrix(Jw1.nm.matrix, 2, 0, 1, 3, 0, 3);
                    this.mask.Constr_N(nc).Get_Cq_b().PasteClippedMatrix(Jw2.nm.matrix, 2, 0, 1, 3, 0, 3);
                    nc++;
                }

            }
        }

        /// If some constraint is redundant, return to normal state
        public override int RestoreRedundant()
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

        public override int GetDOC() { return ndoc; }
        public override int GetDOC_c() { return ndoc_c; }
        public override int GetDOC_d() { return ndoc_d; }

        //
        // STATE FUNCTIONS
        //

        // (override/implement interfaces for global state vectors, see ChPhysicsItem for comments.)
        public override void IntStateGatherReactions(int off_L, ref ChVectorDynamic<double> L)
        {
            if (!this.IsActive())
                return;

            int nc = 0;
            if (c_x)
            {
                if (mask.Constr_N(nc).IsActive())
                    L.matrix[off_L + nc] = -react_force.x;
                nc++;
            }
            if (c_y)
            {
                if (mask.Constr_N(nc).IsActive())
                    L.matrix[off_L + nc] = -react_force.y;
                nc++;
            }
            if (c_z)
            {
                if (mask.Constr_N(nc).IsActive())
                    L.matrix[off_L + nc] = -react_force.z;
                nc++;
            }
            if (c_rx)
            {
                if (mask.Constr_N(nc).IsActive())
                    L.matrix[off_L + nc] = -2.0 * react_torque.x;
                nc++;
            }
            if (c_ry)
            {
                if (mask.Constr_N(nc).IsActive())
                    L.matrix[off_L + nc] = -2.0 * react_torque.y;
                nc++;
            }
            if (c_rz)
            {
                if (mask.Constr_N(nc).IsActive())
                    L.matrix[off_L + nc] = -2.0 * react_torque.z;
                nc++;
            }
        }
        public override void IntStateScatterReactions(int off_L, ChVectorDynamic<double> L)
        {
            react_force = new ChVector(0, 0, 0);
            react_torque = new ChVector(0, 0, 0);

            if (!this.IsActive())
                return;

            int nc = 0;
            if (c_x)
            {
                if (mask.Constr_N(nc).IsActive())
                    react_force.x = -L.matrix[off_L + nc];
                nc++;
            }
            if (c_y)
            {
                if (mask.Constr_N(nc).IsActive())
                    react_force.y = -L.matrix[off_L + nc];
                nc++;
            }
            if (c_z)
            {
                if (mask.Constr_N(nc).IsActive())
                    react_force.z = -L.matrix[off_L + nc];
                nc++;
            }
            if (c_rx)
            {
                if (mask.Constr_N(nc).IsActive())
                    react_torque.x = -0.5 * L.matrix[off_L + nc];
                nc++;
            }
            if (c_ry)
            {
                if (mask.Constr_N(nc).IsActive())
                    react_torque.y = -0.5 * L.matrix[off_L + nc];
                nc++;
            }
            if (c_rz)
            {
                if (mask.Constr_N(nc).IsActive())
                    react_torque.z = -0.5 * L.matrix[off_L + nc];
                nc++;
            }
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
            for (int i = 0; i < mask.nconstr; i++)
            {
                if (mask.Constr_N(i).IsActive())
                {
                    if (do_clamp)
                    {
                        if (mask.Constr_N(i).IsUnilateral())
                            Qc.matrix[off_L + cnt] += ChMaths.ChMax(c * C.matrix.ElementN(cnt), -recovery_clamp);
                        else
                            Qc.matrix[off_L + cnt] += ChMaths.ChMin(ChMaths.ChMax(c * C.matrix.ElementN(cnt), -recovery_clamp), recovery_clamp);
                    }
                    else
                        Qc.matrix[off_L + cnt] += c * C.matrix.ElementN(cnt);
                    cnt++;
                }
            }
        }
        public override void IntLoadConstraint_Ct(int off, ref ChVectorDynamic<double> Qc, double c)
        {
            // NOT NEEDED BECAUSE NO RHEONOMIC TERM
        }
        public override void IntToDescriptor(int off_v,
                                     ChStateDelta v,
                                     ChVectorDynamic<double> R,
                                     int off_L,
                                     ChVectorDynamic<double> L,
                                     ChVectorDynamic<double> Qc)
        {
            int cnt = 0;
            for (int i = 0; i < mask.nconstr; i++)
            {
                if (mask.Constr_N(i).IsActive())
                {
                    mask.Constr_N(i).Set_l_i(L.matrix[off_L + cnt]);
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
            for (int i = 0; i < mask.nconstr; i++)
            {
                if (mask.Constr_N(i).IsActive())
                {
                    L.matrix[off_L + cnt] = mask.Constr_N(i).Get_l_i();
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

            for (int i = 0; i < mask.nconstr; i++)
            {
                if (mask.Constr_N(i).IsActive())
                    mdescriptor.InsertConstraint(mask.Constr_N(i));
            }
        }
        public override void ConstraintsBiReset()
        {
            if (!this.IsActive())
                return;

            for (int i = 0; i < mask.nconstr; i++)
            {
                mask.Constr_N(i).Set_b_i(0.0);
            }
        }
        public override void ConstraintsBiLoad_C(double factor = 1, double recovery_clamp = 0.1, bool do_clamp = false)
        {
            if (!this.IsActive())
                return;

            //***TEST***
            /*
                GetLog()<< "cload: " ;
                if (this.c_x) GetLog()<< " x";
                if (this.c_y) GetLog()<< " y";
                if (this.c_z) GetLog()<< " z";
                if (this.c_rx) GetLog()<< " Rx";
                if (this.c_ry) GetLog()<< " Ry";
                if (this.c_rz) GetLog()<< " Rz";
                GetLog()<< *this.C << "\n";
            */
            int cnt = 0;
            for (int i = 0; i < mask.nconstr; i++)
            {
                if (mask.Constr_N(i).IsActive())
                {
                    if (do_clamp)
                    {
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
            if (!this.IsActive())
                return;

            // NOT NEEDED BECAUSE NO RHEONOMIC TERM
        }
        public override void ConstraintsLoadJacobians()
        {
            // already loaded when doing Update (which used the matrices of the scalar constraint objects)
        }
        public override void ConstraintsFetch_react(double factor = 1)
        {
            react_force = new ChVector(0, 0, 0);
            react_torque = new ChVector(0, 0, 0);

            if (!this.IsActive())
                return;

            int nc = 0;
            if (c_x)
            {
                if (mask.Constr_N(nc).IsActive())
                    react_force.x = -mask.Constr_N(nc).Get_l_i() * factor;
                nc++;
            }
            if (c_y)
            {
                if (mask.Constr_N(nc).IsActive())
                    react_force.y = -mask.Constr_N(nc).Get_l_i() * factor;
                nc++;
            }
            if (c_z)
            {
                if (mask.Constr_N(nc).IsActive())
                    react_force.z = -mask.Constr_N(nc).Get_l_i() * factor;
                nc++;
            }
            if (c_rx)
            {
                if (mask.Constr_N(nc).IsActive())
                    react_torque.x = -0.5 * mask.Constr_N(nc).Get_l_i() * factor;
                nc++;
            }
            if (c_ry)
            {
                if (mask.Constr_N(nc).IsActive())
                    react_torque.y = -0.5 * mask.Constr_N(nc).Get_l_i() * factor;
                nc++;
            }
            if (c_rz)
            {
                if (mask.Constr_N(nc).IsActive())
                    react_torque.z = -0.5 * mask.Constr_N(nc).Get_l_i() * factor;
                nc++;
            }
        }

        protected void SetupLinkMask()
        {
            int nc = 0;
            if (c_x)
                nc++;
            if (c_y)
                nc++;
            if (c_z)
                nc++;
            if (c_rx)
                nc++;
            if (c_ry)
                nc++;
            if (c_rz)
                nc++;

            mask.ResetNconstr(nc);

          //  if (C != null)
          //      C = null;
            C = new ChMatrixDynamic<double>(nc, 1);

            ChangedLinkMask();
        }
        protected void ChangedLinkMask()
        {
            ndoc = mask.GetMaskDoc();
            ndoc_c = mask.GetMaskDoc_c();
            ndoc_d = mask.GetMaskDoc_d();
        }
    }

}
