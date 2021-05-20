using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;


namespace chrono
{
    /// Class for modeling a composite revolute-spherical joint between two
    /// two ChBodyFrame objects.  This joint is defined through a point and
    /// direction on the first body (the revolute side), a point on the second
    /// body (the spherical side), and a distance.  Kinematically, the two points
    /// are maintained at the prescribed distance while the vector between the
    /// two points is always perpendicular to the provided direction of the
    /// revolute joint.
    public class ChLinkRevoluteSpherical : ChLink
    {
        public ChBody body1;
        public ChBody body2;

        private ChVector sPosition1 = new ChVector(0, 0, 0);
        private ChVector sPosition2 = new ChVector(0, 0, 0);

        public ChLinkRevoluteSpherical() {
            m_pos1 = new ChVector(0, 0, 0);
            m_pos2 = new ChVector(0, 0, 0);
            m_dir1 = new ChVector(0, 0, 1);
            m_dist = 0;
            m_cur_dist = 0;
            m_cur_dot = 0;
            m_C = new ChMatrixDynamic<double>(2, 1);

            m_multipliers[0] = 0;
            m_multipliers[1] = 0;
        }

        public ChLinkRevoluteSpherical(ChLinkRevoluteSpherical other) {
            Body1 = other.Body1;
            Body2 = other.Body2;
           // system = other.system;

            m_pos1 = other.m_pos1;
            m_pos2 = other.m_pos2;
            m_dir1 = other.m_dir1;
            m_dist = other.m_dist;
            m_cur_dist = other.m_cur_dist;
            m_cur_dot = other.m_cur_dot;

            m_cnstr_dist.SetVariables(other.Body1.Variables(), other.Body2.Variables());
            m_cnstr_dot.SetVariables(other.Body1.Variables(), other.Body2.Variables());

            m_multipliers[0] = other.m_multipliers[0];
            m_multipliers[1] = other.m_multipliers[1];
        }

        public void Start()
        {
            ChCoordsys csys = new ChCoordsys(Utils.ToChrono(transform.position), Utils.ToChrono(transform.rotation));
            Initialize(body1.BodyFrame, body2.BodyFrame, csys, m_dist);

            ChSystem.system.AddLink(this);
        }

        /// "Virtual" copy ructor (covariant return type).
        public override ChObj Clone()  { return new ChLinkRevoluteSpherical(this); }

        public void Update()
        {
            sPosition1 = GetPoint1Abs();
            sPosition2 = GetPoint2Abs();
        }

        protected virtual void OnDrawGizmos()
        {
            if (Application.isPlaying)
            {
                Gizmos.color = new Color(0, 255, 0);
                Gizmos.DrawLine(new Vector3((float)sPosition1.x, (float)sPosition1.y, (float)sPosition1.z), new Vector3((float)sPosition2.x, (float)sPosition2.y, (float)sPosition2.z));
            }
        }

        /// Get the number of (bilateral) raints introduced by this joint.
        public override int GetDOC_c() { return 2; }

        /// Get the point on Body1 (revolute side), expressed in Body1 coordinate system.
        public ChVector GetPoint1Rel() { return m_pos1; }
        /// Get the direction of the revolute joint, expressed in Body1 coordinate system.
        public ChVector GetDir1Rel() { return m_dir1; }
        /// Get the point on Body2 (spherical side), expressed in Body2 coordinate system.
        public ChVector GetPoint2Rel() { return m_pos2; }

        /// Get the imposed distance (length of massless connector).
        public double GetImposedDistance() { return m_dist; }
        /// Get the current distance between the two points.
        public double GetCurrentDistance() { return m_cur_dist; }

        /// Get the point on Body1 (revolute side), expressed in absolute coordinate system.
        public ChVector GetPoint1Abs() { return Body1.TransformPointLocalToParent(m_pos1); }
        /// Get the direction of the revolute joint, expressed in absolute coordinate system.
        public ChVector GetDir1Abs() { return Body1.TransformDirectionLocalToParent(m_dir1); }
        /// Get the point on Body2 (spherical side), expressed in absolute coordinate system.
        public ChVector GetPoint2Abs() { return Body2.TransformPointLocalToParent(m_pos2); }

        /// Get the link coordinate system, expressed relative to Body2 (spherical side).
        /// This represents the 'main' reference of the link: reaction forces
        /// and reaction torques are reported in this coordinate system.
        public override ChCoordsys GetLinkRelativeCoords() {
            ChVector pos1 = Body2.TransformPointParentToLocal(Body1.TransformPointLocalToParent(m_pos1));
            ChMatrix33<double> A = new ChMatrix33<double>(0);

            ChVector u = (m_pos2 - pos1).GetNormalized();
            ChVector w = Body2.TransformDirectionParentToLocal(Body1.TransformDirectionLocalToParent(m_dir1));
            ChVector v = ChVector.Vcross(w, u);
            A.Set_A_axis(u, v, w);

            return new ChCoordsys(pos1, A.Get_A_quaternion());
        }

        /// Get the joint violation (residuals of the raint equations)
        public ChMatrixDynamic<double> GetC() { return m_C; }

        /// Initialize this joint by specifying the two bodies to be connected, a
        /// coordinate system specified in the absolute frame, and the distance of
        /// the massless connector.  The composite joint is ructed such that the
        /// direction of the revolute joint is aligned with the z axis of the specified
        /// coordinate system and the spherical joint is at the specified distance
        /// along the x axis.
        public void Initialize(ChBodyFrame body1,  //< first frame (revolute side)
                    ChBodyFrame body2,  //< second frame (spherical side)
                    ChCoordsys csys,  //< joint coordinate system (in absolute frame)
                    double distance            //< imposed distance
                    )
        {
            Body1 = body1;
            Body2 = body2;

            m_cnstr_dist.SetVariables(Body1.Variables(), Body2.Variables());
            m_cnstr_dot.SetVariables(Body1.Variables(), Body2.Variables());

            ChVector x_Axis = csys.rot.GetXaxis();
            ChVector z_axis = csys.rot.GetZaxis();

            m_pos1 = Body1.TransformPointParentToLocal(csys.pos);
            m_dir1 = Body1.TransformDirectionParentToLocal(z_axis);
            m_pos2 = Body2.TransformPointParentToLocal(csys.pos + distance * x_Axis);

            m_dist = distance;
            m_cur_dist = distance;
            m_cur_dot = 0;
        }

        /// Initialize this joint by specifying the two bodies to be connected, a point
        /// and a direction on body1 defining the revolute joint, and a point on the
        /// second body defining the spherical joint. If local = true, it is assumed
        /// that these quantities are specified in the local body frames. Otherwise,
        /// it is assumed that they are specified in the absolute frame. The imposed
        /// distance between the two points can be either inferred from the provided
        /// configuration (auto_distance = true) or specified explicitly.
        public void Initialize(ChBodyFrame body1,  //< first frame (revolute side)
                    ChBodyFrame body2,  //< second frame (spherical side)
                    bool local,                 //< true if data given in body local frames
                    ChVector pos1,     //< point on first frame (center of revolute)
                    ChVector dir1,     //< direction of revolute on first frame
                    ChVector pos2,     //< point on second frame (center of spherical)
                    bool auto_distance = true,  //< true if imposed distance equal to |pos1 - po2|
                    double distance = 0         //< imposed distance (used only if auto_distance = false)
                    )
        {
            Body1 = body1;
            Body2 = body2;

            m_cnstr_dist.SetVariables(Body1.Variables(), Body2.Variables());
            m_cnstr_dot.SetVariables(Body1.Variables(), Body2.Variables());

            ChVector pos1_abs;
            ChVector pos2_abs;
            ChVector dir1_abs;

            if (local)
            {
                m_pos1 = pos1;
                m_pos2 = pos2;
                m_dir1 = ChVector.Vnorm(dir1);
                pos1_abs = Body1.TransformPointLocalToParent(m_pos1);
                pos2_abs = Body2.TransformPointLocalToParent(m_pos2);
                dir1_abs = Body1.TransformDirectionLocalToParent(m_dir1);
            }
            else
            {
                pos1_abs = pos1;
                pos2_abs = pos2;
                dir1_abs = ChVector.Vnorm(dir1);
                m_pos1 = Body1.TransformPointParentToLocal(pos1_abs);
                m_pos2 = Body2.TransformPointParentToLocal(pos2_abs);
                m_dir1 = Body1.TransformDirectionParentToLocal(dir1_abs);
            }

            ChVector d12_abs = pos2_abs - pos1_abs;

            m_cur_dist = d12_abs.Length();
            m_dist = auto_distance ? m_cur_dist : distance;

            m_cur_dot = ChVector.Vdot(d12_abs, dir1_abs);
        }

        //
        // UPDATING FUNCTIONS
        //

        /// Perform the update of this joint at the specified time: compute jacobians,
        /// raint violations, etc. and cache in internal structures
        public override void update(double time, bool update_assets = true)
        {
            // Inherit time changes of parent class (ChLink)
            base.update(time, update_assets);

            // Express the body locations and direction in absolute frame
            ChVector pos1_abs = Body1.TransformPointLocalToParent(m_pos1);
            ChVector pos2_abs = Body2.TransformPointLocalToParent(m_pos2);
            ChVector dir1_abs = Body1.TransformDirectionLocalToParent(m_dir1);
            ChVector d12_abs = pos2_abs - pos1_abs;

            // Update current distance and dot product
            m_cur_dist = d12_abs.Length();
            m_cur_dot = ChVector.Vdot(d12_abs, dir1_abs);

            // Calculate a unit vector in the direction d12, expressed in absolute frame
            // Then express it in the two body frames
            ChVector u12_abs = d12_abs / m_cur_dist;
            ChVector u12_loc1 = Body1.TransformDirectionParentToLocal(u12_abs);
            ChVector u12_loc2 = Body2.TransformDirectionParentToLocal(u12_abs);

            // Express the direction vector in the frame of body 2
            ChVector dir1_loc2 = Body2.TransformDirectionParentToLocal(dir1_abs);

            // Cache violation of the distance constraint
            m_C.matrix.SetElement(0, 0, m_cur_dist - m_dist);

            // Compute Jacobian of the distance constraint
            //    ||pos2_abs - pos1_abs|| - dist = 0
            {
                ChVector Phi_r1 = -u12_abs;
                ChVector Phi_pi1 = ChVector.Vcross(u12_loc1, m_pos1);

                m_cnstr_dist.Get_Cq_a().ElementN(0) = Phi_r1.x;
                m_cnstr_dist.Get_Cq_a().ElementN(1) = Phi_r1.y;
                m_cnstr_dist.Get_Cq_a().ElementN(2) = Phi_r1.z;

                m_cnstr_dist.Get_Cq_a().ElementN(3) = Phi_pi1.x;
                m_cnstr_dist.Get_Cq_a().ElementN(4) = Phi_pi1.y;
                m_cnstr_dist.Get_Cq_a().ElementN(5) = Phi_pi1.z;

                ChVector Phi_r2 = u12_abs;
                ChVector Phi_pi2 = -ChVector.Vcross(u12_loc2, m_pos2);

                m_cnstr_dist.Get_Cq_b().ElementN(0) = Phi_r2.x;
                m_cnstr_dist.Get_Cq_b().ElementN(1) = Phi_r2.y;
                m_cnstr_dist.Get_Cq_b().ElementN(2) = Phi_r2.z;

                m_cnstr_dist.Get_Cq_b().ElementN(3) = Phi_pi2.x;
                m_cnstr_dist.Get_Cq_b().ElementN(4) = Phi_pi2.y;
                m_cnstr_dist.Get_Cq_b().ElementN(5) = Phi_pi2.z;
            }

            // Cache violation of the dot constraint
            m_C.matrix.SetElement(1, 0, m_cur_dot);

            // Compute Jacobian of the dot constraint
            //    dot(dir1_abs, pos2_abs - pos1_abs) = 0
            {
                ChVector Phi_r1 = -dir1_abs;
                ChVector Phi_pi1 = ChVector.Vcross(m_dir1, m_pos1) - ChVector.Vcross(u12_loc1, m_pos1);

                m_cnstr_dot.Get_Cq_a().ElementN(0) = Phi_r1.x;
                m_cnstr_dot.Get_Cq_a().ElementN(1) = Phi_r1.y;
                m_cnstr_dot.Get_Cq_a().ElementN(2) = Phi_r1.z;

                m_cnstr_dot.Get_Cq_a().ElementN(3) = Phi_pi1.x;
                m_cnstr_dot.Get_Cq_a().ElementN(4) = Phi_pi1.y;
                m_cnstr_dot.Get_Cq_a().ElementN(5) = Phi_pi1.z;

                ChVector Phi_r2 = dir1_abs;
                ChVector Phi_pi2 = -ChVector.Vcross(dir1_loc2, m_pos2);

                m_cnstr_dot.Get_Cq_b().ElementN(0) = Phi_r2.x;
                m_cnstr_dot.Get_Cq_b().ElementN(1) = Phi_r2.y;
                m_cnstr_dot.Get_Cq_b().ElementN(2) = Phi_r2.z;

                m_cnstr_dot.Get_Cq_b().ElementN(3) = Phi_pi2.x;
                m_cnstr_dot.Get_Cq_b().ElementN(4) = Phi_pi2.y;
                m_cnstr_dot.Get_Cq_b().ElementN(5) = Phi_pi2.z;
            }
        }

        //
        // STATE FUNCTIONS
        //

        public override void IntStateGatherReactions(int off_L, ref ChVectorDynamic<double> L)
        {
            if (!this.IsActive())
                return;

            L.matrix[off_L + 0] = m_multipliers[0];
            L.matrix[off_L + 1] = m_multipliers[1];
        }

        public override void IntStateScatterReactions( int off_L,  ChVectorDynamic<double> L) {
            if (!this.IsActive())
                return;

            m_multipliers[0] = L.matrix[off_L + 0];
            m_multipliers[1] = L.matrix[off_L + 1];

            // Also compute 'intuitive' reactions:

            double lam_dist = m_multipliers[0];  // ||pos2_abs - pos1_abs|| - dist = 0
            double lam_dot = m_multipliers[1];   // dot(dir1_abs, pos2_abs - pos1_abs) = 0

            // Calculate the reaction torques and forces on Body 2 in the joint frame
            // (Note: origin of the joint frame is at the center of the revolute joint
            //  which is defined on body 1, the x-axis is along the vector from the
            //  point on body 1 to the point on body 2.  The z axis is along the revolute
            //  axis defined for the joint)
            react_force.x = lam_dist;
            react_force.y = 0;
            react_force.z = lam_dot;

            react_torque.x = 0;
            react_torque.y = -m_cur_dist * lam_dot;
            react_torque.z = 0;
        }

        public override void IntLoadResidual_CqL(int off_L,
                                          ref ChVectorDynamic<double> R,
                                          ChVectorDynamic<double> L,
                                          double c)
        {
            m_cnstr_dist.MultiplyTandAdd(R.matrix, L.matrix[off_L + 0] * c);
            m_cnstr_dot.MultiplyTandAdd(R.matrix, L.matrix[off_L + 1] * c);
        }

        public override void IntLoadConstraint_C( int off_L,
                                     ref ChVectorDynamic<double> Qc,
                                     double c,
                                     bool do_clamp,
                                     double recovery_clamp) {
            if (!IsActive())
                return;

            double cnstr_dist_violation =
                do_clamp ? ChMaths.ChMin(ChMaths.ChMax(c * (m_cur_dist - m_dist), -recovery_clamp), recovery_clamp) : c * (m_cur_dist - m_dist);

            double cnstr_dot_violation =
                do_clamp ? ChMaths.ChMin(ChMaths.ChMax(c * m_cur_dot, -recovery_clamp), recovery_clamp) : c * m_cur_dot;

            Qc.matrix[off_L + 0] += cnstr_dist_violation;
            Qc.matrix[off_L + 1] += cnstr_dot_violation;
        }

        public override void IntToDescriptor( int off_v,
                                  ChStateDelta v,
                                  ChVectorDynamic<double> R,
                                  int off_L,
                                  ChVectorDynamic<double> L,
                                  ChVectorDynamic<double> Qc) {
            if (!IsActive())
                return;

            m_cnstr_dist.Set_l_i(L.matrix[off_L + 0]);
            m_cnstr_dot.Set_l_i(L.matrix[off_L + 1]);

            m_cnstr_dist.Set_b_i(Qc.matrix[off_L + 0]);
            m_cnstr_dot.Set_b_i(Qc.matrix[off_L + 1]);
        }

        public override void IntFromDescriptor( int off_v,
                                   ref ChStateDelta v,
                                    int off_L,
                                   ref ChVectorDynamic<double> L) {
            if (!IsActive())
                return;

            L.matrix[off_L + 0] = m_cnstr_dist.Get_l_i();
            L.matrix[off_L + 1] = m_cnstr_dot.Get_l_i();
        }

        //
        // SOLVER INTERFACE
        //

        public override void InjectConstraints(ref ChSystemDescriptor descriptor) {
            if (!IsActive())
                return;

            descriptor.InsertConstraint(m_cnstr_dist);
            descriptor.InsertConstraint(m_cnstr_dot);
        }

        public override void ConstraintsBiReset() {
            m_cnstr_dist.Set_b_i(0.0);
            m_cnstr_dot.Set_b_i(0.0);
        }

        public override void ConstraintsBiLoad_C(double factor = 1, double recovery_clamp = 0.1, bool do_clamp = false) {
            if (!IsActive())
                return;

            double cnstr_dist_violation = do_clamp
                                              ? ChMaths.ChMin(ChMaths.ChMax(factor * (m_cur_dist - m_dist), -recovery_clamp), recovery_clamp)
                                              : factor * (m_cur_dist - m_dist);

            double cnstr_dot_violation =
                do_clamp ? ChMaths.ChMin(ChMaths.ChMax(factor * m_cur_dot, -recovery_clamp), recovery_clamp) : factor * m_cur_dot;

            m_cnstr_dist.Set_b_i(m_cnstr_dist.Get_b_i() + cnstr_dist_violation);
            m_cnstr_dot.Set_b_i(m_cnstr_dot.Get_b_i() + cnstr_dot_violation);
        }

        public override void ConstraintsLoadJacobians() {
            // Nothing to do here. Jacobians were loaded in Update().
        }
        public override void ConstraintsFetch_react(double factor = 1) {
            // Extract the Lagrange multipliers for the distance and for
            // the dot constraint.
            double lam_dist = m_cnstr_dist.Get_l_i();  // ||pos2_abs - pos1_abs|| - dist = 0
            double lam_dot = m_cnstr_dot.Get_l_i();    // dot(dir1_abs, pos2_abs - pos1_abs) = 0

            // Note that the Lagrange multipliers must be multiplied by 'factor' to
            // convert from reaction impulses to reaction forces.
            lam_dist *= factor;
            lam_dot *= factor;

            // Calculate the reaction torques and forces on Body 2 in the joint frame
            // (Note: origin of the joint frame is at the center of the revolute joint
            //  which is defined on body 1, the x-axis is along the vector from the
            //  point on body 1 to the point on body 2.  The z axis is along the revolute
            //  axis defined for the joint)
            react_force.x = lam_dist;
            react_force.y = 0;
            react_force.z = lam_dot;

            react_torque.x = 0;
            react_torque.y = -m_cur_dist * lam_dot;
            react_torque.z = 0;
        }

        //
        // EXTRA REACTION FORCE & TORQUE FUNCTIONS
        //

        public ChVector Get_react_force_body1() {
            // Calculate the reaction forces on Body 1 in the joint frame
            // (Note: origin of the joint frame is at the center of the revolute joint
            //  which is defined on body 1, the x-axis is along the vector from the
            //  point on body 1 to the point on body 2.  The z axis is along the revolute
            //  axis defined for the joint)
            //  react_force = (-lam_dist,0,-lam_dot)

            return -react_force;
        }
        public ChVector Get_react_torque_body1() {
            // Calculate the reaction forces on Body 1 in the joint frame
            // (Note: origin of the joint frame is at the center of the revolute joint
            //  which is defined on body 1, the x-axis is along the vector from the
            //  point on body 1 to the point on body 2.  The z axis is along the revolute
            //  axis defined for the joint)
            //  react_torque = (0,m_cur_dist*lam_dot,0)

            return -react_torque;
        }
        public ChVector Get_react_force_body2() {
            // Calculate the reaction torques on Body 2 in the joint frame at the spherical joint
            // (Note: the joint frame x-axis is along the vector from the
            //  point on body 1 to the point on body 2.  The z axis is along the revolute
            //  axis defined for the joint)
            //  react_force = (lam_dist,0,lam_dot)
            return react_force;
        }
        public ChVector Get_react_torque_body2() {
            // Calculate the reaction torques on Body 2 in the joint frame at the spherical joint
            // (Note: the joint frame x-axis is along the vector from the
            //  point on body 1 to the point on body 2.  The z axis is along the revolute
            //  axis defined for the joint)
            //  react_torque = (0,0,0)
            return ChVector.VNULL;
        }



        public ChVector m_pos1;  //< point on first frame (in local frame)
        public ChVector m_pos2;  //< point on second frame (in local frame)
        public ChVector m_dir1;  //< direction of revolute on first frame (in local frame)
        public double m_dist;      //< imposed distance between pos1 and pos2

        public double m_cur_dist;  //< actual distance between pos1 and pos2
        public double m_cur_dot;   //< actual value of dot raint

        public ChConstraintTwoBodies m_cnstr_dist = new ChConstraintTwoBodies();  //< raint: ||pos2_abs - pos1_abs|| - dist = 0
        ChConstraintTwoBodies m_cnstr_dot = new ChConstraintTwoBodies();   //< raint: dot(dir1_abs, pos2_abs - pos1_abs) = 0

        public ChMatrixDynamic<double> m_C;  ////< current raint violations

        public double[] m_multipliers = new double[2];  //< Lagrange multipliers
    }
}
