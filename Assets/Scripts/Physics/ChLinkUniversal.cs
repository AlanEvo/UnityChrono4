using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;



namespace chrono
{
    /// Class for modeling a universal joint between two two ChBodyFrame objects.
    /// This joint is defined through 4 constraint equations between two marker
    /// frames, one on each body.  Kinematically, these constraints impose the
    /// condition that the two marker origins coincide (3 constraints) and that
    /// two directions (one on each body) are always perpendicular. Together,
    /// these constraints model the cross in a physical universal joint.
    public class ChLinkUniversal : ChLink
    {
        public ChBody body1;
        public ChBody body2;

        public ChLinkUniversal()
        {
            m_C = new ChMatrixDynamic<double>(4, 1);

            m_multipliers[0] = 0;
            m_multipliers[1] = 0;
            m_multipliers[2] = 0;
            m_multipliers[3] = 0;
        }
        public ChLinkUniversal(ChLinkUniversal other)
        {
            Body1 = other.Body1;
            Body2 = other.Body2;
            //system = other.system;

            m_frame1 = other.m_frame1;
            m_frame2 = other.m_frame2;

            m_cnstr_x.SetVariables(other.Body1.Variables(), other.Body2.Variables());
            m_cnstr_y.SetVariables(other.Body1.Variables(), other.Body2.Variables());
            m_cnstr_z.SetVariables(other.Body1.Variables(), other.Body2.Variables());
            m_cnstr_dot.SetVariables(other.Body1.Variables(), other.Body2.Variables());

            m_multipliers[0] = other.m_multipliers[0];
            m_multipliers[1] = other.m_multipliers[1];
            m_multipliers[2] = other.m_multipliers[2];
            m_multipliers[3] = other.m_multipliers[3];
        }

        /// "Virtual" copy constructor (covariant return type).
        public override ChObj Clone() { return new ChLinkUniversal(this); }

        public void Start()
        {
            ChFrame<double> frame = new ChFrame<double>(Utils.ToChrono(transform.position), Utils.ToChrono(transform.rotation));
            Initialize(body1.BodyFrame, body2.BodyFrame, frame);

            ChSystem msystem = FindObjectOfType<ChSystem>();
            msystem.AddLink(this);
        }

        protected virtual void OnDrawGizmos()
        {
            // There must be a claner way of calculating these quaternions?
            Color color = Color.green;
            UnityEngine.Quaternion offset = new UnityEngine.Quaternion(0.707f, 0, -0.707f, 0);
            UnityEngine.Quaternion offset2 = new UnityEngine.Quaternion(0.707f, 0, 0.707f, 0);
            UnityEngine.Quaternion offset3 = new UnityEngine.Quaternion(0.707f, 0, 0, 0.707f);
            if (body1 != null) {
                float body1angle = body1.transform.rotation.eulerAngles.x;
                DrawHalfEllipse(transform.position, transform.up, body1.transform.rotation * offset, body1angle, 0.25f * transform.localScale.x, 0.25f * transform.localScale.y, 32, color);
            }
            if (body2 != null) {
                float body2angle = body2.transform.rotation.eulerAngles.x;
                DrawHalfEllipse(transform.position, transform.right, body2.transform.rotation * offset2 * offset3, body2angle, 0.25f * transform.localScale.x, 0.25f * transform.localScale.y, 32, color);
            }
        }

        private static void DrawHalfEllipse(Vector3 pos, Vector3 forward, UnityEngine.Quaternion rotation, float ang, float radiusX, float radiusY, int segments, Color color, float duration = 0)
        {
            float angle = 0;// rotation;           
            UnityEngine.Quaternion rot = rotation;//UnityEngine.Quaternion.LookRotation(forward, up);
            Vector3 lastPoint = Vector3.zero;
            Vector3 thisPoint = Vector3.zero;

            for (int i = 0; i < segments + 1; i++)
            {
                thisPoint.x = Mathf.Sin(Mathf.Deg2Rad * angle) * radiusX;
                thisPoint.y = Mathf.Cos(Mathf.Deg2Rad * angle) * radiusY;

                if (i > 0)
                {
                    Debug.DrawLine(rot * lastPoint + pos, rot * thisPoint + pos, color, duration);
                }

                lastPoint = thisPoint;
                angle += 180f / segments;
            }
        }

        /// Get the number of (bilateral) constraints introduced by this joint.
        public override int GetDOC_c() { return 4; }

        /// Get the link coordinate system, expressed relative to Body2.
        public override ChCoordsys GetLinkRelativeCoords() { return m_frame2.GetCoord(); }

        /// Get the joint frame on Body1, expressed in Body1 coordinate system.
        public ChFrame<double> GetFrame1Rel() { return m_frame1; }
        /// Get the joint frame on Body2, expressed in Body2 coordinate system.
        public ChFrame<double> GetFrame2Rel() { return m_frame2; }

        /// Get the joint frame on Body1, expressed in absolute coordinate system.
        public ChFrame<double> GetFrame1Abs() { return ChFrame<double>.BitShiftRight(m_frame1, Body1); }
        /// Get the joint frame on Body2, expressed in absolute coordinate system.
        public ChFrame<double> GetFrame2Abs() { return ChFrame<double>.BitShiftRight(m_frame2, Body2); }

        /// Get the joint violation (residuals of the constraint equations)
        public ChMatrix GetC() { return m_C; }

        /// Initialize this joint by specifying the two bodies to be connected and a
        /// joint frame specified in the absolute frame. Two local joint frames, one
        /// on each body, are constructed so that they coincide with the specified
        /// global joint frame at the current configuration. The kinematics of the
        /// universal joint are obtained by imposing that the origins of these two
        /// frames are the same and that the X axis of the joint frame on body 1 and
        /// the Y axis of the joint frame on body 2 are perpendicular.
        public void Initialize(ChBodyFrame body1,  //< first body frame
                    ChBodyFrame body2,  //< second body frame
                    ChFrame<double> frame  //< joint frame (in absolute frame)
                    )
        {
            Body1 = body1;
            Body2 = body2;

            m_cnstr_x.SetVariables(Body1.Variables(), Body2.Variables());
            m_cnstr_y.SetVariables(Body1.Variables(), Body2.Variables());
            m_cnstr_z.SetVariables(Body1.Variables(), Body2.Variables());
            m_cnstr_dot.SetVariables(Body1.Variables(), Body2.Variables());

            ((ChFrame<double>)Body1).TransformParentToLocal(frame, m_frame1);
            ((ChFrame<double>)Body2).TransformParentToLocal(frame, m_frame2);

            m_u1_tilde.Set_X_matrix(m_frame1.GetA().Get_A_Xaxis());
            m_v2_tilde.Set_X_matrix(m_frame2.GetA().Get_A_Yaxis());

            m_C.SetElement(0, 0, 0.0);
            m_C.SetElement(1, 0, 0.0);
            m_C.SetElement(2, 0, 0.0);
            m_C.SetElement(3, 0, 0.0);
        }

        /// Initialize this joint by specifying the two bodies to be connected and the
        /// joint frames on each body. If local = true, it is assumed that these quantities
        /// are specified in the local body frames. Otherwise, it is assumed that they are
        /// specified in the absolute frame.
        public void Initialize(ChBodyFrame body1,  //< first body frame
                                ChBodyFrame body2,  //< second body frame
                    bool local,               //< true if data given in body local frames
                    ChFrame<double> frame1,  //< joint frame on body 1
                    ChFrame<double> frame2   //< joint frame on body 2
                    )
        {
            Body1 = body1;
            Body2 = body2;

            m_cnstr_x.SetVariables(Body1.Variables(), Body2.Variables());
            m_cnstr_y.SetVariables(Body1.Variables(), Body2.Variables());
            m_cnstr_z.SetVariables(Body1.Variables(), Body2.Variables());
            m_cnstr_dot.SetVariables(Body1.Variables(), Body2.Variables());

            ChFrame<double> frame1_abs;
            ChFrame<double> frame2_abs;

            if (local)
            {
                m_frame1 = frame1;
                m_frame2 = frame2;
                frame1_abs = ChFrame<double>.BitShiftRight(frame1, Body1);
                frame2_abs = ChFrame<double>.BitShiftRight(frame2, Body2);
            }
            else
            {
                ((ChFrame<double>)Body1).TransformParentToLocal(frame1, m_frame1);
                ((ChFrame<double>)Body2).TransformParentToLocal(frame2, m_frame2);
                frame1_abs = frame1;
                frame2_abs = frame2;
            }

            m_u1_tilde.Set_X_matrix(m_frame1.GetA().Get_A_Xaxis());
            m_v2_tilde.Set_X_matrix(m_frame2.GetA().Get_A_Yaxis());

            m_C.SetElement(0, 0, frame2_abs.coord.pos.x - frame1_abs.coord.pos.x);
            m_C.SetElement(1, 0, frame2_abs.coord.pos.y - frame1_abs.coord.pos.y);
            m_C.SetElement(2, 0, frame2_abs.coord.pos.z - frame1_abs.coord.pos.z);
            m_C.SetElement(3, 0, ChVector.Vdot(frame1_abs.GetA().Get_A_Xaxis(), frame2_abs.GetA().Get_A_Yaxis()));
        }

        //
        // UPDATING FUNCTIONS
        //

        /// Perform the update of this joint at the specified time: compute jacobians
        /// and constraint violations, cache in internal structures
        public override void update(double time, bool update_assets = true)
        {
            // Inherit time changes of parent class
            base.UpdateTime(time);

            // Express the joint frames in absolute frame
            ChFrame<double> frame1_abs = ChFrame<double>.BitShiftRight(m_frame1, Body1);
            ChFrame<double> frame2_abs = ChFrame<double>.BitShiftRight(m_frame2, Body2);

            // Calculate violations of the spherical constraints
            m_C.SetElement(0, 0, frame2_abs.coord.pos.x - frame1_abs.coord.pos.x);
            m_C.SetElement(1, 0, frame2_abs.coord.pos.y - frame1_abs.coord.pos.y);
            m_C.SetElement(2, 0, frame2_abs.coord.pos.z - frame1_abs.coord.pos.z);

            // Compute Jacobian of the spherical constraints
            //    pos2_abs - pos1_abs = 0
            {
                ChMatrix33<double> tilde1 = new ChMatrix33<double>();
                ChMatrix33<double> tilde2 = new ChMatrix33<double>();
                tilde1.Set_X_matrix(m_frame1.coord.pos);
                tilde2.Set_X_matrix(m_frame2.coord.pos);
                ChMatrix33<double> Phi_pi1 = Body1.GetA() * tilde1;
                ChMatrix33<double> Phi_pi2 = Body2.GetA() * tilde2;

                m_cnstr_x.Get_Cq_a().ElementN(0) = -1;
                m_cnstr_x.Get_Cq_b().ElementN(0) = +1;
                m_cnstr_x.Get_Cq_a().ElementN(1) = 0;
                m_cnstr_x.Get_Cq_b().ElementN(1) = 0;
                m_cnstr_x.Get_Cq_a().ElementN(2) = 0;
                m_cnstr_x.Get_Cq_b().ElementN(2) = 0;
                m_cnstr_x.Get_Cq_a().ElementN(3) = Phi_pi1[0, 0];
                m_cnstr_x.Get_Cq_b().ElementN(3) = -Phi_pi2[0, 0];
                m_cnstr_x.Get_Cq_a().ElementN(4) = Phi_pi1[0, 1];
                m_cnstr_x.Get_Cq_b().ElementN(4) = -Phi_pi2[0, 1];
                m_cnstr_x.Get_Cq_a().ElementN(5) = Phi_pi1[0, 2];
                m_cnstr_x.Get_Cq_b().ElementN(5) = -Phi_pi2[0, 2];

                m_cnstr_y.Get_Cq_a().ElementN(0) = 0;
                m_cnstr_y.Get_Cq_b().ElementN(0) = 0;
                m_cnstr_y.Get_Cq_a().ElementN(1) = -1;
                m_cnstr_y.Get_Cq_b().ElementN(1) = +1;
                m_cnstr_y.Get_Cq_a().ElementN(2) = 0;
                m_cnstr_y.Get_Cq_b().ElementN(2) = 0;
                m_cnstr_y.Get_Cq_a().ElementN(3) = Phi_pi1[1, 0];
                m_cnstr_y.Get_Cq_b().ElementN(3) = -Phi_pi2[1, 0];
                m_cnstr_y.Get_Cq_a().ElementN(4) = Phi_pi1[1, 1];
                m_cnstr_y.Get_Cq_b().ElementN(4) = -Phi_pi2[1, 1];
                m_cnstr_y.Get_Cq_a().ElementN(5) = Phi_pi1[1, 2];
                m_cnstr_y.Get_Cq_b().ElementN(5) = -Phi_pi2[1, 2];

                m_cnstr_z.Get_Cq_a().ElementN(0) = 0;
                m_cnstr_z.Get_Cq_b().ElementN(0) = 0;
                m_cnstr_z.Get_Cq_a().ElementN(1) = 0;
                m_cnstr_z.Get_Cq_b().ElementN(1) = 0;
                m_cnstr_z.Get_Cq_a().ElementN(2) = -1;
                m_cnstr_z.Get_Cq_b().ElementN(2) = +1;
                m_cnstr_z.Get_Cq_a().ElementN(3) = Phi_pi1[2, 0];
                m_cnstr_z.Get_Cq_b().ElementN(3) = -Phi_pi2[2, 0];
                m_cnstr_z.Get_Cq_a().ElementN(4) = Phi_pi1[2, 1];
                m_cnstr_z.Get_Cq_b().ElementN(4) = -Phi_pi2[2, 1];
                m_cnstr_z.Get_Cq_a().ElementN(5) = Phi_pi1[2, 2];
                m_cnstr_z.Get_Cq_b().ElementN(5) = -Phi_pi2[2, 2];
            }

            // Calculate violation of the dot constraint
            ChVector u1 = frame1_abs.GetA().Get_A_Xaxis();
            ChVector v2 = frame2_abs.GetA().Get_A_Yaxis();

            m_C.SetElement(3, 0, ChVector.Vdot(u1, v2));

            // Compute Jacobian of the dot constraint
            //    dot(u1_abs, v2_abs) = 0
            {
                ChMatrix33<double> mat1 = Body1.GetA() * m_u1_tilde;
                ChMatrix33<double> mat2 = Body2.GetA() * m_v2_tilde;
                ChVector Phi_pi1 = mat1.MatrT_x_Vect(v2);
                ChVector Phi_pi2 = mat2.MatrT_x_Vect(u1);

                m_cnstr_dot.Get_Cq_a().ElementN(0) = 0;
                m_cnstr_dot.Get_Cq_a().ElementN(1) = 0;
                m_cnstr_dot.Get_Cq_a().ElementN(2) = 0;
                m_cnstr_dot.Get_Cq_a().ElementN(3) = -Phi_pi1.x;
                m_cnstr_dot.Get_Cq_a().ElementN(4) = -Phi_pi1.y;
                m_cnstr_dot.Get_Cq_a().ElementN(5) = -Phi_pi1.z;

                m_cnstr_dot.Get_Cq_b().ElementN(0) = 0;
                m_cnstr_dot.Get_Cq_b().ElementN(1) = 0;
                m_cnstr_dot.Get_Cq_b().ElementN(2) = 0;
                m_cnstr_dot.Get_Cq_b().ElementN(3) = -Phi_pi2.x;
                m_cnstr_dot.Get_Cq_b().ElementN(4) = -Phi_pi2.y;
                m_cnstr_dot.Get_Cq_b().ElementN(5) = -Phi_pi2.z;
            }
        }

        //
        // STATE FUNCTIONS
        //

        // (override/implement interfaces for global state vectors, see ChPhysicsItem for comments.)
        public override void IntStateGatherReactions(int off_L, ref ChVectorDynamic<double> L)
        {
            if (!this.IsActive())
                return;

            L[off_L + 0] = m_multipliers[0];
            L[off_L + 1] = m_multipliers[1];
            L[off_L + 2] = m_multipliers[2];
            L[off_L + 3] = m_multipliers[3];
        }
        public override void IntStateScatterReactions(int off_L, ChVectorDynamic<double> L)
        {
            if (!this.IsActive())
                return;

            m_multipliers[0] = L[off_L + 0];
            m_multipliers[1] = L[off_L + 1];
            m_multipliers[2] = L[off_L + 2];
            m_multipliers[3] = L[off_L + 3];

            // Also compute 'intuitive' reactions:

            // Extract the Lagrange multipliers for the 3 spherical constraints and for
            // the dot constraint.
            ChVector lam_sph = new ChVector(m_multipliers[0], m_multipliers[1], m_multipliers[2]);
            double lam_dot = m_multipliers[3];

            // Calculate the reaction force and torque acting on the 2nd body at the joint
            // location, expressed in the joint reference frame.  Taking into account the
            // sign with which Lagrange multipliers show up in the EOM in Chrono, we get:
            //   F = C^T * A_2^T * Phi_r2^T * lam
            //   T = C^T * ( Phi_pi2^T - tilde(s2') * A_2^T * Phi_r2^T ) * lam
            // For the universal joint, after some manipulations, we have:
            //   F = C^T * A_2^T * lam_sph
            //   T = C^T * tilde(v2') *A_2^T * u1 * lam_dot
            //     = -C^T * [A_2 * tilde(v2')]^T * u1 * lam_dot

            // Reaction force
            ChVector F2 = Body2.GetA().MatrT_x_Vect(lam_sph);
            react_force = m_frame2.GetA().MatrT_x_Vect(F2);

            // Reaction torque
            ChVector u1 = Body1.TransformDirectionLocalToParent(m_frame1.GetA().Get_A_Xaxis());
            ChMatrix33<double> mat2 = Body2.GetA() * m_v2_tilde;
            ChVector T2 = mat2.MatrT_x_Vect(u1) * lam_dot;
            react_torque = -m_frame2.GetA().MatrT_x_Vect(T2);
        }
        public override void IntLoadResidual_CqL(int off_L,
                                         ref ChVectorDynamic<double> R,
                                         ChVectorDynamic<double> L,
                                         double c)
        {
            if (!IsActive())
                return;

            m_cnstr_x.MultiplyTandAdd(R, L[off_L + 0] * c);
            m_cnstr_y.MultiplyTandAdd(R, L[off_L + 1] * c);
            m_cnstr_z.MultiplyTandAdd(R, L[off_L + 2] * c);
            m_cnstr_dot.MultiplyTandAdd(R, L[off_L + 3] * c);
        }
        public override void IntLoadConstraint_C(int off_L,
                                         ref ChVectorDynamic<double> Qc,
                                         double c,
                                         bool do_clamp,
                                         double recovery_clamp)
        {
            if (!IsActive())
                return;

            double cnstr_x_violation = c * m_C.GetElement(0, 0);
            double cnstr_y_violation = c * m_C.GetElement(1, 0);
            double cnstr_z_violation = c * m_C.GetElement(2, 0);
            double cnstr_dot_violation = c * m_C.GetElement(3, 0);

            if (do_clamp)
            {
                cnstr_x_violation = ChMaths.ChMin(ChMaths.ChMax(cnstr_x_violation, -recovery_clamp), recovery_clamp);
                cnstr_y_violation = ChMaths.ChMin(ChMaths.ChMax(cnstr_y_violation, -recovery_clamp), recovery_clamp);
                cnstr_z_violation = ChMaths.ChMin(ChMaths.ChMax(cnstr_z_violation, -recovery_clamp), recovery_clamp);
                cnstr_dot_violation = ChMaths.ChMin(ChMaths.ChMax(cnstr_dot_violation, -recovery_clamp), recovery_clamp);
            }
            Qc[off_L + 0] += cnstr_x_violation;
            Qc[off_L + 1] += cnstr_y_violation;
            Qc[off_L + 2] += cnstr_z_violation;
            Qc[off_L + 3] += cnstr_dot_violation;
        }
        public override void IntToDescriptor(int off_v,
                                     ChStateDelta v,
                                     ChVectorDynamic<double> R,
                                     int off_L,
                                     ChVectorDynamic<double> L,
                                     ChVectorDynamic<double> Qc)
        {
            if (!IsActive())
                return;

            m_cnstr_x.Set_l_i(L[off_L + 0]);
            m_cnstr_y.Set_l_i(L[off_L + 1]);
            m_cnstr_z.Set_l_i(L[off_L + 2]);
            m_cnstr_dot.Set_l_i(L[off_L + 3]);

            m_cnstr_x.Set_b_i(Qc[off_L + 0]);
            m_cnstr_y.Set_b_i(Qc[off_L + 1]);
            m_cnstr_z.Set_b_i(Qc[off_L + 2]);
            m_cnstr_dot.Set_b_i(Qc[off_L + 3]);
        }
        public override void IntFromDescriptor(int off_v,
                                       ref ChStateDelta v,
                                       int off_L,
                                       ref ChVectorDynamic<double> L)
        {
            if (!IsActive())
                return;

            L[off_L + 0] = m_cnstr_x.Get_l_i();
            L[off_L + 1] = m_cnstr_y.Get_l_i();
            L[off_L + 2] = m_cnstr_z.Get_l_i();
            L[off_L + 3] = m_cnstr_dot.Get_l_i();
        }

        //
        // SOLVER INTERFACE
        //

        public override void InjectConstraints(ref ChSystemDescriptor descriptor)
        {
            if (!IsActive())
                return;

            descriptor.InsertConstraint(m_cnstr_x);
            descriptor.InsertConstraint(m_cnstr_y);
            descriptor.InsertConstraint(m_cnstr_z);
            descriptor.InsertConstraint(m_cnstr_dot);
        }
        public override void ConstraintsBiReset()
        {
            m_cnstr_x.Set_b_i(0.0);
            m_cnstr_y.Set_b_i(0.0);
            m_cnstr_z.Set_b_i(0.0);
            m_cnstr_dot.Set_b_i(0.0);
        }
        public override void ConstraintsBiLoad_C(double factor = 1, double recovery_clamp = 0.1, bool do_clamp = false)
        {
            if (!IsActive())
                return;

            double cnstr_x_violation = factor * m_C.GetElement(0, 0);
            double cnstr_y_violation = factor * m_C.GetElement(1, 0);
            double cnstr_z_violation = factor * m_C.GetElement(2, 0);
            double cnstr_dot_violation = factor * m_C.GetElement(3, 0);

            if (do_clamp)
            {
                cnstr_x_violation = ChMaths.ChMin(ChMaths.ChMax(cnstr_x_violation, -recovery_clamp), recovery_clamp);
                cnstr_y_violation = ChMaths.ChMin(ChMaths.ChMax(cnstr_y_violation, -recovery_clamp), recovery_clamp);
                cnstr_z_violation = ChMaths.ChMin(ChMaths.ChMax(cnstr_z_violation, -recovery_clamp), recovery_clamp);
                cnstr_dot_violation = ChMaths.ChMin(ChMaths.ChMax(cnstr_dot_violation, -recovery_clamp), recovery_clamp);
            }

            m_cnstr_x.Set_b_i(m_cnstr_x.Get_b_i() + cnstr_x_violation);
            m_cnstr_y.Set_b_i(m_cnstr_y.Get_b_i() + cnstr_y_violation);
            m_cnstr_z.Set_b_i(m_cnstr_z.Get_b_i() + cnstr_z_violation);
            m_cnstr_dot.Set_b_i(m_cnstr_dot.Get_b_i() + cnstr_dot_violation);
        }
        public override void ConstraintsLoadJacobians()
        {
            // Nothing to do here. Jacobians were loaded in Update().
        }
        public override void ConstraintsFetch_react(double factor = 1)
        {
            // Extract the Lagrange multipliers for the 3 spherical constraints and for
            // the dot constraint.
            ChVector lam_sph = new ChVector(m_cnstr_x.Get_l_i(), m_cnstr_y.Get_l_i(), m_cnstr_z.Get_l_i());
            double lam_dot = m_cnstr_dot.Get_l_i();

            // Note that the Lagrange multipliers must be multiplied by 'factor' to
            // convert from reaction impulses to reaction forces.
            lam_sph *= factor;
            lam_dot *= factor;

            // Calculate the reaction force and torque acting on the 2nd body at the joint
            // location, expressed in the joint reference frame.  Taking into account the
            // sign with which Lagrange multipliers show up in the EOM in Chrono, we get:
            //   F = C^T * A_2^T * Phi_r2^T * lam
            //   T = C^T * ( Phi_pi2^T - tilde(s2') * A_2^T * Phi_r2^T ) * lam
            // For the universal joint, after some manipulations, we have:
            //   F = C^T * A_2^T * lam_sph
            //   T = C^T * tilde(v2') *A_2^T * u1 * lam_dot
            //     = -C^T * [A_2 * tilde(v2')]^T * u1 * lam_dot

            // Reaction force
            ChVector F2 = Body2.GetA().MatrT_x_Vect(lam_sph);
            react_force = m_frame2.GetA().MatrT_x_Vect(F2);

            // Reaction torque
            ChVector u1 = Body1.TransformDirectionLocalToParent(m_frame1.GetA().Get_A_Xaxis());
            ChMatrix33<double> mat2 = Body2.GetA() * m_v2_tilde;
            ChVector T2 = mat2.MatrT_x_Vect(u1) * lam_dot;
            react_torque = -m_frame2.GetA().MatrT_x_Vect(T2);
        }


        // Joint frames (in body local frames)
        private ChFrame<double> m_frame1 = new ChFrame<double>();  //< joint frame on body 1
        private ChFrame<double> m_frame2 = new ChFrame<double>();  //< joint frame on body 2

        // Cached matrices
        private ChMatrix33<double> m_u1_tilde = new ChMatrix33<double>();
        private ChMatrix33<double> m_v2_tilde = new ChMatrix33<double>();

        // The constraint objects
        private ChConstraintTwoBodies m_cnstr_x = new ChConstraintTwoBodies();    ///< constraint: x1_abs - x2_abs = 0
        private ChConstraintTwoBodies m_cnstr_y = new ChConstraintTwoBodies();    ///< constraint: y1_abs - y2_abs = 0
        private ChConstraintTwoBodies m_cnstr_z = new ChConstraintTwoBodies();    ///< constraint: z1_abs - z2_abs = 0
        private ChConstraintTwoBodies m_cnstr_dot = new ChConstraintTwoBodies();  ///< constraint: dot(u1_abs, v2_abs) = 0

        private ChMatrix m_C = new ChMatrix();                //< current constraint violations

        private double[] m_multipliers = new double[4];  //< Lagrange multipliers

    };  

}
