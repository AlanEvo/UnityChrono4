using System.Collections;
using System.Collections.Generic;
using UnityEngine;


namespace chrono
{

    /// A motor that enforces the angular speed w(t) between two frames on two bodies, using a rheonomic constraint.
    /// Note: no compliance is allowed, so if the actuator hits an undeformable obstacle it hits a pathological
    /// situation and the solver result can be unstable/unpredictable.
    /// Think at it as a servo drive with "infinitely stiff" control.
    /// This type of motor is very easy to use, stable and efficient, and should be used if the 'infinitely stiff'
    /// control assumption is a good approximation of what you simulate (e.g., very good and reactive controllers).
    /// By default it is initialized with constant angular speed: df/dt = 1.
    /// Use SetSpeedFunction() to change to other speed functions.

    public class ChLinkMotorRotationSpeed : ChLinkMotorRotation
    {
        public ChBody body1;
        public ChBody body2;

        public ChLinkMotorRotationSpeed()
        {
            variable.GetMass()[0, 0] = 1.0;
            variable.GetInvMass()[0, 0] = 1.0;

            m_func = new ChFunction_Const(1.0);

            rot_offset = 0;

            aux_dt = 0;  // used for integrating speed, = rot
            aux_dtdt = 0;

            avoid_angle_drift = true;
        }
        public ChLinkMotorRotationSpeed(ChLinkMotorRotationSpeed other)
        {
            variable = other.variable;
            rot_offset = other.rot_offset;
            aux_dt = other.aux_dt;
            aux_dtdt = other.aux_dtdt;
            avoid_angle_drift = other.avoid_angle_drift;
        }

        /// "Virtual" copy constructor (covariant return type).
        public override ChObj Clone() { return new ChLinkMotorRotationSpeed(this); }

        // NOTE this class link is set to Start because it needs to be created after the ChBody/BodyFrame, otherwise it's not
        // initialized by the time it gets to here.
        public void Start()
        {
            ChFrame<double> frame = new ChFrameMoving<double>(Utils.ToChrono(transform.position), Utils.ToChrono(transform.rotation));
            Initialize(body1.BodyFrame, body2.BodyFrame, frame);

            // Get a handle to the associated function component and set the motor's function
            var fun_component = this.GetComponent<ChFunction>();
            if (fun_component != null)
            {
                SetSpeedFunction(fun_component);
            }

            ChSystem msystem = FindObjectOfType<ChSystem>();
            msystem.AddLink(this);
        }

        private static void DrawEllipse(Vector3 pos, Vector3 forward, Vector3 up, float radiusX, float radiusY, int segments, Color color, float duration = 0)
        {
            float angle = 0f;
            UnityEngine.Quaternion rot = UnityEngine.Quaternion.LookRotation(forward, up);
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
                angle += 360f / segments;
            }
        }

        protected virtual void OnDrawGizmos()
        {
            if (Application.isPlaying)
            {
                Color color = Color.red;
                DrawEllipse(new Vector3((float)this.GetLinkAbsoluteCoords().pos.x, (float)this.GetLinkAbsoluteCoords().pos.y, (float)this.GetLinkAbsoluteCoords().pos.z), transform.forward, transform.up, 0.25f * transform.localScale.x, 0.25f * transform.localScale.y, 32, color);
            }
            else
            {
                Color color = Color.red;
                DrawEllipse(transform.position, transform.forward, transform.up, 0.25f * transform.localScale.x, 0.25f * transform.localScale.y, 32, color);
            }
        }

        /// Set the angular speed function of time w(t).
        /// To prevent acceleration pikes, this function should be C0 continuous.
        public void SetSpeedFunction(ChFunction function) { SetMotorFunction(function); }

        /// Get the angular speed function w(t).
        public ChFunction GetSpeedFunction()
        {
            return GetMotorFunction();
        }

        /// Get initial offset, in [rad]. By default = 0.
        public void SetAngleOffset(double mo) { rot_offset = mo; }

        /// Get initial offset, in [rad].
        public double GetAngleOffset() { return rot_offset; }

        /// Set if the constraint must avoid angular drift. If true, it
        /// means that the constraint is satisfied also at the rotation level,
        /// by integrating the velocity in a separate auxiliary state. Default, true.
        public void SetAvoidAngleDrift(bool mb) { this.avoid_angle_drift = mb; }

        /// Set if the constraint is in "avoid angle drift" mode.
        public bool GetAvoidAngleDrift() { return this.avoid_angle_drift; }

        /// Get the current actuator reaction torque [Nm]
        public override double GetMotorTorque() { return -this.react_torque.z; }

        public override void update(double mytime, bool update_assets)
        {
            // Inherit parent class:
            base.update(mytime, update_assets);

            // Override the rotational jacobian [Cq] and the rotational residual C,
            // by assuming an additional hidden frame that rotates about frame2:

            if (this.Body1 != null && this.Body2 != null)
            {
                ChFrame<double> aframe1 = ChFrame<double>.BitShiftRight(this.frame1 , (this.Body1));
                ChFrame<double> aframe2 = ChFrame<double>.BitShiftRight(this.frame2 , (this.Body2));

                ChFrame<double> aframe12 = new ChFrame<double>();
                aframe2.TransformParentToLocal(aframe1, aframe12);

                ChFrame<double> aframe2rotating = new ChFrame<double>();

                double aux_rotation;

                if (this.avoid_angle_drift)
                {
                    aux_rotation = this.aux_dt + this.rot_offset;
                }
                else
                {
                    // to have it aligned to current rot, to allow C=0.
                    aux_rotation = aframe12.GetRot().Q_to_Rotv().z;
                }

                aframe2rotating.SetRot(aframe2.GetRot() * ChQuaternion.Q_from_AngAxis2(aux_rotation, ChVector.VECT_Z));

                // TODO this needs to be addressed, with it it causes rotation problems, seems to work fine without the TransformParentToLocal?
                ChFrame<double> aframe12rotating = new ChFrame<double>();
                                                                         // aframe2rotating.TransformParentToLocal(aframe1, aframe12rotating);

                ChMatrix33<double> Jw1 = new ChMatrix33<double>();
                ChMatrix33<double> Jw2 = new ChMatrix33<double>();
                ChMatrix33<double> mtempM = new ChMatrix33<double>();
                ChMatrix33<double> mtempQ = new ChMatrix33<double>();

                ChMatrix33<double> abs_plane_rotating = aframe2rotating.GetA();

                Jw1.nm.matrix.MatrTMultiply(abs_plane_rotating.nm.matrix, Body1.GetA().nm.matrix);
                Jw2.nm.matrix.MatrTMultiply(abs_plane_rotating.nm.matrix, Body2.GetA().nm.matrix);

                Jw2.nm.matrix.MatrNeg();

                // TODO this also needs to be addressed, with it it causes rotation problems/

                // Premultiply by Jw1 and Jw2 by  0.5*[Fp(q_resid)]' to get residual as imaginary part of a quaternion.
                /* mtempM.Set_X_matrix((aframe12rotating.GetRot().GetVector()) * 0.5);
                 mtempM[0, 0] = 0.5 * aframe12rotating.GetRot().e0;
                 mtempM[1, 1] = 0.5 * aframe12rotating.GetRot().e0;
                 mtempM[2, 2] = 0.5 * aframe12rotating.GetRot().e0;
                 mtempQ.MatrTMultiply(mtempM, Jw1);
                 Jw1 = mtempQ;
                 mtempQ.MatrTMultiply(mtempM, Jw2);
                 Jw2 = mtempQ;*/

                int nc = 0;

                if (c_x)
                {
                    nc++;
                }
                if (c_y)
                {
                    nc++;
                }
                if (c_z)
                {
                    nc++;
                }
                if (c_rx)
                {
                    this.C.matrix.ElementN(nc) = aframe12rotating.GetRot().e1;
                    this.mask.Constr_N(nc).Get_Cq_a().FillElem(0);
                    this.mask.Constr_N(nc).Get_Cq_b().FillElem(0);
                    this.mask.Constr_N(nc).Get_Cq_a().PasteClippedMatrix(Jw1.nm.matrix, 0, 0, 1, 3, 0, 3);
                    this.mask.Constr_N(nc).Get_Cq_b().PasteClippedMatrix(Jw2.nm.matrix, 0, 0, 1, 3, 0, 3);
                    nc++;
                }
                if (c_ry)
                {
                    this.C.matrix.ElementN(nc) = aframe12rotating.GetRot().e2;
                    this.mask.Constr_N(nc).Get_Cq_a().FillElem(0);
                    this.mask.Constr_N(nc).Get_Cq_b().FillElem(0);
                    this.mask.Constr_N(nc).Get_Cq_a().PasteClippedMatrix(Jw1.nm.matrix, 1, 0, 1, 3, 0, 3);
                    this.mask.Constr_N(nc).Get_Cq_b().PasteClippedMatrix(Jw2.nm.matrix, 1, 0, 1, 3, 0, 3);
                    nc++;
                }
                if (c_rz)
                {
                    this.C.matrix.ElementN(nc) = aframe12rotating.GetRot().e3;
                    this.mask.Constr_N(nc).Get_Cq_a().FillElem(0);
                    this.mask.Constr_N(nc).Get_Cq_b().FillElem(0);
                    this.mask.Constr_N(nc).Get_Cq_a().PasteClippedMatrix(Jw1.nm.matrix, 2, 0, 1, 3, 0, 3);
                    this.mask.Constr_N(nc).Get_Cq_b().PasteClippedMatrix(Jw2.nm.matrix, 2, 0, 1, 3, 0, 3);
                    nc++;
                }
            }
        }

        //
        // STATE FUNCTIONS
        //

        public override int GetDOF() { return 1; }

        public override void IntStateGather(int off_x,
                                    ref ChState x,
                                    int off_v,
                                    ref ChStateDelta v,
                                    ref double T)
        {
            x.matrix[off_x] = 0;  // aux;
            v.matrix[off_v] = aux_dt;
            T = GetChTime();
        }
        public override void IntStateScatter(int off_x,
                                     ChState x,
                                     int off_v,
                                     ChStateDelta v,
                                     double T)
        {
            // aux = x(off_x);
            aux_dt = v.matrix[off_v];
        }
        public override void IntStateGatherAcceleration(int off_a, ref ChStateDelta a)
        {
            a.matrix[off_a] = aux_dtdt;
        }
        public override void IntStateScatterAcceleration(int off_a, ChStateDelta a)
        {
            aux_dtdt = a.matrix[off_a];
        }
        public override void IntLoadResidual_F(int off, ref ChVectorDynamic<double> R, double c)
        {
            double imposed_speed = m_func.Get_y(this.GetChTime());
            R.matrix[off] += imposed_speed * c;
        }
        public override void IntLoadResidual_Mv(int off,
                                        ref ChVectorDynamic<double> R,
                                        ChVectorDynamic<double> w,
                                        double c)
        {
            R.matrix[off] += c * 1.0 * w.matrix[off];
        }
        public override void IntToDescriptor(int off_v,
                                     ChStateDelta v,
                                     ChVectorDynamic<double> R,
                                     int off_L,
                                     ChVectorDynamic<double> L,
                                     ChVectorDynamic<double> Qc)
        {
            // inherit parent
            base.IntToDescriptor(off_v, v, R, off_L, L, Qc);

            this.variable.Get_qb().matrix[0, 0] = v.matrix[off_v];
            this.variable.Get_fb().matrix[0, 0] = R.matrix[off_v];
        }
        public override void IntFromDescriptor(int off_v,
                                       ref ChStateDelta v,
                                       int off_L,
                                       ref ChVectorDynamic<double> L)
        {
            // inherit parent
            base.IntFromDescriptor(off_v, ref v, off_L, ref L);

            v.matrix[off_v] = this.variable.Get_qb().matrix[0, 0];
        }

        public override void IntLoadConstraint_Ct(int off_L, ref ChVectorDynamic<double> Qc, double c)
        {
            double mCt = -0.5 * m_func.Get_y(this.GetChTime());

            int ncrz = mask.nconstr - 1;
            if (mask.Constr_N(ncrz).IsActive())
            {
                Qc.matrix[off_L + ncrz] += c * mCt;
            }
        }

        //
        // SOLVER INTERFACE (OLD)
        //

        public override void VariablesFbReset()
        {
            variable.Get_fb().matrix.FillElem(0.0);
        }
        public override void VariablesFbLoadForces(double factor = 1)
        {
            double imposed_speed = m_func.Get_y(this.GetChTime());
            variable.Get_fb().matrix.ElementN(0) += imposed_speed * factor;
        }
        public override void VariablesQbLoadSpeed()
        {
            // set current speed in 'qb', it can be used by the solver when working in incremental mode
            variable.Get_qb().matrix.SetElement(0, 0, aux_dt);
        }
        public override void VariablesFbIncrementMq()
        {
            variable.Compute_inc_Mb_v(ref variable.Get_fb().matrix, variable.Get_qb().matrix);
        }
        public override void VariablesQbSetSpeed(double step = 0)
        {
            double old_dt = aux_dt;

            // from 'qb' vector, sets body speed, and updates auxiliary data
            aux_dt = variable.Get_qb().matrix.GetElement(0, 0);

            // Compute accel. by BDF (approximate by differentiation); not needed
        }
        public override void InjectVariables(ref ChSystemDescriptor mdescriptor)
        {
            variable.SetDisabled(!IsActive());

            mdescriptor.InsertVariables(variable);
        }

        public override void ConstraintsBiLoad_Ct(double factor = 1)
        {
            if (!this.IsActive())
                return;

            double mCt = -0.5 * m_func.Get_y(this.GetChTime());
            int ncrz = mask.nconstr - 1;
            if (mask.Constr_N(ncrz).IsActive())
            {
                mask.Constr_N(ncrz).Set_b_i(mask.Constr_N(ncrz).Get_b_i() + factor * mCt);
            }
        }


        private double rot_offset;

        private ChVariablesGeneric variable = new ChVariablesGeneric();

        private double aux_dt;  // used for integrating speed, = angle
        private double aux_dtdt;

        private bool avoid_angle_drift;
    }

  
}