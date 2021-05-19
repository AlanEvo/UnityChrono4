using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono
{

    /// A motor that enforces the rotation angle r(t) between two frames on two bodies, using a rheonomic constraint.
    /// The r(t) angle of frame A rotating on Z axis of frame B, is imposed via an exact function of time f(t),
    /// and an optional angle offset:
    ///    r(t) = f(t) + offset
    /// Note: no compliance is allowed, so if the actuator hits an undeformable obstacle it hits a pathological
    /// situation and the solver result can be unstable/unpredictable.
    /// Think at it as a servo drive with "infinitely stiff" control.
    /// This type of motor is very easy to use, stable and efficient, and should be used if the 'infinitely stiff'
    /// control assumption  is a good approximation of what you simulate (e.g., very good and reactive controllers).
    /// By default it is initialized with linear ramp: df/dt= 1.
    /// Use SetAngleFunction() to change to other motion functions.

    public class ChLinkMotorRotationAngle : ChLinkMotorRotation
    {
        public ChBody body1;
        public ChBody body2;

        public ChLinkMotorRotationAngle()
        {
            // default motion function: ramp with initial value y(0) = 0 and slope dy/dt = 1
            m_func = new ChFunction_Ramp(0.0, 1.0);

            rot_offset = 0;
        }
        public ChLinkMotorRotationAngle(ChLinkMotorRotationAngle other)
        {
            rot_offset = other.rot_offset;
        }

        /// "Virtual" copy constructor (covariant return type).
        public override ChObj Clone() { return new ChLinkMotorRotationAngle(this); }

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
                SetAngleFunction(fun_component);
            }

            ChSystem msystem = FindObjectOfType<ChSystem>();
            msystem.AddLink(this);
        }

        protected virtual void OnDrawGizmos()
        {

            Gizmos.color = new Color(0, 255, 0);
            Gizmos.DrawLine(transform.position, transform.position + (transform.forward * 0.2f));

        }

            /// Set the rotation angle function of time a(t).
            /// This function should be C0 continuous and, to prevent acceleration spikes,
            /// it should ideally be C1 continuous.
            public void SetAngleFunction(ChFunction function) { SetMotorFunction(function); }

        /// Get the rotation angle function f(t).
        public ChFunction GetAngleFunction() { return GetMotorFunction();
}

        /// Get initial angle offset for f(t)=0, in [rad]. Rotation on Z of the two axes
        /// will be r(t) = f(t) + offset.
        /// By default, offset = 0
        public void SetAngleOffset(double mo) { rot_offset = mo; }

        /// Get initial offset for f(t)=0, in [rad]
        public double GetAngleOffset() { return rot_offset; }

        /// Get the current actuator reaction torque [Nm]
        public override double GetMotorTorque() { return -this.react_torque.z; }

        public override void update(double mytime, bool update_assets) {
            // Inherit parent class:
            base.update(mytime, update_assets);

            // Override the rotational jacobian [Cq] and the rotational residual C,
            // by assuming an additional hidden frame that rotates about frame2:

            if (this.Body1 != null && this.Body2 != null)
            {
                ChFrame<double> aframe1 = this.frame1.BitShiftRight(this.Body1);
                ChFrame<double> aframe2 = this.frame2.BitShiftRight(this.Body2);

                ChFrame<double> aframe12 = new ChFrame<double>();// ChFrame<double>.FNULL;
                aframe2.TransformParentToLocal(aframe1, aframe12);

                ChFrame<double> aframe2rotating = new ChFrame<double>();// ChFrame<double>.FNULL;

                double aux_rotation;

                aux_rotation = m_func.Get_y(mytime) + rot_offset;

                aframe2rotating.SetRot(aframe2.GetRot() * ChQuaternion.Q_from_AngAxis2(aux_rotation, ChVector.VECT_Z));

                ChFrame<double> aframe12rotating = new ChFrame<double>();
               // aframe2rotating.TransformParentToLocal(aframe1, aframe12rotating);

                ChMatrix33<double> Jw1 = new ChMatrix33<double>(0), Jw2 = new ChMatrix33<double>(0);
                ChMatrix33<double> mtempM = new ChMatrix33<double>(0), mtempQ = new ChMatrix33<double>(0);

                ChMatrix33<double> abs_plane_rotating = aframe2rotating.GetA();

                Jw1.nm.matrix.MatrTMultiply(abs_plane_rotating.nm.matrix, Body1.GetA().nm.matrix);
                Jw2.nm.matrix.MatrTMultiply(abs_plane_rotating.nm.matrix, Body2.GetA().nm.matrix);

                Jw2.nm.matrix.MatrNeg();

                // Premultiply by Jw1 and Jw2 by  0.5*[Fp(q_resid)]' to get residual as imaginary part of a quaternion.
              /*  mtempM.Set_X_matrix((aframe12rotating.GetRot().GetVector()) * 0.5);
                mtempM.nm.matrix[0, 0] = 0.5 * aframe12rotating.GetRot().e0;
                mtempM.nm.matrix[1, 1] = 0.5 * aframe12rotating.GetRot().e0;
                mtempM.nm.matrix[2, 2] = 0.5 * aframe12rotating.GetRot().e0;
                mtempQ.nm.matrix.MatrTMultiply(mtempM.nm.matrix, Jw1.nm.matrix);
                Jw1 = mtempQ;
                mtempQ.nm.matrix.MatrTMultiply(mtempM.nm.matrix, Jw2.nm.matrix);
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
        public override void IntLoadConstraint_Ct(int off_L, ref ChVectorDynamic<double> Qc, double c) {
            double mCt = -0.5 * m_func.Get_y_dx(this.GetChTime());
            int ncrz = mask.nconstr - 1;
            if (mask.Constr_N(ncrz).IsActive())
            {
                Qc.matrix[off_L + ncrz] += c * mCt;
            }
        }

        //
        // SOLVER INTERFACE (OLD)
        //
        public override void ConstraintsBiLoad_Ct(double factor = 1) {
            if (!this.IsActive())
                return;

            double mCt = -0.5 * m_func.Get_y_dx(this.GetChTime());
            int ncrz = mask.nconstr - 1;
            if (mask.Constr_N(ncrz).IsActive())
            {
                mask.Constr_N(ncrz).Set_b_i(mask.Constr_N(ncrz).Get_b_i() + factor * mCt);
            }
        }

        private double rot_offset;

    }

}
