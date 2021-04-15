using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono
{

    /// A motor that applies a torque between two frames on two bodies.
    /// Differently from the ChLinkMotorRotationAngle and ChLinkMotorRotationSpeed,
    /// this does not enforce precise motion via constraint.
    /// Example of application:
    /// - mimic a PID controlled system with some feedback (user-defined)
    /// - force that is updated by a cosimulation
    /// - force from a human-in-the-loop setpoint
    /// Use SetTorqueFunction() to change to other torque function (default zero torque).

    public class ChLinkMotorRotationTorque : ChLinkMotorRotation
    {
        public ChBody body1;
        public ChBody body2;

        public ChLinkMotorRotationTorque()
        {
            this.c_rz = false;
            SetupLinkMask();

            m_func = new ChFunction_Const(0.0);
        }
        public ChLinkMotorRotationTorque(ChLinkMotorRotationTorque other)
            : base(other){ }

        /// "Virtual" copy constructor (covariant return type).
        public override ChObj Clone() { return new ChLinkMotorRotationTorque(this); }

        // NOTE this class link is set to Start because it needs to be created after the ChBody/BodyFrame, otherwise it's not
        // initialized by the time it gets to here.
        public void Start()
        {
            ChFrame<double> frame = new ChFrameMoving<double>(Utils.ToChrono(transform.position), Utils.ToChrono(transform.rotation));
            Initialize(body1.BodyFrame, body2.BodyFrame, frame);

            ChSystem msystem = FindObjectOfType<ChSystem>();
            msystem.AddLink(this);

            // Get a handle to the associated function component and set the motor's function
            var fun_component = this.GetComponent<ChFunction>();
            if (fun_component != null)
            {
                SetTorqueFunction(fun_component);
            }


        }



        /// Set the torque function of time T(t).
        public void SetTorqueFunction(ChFunction function) { SetMotorFunction(function); }

        /// Gets the torque function F(t).
        public ChFunction GetTorqueFunction() { return GetMotorFunction();
}

        /// Get the current actuator reaction torque.
        public override double GetMotorTorque()  { return m_func.Get_y(GetChTime()); }

        public override void update(double mytime, bool update_assets) {
            base.update(mytime, update_assets);
        }

        //
        // STATE FUNCTIONS
        //
        public override void IntLoadResidual_F(int off, ref ChVectorDynamic<double> R, double c) {
            // compute instant torque
            double mT = m_func.Get_y(this.GetChTime());

            ChFrame<double> aframe1 = ChFrame<double>.BitShiftRight(this.frame1 , (this.Body1));
            ChFrame<double> aframe2 = ChFrame<double>.BitShiftRight(this.frame2 , (this.Body2));
            ChVector m_abs_torque = aframe2.GetA().Matr_x_Vect(new ChVector(0, 0, mT));

            if (Body2.Variables().IsActive())
            {
                R.PasteSumVector(Body2.TransformDirectionParentToLocal(m_abs_torque) * -c, Body2.Variables().GetOffset() + 3,
                                 0);
            }

            if (Body1.Variables().IsActive())
            {
                R.PasteSumVector(Body1.TransformDirectionParentToLocal(m_abs_torque) * c, Body1.Variables().GetOffset() + 3,
                                 0);
            }
        }

        //
        // SOLVER INTERFACE (OLD)
        //
        public override void ConstraintsFbLoadForces(double factor = 1) {
            // compute instant torque
            double mT = m_func.Get_y(this.GetChTime());

            ChFrame<double> aframe1 = ChFrame<double>.BitShiftRight(this.frame1 , (this.Body1));
            ChFrame<double> aframe2 = ChFrame<double>.BitShiftRight(this.frame2 , (this.Body2));
            ChVector m_abs_torque = aframe2.GetA().Matr_x_Vect(new ChVector(0, 0, mT));

            Body2.Variables().Get_fb().PasteSumVector(Body2.TransformDirectionParentToLocal(m_abs_torque) * -factor, 3, 0);

            Body1.Variables().Get_fb().PasteSumVector(Body1.TransformDirectionParentToLocal(m_abs_torque) * factor, 3, 0);
        }
    }

}
