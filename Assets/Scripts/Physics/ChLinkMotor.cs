using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono
{

    /// Base class for all "motor" constraints between
    /// two frames on two bodies.
    /// Look for children classes for specialized behaviors,
    /// for example chrono::ChLinkMotorRotationAngle.

    public class ChLinkMotor : ChLinkMateGeneric
    {

        public ChLinkMotor()
        {
            m_func = new ChFunction_Const(0); // defaults to no motion.
        }
        public ChLinkMotor(ChLinkMotor other)
        {
            m_func = other.m_func;
        }

        /// Set the actuation function of time F(t).
        /// The return value of this function has different meanings in various derived classes
        /// and can represent a position, angle, linear speed, angular speed, force, or torque.
        /// If controlling a position-level quantity (position or angle), this function must be
        /// C0 continuous (ideally C1 continuous to prevent spikes in accelerations).
        /// If controlling a velocity-level quantity (linear on angular speed), this function 
        /// should ideally be C0 continuous to prevent acceleration spikes.
        public void SetMotorFunction(ChFunction function) { m_func = function; }

        /// Get the actuation function F(t).
        public ChFunction GetMotorFunction() { return m_func; }

        /// "Virtual" copy constructor (covariant return type).
        public override ChObj Clone()  { return new ChLinkMotor(this); }

        /// Update state of the LinkMotor.
        public override void update(double mytime, bool update_assets) {
            base.update(mytime, update_assets);
            m_func.update(mytime);
        }

        protected ChFunction m_func;
    }

}
