using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono
{

    /// Base class for all linear "motor" constraints between
    /// two frames on two bodies. Motors of this type assume that
    /// the actuator is directed along X direction of the master frame.
    /// Look for children classes for specialized behaviors, for example
    /// ex. chrono::ChLinkMotorLinearPosition

    public abstract class ChLinkMotorLinear : ChLinkMotor {

        /// Type of guide constraint
        public enum GuideConstraint { FREE, PRISMATIC, SPHERICAL };

        public ChLinkMotorLinear() {
            this.SetGuideConstraint(GuideConstraint.PRISMATIC);

            mpos = 0;
            mpos_dt = 0;
            mpos_dtdt = 0;
        }
        public ChLinkMotorLinear(ChLinkMotorLinear other) {
            mpos = other.mpos;
            mpos_dt = other.mpos_dt;
            mpos_dtdt = other.mpos_dtdt;
        }

        /// "Virtual" copy constructor (covariant return type).
        // virtual ChLinkMotorLinear* Clone() const override { return new ChLinkMotorLinear(*this); }

        /// Sets which movements (of frame 1 respect to frame 2) are constrained.
        /// By default, acts as a pure prismatic guide.
        /// Note that the x direction is the motorized one, and is never affected by
        /// this option.
        public void SetGuideConstraint(GuideConstraint mconstraint) {
            if (mconstraint == GuideConstraint.FREE)
            {
                this.c_y = false;
                this.c_z = false;
                this.c_rx = false;
                this.c_ry = false;
                this.c_rz = false;
                SetupLinkMask();
            }
            if (mconstraint == GuideConstraint.PRISMATIC)
            {
                this.c_y = true;
                this.c_z = true;
                this.c_rx = true;
                this.c_ry = true;
                this.c_rz = true;
                SetupLinkMask();
            }
            if (mconstraint == GuideConstraint.SPHERICAL)
            {
                this.c_y = true;
                this.c_z = true;
                this.c_rx = false;
                this.c_ry = false;
                this.c_rz = false;
                SetupLinkMask();
            }
        }

        /// Sets which movements (of frame 1 respect to frame 2) are constrained.
        /// By default, acts as a pure prismatic guide.
        /// Note that the x direction is the motorized one, and is never affected by
        /// this option.
        public void SetGuideConstraint(bool mc_y, bool mc_z, bool mc_rx, bool mc_ry, bool mc_rz) {
            this.c_y = mc_y;
            this.c_z = mc_z;
            this.c_rx = mc_rx;
            this.c_ry = mc_ry;
            this.c_rz = mc_rz;
            SetupLinkMask();
        }

        /// Get the current actuator displacement [m], including error etc.
        public virtual double GetMotorPos() { return mpos; }
        /// Get the current actuator speed [m/s], including error etc.
        public virtual double GetMotorPos_dt() { return mpos_dt; }
        /// Get the current actuator acceleration [m/s^2], including error etc.
        public virtual double GetMotorPos_dtdt() { return mpos_dtdt; }
        /// Get the current actuator reaction force [N]
        public abstract double GetMotorForce();

        public override void update(double mytime, bool update_assets) {
            // Inherit parent class:
            base.update(mytime, update_assets);

            // compute aux data for future reference (istantaneous pos speed accel)
            ChFrameMoving<double> aframe1 = ChFrameMoving<double>.BitShiftRight((ChFrameMoving<double>)(this.frame1) , (this.Body1));
            ChFrameMoving<double> aframe2 = ChFrameMoving<double>.BitShiftRight((ChFrameMoving<double>)(this.frame2) , (this.Body2));
            ChFrameMoving<double> aframe12 = new ChFrameMoving<double>();
            aframe2.TransformParentToLocal(aframe1, aframe12);
            this.mpos = aframe12.GetPos().x;
            this.mpos_dt = aframe12.GetPos_dt().x;
            this.mpos_dtdt = aframe12.GetPos_dtdt().x;
        }

        // aux data for optimization
        protected double mpos;
        protected double mpos_dt;
        protected double mpos_dtdt;
    }

}
