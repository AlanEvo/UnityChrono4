using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;

namespace chrono
{

    /// Base class for all rotational "motor" constraints between
    /// two frames on two bodies. Motors of this type assume that
    /// the spindle is directed along Z direction of the master frame.
    /// Look for children classes for specialized behaviors,
    /// for example chrono::ChLinkMotorRotationAngle

    public class ChLinkMotorRotation : ChLinkMotor
    {

        /// Type of guide constraint
        public enum SpindleConstraint { FREE, REVOLUTE, CYLINDRICAL, OLDHAM };
        public SpindleConstraint sConstraint = SpindleConstraint.FREE;

        public ChLinkMotorRotation() {
            this.SetSpindleConstraint(SpindleConstraint.REVOLUTE);

            mrot = 0;
            mrot_dt = 0;
            mrot_dtdt = 0;
        }
        public ChLinkMotorRotation(ChLinkMotorRotation other) {
            mrot = other.mrot;
            mrot_dt = other.mrot_dt;
            mrot_dtdt = other.mrot_dtdt;
        }

        /// Sets which movements (of frame 1 respect to frame 2) are constrained.
        /// By default, acts as bearing, like a revolute joint.
        /// Note that the Z direction is the motorized one, and is never affected by
        /// this option.
        public void SetSpindleConstraint(SpindleConstraint mconstraint) {
            if (mconstraint == SpindleConstraint.FREE)
            {
                this.c_x = false;
                this.c_y = false;
                this.c_z = false;
                this.c_rx = false;
                this.c_ry = false;
                SetupLinkMask();
            }
            if (mconstraint == SpindleConstraint.REVOLUTE)
            {
                this.c_x = true;
                this.c_y = true;
                this.c_z = true;
                this.c_rx = true;
                this.c_ry = true;
                SetupLinkMask();
            }
            if (mconstraint == SpindleConstraint.CYLINDRICAL)
            {
                this.c_x = true;
                this.c_y = true;
                this.c_z = false;
                this.c_rx = true;
                this.c_ry = true;
                SetupLinkMask();
            }
            if (mconstraint == SpindleConstraint.OLDHAM)
            {
                this.c_x = false;
                this.c_y = false;
                this.c_z = false;
                this.c_rx = true;
                this.c_ry = true;
                SetupLinkMask();
            }
        }

        /// Sets which movements (of frame 1 respect to frame 2) are constrained.
        /// By default, acts as bearing, like a revolute joint.
        /// Note that the Z direction is the motorized one, and is never affected by
        /// this option.
        public void SetSpindleConstraint(bool mc_x, bool mc_y, bool mc_z, bool mc_rx, bool mc_ry) {
            this.c_x = mc_x;
            this.c_y = mc_y;
            this.c_z = mc_z;
            this.c_rx = mc_rx;
            this.c_ry = mc_ry;
            SetupLinkMask();
        }

        /// Get the current actuator rotation [rad], including error etc.
        /// This rotation keeps track of multiple turns, so it is not limited in periodic -PI..+PI,
        /// and rotation accumulates indefinitely. Use GetMotorRotTurns() and GetMotorRotPeriodic() otherwise.
        public virtual double GetMotorRot() { return mrot; }

        /// In case of multi-turns, gets the current actuator number of (integer) rotations,
        public virtual int GetMotorRotTurns() { return (int)(mrot / ChMaths.CH_C_2PI); }

        /// In case of multi-turns, gets the current actuator rotation angle [rad], in periodic -PI..+PI.
        public virtual double GetMotorRotPeriodic() { return Mathf.Repeat((float)mrot, (float)ChMaths.CH_C_2PI); }

        /// Get the current actuator speed [rad/s], including error etc.
        public virtual double GetMotorRot_dt() { return mrot_dt; }

        /// Get the current actuator acceleration [rad/s^2], including error etc.
        public virtual double GetMotorRot_dtdt() { return mrot_dtdt; }

        /// Get the current actuator reaction torque [Nm]
        public virtual double GetMotorTorque() {
            return 0;
        }

        public override void update(double mytime, bool update_assets) {
            // Inherit parent class:
            base.update(mytime, update_assets);

            // compute aux data for future reference (istantaneous pos speed accel)
            ChFrameMoving<double> aframe1 = ChFrameMoving<double>.BitShiftRight((ChFrameMoving<double>)this.frame1, (this.Body1));
            ChFrameMoving<double> aframe2 = ChFrameMoving<double>.BitShiftRight((ChFrameMoving<double>)this.frame2, (this.Body2));
            ChFrameMoving<double> aframe12 = new ChFrameMoving<double>();
            aframe2.TransformParentToLocal(aframe1, aframe12);


            // multi-turn rotation code
            double last_totrot = this.mrot;
            double last_rot = (double)decimal.Remainder(Convert.ToDecimal(last_totrot), Convert.ToDecimal(ChMaths.CH_C_2PI));
            double last_turns = last_totrot - last_rot;
            double new_rot = (double)decimal.Remainder(Convert.ToDecimal(aframe12.GetRot().Q_to_Rotv().z), Convert.ToDecimal(ChMaths.CH_C_2PI));
            this.mrot = last_turns + new_rot;
            if (Math.Abs(new_rot + ChMaths.CH_C_2PI - last_rot) < Math.Abs(new_rot - last_rot))
                this.mrot = last_turns + new_rot + ChMaths.CH_C_2PI;
            if (Math.Abs(new_rot - ChMaths.CH_C_2PI - last_rot) < Math.Abs(new_rot - last_rot))
                this.mrot = last_turns + new_rot - ChMaths.CH_C_2PI;

            this.mrot_dt = aframe12.GetWvel_loc().z;
            this.mrot_dtdt = aframe12.GetWacc_loc().z;
        }

        // aux data for optimization
        protected double mrot;
        protected double mrot_dt;
        protected double mrot_dtdt;
    }
}


