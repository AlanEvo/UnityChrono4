using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;

namespace chrono
{


    /// Base class for all "motors" between two 1D elements of ChShaft class.
    /// You can consider the usage of its inherited  classes such as ChShaftsMotor,
    /// ChShaftsMotorAngle, ChShaftsMotorSpeed or ChShaftsMotorTorque.
    public abstract class ChShaftsMotorBase : ChShaftsCouple
    {

        public ChShaftsMotorBase() { }
        public ChShaftsMotorBase(ChShaftsMotorBase other) { }

        /// Get the actual angle rotation [rad] of the motor, in terms of phase of shaft 1 respect to 2.
        public virtual double GetMotorRot() { return (shaft1.GetPos() - shaft2.GetPos()); }
        /// Get the actual speed [rad/s] of the motor, in terms of speed of shaft 1 respect to 2.
        public virtual double GetMotorRot_dt() { return (shaft1.GetPos_dt() - shaft2.GetPos_dt()); }
        /// Get the actual acceleration [rad/s^2] of the motor, in terms of accel. of shaft 1 respect to 2.
        public virtual double GetMotorRot_dtdt() { return (shaft1.GetPos_dtdt() - shaft2.GetPos_dtdt()); }

        /// In case of multi-turns, gets the current actuator number of (integer) rotations:
        public virtual int GetMotorRotTurns() { return (int)(GetMotorRot() / ChMaths.CH_C_2PI); }

        /// In case of multi-turns, gets the current actuator rotation angle [rad], in periodic -PI..+PI.
       // public virtual double GetMotorRotPeriodic() { return Math.Mod(GetMotorRot(), ChMaths.CH_C_2PI); }

        /// Get the current motor torque between shaft2 and shaft1, expressed as applied to shaft1
        public abstract double GetMotorTorque();

        /// Get the reaction torque exchanged between the two shafts,
        /// considered as applied to the 1st axis.
        public override double GetTorqueReactionOn1() { return (GetMotorTorque()); }

        /// Get the reaction torque exchanged between the two shafts,
        /// considered as applied to the 2nd axis.
        public override double GetTorqueReactionOn2() { return -(GetMotorTorque()); }

    }
}
