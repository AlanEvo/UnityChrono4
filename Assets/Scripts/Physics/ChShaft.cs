using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;


namespace chrono
{
    ///  Class for one-degree-of-freedom mechanical parts with associated
    ///  inertia (mass, or J moment of inertial for rotating parts).
    ///  In most cases these represent shafts that can be used to build 1D models
    ///  of power trains. This is more efficient than simulating power trains
    ///  modeled with full 3D ChBody objects.
    public class ChShaft : ChPhysicsItem
    {

        private double torque;  //< The torque acting on shaft (force, if used as linear DOF)

        private double pos;       //< shaft angle
        private double pos_dt;    //< shaft angular velocity
        private double pos_dtdt;  //< shaft angular acceleration

        private double inertia;  //< shaft J moment of inertia (or mass, if used as linear DOF)

        private ChVariablesShaft variables = new ChVariablesShaft();  //< used as an interface to the solver

        private float max_speed;  //< limit on linear speed

        private float sleep_time;
        private float sleep_minspeed;
        private float sleep_minwvel;
        private float sleep_starttime;

        private bool mfixed;
        private bool limitspeed;
        private bool sleeping;
        private bool use_sleeping;

        private int id;  //< shaft id used for internal indexing


        public ChShaft()
        {
            torque = 0;
            pos = 0;
            pos_dt = 0;
            pos_dtdt = 0;
            inertia = 1;
            mfixed = false;
            limitspeed = false;
            max_speed = 10.0f;
            sleep_time = 0.6f;
            sleep_starttime = 0;
            sleep_minspeed = 0.1f;
            sleep_minwvel = 0.04f;
            sleeping = false;
            SetUseSleeping(true);
            variables.SetShaft(this);
        }
        public ChShaft(ChShaft other) {
            torque = other.torque;
           // system = other.system;
            pos = other.pos;
            pos_dt = other.pos_dt;
            pos_dtdt = other.pos_dtdt;
            inertia = other.inertia;
            mfixed = other.mfixed;
            sleeping = other.sleeping;
            limitspeed = other.limitspeed;

            variables = other.variables;

            max_speed = other.max_speed;

            sleep_time = other.sleep_time;
            sleep_starttime = other.sleep_starttime;
            sleep_minspeed = other.sleep_minspeed;
            sleep_minwvel = other.sleep_minwvel;
        }

        /// "Virtual" copy constructor (covariant return type).
        public override ChObj Clone() { return new ChShaft(this); }

        //
        // FLAGS
        //

        /// Sets the 'fixed' state of the shaft. If true, it does not rotate
        /// despite constraints, forces, etc.
        public void SetShaftFixed(bool mev)
        {
            mfixed = mev;
            variables.SetDisabled(mev);
        }

        public bool GetShaftFixed() { return mfixed; }

        /// Trick. Set the maximum shaft velocity (beyond this limit it will
        /// be clamped). This is useful in virtual reality and real-time
        /// simulations.
        /// The realism is limited, but the simulation is more stable.
        public void SetLimitSpeed(bool mlimit) { limitspeed = mlimit; }
        public bool GetLimitSpeed() { return limitspeed; }

        /// Trick. If use sleeping= true, shafts which do not rotate
        /// for too long time will be deactivated, for optimization.
        /// The realism is limited, but the simulation is faster.
        public void SetUseSleeping(bool ms) { use_sleeping = ms; }
        public bool GetUseSleeping() { return use_sleeping; }

        /// Force the shaft in sleeping mode or not (usually this state change is not
        /// handled by users, anyway, because it is mostly automatic).
        public void SetSleeping(bool ms) { sleeping = ms; }
        /// Tell if the shaft is actually in sleeping state.
        public bool GetSleeping() { return sleeping; }

        /// Put the shaft in sleeping state if requirements are satisfied.
        public bool TrySleeping() {
            if (GetUseSleeping())
            {
                if (GetSleeping())
                    return true;

                if (Math.Abs(pos_dt) < sleep_minspeed)
                {
                    if ((GetChTime() - sleep_starttime) > sleep_time)
                    {
                        SetSleeping(true);
                        return true;
                    }
                }
                else
                {
                    sleep_starttime = (float)(GetChTime());
                }
            }
            return false;
        }

        /// Tell if the body is active, i.e. it is neither fixed to ground nor
        /// it is in sleep mode.
        public bool IsActive() { return !(sleeping || mfixed); }

        //
        // FUNCTIONS
        //

        /// Set the shaft id for indexing (only used internally)
        public void SetId(int identifier) { id = identifier; }

        /// Get the shaft id for indexing (only used internally)
        public int GetId() { return id; }

        /// Number of coordinates of the shaft
        public override int GetDOF() { return 1; }

        /// Returns reference to the encapsulated ChVariables,
        public ChVariablesShaft Variables() { return variables; }

        //
        // STATE FUNCTIONS

        //

        // (override/implement interfaces for global state vectors, see ChPhysicsItem for comments.)
        public override void IntStateGather(int off_x,
                            ref ChState x,
                            int off_v,
                            ref ChStateDelta v,
                            ref double T) {
            x[off_x] = pos;
            v[off_v] = pos_dt;
            T = GetChTime();
        }

        public override void IntStateScatter(int off_x,
                                 ChState x,
                                 int off_v,
                                 ChStateDelta v,
                                 double T) {
            SetPos(x[off_x]);
            SetPos_dt(v[off_v]);
            update(T);
        }

        public override void IntStateGatherAcceleration(int off_a, ref ChStateDelta a) {
            a[off_a] = pos_dtdt;
        }
        public override void IntStateScatterAcceleration(int off_a, ChStateDelta a) {
            SetPos_dtdt(a[off_a]);
        }
        public override void IntLoadResidual_F(int off, ref ChVectorDynamic<double> R, double c) {
            // add applied forces to 'fb' vector
            R[off] += torque * c;
        }
        public override void IntLoadResidual_Mv(int off,
                                    ref ChVectorDynamic<double> R,
                                    ChVectorDynamic<double> w,
                                    double c) {
            R[off] += c * inertia * w[off];
        }
        public override void IntToDescriptor(int off_v,
                                 ChStateDelta v,
                                 ChVectorDynamic<double> R,
                                 int off_L,
                                 ChVectorDynamic<double> L,
                                 ChVectorDynamic<double> Qc) {
            variables.Get_qb()[0, 0] = v[off_v];
            variables.Get_fb()[0, 0] = R[off_v];
        }
        public override void IntFromDescriptor(int off_v,
                                   ref ChStateDelta v,
                                   int off_L,
                                   ref ChVectorDynamic<double> L) {
            v[off_v] = variables.Get_qb()[0, 0];
        }

        //
        // SOLVER FUNCTIONS
        //

        // Override/implement system functions of ChPhysicsItem
        // (to assemble/manage data for system solver)

        /// Sets the 'fb' part of the encapsulated ChVariables to zero.
        public override void VariablesFbReset() {
            variables.Get_fb().FillElem(0.0);
        }

        /// Adds the current torques in the 'fb' part: qf+=torques*factor
        public override void VariablesFbLoadForces(double factor = 1) {
            // add applied torques to 'fb' vector
            variables.Get_fb().ElementN(0) += torque * factor;
        }

        /// Initialize the 'qb' part of the ChVariables with the
        /// current value of shaft speed. Note: since 'qb' is the unknown , this
        /// function seems unnecessary, unless used before VariablesFbIncrementMq()
        public override void VariablesQbLoadSpeed() {
            // set current speed in 'qb', it can be used by the solver when working in incremental mode
            variables.Get_qb().SetElement(0, 0, pos_dt);
        }

        /// Adds M*q (masses multiplied current 'qb') to Fb, ex. if qb is initialized
        /// with v_old using VariablesQbLoadSpeed, this method can be used in
        /// timestepping schemes that do: M*v_new = M*v_old + forces*dt
        public override void VariablesFbIncrementMq() {
            variables.Compute_inc_Mb_v(ref variables.Get_fb(), variables.Get_qb());
        }

        /// Fetches the shaft speed from the 'qb' part of the ChVariables (does not
        /// updates the full shaft state) and sets it as the current shaft speed.
        /// If 'step' is not 0, also computes the approximate acceleration of
        /// the shaft using backward differences, that is  accel=(new_speed-old_speed)/step.
        public override void VariablesQbSetSpeed(double step = 0) {
            double old_dt = pos_dt;

            // from 'qb' vector, sets body speed, and updates auxiliary data
            pos_dt = variables.Get_qb().GetElement(0, 0);

            // apply limits (if in speed clamping mode) to speeds.
            ClampSpeed();

            // Compute accel. by BDF (approximate by differentiation);
            if (step != 0)
            {
                pos_dtdt = (pos_dt - old_dt) / step;
            }
        }

        /// Increment shaft position by the 'qb' part of the ChVariables,
        /// multiplied by a 'step' factor.
        ///     pos+=qb*step
        public override void VariablesQbIncrementPosition(double dt_step) {
            if (!IsActive())
                return;

            // Updates position with incremental action of speed contained in the
            // 'qb' vector:  pos' = pos + dt * speed   , like in an Eulero step.

            double newspeed = variables.Get_qb().GetElement(0, 0);

            // ADVANCE POSITION: pos' = pos + dt * vel
            pos = pos + newspeed * dt_step;
        }

        /// Tell to a system descriptor that there are variables of type
        /// ChVariables in this object (for further passing it to a solver)
        public override void InjectVariables(ref ChSystemDescriptor mdescriptor) {
            variables.SetDisabled(!IsActive());

            mdescriptor.InsertVariables(variables);
        }

        // Other functions

        /// Set no speed and no accelerations (but does not change the position)
        public override void SetNoSpeedNoAcceleration() {
            pos_dt = 0;
            pos_dtdt = 0;
        }

        /// Set the torque applied to the shaft
        public void SetAppliedTorque(double mtorque) { torque = mtorque; }
        /// Get the torque applied to the shaft
        public double GetAppliedTorque() { return torque; }

        /// Set the angular position
        public void SetPos(double mp) { pos = mp; }
        /// Get the angular position
        public double GetPos() { return pos; }

        /// Set the angular velocity
        public void SetPos_dt(double mp) { pos_dt = mp; }
        /// Get the angular velocity
        public double GetPos_dt() { return pos_dt; }

        /// Set the angular acceleration
        public void SetPos_dtdt(double mp) { pos_dtdt = mp; }
        /// Get the angular acceleration
        public double GetPos_dtdt() { return pos_dtdt; }

        /// Inertia of the shaft. Must be positive.
        /// Try not to mix bodies with too high/too low values of mass, for numerical stability.
        public void SetInertia(double newJ) {
           // Debug.Assert(newJ > 0.0);
            if (newJ <= 0.0)
                return;
            inertia = newJ;
            variables.SetInertia(newJ);
        }

        public double GetInertia() { return inertia; }

        /// Trick. Set the maximum velocity (beyond this limit it will
        /// be clamped). This is useful in virtual reality and real-time
        /// simulations, to increase robustness at the cost of realism.
        /// This limit is active only if you set  SetLimitSpeed(true);
        public void SetMaxSpeed(float m_max_speed) { max_speed = m_max_speed; }
        public float GetMaxSpeed() { return max_speed; }

        /// When this function is called, the speed of the shaft is clamped
        /// into limits posed by max_speed and max_wvel  - but remember to
        /// put the shaft in the SetLimitSpeed(true) mode.
        public void ClampSpeed() {
            if (GetLimitSpeed())
            {
                if (pos_dt > max_speed)
                    pos_dt = max_speed;
                if (pos_dt < -max_speed)
                    pos_dt = -max_speed;
            }
        }

        /// Set the amount of time which must pass before going automatically in
        /// sleep mode when the shaft has very small movements.
        public void SetSleepTime(float m_t) { sleep_time = m_t; }
        public float GetSleepTime() { return sleep_time; }

        /// Set the max linear speed to be kept for 'sleep_time' before freezing.
        public void SetSleepMinSpeed(float m_t) { sleep_minspeed = m_t; }
        public float GetSleepMinSpeed() { return sleep_minspeed; }

        /// Set the max linear speed to be kept for 'sleep_time' before freezing.
        public void SetSleepMinWvel(float m_t) { sleep_minwvel = m_t; }
        public float GetSleepMinWvel() { return sleep_minwvel; }

        //
        // UPDATE FUNCTIONS
        //

        /// Update all auxiliary data of the shaft at given time
        public override void update(double mytime, bool update_assets = true) {
            // Update parent class too
            base.update(mytime, update_assets);

            // Class update

            // TrySleeping();    // See if the body can fall asleep; if so, put it to sleeping
            ClampSpeed();  // Apply limits (if in speed clamping mode) to speeds
        }

    }
}
