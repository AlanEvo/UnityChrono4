using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono
{


    /// A motor that enforces the angular speed w(t) between
    /// two ChShaft shafts, using a rheonomic constraint.
    /// Note: no compliance is allowed, so if the actuator hits an undeformable 
    /// obstacle it hits a pathological situation and the solver result
    /// can be unstable/unpredictable.
    /// Think at it as a servo drive with "infinitely stiff" control.
    /// This type of motor is very easy to use, stable and efficient,
    /// and should be used if the 'infinitely stiff' control assumption 
    /// is a good approximation of what you simulate (ex very good and
    /// reactive controllers).
    /// By default it is initialized with constant angular speed: df/dt= 1 rad/s, use
    /// SetSpeedFunction() to change to other speed functions.

    public class ChShaftsMotorSpeed : ChShaftsMotorBase
    {
        private ChFunction f_speed;
        private double rot_offset;

        private ChVariablesGeneric variable = new ChVariablesGeneric();

        private double aux_dt; // used for integrating speed, = angle
        private double aux_dtdt;

        private bool avoid_angle_drift;

        private double motor_torque;
        private ChConstraintTwoGeneric constraint;  //< used as an interface to the solver


        public ChShaftsMotorSpeed()
        {
            motor_torque = 0;
            this.variable.GetMass()[0, 0] = 1.0;
            this.variable.GetInvMass()[0, 0] = 1.0;

            this.f_speed = new ChFunction_Const(1.0);

            this.rot_offset = 0;

            this.aux_dt = 0; // used for integrating speed, = pos
            this.aux_dtdt = 0;

            this.avoid_angle_drift = true;
        }
        public ChShaftsMotorSpeed(ChShaftsMotorSpeed other)
        {
            this.variable = other.variable;

            this.f_speed = other.f_speed;

            this.rot_offset = other.rot_offset;

            this.aux_dt = other.aux_dt;
            this.aux_dtdt = other.aux_dtdt;

            this.avoid_angle_drift = other.avoid_angle_drift;
        }

        /// "Virtual" copy constructor (covariant return type).
        public override ChObj Clone() { return new ChShaftsMotorSpeed(this); }

        public void Start()
        {            
            Initialize(shaft1, shaft2);

            // Get a handle to the associated function component and set the motor's function
            var fun_component = this.GetComponent<ChFunction>();
            if (fun_component != null)
            {
                SetSpeedFunction(fun_component);
            }

            ChSystem msystem = FindObjectOfType<ChSystem>();
           // msystem.Add(this);

        }


        /// Sets the angular speed function w(t), in [rad/s]. It is a function of time. 
        /// Best if C0 continuous, otherwise it gives peaks in accelerations.
        public void SetSpeedFunction(ChFunction mf) { f_speed = mf; }

        /// Gets the speed function w(t). In [rad/s].
        public ChFunction GetSpeedFunction() { return f_speed; }


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


        /// Use this function after gear creation, to initialize it, given
        /// two shafts to join. The first shaft is the 'output' shaft of the motor,
        /// the second is the 'truss', often fixed and not rotating.
        /// The torque is applied to the output shaft, while the truss shafts
        /// gets the same torque but with opposite sign.
        /// Each shaft must belong to the same ChSystem.
        public override bool Initialize(ChShaft mshaft1,  //< first  shaft to join (motor output shaft)
                    ChShaft mshaft2   //< second shaft to join (motor truss)
                    ) {
            // Parent class initialize
            if (!base.Initialize(mshaft1, mshaft2))
                return false;

            ChShaft mm1 = mshaft1;
            ChShaft mm2 = mshaft2;

            constraint.SetVariables(mm1.Variables(), mm2.Variables());

            SetSystem(shaft1.GetSystem());

            return true;
        }


        /// Get the current motor torque between shaft2 and shaft1, expressed as applied to shaft1
        public override double GetMotorTorque() { return motor_torque; }

        /// Update all auxiliary data 
        public override void update(double mytime, bool update_assets = true) {
            // Inherit time changes of parent class
            base.update(mytime, update_assets);

            // update class data

            this.f_speed.update(mytime); // call callbacks if any
        }


        //
        // STATE FUNCTIONS
        //

        /// Number of scalar constraints
        public override int GetDOF() { return 1; }
        public override int GetDOC_c() { return 1; }

        // (override/implement interfaces for global state vectors, see ChPhysicsItem for comments.)
        public override void IntStateGather(int off_x, ref ChState x, int off_v, ref ChStateDelta v, ref double T) {
            x.matrix[off_x] = 0;//aux;
            v.matrix[off_v] = aux_dt;
            T = GetChTime();
        }
        public override void IntStateScatter(int off_x, ChState x, int off_v, ChStateDelta v, double T) {
            //aux = x(off_x);
            aux_dt = v.matrix[off_v];
        }
        public override void IntStateGatherReactions(int off_L, ref ChVectorDynamic<double> L) {
            L.matrix[off_L] = motor_torque;
        }
        public override void IntStateScatterReactions(int off_L, ChVectorDynamic<double> L) {
            motor_torque = L.matrix[off_L];
        }
        public override void IntStateGatherAcceleration(int off_a, ref ChStateDelta a) {
            a.matrix[off_a] = aux_dtdt;
        }
        public override void IntStateScatterAcceleration(int off_a, ChStateDelta a) {
            aux_dtdt = a.matrix[off_a];
        }
        public override void IntLoadResidual_F(int off, ref ChVectorDynamic<double> R, double c) {
            double imposed_speed = this.f_speed.Get_y(this.GetChTime());
            R.matrix[off] += imposed_speed * c;
        }
        public override void IntLoadResidual_Mv(int off, ref ChVectorDynamic<double> R, ChVectorDynamic<double> w, double c) {
            R.matrix[off] += c * 1.0 * w.matrix[off];
        }
        public override void IntLoadResidual_CqL(int off_L, ref ChVectorDynamic<double> R, ChVectorDynamic<double> L, double c) {
            constraint.MultiplyTandAdd(R.matrix, L.matrix[off_L] * c);
        }
        public override void IntLoadConstraint_C(int off_L, ref ChVectorDynamic<double> Qc, double c, bool do_clamp, double recovery_clamp) {
            // Add the time-dependent term in residual C as 
            //   C = d_error - d_setpoint - d_offset
            // with d_error = x_pos_A-x_pos_B, and d_setpoint = x(t)
            double C;
            if (this.avoid_angle_drift)
                C = this.GetMotorRot() - aux_dt - this.rot_offset;
            else
                C = 0.0;

            double res = c * C;

            if (do_clamp)
            {
                res = ChMaths.ChMin(ChMaths.ChMax(res, -recovery_clamp), recovery_clamp);
            }
            Qc.matrix[off_L] += res;
        }
        public override void IntLoadConstraint_Ct(int off_L, ref ChVectorDynamic<double> Qc, double c) {
            double ct = -this.f_speed.Get_y(this.GetChTime());
            Qc.matrix[off_L] += c * ct;
        }
        public override void IntToDescriptor(int off_v, ChStateDelta v, ChVectorDynamic<double> R, int off_L, ChVectorDynamic<double> L, ChVectorDynamic<double> Qc) {
            constraint.Set_l_i(L.matrix[off_L]);
            constraint.Set_b_i(Qc.matrix[off_L]);

            this.variable.Get_qb().matrix[0, 0] = v.matrix[off_v];
            this.variable.Get_fb().matrix[0, 0] = R.matrix[off_v];
        }
        public override void IntFromDescriptor(int off_v, ref ChStateDelta v, int off_L, ref ChVectorDynamic<double> L) {
            L.matrix[off_L] = constraint.Get_l_i();

            v.matrix[off_v] = this.variable.Get_qb().matrix[0, 0];
        }

        // Old...

        public override void InjectConstraints(ref ChSystemDescriptor mdescriptor) {
            mdescriptor.InsertConstraint(constraint);
        }
        public override void InjectVariables(ref ChSystemDescriptor mdescriptor) {
            mdescriptor.InsertVariables(variable);
        }
        public override void ConstraintsBiReset() {
            constraint.Set_b_i(0.0);
        }
        public override void ConstraintsBiLoad_C(double factor = 1, double recovery_clamp = 0.1, bool do_clamp = false) {
            double C;
            if (this.avoid_angle_drift)
                C = this.GetMotorRot() - aux_dt - this.rot_offset;
            else
                C = 0.0;

            double res = factor * C;

            if (do_clamp)
            {
                res = ChMaths.ChMin(ChMaths.ChMax(res, -recovery_clamp), recovery_clamp);
            }

            constraint.Set_b_i(constraint.Get_b_i() + res);
        }
        public override void ConstraintsBiLoad_Ct(double factor = 1) {
            double ct = -this.f_speed.Get_y(this.GetChTime());
            constraint.Set_b_i(constraint.Get_b_i() + factor * ct);
        }
        public override void ConstraintsLoadJacobians() {
            constraint.Get_Cq_a().SetElement(0, 0, 1);
            constraint.Get_Cq_b().SetElement(0, 0, -1);
        }
        public override void ConstraintsFetch_react(double factor = 1) {
            motor_torque = -constraint.Get_l_i() * factor;
        }
        public override void VariablesFbReset() {
            variable.Get_fb().matrix.FillElem(0.0);
        }
        public override void VariablesFbLoadForces(double factor = 1) {
            double imposed_speed = this.f_speed.Get_y(this.GetChTime());
            variable.Get_fb().matrix.ElementN(0) += imposed_speed * factor;
        }
        public override void VariablesQbLoadSpeed() {
            // set current speed in 'qb', it can be used by the solver when working in incremental mode
            variable.Get_qb().matrix.SetElement(0, 0, aux_dt);
        }
        public override void VariablesFbIncrementMq() {
            variable.Compute_inc_Mb_v(ref variable.Get_fb().matrix, variable.Get_qb().matrix);
        }
        public override void VariablesQbSetSpeed(double step = 0) {
            double old_dt = aux_dt;

            // from 'qb' vector, sets body speed, and updates auxiliary data
            aux_dt = variable.Get_qb().matrix.GetElement(0, 0);

            // Compute accel. by BDF (approximate by differentiation); not needed
        }

    }
}
