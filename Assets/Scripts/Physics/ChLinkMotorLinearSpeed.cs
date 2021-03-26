using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono
{

    /// A linear motor that enforces the speed v(t) between two frames on two bodies, using a rheonomic constraint.
    /// Note: no compliance is allowed, so if the actuator hits an undeformable obstacle it hits a pathological
    /// situation and the solver result can be unstable/unpredictable.
    /// Think at it as a servo drive with "infinitely stiff" control.
    /// This type of motor is very easy to use, stable and efficient, and should be used if the 'infinitely stiff'
    /// control assumption is a good approximation of what you simulate (e.g., very good and reactive controllers).
    /// By default it is initialized with constant speed: df/dt= 1.
    /// Use SetSpeedFunction() to change to other speed functions.

    public class ChLinkMotorLinearSpeed : ChLinkMotorLinear
    {
        public ChBody body1;
        public ChBody body2;

        public ChLinkMotorLinearSpeed() {
            variable.GetMass()[0, 0] = 1.0;
            variable.GetInvMass()[0, 0] = 1.0;

            m_func = new ChFunction_Const(1.0);

            pos_offset = 0;

            aux_dt = 0;  // used for integrating speed, = pos
            aux_dtdt = 0;

            avoid_position_drift = true;
        }
        public ChLinkMotorLinearSpeed(ChLinkMotorLinearSpeed other): base(other)
        {
            variable = other.variable;
            pos_offset = other.pos_offset;
            aux_dt = other.aux_dt;
            aux_dtdt = other.aux_dtdt;
            avoid_position_drift = other.avoid_position_drift;
        }

        /// "Virtual" copy constructor (covariant return type).
        public override ChObj Clone()  { return new ChLinkMotorLinearSpeed(this); }

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

        void FixedUpdate()
        {
            var csys = GetLinkAbsoluteCoords();
            transform.position = Utils.FromChrono(csys.pos);
            transform.rotation = Utils.FromChrono(csys.rot);
        }

        /// Set the speed function of time v(t).
        /// To prevent acceleration pikes, this function should be C0 continuous.
        public void SetSpeedFunction(ChFunction function) { SetMotorFunction(function); }

        /// Get the speed function v(t).
        public ChFunction GetSpeedFunction() { return GetMotorFunction(); }

        /// Get initial offset, by default = 0.
        public void SetMotionOffset(double mo) { pos_offset = mo; }

        /// Get initial offset.
        public double GetMotionOffset() { return pos_offset; }

        /// Set if the constraint must avoid position drift. If true, it
        /// means that the constraint is satisfied also at the position level,
        /// by integrating the velocity in a separate auxiliary state. Default, true.
        public void SetAvoidPositionDrift(bool mb) { this.avoid_position_drift = mb; }

        /// Set if the constraint is in "avoid position drift" mode.
        public bool GetAvoidPositionDrift() { return this.avoid_position_drift; }

        /// Get the current actuator reaction force [N]
        public override double GetMotorForce() { return -this.react_force.x; }

        public override void update(double mytime, bool update_assets) {
            // Inherit parent class:
            base.update(mytime, update_assets);

            // Add the time-dependent term in residual C as
            //   C = d_error - d_setpoint - d_offset
            // with d_error = x_pos_A- x_pos_B, and d_setpoint = x(t)
            if (this.avoid_position_drift)
                C.ElementN(0) = this.mpos - aux_dt - this.pos_offset;
            else
                C.ElementN(0) = 0.0;
        }

        //
        // STATE FUNCTIONS
        //

        public override int GetDOF() { return 1; }

        public override void IntStateGather(int off_x,
                                ref ChState x,
                                int off_v,
                                ref ChStateDelta v,
                                ref double T) {
            x[off_x] = 0;  // aux;
            v[off_v] = aux_dt;
            T = GetChTime();
        }

        public override void IntStateScatter(int off_x,
                                 ChState x,
                                 int off_v,
                                 ChStateDelta v,
                                 double T) {
            // aux = x(off_x);
            aux_dt = v[off_v];
        }
        public override void IntStateGatherAcceleration(int off_a, ref ChStateDelta a) {
            a[off_a] = aux_dtdt;
        }
        public override void IntStateScatterAcceleration(int off_a, ChStateDelta a) {
            aux_dtdt = a[off_a];
        }
        public override void IntLoadResidual_F(int off, ref ChVectorDynamic<double> R, double c) {
            double imposed_speed = m_func.Get_y(this.GetChTime());
            R[off] += imposed_speed * c;
        }
        public override void IntLoadResidual_Mv(int off,
                                    ref ChVectorDynamic<double> R,
                                    ChVectorDynamic<double> w,
                                    double c) {
            R[off] += c * 1.0 * w[off];
        }
        public override void IntToDescriptor(int off_v,
                                 ChStateDelta v,
                                 ChVectorDynamic<double> R,
                                 int off_L,
                                 ChVectorDynamic<double> L,
                                 ChVectorDynamic<double> Qc) {
            // inherit parent
            base.IntToDescriptor(off_v, v, R, off_L, L, Qc);

            this.variable.Get_qb()[0, 0] = v[off_v];
            this.variable.Get_fb()[0, 0] = R[off_v];
        }
        public override void IntFromDescriptor(int off_v,
                                   ref ChStateDelta v,
                                   int off_L,
                                   ref ChVectorDynamic<double> L) {
            // inherit parent
            base.IntFromDescriptor(off_v, ref v, off_L, ref L);

            v[off_v] = this.variable.Get_qb()[0, 0];
        }

        public override void IntLoadConstraint_Ct(int off_L, ref ChVectorDynamic<double> Qc, double c) {
            double mCt = -m_func.Get_y(this.GetChTime());
            if (mask.Constr_N(0).IsActive())
            {
                Qc[off_L + 0] += c * mCt;
            }
        }

        //
        // SOLVER INTERFACE (OLD)
        //

        public override void VariablesFbReset() {
            variable.Get_fb().FillElem(0.0);
        }
        public override void VariablesFbLoadForces(double factor = 1) {
            double imposed_speed = m_func.Get_y(this.GetChTime());
            variable.Get_fb().ElementN(0) += imposed_speed * factor;
        }
        public override void VariablesQbLoadSpeed() {
            // set current speed in 'qb', it can be used by the solver when working in incremental mode
            variable.Get_qb().SetElement(0, 0, aux_dt);
        }
        public override void VariablesFbIncrementMq() {
            variable.Compute_inc_Mb_v(ref variable.Get_fb(), variable.Get_qb());
        }
        public override void VariablesQbSetSpeed(double step = 0) {
            double old_dt = aux_dt;

            // from 'qb' vector, sets body speed, and updates auxiliary data
            aux_dt = variable.Get_qb().GetElement(0, 0);

            // Compute accel. by BDF (approximate by differentiation); not needed
        }
        public override void InjectVariables(ref ChSystemDescriptor mdescriptor) {
            variable.SetDisabled(!IsActive());

            mdescriptor.InsertVariables(variable);
        }

        public override void ConstraintsBiLoad_Ct(double factor = 1) {
            if (!this.IsActive())
                return;

            double mCt = -m_func.Get_y(this.GetChTime());
            if (mask.Constr_N(0).IsActive())
            {
                mask.Constr_N(0).Set_b_i(mask.Constr_N(0).Get_b_i() + factor * mCt);
            }
        }


        private double pos_offset;

        private ChVariablesGeneric variable = new ChVariablesGeneric();

        private double aux_dt;  // used for integrating speed, = pos
        private double aux_dtdt;

        private bool avoid_position_drift;
    }



}