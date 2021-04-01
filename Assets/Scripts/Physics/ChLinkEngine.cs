using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;


namespace chrono
{
    /// Class for links representing engines between two rigid bodies.
    /// Note that the engine can be in 'impose relative rotation' mode,
    /// as well as in 'impose speed' etc. It can also be used to represent
    /// an engine with a torque/speed custom curve. etc.
    /// **NOTE! THIS IS OBSOLETE**. Prefer using the new classes
    /// inherited from chrono::ChLinkMotor.
    public class ChLinkEngine : ChLinkLock
    {

        public enum eCh_eng_mode
        {
            ENG_MODE_ROTATION = 0,
            ENG_MODE_SPEED,
            ENG_MODE_TORQUE,
            ENG_MODE_KEY_ROTATION,
            ENG_MODE_KEY_POLAR,
            ENG_MODE_TO_POWERTRAIN_SHAFT
        };

        public enum eCh_shaft_mode
        {
            ENG_SHAFT_LOCK = 0,   //< shafts of motor and user (markers 1 and 2) are stiffly joined
            ENG_SHAFT_PRISM,      //< shafts of motor and user (markers 1 and 2) can shift along shaft (Z axis)
            ENG_SHAFT_OLDHAM,     //< shafts of motor and user (markers 1 and 2) may be parallel shifting on X and Y
            ENG_SHAFT_UNIVERSAL,  //< not yet used
            ENG_SHAFT_CARDANO     //< not yet used
        };

        protected ChFunction rot_funct;  ///< rotation(t) function
        protected ChFunction spe_funct;  ///< speed(t) function
        protected ChFunction tor_funct;  ///< torque(t) function
        protected ChFunction torque_w;   ///< torque(w) function

        protected bool learn;  //< if true, the actuator does not apply raint, just records the motion into its rot_function.

        protected bool impose_reducer;  //< if true, speed torque or rotation are imposed to the fast (motor) shaft, before reducer!

        protected double mot_rot;         //< current rotation (read only)
        protected double mot_rot_dt;      //< current ang speed (read only)
        protected double mot_rot_dtdt;    //< current ang acc  (read only)
        protected double mot_torque;      //< current motor torque (read only)
        protected double mot_rerot;       //< current rotation (read only)  before reducer
        protected double mot_rerot_dt;    //< current ang speed (read only) before reducer
        protected double mot_rerot_dtdt;  //< current ang acc  (read only)  before reducer
        protected double mot_retorque;    //< current motor torque (read only) before reducer

        public double mot_tau;      //< motor: transmission ratio
        public double mot_eta;      //< motor: transmission efficiency
        public double mot_inertia;  //< motor: inertia (added to system)

        public eCh_eng_mode eng_mode = eCh_eng_mode.ENG_MODE_KEY_ROTATION;  //< mode of controlling the motor (by rotation, speed etc.)

        public eCh_shaft_mode shaft_mode = eCh_shaft_mode.ENG_SHAFT_LOCK;  //< mode of imposing raints on extra (non-z) degrees of freedom

        protected ChFunction rot_funct_x;  //< rotation(t) function for keyframe polar motor
        protected ChFunction rot_funct_y;  //< rotation(t) function for keyframe polar motor
        protected double last_r3time;               //< internal:for backward differentiation to compute speed in keyframe mode
        protected double last_r3mot_rot;            //< internal:for backward differentiation to compute speed in keyframe mode
        protected double last_r3mot_rot_dt;         //< internal:for backward differentiation to compute speed in keyframe mode
        protected  ChQuaternion last_r3relm_rot = new  ChQuaternion(0, 0, 0, 0);       //< internal:for backward differentiation to compute speed in keyframe mode
        protected  ChQuaternion last_r3relm_rot_dt = new  ChQuaternion(0, 0, 0, 0);    //< internal
        protected  ChQuaternion keyed_polar_rotation = new  ChQuaternion(0, 0, 0, 0);  //< internal

        protected ChShaft innershaft1;// = new ChShaft();            //< used in ENG_MODE_TO_POWERTRAIN_SHAFT
        protected ChShaft innershaft2;// = new ChShaft();            //< used in ENG_MODE_TO_POWERTRAIN_SHAFT
        protected ChShaftsBody innerconstraint1;// = new ChShaftsBody();  //< used in ENG_MODE_TO_POWERTRAIN_SHAFT
        protected ChShaftsBody innerconstraint2;// = new ChShaftsBody();  //< used in ENG_MODE_TO_POWERTRAIN_SHAFT
        protected double torque_react1;
        protected double torque_react2;




        public ChLinkEngine()
        {
            mot_rot = 0;
            mot_rot_dt = 0;
            mot_rot_dtdt = 0;
            mot_rerot = 0;
            mot_rerot_dt = 0;
            mot_rerot_dtdt = 0;
            mot_torque = 0;
            mot_retorque = 0;
            last_r3mot_rot = 0;
            last_r3mot_rot_dt = 0;
            last_r3relm_rot = new ChQuaternion(0, 0, 0, 0);// ChQuaternion.QUNIT;
            last_r3relm_rot_dt = new ChQuaternion(0, 0, 0, 0);//ChQuaternion.QNULL;
            last_r3time = 0;
            keyed_polar_rotation = new ChQuaternion(0, 0, 0, 0);// ChQuaternion.QNULL;
            impose_reducer = false;
            mot_tau = 1;
            mot_eta = 1;
            mot_inertia = 0;
            torque_react2 = 0;
           // eng_mode = eng_mode;
            learn = false;
            rot_funct = new ChFunction_Const(0);
            spe_funct = new ChFunction_Const(0);
            tor_funct = new ChFunction_Const(0);
            torque_w = new ChFunction_Const(1);

            rot_funct_x = new ChFunction_Const(0);
            rot_funct_y = new ChFunction_Const(0);

            // Mask: initialize our LinkMaskLF (lock formulation mask)
            // to E3 only.
            ChLinkMaskLF masklf = new ChLinkMaskLF();
            mask = masklf;
            ((ChLinkMaskLF)mask).SetLockMask(true, false, false, false, false, false, true);
            ChangedLinkMask();

            // Mask: initialize remaining LinkMaskLF (lock formulation mask) for the engine.
            // All shaft modes at least are setting the lock on E3 (z-rotation) coordinate.
            Set_shaft_mode( eCh_shaft_mode.ENG_SHAFT_LOCK);
        }

        public ChLinkEngine(ChLinkEngine other) { 
        
        }

        public new void Start()
        {
            innerconstraint1 = gameObject.AddComponent<ChShaftsBody>();
            innerconstraint2 = gameObject.AddComponent<ChShaftsBody>();
            innershaft1 = gameObject.AddComponent<ChShaft>();
            innershaft2 = gameObject.AddComponent<ChShaft>();

            ChCoordsys<double> pos = new ChCoordsys<double>();
            pos.pos.x = transform.position.x;
            pos.pos.y = transform.position.y;
            pos.pos.z = transform.position.z;
            pos.rot.e0 = transform.rotation.w;
            pos.rot.e1 = transform.rotation.x;
            pos.rot.e2 = transform.rotation.y;
            pos.rot.e3 = transform.rotation.z;
            //ChFrame<double> frame = new ChFrame<double>(ToChrono(transform.position), ToChrono(transform.rotation));
            Initialize(body1, body2, pos);
            Set_shaft_mode(shaft_mode);
            Set_eng_mode(eng_mode);

            // Get a handle to the associated function component and set the motor's function
            var fun_component = this.GetComponent<ChFunction>();
            if (fun_component != null)
            {
                Set_spe_funct(fun_component);
            }

            ChSystem msystem = FindObjectOfType<ChSystem>();
            msystem.AddLink(this);
        }



        /// "Virtual" copy ructor (covariant return type).
        public override ChObj Clone() { return new ChLinkEngine(this); }

        public void FixedUpdate()
        {
           // var frame = this.GetLinkAbsoluteCoords();
           // transform.position = FromChrono(frame.GetPos());
           // transform.rotation = FromChrono(frame.GetRot());
        }

        public static ChVector ToChrono(Vector3 v)
        {
            return new ChVector(v.x, v.y, v.z);
        }

        public static  ChQuaternion ToChrono(UnityEngine.Quaternion q)
        {
            return new  ChQuaternion(q.w, q.x, q.y, q.z);
        }

        public static Vector3 FromChrono(ChVector v)
        {
            return new Vector3((float)v.x, (float)v.y, (float)v.z);
        }
        public static UnityEngine.Quaternion FromChrono( ChQuaternion q)
        {
            return new UnityEngine.Quaternion((float)q.e1, (float)q.e2, (float)q.e3, (float)q.e0);
        }

        /// Updates motion laws, etc. for the impose rotation / impose speed modes
        public override void UpdateTime(double mytime) {
            // First, inherit to parent class
            base.UpdateTime(mytime);

            if (!IsActive())
                return;

            // DEFAULTS compute rotation vars...
            // by default for torque control..

            motion_axis = ChVector.VECT_Z;  // motion axis is always the marker2 Z axis (in m2 relative coords)
            mot_rot = relAngle;
            mot_rot_dt = ChVector.Vdot(relWvel, motion_axis);
            mot_rot_dtdt = ChVector.Vdot(relWacc, motion_axis);
            mot_rerot = mot_rot / mot_tau;
            mot_rerot_dt = mot_rot_dt / mot_tau;
            mot_rerot_dtdt = mot_rot_dtdt / mot_tau;

            // nothing more to do here for torque control
            if (eng_mode == eCh_eng_mode.ENG_MODE_TORQUE)
                return;

            // If LEARN MODE, just record motion
            if (learn)
            {
                deltaC.pos = new ChVector(0, 0, 0);
                deltaC_dt.pos = new ChVector(0, 0, 0);
                deltaC_dtdt.pos = new ChVector(0, 0, 0);
                if (!(limit_Rx.Get_active() || limit_Ry.Get_active() || limit_Rz.Get_active()))
                {
                    deltaC.rot = new ChQuaternion(0, 0, 0, 0);// ChQuaternion.QUNIT;
                    deltaC_dt.rot = new ChQuaternion(0, 0, 0, 0); //ChQuaternion.QNULL;
                    deltaC_dtdt.rot = new ChQuaternion(0, 0, 0, 0);// ChQuaternion.QNULL;
                }

                if (eng_mode == eCh_eng_mode.ENG_MODE_ROTATION)
                {
                    if (rot_funct.Get_Type() != ChFunction.FunctionType.FUNCT_RECORDER)
                        rot_funct = new ChFunction_Recorder();

                    // record point
                    double rec_rot = relAngle;  // ***TO DO*** compute also rotations with cardano mode?
                    if (impose_reducer)
                        rec_rot = rec_rot / mot_tau;
                    ChFunction_Recorder rec = (ChFunction_Recorder)rot_funct;
                    rec.AddPoint(mytime, rec_rot, 1);  // x=t
                }

                if (eng_mode == eCh_eng_mode.ENG_MODE_SPEED)
                {
                    if (spe_funct.Get_Type() != ChFunction.FunctionType.FUNCT_RECORDER)
                        spe_funct = new ChFunction_Recorder();

                    // record point
                    double rec_spe = ChVector.Vlength(relWvel);  // ***TO DO*** compute also with cardano mode?
                    if (impose_reducer)
                        rec_spe = rec_spe / mot_tau;
                    ChFunction_Recorder rec = (ChFunction_Recorder)spe_funct;
                    rec.AddPoint(mytime, rec_spe, 1);  //  x=t
                }
            }

            if (learn)
                return;  // no need to go on further...--.>>>

            // Impose relative positions/speeds

            deltaC.pos = new ChVector(0, 0, 0);
            deltaC_dt.pos = new ChVector(0, 0, 0);
            deltaC_dtdt.pos = new ChVector(0, 0, 0);

            if (eng_mode == eCh_eng_mode.ENG_MODE_ROTATION)
            {
                if (impose_reducer)
                {
                    mot_rerot = rot_funct.Get_y(ChTime);
                    mot_rerot_dt = rot_funct.Get_y_dx(ChTime);
                    mot_rerot_dtdt = rot_funct.Get_y_dxdx(ChTime);
                    mot_rot = mot_rerot * mot_tau;
                    mot_rot_dt = mot_rerot_dt * mot_tau;
                    mot_rot_dtdt = mot_rerot_dtdt * mot_tau;
                }
                else
                {
                    mot_rot = rot_funct.Get_y(ChTime);
                    mot_rot_dt = rot_funct.Get_y_dx(ChTime);
                    mot_rot_dtdt = rot_funct.Get_y_dxdx(ChTime);
                    mot_rerot = mot_rot / mot_tau;
                    mot_rerot_dt = mot_rot_dt / mot_tau;
                    mot_rerot_dtdt = mot_rot_dtdt / mot_tau;
                }
                deltaC.rot =  ChQuaternion.Q_from_AngAxis2(mot_rot, motion_axis);
                deltaC_dt.rot =  ChQuaternion.Qdt_from_AngAxis(deltaC.rot, mot_rot_dt, motion_axis);
                deltaC_dtdt.rot =  ChQuaternion.Qdtdt_from_AngAxis(mot_rot_dtdt, motion_axis, deltaC.rot, deltaC_dt.rot);
            }

            if (eng_mode == eCh_eng_mode.ENG_MODE_SPEED)
            {
                if (impose_reducer)
                {
                    mot_rerot_dt = spe_funct.Get_y(ChTime);
                    mot_rerot_dtdt = spe_funct.Get_y_dx(ChTime);
                    mot_rot_dt = mot_rerot_dt * mot_tau;
                    mot_rot_dtdt = mot_rerot_dtdt * mot_tau;
                }
                else
                {
                    mot_rot_dt = spe_funct.Get_y(ChTime);
                    mot_rot_dtdt = spe_funct.Get_y_dx(ChTime);
                    mot_rerot_dt = mot_rot_dt / mot_tau;
                    mot_rerot_dtdt = mot_rot_dtdt / mot_tau;
                }
                deltaC.rot = ChQuaternion.Qnorm(GetRelM().rot);  // just keep current position, -assume always good after integration-.
                ChMatrix33<double> relA = new ChMatrix33<double>();
                relA.Set_A_quaternion(GetRelM().rot);  // ..but adjust to keep Z axis aligned to shaft, anyway!
                ChVector displaced_z_axis = relA.Get_A_Zaxis();
                ChVector adjustment = ChVector.Vcross(displaced_z_axis, ChVector.VECT_Z);
                deltaC.rot = ChQuaternion.Q_from_AngAxis2(ChVector.Vlength(adjustment), ChVector.Vnorm(adjustment)) % deltaC.rot;
                deltaC_dt.rot = ChQuaternion.Qdt_from_AngAxis(deltaC.rot, mot_rot_dt, motion_axis);
                deltaC_dtdt.rot = ChQuaternion.Qdtdt_from_AngAxis(mot_rot_dtdt, motion_axis, deltaC.rot, deltaC_dt.rot);
            }
        }
        /// Updates torque for the impose torque mode
        public override void UpdateForces(double mytime) {
            // First, inherit to parent class
            base.UpdateForces(mytime);

           // Debug.Log("const2 " + this.constraints.Count);

            if (!IsActive())
                return;

            // DEFAULTS set null torques
            mot_torque = 0;
            mot_retorque = 0;

            if (eng_mode == eCh_eng_mode.ENG_MODE_TORQUE)
            {
                // in torque mode, apply the torque vector to both m1 and m2
                //  -  M= f(t)
                double my_torque = Get_tor_funct().Get_y(ChTime);

                if (impose_reducer)
                {
                    my_torque = my_torque * Get_torque_w_funct().Get_y(mot_rerot_dt);
                    mot_retorque = my_torque;
                    mot_torque = mot_retorque * (mot_eta / mot_tau);
                }
                else
                {
                    my_torque = my_torque * Get_torque_w_funct().Get_y(mot_rot_dt);
                    mot_torque = my_torque;
                    mot_retorque = mot_retorque * (mot_tau / mot_eta);
                }

                ChVector mv_torque = ChVector.Vmul(motion_axis, mot_torque);

                // +++ADD TO LINK TORQUE VECTOR
                C_torque = ChVector.Vadd(C_torque, mv_torque);
            }

            if ((eng_mode == eCh_eng_mode.ENG_MODE_ROTATION) || (eng_mode == eCh_eng_mode.ENG_MODE_SPEED) || (eng_mode == eCh_eng_mode.ENG_MODE_KEY_ROTATION))
            {
                mot_torque = react_torque.z;
                mot_retorque = mot_torque * (mot_tau / mot_eta) + mot_rerot_dtdt * mot_inertia;
            }

            if (eng_mode == eCh_eng_mode.ENG_MODE_SPEED)
            {
                // trick: zeroes Z rotat. violation to tell that rot.position is always ok
                if (C.GetRows() != 0)
                    C.SetElement(C.GetRows() - 1, 0, 0.0);
            }
        }
        /// Updates the r3d time, so perform differentiation for computing speed in case of keyframed motion

        public override void UpdatedExternalTime(double prevtime, double time) {
            last_r3time = ChTime;
            last_r3mot_rot = Get_mot_rot();
            last_r3mot_rot_dt = Get_mot_rot_dt();
            last_r3relm_rot = GetRelM().rot;
            last_r3relm_rot_dt = GetRelM_dt().rot;
        }

        /// Sets up the markers associated with the engine link
        public override void SetUpMarkers(ChMarker mark1, ChMarker mark2) {
            base.SetUpMarkers(mark1, mark2);

            if (Body1 != null && Body2 != null)
            {
                // Note: we wrap Body1 and Body2 in shared_ptr with custom no-op destructors
                // so that the two objects are not destroyed when these shared_ptr go out of
                // scope (since Body1 and Body2 are still managed through other shared_ptr).
                ChBodyFrame b1 = new ChBodyFrame(Body1);
                ChBodyFrame b2 = new ChBodyFrame(Body2);
                if (innerconstraint1)
                    innerconstraint1.Initialize(innershaft1, b1, ChVector.VECT_Z);
                if (innerconstraint2)
                    innerconstraint2.Initialize(innershaft2, b2, ChVector.VECT_Z);
            }
        }

        // data get/set
        public ChFunction Get_rot_funct() { return rot_funct; }
        public ChFunction Get_spe_funct() { return spe_funct; }
        public ChFunction Get_tor_funct() { return tor_funct; }
        public ChFunction Get_torque_w_funct() { return torque_w; }

        public void Set_rot_funct(ChFunction mf) { rot_funct = mf; }
        public void Set_spe_funct(ChFunction mf) { spe_funct = mf; }
        public void Set_tor_funct(ChFunction mf) { tor_funct = mf; }
        public void Set_torque_w_funct(ChFunction mf) { torque_w = mf; }

        public ChFunction Get_rot_funct_x() { return rot_funct_x; }
        public ChFunction Get_rot_funct_y() { return rot_funct_y; }
        public  ChQuaternion GetKeyedPolarRotation() { return keyed_polar_rotation; }

        public void Set_rot_funct_x(ChFunction mf) { rot_funct_x = mf; }
        public void Set_rot_funct_y(ChFunction mf) { rot_funct_y = mf; }
        public void SetKeyedPolarRotation(ChQuaternion mq) { keyed_polar_rotation = mq; }

        public bool Get_learn() { return learn; }
        public bool Get_impose_reducer() { return impose_reducer; }

        public void Set_learn(bool mset) {
            learn = mset;

            if ((eng_mode == eCh_eng_mode.ENG_MODE_ROTATION) || (eng_mode == eCh_eng_mode.ENG_MODE_SPEED) || (eng_mode == eCh_eng_mode.ENG_MODE_KEY_ROTATION))
            {
                if (mset)
                    ((ChLinkMaskLF)mask).Constr_E3().SetMode(eChConstraintMode.CONSTRAINT_FREE);
                else
                    ((ChLinkMaskLF)mask).Constr_E3().SetMode(eChConstraintMode.CONSTRAINT_LOCK);

                ChangedLinkMask();
            }

            if (eng_mode == eCh_eng_mode.ENG_MODE_KEY_POLAR)
            {
                if (mset)
                {
                    ((ChLinkMaskLF)mask).Constr_E1().SetMode(eChConstraintMode.CONSTRAINT_FREE);
                    ((ChLinkMaskLF)mask).Constr_E2().SetMode(eChConstraintMode.CONSTRAINT_FREE);
                    ((ChLinkMaskLF)mask).Constr_E3().SetMode(eChConstraintMode.CONSTRAINT_FREE);
                }
                else
                {
                    ((ChLinkMaskLF)mask).Constr_E1().SetMode(eChConstraintMode.CONSTRAINT_LOCK);
                    ((ChLinkMaskLF)mask).Constr_E2().SetMode(eChConstraintMode.CONSTRAINT_LOCK);
                    ((ChLinkMaskLF)mask).Constr_E3().SetMode(eChConstraintMode.CONSTRAINT_LOCK);
                }
                ChangedLinkMask();
            }

            if (eng_mode == eCh_eng_mode.ENG_MODE_ROTATION && rot_funct.Get_Type() != ChFunction.FunctionType.FUNCT_RECORDER)
                rot_funct = new ChFunction_Recorder();

            if (eng_mode == eCh_eng_mode.ENG_MODE_SPEED && spe_funct.Get_Type() != ChFunction.FunctionType.FUNCT_RECORDER)
                spe_funct = new ChFunction_Recorder();
        }
        public void Set_impose_reducer(bool mset) { impose_reducer = mset; }

        public eCh_eng_mode Get_eng_mode() { return eng_mode; }

        public void Set_eng_mode(eCh_eng_mode mset) {
            if (Get_learn())
                Set_learn(false);  // reset learn state when changing mode

            if (eng_mode != mset)
            {
                eng_mode = mset;

                switch (eng_mode)
                {
                    case eCh_eng_mode.ENG_MODE_ROTATION:
                    case eCh_eng_mode.ENG_MODE_SPEED:
                    case eCh_eng_mode.ENG_MODE_KEY_ROTATION:
                        ((ChLinkMaskLF)mask).Constr_E3().SetMode(eChConstraintMode.CONSTRAINT_LOCK);
                        break;
                    case eCh_eng_mode.ENG_MODE_KEY_POLAR:
                        ((ChLinkMaskLF)mask).Constr_E1().SetMode(eChConstraintMode.CONSTRAINT_LOCK);
                        ((ChLinkMaskLF)mask).Constr_E2().SetMode(eChConstraintMode.CONSTRAINT_LOCK);
                        ((ChLinkMaskLF)mask).Constr_E3().SetMode(eChConstraintMode.CONSTRAINT_LOCK);
                        break;
                    case eCh_eng_mode.ENG_MODE_TORQUE:
                        ((ChLinkMaskLF)mask).Constr_E3().SetMode(eChConstraintMode.CONSTRAINT_FREE);
                        break;
                    case eCh_eng_mode.ENG_MODE_TO_POWERTRAIN_SHAFT:
                        ((ChLinkMaskLF)mask).Constr_E3().SetMode(eChConstraintMode.CONSTRAINT_FREE);
                        innershaft1 = new ChShaft();
                        innershaft2 = new ChShaft();
                        innerconstraint1 = new ChShaftsBody();
                        innerconstraint2 = new ChShaftsBody();
                        SetUpMarkers(marker1, marker2);  // to initialize innerconstraint1 innerconstraint2
                        break;
                }

                ChangedLinkMask();  // update all from new mask
            }

            if (eng_mode == eCh_eng_mode.ENG_MODE_KEY_ROTATION && rot_funct.Get_Type() != ChFunction.FunctionType.FUNCT_CONST)
                rot_funct = new ChFunction_Const();

            if (eng_mode == eCh_eng_mode.ENG_MODE_KEY_POLAR)
            {
                if (rot_funct.Get_Type() != ChFunction.FunctionType.FUNCT_CONST)
                    rot_funct = new ChFunction_Const();
                if (rot_funct_x.Get_Type() != ChFunction.FunctionType.FUNCT_CONST)
                    rot_funct_x = new ChFunction_Const();
                if (rot_funct_y.Get_Type() != ChFunction.FunctionType.FUNCT_CONST)
                    rot_funct_y = new ChFunction_Const();
            }
        }

        public eCh_shaft_mode Get_shaft_mode() { return shaft_mode; }

        public void Set_shaft_mode(eCh_shaft_mode mset)
        {
            shaft_mode = mset;

            eChConstraintMode curr_mode_z = ((ChLinkMaskLF)mask).Constr_E3().GetMode();

            switch (shaft_mode)
            {
                case eCh_shaft_mode.ENG_SHAFT_PRISM:
                    ((ChLinkMaskLF)mask).SetLockMask(true, true, false, false, true, true, true);
                    break;
                case eCh_shaft_mode.ENG_SHAFT_UNIVERSAL:
                    ((ChLinkMaskLF)mask).SetLockMask(true, true, true, false, false, false, true);
                    break;
                case eCh_shaft_mode.ENG_SHAFT_CARDANO:
                    ((ChLinkMaskLF)mask).SetLockMask(false, false, false, false, false, false, true);
                    break;
                case eCh_shaft_mode.ENG_SHAFT_OLDHAM:
                    ((ChLinkMaskLF)mask).SetLockMask(false, false, false, false, true, true, true);
                    break;
                case eCh_shaft_mode.ENG_SHAFT_LOCK:
                default:
                    ((ChLinkMaskLF)mask).SetLockMask(true, true, true, false, true, true, true);
                    break;
            }

            ((ChLinkMaskLF)mask).Constr_E3().SetMode(curr_mode_z);

            // change data
            ChangedLinkMask();
        }

        public double Get_mot_rot() { return mot_rot; }
        public double Get_mot_rot_dt()  { return mot_rot_dt; }
        public double Get_mot_rot_dtdt()  { return mot_rot_dtdt; }
        public double Get_mot_torque()  { return mot_torque; }
        public double Get_mot_rerot()  { return mot_rerot; }
        public double Get_mot_rerot_dt()  { return mot_rerot_dt; }
        public double Get_mot_rerot_dtdt()  { return mot_rerot_dtdt; }
        public double Get_mot_retorque()  { return mot_retorque; }
        public double Get_mot_tau()  { return mot_tau; }
        public double Get_mot_eta()  { return mot_eta; }
        public double Get_mot_inertia()  { return mot_inertia; }

        public void Set_mot_tau(double mtau) { mot_tau = mtau; }
        public void Set_mot_eta(double meta) { mot_eta = meta; }
        public void Set_mot_inertia(double min) { mot_inertia = min; }

        // Access the inner 1D shaft connected to the rotation of body1 about dir of motor shaft,
        // if in CH_ENG_MODE_TO_POWERTRAIN_SHAFT. The shaft can be
        // connected to other shafts with ChShaftsClutch or similar items.
        public ChShaft GetInnerShaft1()  { return innershaft1; }
        // Access the inner 1D shaft connected to the rotation of body2 about dir of motor shaft,
        // if in CH_ENG_MODE_TO_POWERTRAIN_SHAFT. The shaft can be
        // connected to other shafts with ChShaftsClutch or similar items.
        public ChShaft GetInnerShaft2()  { return innershaft2; }
        // Get the torque between body 1 and inner shaft 1.
        // Note: use only if in CH_ENG_MODE_TO_POWERTRAIN_SHAFT.
        public double GetInnerTorque1()  { return torque_react1; }
        // Get the torque between body 2 and inner shaft 2.
        // Note: use only if in CH_ENG_MODE_TO_POWERTRAIN_SHAFT.
        public double GetInnerTorque2()  { return torque_react2; }

        //
        // STATE FUNCTIONS
        //

        public override int GetDOF() {
            if (eng_mode == eCh_eng_mode.ENG_MODE_TO_POWERTRAIN_SHAFT)
            {                
                return (2 + base.GetDOF());
            }
            return base.GetDOF();
        }
        public override int GetDOC_c() {
            if (eng_mode == eCh_eng_mode.ENG_MODE_TO_POWERTRAIN_SHAFT)
            {
                return (2 + base.GetDOC_c());
            }
            return base.GetDOC_c();
        }

        // (override/implement interfaces for global state vectors, see ChPhysicsItem for comments.)
        // (beyond the base link implementations, it also have to
        // add the raint coming from the inner shaft etc.)
        public override void IntStateGather(int off_x,
                                ref ChState x,
                                int off_v,
                                ref ChStateDelta v,
                                ref double T) {
            // First, inherit to parent class
            base.IntStateGather(off_x, ref x, off_v, ref v, ref T);

            if (eng_mode == eCh_eng_mode.ENG_MODE_TO_POWERTRAIN_SHAFT)
            {
                innershaft1.IntStateGather(off_x + 0, ref x, off_v + 0, ref v, ref T);
                innershaft2.IntStateGather(off_x + 1,ref  x, off_v + 1, ref v, ref T);
            }
        }
        public override void IntStateScatter(int off_x,
                                  ChState x,
                                  int off_v,
                                  ChStateDelta v,
                                  double T) {
            // First, inherit to parent class
            base.IntStateScatter(off_x, x, off_v, v, T);

            if (eng_mode == eCh_eng_mode.ENG_MODE_TO_POWERTRAIN_SHAFT)
            {
                innershaft1.IntStateScatter(off_x + 0, x, off_v + 0, v, T);
                innershaft2.IntStateScatter(off_x + 1, x, off_v + 1, v, T);
            }
        }
        public override void IntStateGatherAcceleration(int off_a, ref ChStateDelta a) {
            // First, inherit to parent class
            base.IntStateGatherAcceleration(off_a, ref a);

            if (eng_mode == eCh_eng_mode.ENG_MODE_TO_POWERTRAIN_SHAFT)
            {
                innershaft1.IntStateScatterAcceleration(off_a + 0, a);
                innershaft2.IntStateScatterAcceleration(off_a + 1, a);
            }
        }
        public override void IntStateScatterAcceleration(int off_a, ChStateDelta a) {
            // First, inherit to parent class
            base.IntStateScatterAcceleration(off_a, a);

            if (eng_mode == eCh_eng_mode.ENG_MODE_TO_POWERTRAIN_SHAFT)
            {
                innershaft1.IntStateScatterAcceleration(off_a + 0, a);
                innershaft2.IntStateScatterAcceleration(off_a + 1, a);
            }
        }
        public override void IntStateIncrement(int off_x,
                                    ref ChState x_new,
                                    ChState x,
                                    int off_v,
                                    ChStateDelta Dv) {
            // First, inherit to parent class
            base.IntStateIncrement(off_x, ref x_new, x, off_v, Dv);

            if (eng_mode == eCh_eng_mode.ENG_MODE_TO_POWERTRAIN_SHAFT)
            {
                innershaft1.IntStateIncrement(off_x + 0, ref x_new, x, off_v + 0, Dv);
                innershaft2.IntStateIncrement(off_x + 1, ref x_new, x, off_v + 1, Dv);
            }
        }
        public override void IntStateGatherReactions(int off_L, ref ChVectorDynamic<double> L) {
            // First, inherit to parent class
            base.IntStateGatherReactions(off_L, ref L);

            if (eng_mode == eCh_eng_mode.ENG_MODE_TO_POWERTRAIN_SHAFT)
            {
                innerconstraint1.IntStateGatherReactions(off_L + 0, ref L);
                innerconstraint2.IntStateGatherReactions(off_L + 1, ref L);
            }
        }
        public override void IntStateScatterReactions(int off_L, ChVectorDynamic<double> L) {
            // First, inherit to parent class
            base.IntStateScatterReactions(off_L, L);

            if (eng_mode == eCh_eng_mode.ENG_MODE_TO_POWERTRAIN_SHAFT)
            {
                innerconstraint1.IntStateScatterReactions(off_L + 0, L);
                innerconstraint2.IntStateScatterReactions(off_L + 1, L);
            }
        }
        public override void IntLoadResidual_F(int off, ref ChVectorDynamic<double> R, double c) {
            // First, inherit to parent class
            base.IntLoadResidual_F(off, ref R, c);

            if (eng_mode == eCh_eng_mode.ENG_MODE_TO_POWERTRAIN_SHAFT)
            {
                innershaft1.IntLoadResidual_F(off + 0, ref R, c);
                innershaft2.IntLoadResidual_F(off + 1, ref R, c);
            }
        }
        public override void IntLoadResidual_Mv(int off,
                                    ref ChVectorDynamic<double> R,
                                     ChVectorDynamic<double> w,
                                     double c) {
            // First, inherit to parent class
            base.IntLoadResidual_Mv(off, ref R, w, c);

            if (eng_mode == eCh_eng_mode.ENG_MODE_TO_POWERTRAIN_SHAFT)
            {
                innershaft1.IntLoadResidual_Mv(off + 0, ref R, w, c);
                innershaft2.IntLoadResidual_Mv(off + 1, ref R, w, c);
            }
        }
        public override void IntLoadResidual_CqL(int off_L,
                                     ref ChVectorDynamic<double> R,
                                      ChVectorDynamic<double> L,
                                      double c) {
            // First, inherit to parent class
            base.IntLoadResidual_CqL(off_L, ref R, L, c);

            if (eng_mode == eCh_eng_mode.ENG_MODE_TO_POWERTRAIN_SHAFT)
            {
                innerconstraint1.IntLoadResidual_CqL(off_L, ref R, L, c);
                innerconstraint2.IntLoadResidual_CqL(off_L + 1, ref R, L, c);
            }
        }
        public override void IntLoadConstraint_C(int off_L,
                                     ref ChVectorDynamic<double> Qc,
                                      double c,
                                     bool do_clamp,
                                     double recovery_clamp) {
            // First, inherit to parent class
            base.IntLoadConstraint_C(off_L, ref Qc, c, do_clamp, recovery_clamp);

            if (eng_mode == eCh_eng_mode.ENG_MODE_TO_POWERTRAIN_SHAFT)
            {
                innerconstraint1.IntLoadConstraint_C(off_L, ref Qc, c, do_clamp, recovery_clamp);
                innerconstraint2.IntLoadConstraint_C(off_L + 1, ref Qc, c, do_clamp, recovery_clamp);
            }
        }
        public override void IntLoadConstraint_Ct(int off_L, ref ChVectorDynamic<double> Qc, double c) {
            // First, inherit to parent class
            base.IntLoadConstraint_Ct(off_L, ref Qc, c);

            if (eng_mode == eCh_eng_mode.ENG_MODE_TO_POWERTRAIN_SHAFT)
            {
                innerconstraint1.IntLoadConstraint_Ct(off_L, ref Qc, c);
                innerconstraint2.IntLoadConstraint_Ct(off_L + 1, ref Qc, c);
            }
        }
        public override void IntToDescriptor(int off_v,
                                  ChStateDelta v,
                                  ChVectorDynamic<double> R,
                                  int off_L,
                                  ChVectorDynamic<double> L,
                                  ChVectorDynamic<double> Qc) {
            // First, inherit to parent class
            base.IntToDescriptor(off_v, v, R, off_L, L, Qc);

            if (eng_mode == eCh_eng_mode.ENG_MODE_TO_POWERTRAIN_SHAFT)
            {
                innershaft1.IntToDescriptor(off_v, v, R, off_L, L, Qc);
                innershaft2.IntToDescriptor(off_v + 1, v, R, off_L, L, Qc);
                innerconstraint1.IntToDescriptor(off_v, v, R, off_L, L, Qc);
                innerconstraint2.IntToDescriptor(off_v, v, R, off_L + 1, L, Qc);
            }
        }
        public override void IntFromDescriptor(int off_v,
                                   ref ChStateDelta v,
                                   int off_L,
                                   ref ChVectorDynamic<double> L) {
            // First, inherit to parent class
            base.IntFromDescriptor(off_v, ref v, off_L, ref L);

            if (eng_mode == eCh_eng_mode.ENG_MODE_TO_POWERTRAIN_SHAFT)
            {
                innershaft1.IntFromDescriptor(off_v, ref v, off_L, ref L);
                innershaft2.IntFromDescriptor(off_v + 1, ref v, off_L, ref L);
                innerconstraint1.IntFromDescriptor(off_v, ref v, off_L, ref L);
                innerconstraint2.IntFromDescriptor(off_v, ref v, off_L + 1, ref L);
            }
        }

        //
        // SOLVER INTERFACE
        //

        // Overload system functions of ChPhysicsItem
        // (besides the base link implementations, these also add
        // the raint coming from the inner shaft etc.)
        public override void InjectConstraints(ref ChSystemDescriptor mdescriptor) {
            // First, inherit to parent class
            base.InjectConstraints(ref mdescriptor);

            if (eng_mode == eCh_eng_mode.ENG_MODE_TO_POWERTRAIN_SHAFT)
            {
                innerconstraint1.InjectConstraints(ref mdescriptor);
                innerconstraint2.InjectConstraints(ref mdescriptor);
            }
        }

        public override void ConstraintsBiReset() {
            // First, inherit to parent class
            base.ConstraintsBiReset();

            if (eng_mode == eCh_eng_mode.ENG_MODE_TO_POWERTRAIN_SHAFT)
            {
                innerconstraint1.ConstraintsBiReset();
                innerconstraint2.ConstraintsBiReset();
            }
        }

        public override void ConstraintsBiLoad_C(double factor = 1, double recovery_clamp = 0.1, bool do_clamp = false) {
            // First, inherit to parent class
            base.ConstraintsBiLoad_C(factor, recovery_clamp, do_clamp);

            if (eng_mode == eCh_eng_mode.ENG_MODE_TO_POWERTRAIN_SHAFT)
            {
                innerconstraint1.ConstraintsBiLoad_C(factor, recovery_clamp, do_clamp);
                innerconstraint2.ConstraintsBiLoad_C(factor, recovery_clamp, do_clamp);
            }
        }

        public override void ConstraintsBiLoad_Ct(double factor = 1) {
            // First, inherit to parent class
            base.ConstraintsBiLoad_Ct(factor);

            if (eng_mode == eCh_eng_mode.ENG_MODE_TO_POWERTRAIN_SHAFT)
            {
                // nothing
            }
        }

        public override void ConstraintsLoadJacobians() {
            // First, inherit to parent class
            base.ConstraintsLoadJacobians();

            if (eng_mode == eCh_eng_mode.ENG_MODE_TO_POWERTRAIN_SHAFT)
            {
                innerconstraint1.ConstraintsLoadJacobians();
                innerconstraint2.ConstraintsLoadJacobians();
            }
        }
        public override void ConstraintsFetch_react(double factor = 1) {
            // First, inherit to parent class
            base.ConstraintsFetch_react(factor);

            if (eng_mode == eCh_eng_mode.ENG_MODE_TO_POWERTRAIN_SHAFT)
            {
                innerconstraint1.ConstraintsFetch_react(factor);
                innerconstraint2.ConstraintsFetch_react(factor);
            }
        }
        public override void InjectVariables(ref ChSystemDescriptor mdescriptor) {
            // First, inherit to parent class
            base.InjectVariables(ref mdescriptor);

            if (eng_mode == eCh_eng_mode.ENG_MODE_TO_POWERTRAIN_SHAFT)
            {
                innershaft1.InjectVariables(ref mdescriptor);
                innershaft2.InjectVariables(ref mdescriptor);
            }
        }
        public override void VariablesFbReset() {
            // First, inherit to parent class
            base.VariablesFbReset();

            if (eng_mode == eCh_eng_mode.ENG_MODE_TO_POWERTRAIN_SHAFT)
            {
                innershaft1.VariablesFbReset();
                innershaft2.VariablesFbReset();
            }
        }
        public override void VariablesFbLoadForces(double factor = 1) {
            // First, inherit to parent class
            base.VariablesFbLoadForces(factor);

            if (eng_mode == eCh_eng_mode.ENG_MODE_TO_POWERTRAIN_SHAFT)
            {
                innershaft1.VariablesFbLoadForces(factor);
                innershaft2.VariablesFbLoadForces(factor);
            }
        }
        public override void VariablesQbLoadSpeed() {
            // First, inherit to parent class
            base.VariablesQbLoadSpeed();

            if (eng_mode == eCh_eng_mode.ENG_MODE_TO_POWERTRAIN_SHAFT)
            {
                innershaft1.VariablesQbLoadSpeed();
                innershaft2.VariablesQbLoadSpeed();
            }
        }
        public override void VariablesFbIncrementMq() {
            // inherit parent class
            base.VariablesFbIncrementMq();

            if (eng_mode == eCh_eng_mode.ENG_MODE_TO_POWERTRAIN_SHAFT)
            {
                innershaft1.VariablesFbIncrementMq();
                innershaft2.VariablesFbIncrementMq();
            }
        }
        public override void VariablesQbSetSpeed(double step = 0) {
            // First, inherit to parent class
            base.VariablesQbSetSpeed(step);

            if (eng_mode == eCh_eng_mode.ENG_MODE_TO_POWERTRAIN_SHAFT)
            {
                innershaft1.VariablesQbSetSpeed(step);
                innershaft2.VariablesQbSetSpeed(step);
            }
        }
        public override void VariablesQbIncrementPosition(double step) {
            // First, inherit to parent class
            base.VariablesQbIncrementPosition(step);

            if (eng_mode == eCh_eng_mode.ENG_MODE_TO_POWERTRAIN_SHAFT)
            {
                innershaft1.VariablesQbIncrementPosition(step);
                innershaft2.VariablesQbIncrementPosition(step);
            }
        }

    }
}
