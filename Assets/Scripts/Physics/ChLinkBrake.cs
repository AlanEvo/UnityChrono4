using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono {

    /// Link representing a brake between two rigid bodies,
    /// including the sticking effect.
    /// It could be used to represent also linear brakes.
    /// This constraint can behave also as a clutch.
    ///  ***OBSOLETE*** better add ChLinkEngine in ENG_MODE_TO_POWERTRAIN_SHAFT mode, and add a ChShaftsClutch between
    ///  shafts

    public class ChLinkBrake : ChLinkLock {


        public double brake_torque;  //< applied torque.
        public double stick_ratio;  //< static sticking torque = stick ratio * brake torque (if <1, sticking effect is turned off)

        public ChLinkLock hub;

        public eChBrmode brake_mode = eChBrmode.BRAKE_ROTATION;  //< default works as traditional rotating brake, but can also be linear, on x

        public enum eChBrmode { BRAKE_ROTATION = 0, BRAKE_TRANSLATEX };

        protected int last_dir;     ///< 0= clockwise, 1= anticlockw.  -- internal
        protected bool must_stick;  ///< if true, change DOF mask to add link -- internal


        public ChLinkBrake() {
            brake_torque = 0;
            stick_ratio = 1.1;
            brake_mode = eChBrmode.BRAKE_ROTATION; 
            last_dir = 0;
            must_stick = false;
            // Mask: initialize our LinkMaskLF (lock formulation mask)
            // because this class inherited from LinkLock.
            ((ChLinkMaskLF)mask).SetLockMask(false, false, false, false, false, false, false);

            ChangedLinkMask();
        }
        public ChLinkBrake(ChLinkBrake other) {
            brake_torque = other.brake_torque;
            stick_ratio = other.stick_ratio;
            brake_mode = other.brake_mode;

            last_dir = other.last_dir;
            must_stick = other.must_stick;
        }

        /// "Virtual" copy constructor (covariant return type).
        public override ChObj Clone() { return new ChLinkBrake(this); }

        // Note, always override the Awake on inheritting from LinkLock!
        public override void Awake()
        {
           /* System.Type mbf1 = typeof(ChBody);

            //ChBody mbf1 = new ChBody();
            mbf1.BodyFrame = hub.GetBody1();
            ChBody mbf2 = new ChBody();
            mbf2.BodyFrame = hub.GetBody2();*/

           // Set_brake_mode(brake_mode);

            Initialize(body1, body2, true, hub.GetMarker1().FrameMoving.GetCoord(), hub.GetMarker2().FrameMoving.GetCoord());
            Set_brake_mode(brake_mode);
            ChSystem msystem = FindObjectOfType<ChSystem>();
            msystem.AddLink(this);
            //ChSystem.system.AddLink(this);
        }

        public override void UpdateTime(double time) {
            ChTime = time;
        }

        public override void UpdateForces(double mytime) {
            // First, inherit to parent class
            base.UpdateForces(mytime);

            if (this.IsDisabled())
                return;

            // then, if not sticking,
            if (this.brake_torque != 0)
            {            
                if (brake_mode == eChBrmode.BRAKE_ROTATION)
                {
                    if (((ChLinkMaskLF)mask).Constr_E3().IsActive() == false)
                    {
                        int mdir;

                        ChVector mv_torque = ChVector.Vmul(ChVector.VECT_Z, this.brake_torque);
                        mdir = 0;  // clockwise torque

                        if (ChVector.Vdot(this.relWvel, mv_torque) > 0.0)
                        {                            

                            mv_torque = ChVector.Vmul(mv_torque, -1.0);  // keep torque always opposed to ang speed.
                            mdir = 1;                           // counterclockwise torque
                        }                        

                        if (mdir != this.last_dir)
                            this.must_stick = true;
                        this.last_dir = mdir;

                        // +++ADD TO LINK TORQUE VECTOR
                        C_torque = ChVector.Vadd(C_torque, mv_torque);
                    }
                }
                if (brake_mode == eChBrmode.BRAKE_TRANSLATEX)
                {
                    if (((ChLinkMaskLF)mask).Constr_X().IsActive() == false)
                    {
                        int mdir;

                        ChVector mv_force = ChVector.Vmul(ChVector.VECT_X, this.brake_torque);
                        mdir = 0;  // F-.  rear motion: frontfacing break force

                        if (this.relM_dt.pos.x > 0.0)
                        {
                            mv_force = ChVector.Vmul(mv_force, -1.0);  // break force always opposed to speed
                            mdir = 1;                         // F<-- backfacing breakforce for front motion
                        }

                        if (mdir != this.last_dir)
                            this.must_stick = true;
                        this.last_dir = mdir;

                        // +++ADD TO LINK TORQUE VECTOR
                        C_force = ChVector.Vadd(C_force, mv_force);
                    }
                }
            }

            // turn off sticking feature if stick ration not > 1.0
            if (this.stick_ratio <= 1.0)
                must_stick = false;
        }

        public override void SetDisabled(bool mdis) {
            base.SetDisabled(mdis);

            ((ChLinkMaskLF)mask).Constr_E3().SetMode(eChConstraintMode.CONSTRAINT_FREE);
            ((ChLinkMaskLF)mask).Constr_X().SetMode(eChConstraintMode.CONSTRAINT_FREE);

            ChangedLinkMask();
        }

        public double Get_brake_torque() { return brake_torque; }
        public void Set_brake_torque(double mset) { brake_torque = mset; }
        public double Get_stick_ratio() { return stick_ratio; }
        public void Set_stick_ratio(double mset) { stick_ratio = mset; }
        public eChBrmode Get_brake_mode() { return brake_mode; }
        public void Set_brake_mode(eChBrmode mmode) {
            if (mmode != brake_mode)
            {
                brake_mode = mmode;

                // reset mask for default free brake
                ((ChLinkMaskLF)mask).Constr_E3().SetMode(eChConstraintMode.CONSTRAINT_FREE);
                ((ChLinkMaskLF)mask).Constr_X().SetMode(eChConstraintMode.CONSTRAINT_FREE);

                ChangedLinkMask();
            }
        }
    }

}
