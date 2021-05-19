
using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
using UnityEditor;


namespace chrono
{
    /// Class for linear actuators between two markers,
    /// as the actuator were joined with two spherical
    /// bearing at the origin of the two markers.
    /// **NOTE! THIS IS OBSOLETE**. Prefer using the new classes 
    /// inherited from chrono::ChLinkMotor.
    public class ChLinkLinActuator : ChLinkLock
    {

        protected ChFunction dist_funct = new ChFunction();  //< distance function
        protected bool learn;  //< if true, the actuator does not apply constraint, just records the motion into its dist_function.
        protected bool learn_torque_rotation;  //< if true, the actuator records the torque and rotation.
        public double offset;               /// distance offset
        public bool pos_are_relative;

        protected double mot_tau;                          //< motor: transmission ratio
        protected double mot_eta;                          //< motor: transmission efficiency
        protected double mot_inertia;                      //< motor: inertia (added to system)
        protected ChFunction mot_torque = new ChFunction();  //< motor: recorder of torque
        protected ChFunction mot_rot = new ChFunction();     //< motor: recorder of motor rotation

        protected double mot_rerot;       //< current rotation (read only)  before reducer
        protected double mot_rerot_dt;    //< current ang speed (read only) before reducer
        protected double mot_rerot_dtdt;  //< current ang acc  (read only)  before reducer
        protected double mot_retorque;    //< current motor torque (read only) before reducer

        // TEST
        ChMatrix33<double> ma = new ChMatrix33<double>(0);

        public ChLinkLinActuator()
        {
            learn = false;
            learn_torque_rotation = true;
            offset = 0.1;
            mot_tau = 1;
            mot_eta = 1;
            mot_inertia = 0;
            mot_rerot = 0;
            mot_rerot_dt = 0;
            mot_rerot_dtdt = 0;
            //dist_funct = new ChFunction_Const(0);
          //  mot_torque = new ChFunction_Recorder();
          //  mot_rot = new ChFunction_Recorder();

            // Mask: initialize our LinkMaskLF (lock formulation mask)
            // to X  only. It was a LinkMaskLF because this class inherited from LinkLock.
            ((ChLinkMaskLF)mask).SetLockMask(true, false, false, false, false, false, false);

            ChangedLinkMask();

            mot_rerot = mot_rerot_dt = mot_rerot_dtdt = 0;
        }

        public ChLinkLinActuator(ChLinkLinActuator other)
        {
            learn = other.learn;
            learn_torque_rotation = other.learn_torque_rotation;
            offset = other.offset;

            dist_funct = new ChFunction(other.dist_funct.Clone());
            mot_torque = new ChFunction(other.mot_torque.Clone());
            mot_rot = new ChFunction(other.mot_rot.Clone());

            mot_tau = other.mot_tau;
            mot_eta = other.mot_eta;
            mot_inertia = other.mot_inertia;
        }

        /// "Virtual" copy constructor (covariant return type).
        public override ChObj Clone()
        {
            return new ChLinkLinActuator(this);
        }

        // Note, *always!" override the Awake function inheriting from LinkLock!  Otherwise the LinkLock Awake is executed.
        public override void Awake()
        {
            dist_funct = gameObject.AddComponent<ChFunction_Const>();

            ChVector link_abs = new ChVector(transform.position.x, transform.position.y, transform.position.z);
            ChVector yaxis = new ChVector(0.0, 0.0, 1.0);
            double _offset = 10;
            ChVector pt1 = link_abs;
            ChVector pt2 = link_abs - _offset * yaxis;
            Initialize(body1, body2, pos_are_relative, new ChCoordsys(pt1, ChQuaternion.QUNIT), new ChCoordsys(pt2, ChQuaternion.QUNIT));

            Set_lin_offset(_offset);

            ChSystem.system.AddLink(this);
        }

        // Updates motion laws, marker positions, etc.
        public override void UpdateTime(double mytime)
        {
            // First, inherit to parent class
            base.UpdateTime(mytime);

            // If LEARN MODE, just record motion
            if (learn)
            {
                /*   do not change deltas, in free mode maybe that 'limit on X' changed them
                deltaC.pos = VNULL;
                deltaC_dt.pos = VNULL;
                deltaC_dtdt.pos = VNULL;
                deltaC.rot = QUNIT;
                deltaC_dt.rot = QNULL;
                deltaC_dtdt.rot = QNULL;
                */
               // if (dist_funct.Get_Type() != ChFunction.FunctionType.FUNCT_RECORDER)
                 //   dist_funct = new ChFunction_Recorder();

                // record point
                double rec_dist = ChVector.Vlength(ChVector.Vsub(marker1.GetAbsCoord().pos, marker2.GetAbsCoord().pos));
                rec_dist -= offset;
               // (ChFunction_Recorder)(dist_funct).AddPoint(mytime, rec_dist, 1);  // (x,y,w)  x=t
            }

            // Move (well, rotate...) marker 2 to align it in actuator direction

            // ! Require that the BDF routine of marker won't handle speed and acc.calculus of the moved marker 2!
            marker2.SetMotionType(ChMarker.eChMarkerMotion.M_MOTION_EXTERNAL);

           // ChMatrix33<double> ma = new ChMatrix33<double>(0);
            ma.Set_A_quaternion(marker2.GetAbsCoord().rot);
            ChVector absdist = ChVector.Vsub(marker1.GetAbsCoord().pos, marker2.GetAbsCoord().pos);

            ChVector mx = ChVector.Vnorm(absdist);

            ChVector my = ma.Get_A_Yaxis();
            if (ChVector.Vequal(mx, my))
            {
                if (mx.x == 1.0)
                    my = ChVector.VECT_Y;
                else
                    my = ChVector.VECT_X;
            }
            ChVector mz = ChVector.Vnorm(ChVector.Vcross(mx, my));
            my = ChVector.Vnorm(ChVector.Vcross(mz, mx));

            ma.Set_A_axis(mx, my, mz);

            ChCoordsys newmarkpos;
            ChVector oldpos = marker2.FrameMoving.GetPos();  // backup to avoid numerical err.accumulation
            newmarkpos.pos = marker2.GetAbsCoord().pos;
            newmarkpos.rot = ma.Get_A_quaternion();

            marker2.Impose_Abs_Coord(newmarkpos);  // rotate "main" marker2 into tangent position (may add err.accumulation)
            marker2.FrameMoving.SetPos(oldpos);                // backup to avoid numerical err.accumulation

            if (learn)
                return;  // no need to go on further...--.>>>

            // imposed relative positions/speeds
            deltaC.pos = ChVector.VNULL;
            deltaC.pos.x = dist_funct.Get_y(ChTime) + offset;  // distance is always on M2 'X' axis

            deltaC_dt.pos = ChVector.VNULL;
            deltaC_dt.pos.x = dist_funct.Get_y_dx(ChTime);  // distance speed

            deltaC_dtdt.pos = ChVector.VNULL;
            deltaC_dtdt.pos.x = dist_funct.Get_y_dxdx(ChTime);  // distance acceleration
                                                                // add also the centripetal acceleration if distance vector's rotating,
                                                                // as centripetal acc. of point sliding on a sphere surface.
            ChVector tang_speed = GetRelM_dt().pos;
            tang_speed.x = 0;                       // only z-y coords in relative tang speed vector
            double len_absdist = ChVector.Vlength(absdist);  // don't divide by zero
            if (len_absdist > 1E-6)
                deltaC_dtdt.pos.x -= Math.Pow(ChVector.Vlength(tang_speed), 2) / ChVector.Vlength(absdist);  // An = Adelta -(Vt^2 / r)

            deltaC.rot = ChQuaternion.QUNIT;  // no relative rotations imposed!
            deltaC_dt.rot = ChQuaternion.QNULL;
            deltaC_dtdt.rot = ChQuaternion.QNULL;

            // Compute motor variables
            // double m_rotation;
            // double m_torque;
            mot_rerot = (deltaC.pos.x - offset) / mot_tau;
            mot_rerot_dt = deltaC_dt.pos.x / mot_tau;
            mot_rerot_dtdt = deltaC_dtdt.pos.x / mot_tau;
            mot_retorque = mot_rerot_dtdt * mot_inertia + (react_force.x * mot_tau) / mot_eta;
            //  m_rotation = (deltaC.pos.x() - offset) / mot_tau;
            //  m_torque =  (deltaC_dtdt.pos.x() / mot_tau) * mot_inertia + (react_force.x() * mot_tau) / mot_eta;

            if (learn_torque_rotation)
            {
               // if (mot_torque.Get_Type() != ChFunction.FunctionType.FUNCT_RECORDER)
                //    mot_torque = new ChFunction_Recorder();

               // if (mot_rot.Get_Type() != ChFunction.FunctionType.FUNCT_RECORDER)
                //    mot_rot = new ChFunction_Recorder();

               // std::static_pointer_cast<ChFunction_Recorder>(mot_torque).AddPoint(mytime, mot_retorque, 1);  // (x,y,w)  x=t
               // std::static_pointer_cast<ChFunction_Recorder>(mot_rot).AddPoint(mytime, mot_rerot, 1);        // (x,y,w)  x=t
            }
        }

        // data get/set
        public ChFunction Get_dist_funct() { return dist_funct; }
        public ChFunction Get_motrot_funct() { return mot_rot; }
        public ChFunction Get_mottorque_funct() { return mot_torque; }

        public void Set_dist_funct(ChFunction mf) { dist_funct = mf; }
        public void Set_motrot_funct(ChFunction mf) { mot_rot = mf; }
        public void Set_mottorque_funct(ChFunction mf) { mot_torque = mf; }

        public bool Get_learn() { return learn; }
        void Set_learn(bool mset)
        {
            if (mset)
            {
                SetDisabled(true);  // ..just to show it as a green wireframe...
                Set_learn_torque_rotaton(false);
            }
            else
            {
                SetDisabled(false);
            }

            if (mset)
                ((ChLinkMaskLF)mask).Constr_X().SetMode(eChConstraintMode.CONSTRAINT_FREE);
            else
                ((ChLinkMaskLF)mask).Constr_X().SetMode(eChConstraintMode.CONSTRAINT_LOCK);

            ChangedLinkMask();

            learn = mset;
           // if (dist_funct.Get_Type() != ChFunction.FunctionType.FUNCT_RECORDER)
             //   dist_funct = new ChFunction_Recorder();
        }
        public bool Get_learn_torque_rotaton() { return learn_torque_rotation; }
        public void Set_learn_torque_rotaton(bool mset)
        {
            learn_torque_rotation = mset;
           // if (mot_torque.Get_Type() != ChFunction.FunctionType.FUNCT_RECORDER)
             //   mot_torque = new ChFunction_Recorder();

           // if (mot_rot.Get_Type() != ChFunction.FunctionType.FUNCT_RECORDER)
             //   mot_rot = new ChFunction_Recorder();
        }
        public double Get_lin_offset() { return offset; }
        public void Set_lin_offset(double mset) { offset = mset; }

        public void Set_mot_tau(double mtau) { mot_tau = mtau; }
        public double Get_mot_tau() { return mot_tau; }
        public void Set_mot_eta(double meta) { mot_eta = meta; }
        public double Get_mot_eta() { return mot_eta; }
        void Set_mot_inertia(double min) { mot_inertia = min; }
        public double Get_mot_inertia() { return mot_inertia; }

        // easy fetching of motor-reduced moments or angle-speed-accel.
        public double Get_mot_rerot() { return mot_rerot; }
        public double Get_mot_rerot_dt() { return mot_rerot_dt; }
        public double Get_mot_rerot_dtdt() { return mot_rerot_dtdt; }
        public double Get_mot_retorque() { return mot_retorque; }
    }
}
