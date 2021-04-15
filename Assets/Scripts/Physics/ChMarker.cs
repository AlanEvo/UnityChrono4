using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;

namespace chrono
{
    /// Markers are auxiliary reference frames which belong to rigid bodies and move
    /// together with them. Most often, markers are used as references to build
    /// ChLink() constraints between two rigid bodies.
    /// The ChMarker objects allow also to user-define a motion law of marker respect
    /// to parent ChBody, if needed to represent imposed trajectories etc.
    public class ChMarker : ChObj
    {        
        public ChFrameMoving<double> FrameMoving = new ChFrameMoving<double>();

        public enum eChMarkerMotion
        {
            M_MOTION_FUNCTIONS = 0,  //< marker uses its own x, y, z functions
            M_MOTION_KEYFRAMED = 1,  //< marker moved via external key frames (derivatives obtained with BDF)
            M_MOTION_EXTERNAL = 2,   //< marker moved via external functions (derivatives provided)
        };

        private eChMarkerMotion motion_type;  //< type of marker motion

        private ChFunction motion_X;    //< user imposed motion for X coord, body relative
        private ChFunction motion_Y;    //< user imposed motion for Y coord, body relative
        private ChFunction motion_Z;    //< user imposed motion for Z coord, body relative
        private ChFunction motion_ang;  //< user imposed angle rotation about axis
        ChVector motion_axis = new ChVector(0, 0, 0);      //< this is the axis for the user imposed rotation

        private ChBody Body;  //< points to parent body     

        private ChCoordsys rest_coord;// = ChCoordsys.CSYSNULL;  //< relative resting position for function=0.

        private ChCoordsys last_rel_coord;// = ChCoordsys.CSYSNULL;     //< These values are set for each marker update, and are
        private ChCoordsys last_rel_coord_dt;// = ChCoordsys.CSYSNULL;  //< used internally to guess if there's some external routine
        private double last_time;            //< which moves the marker, so marker motion is guessed by BDF.

        /// Absolute position of frame (expressed in absolute coordinate system).
        /// Computed at each Update() call; Useful for efficiency reasons.
        [SerializeField]
        private ChFrameMoving<double> abs_frame = new ChFrameMoving<double>();  //< absolute frame position

       /* public void Awake()
        {
            FrameMoving = new ChFrameMoving<double>();
            Body = null;
            rest_coord = new ChCoordsys(new ChVector(0, 0, 0), new ChQuaternion(1, 0, 0, 0));//ChCoordsys.CSYSNORM;
            motion_type = eChMarkerMotion.M_MOTION_FUNCTIONS;
            motion_axis = ChVector.VECT_Z;
            last_rel_coord = new ChCoordsys(new ChVector(0, 0, 0), new ChQuaternion(1, 0, 0, 0));//ChCoordsys.CSYSNORM;
            last_rel_coord_dt = new ChCoordsys(new ChVector(0, 0, 0), new ChQuaternion(0, 0, 0, 0));// ChCoordsys.CSYSNULL;
            last_time = 0;
            motion_X = new ChFunction_Const(0);  // default: no motion
            motion_Y = new ChFunction_Const(0);
            motion_Z = new ChFunction_Const(0);
            motion_ang = new ChFunction_Const(0);

            UpdateState();
        }*/

        public ChMarker()
        {
            FrameMoving = new ChFrameMoving<double>();
            Body = null;
            rest_coord = new ChCoordsys(new ChVector(0, 0, 0), new ChQuaternion(1, 0, 0, 0));//ChCoordsys.CSYSNORM;
            motion_type = eChMarkerMotion.M_MOTION_FUNCTIONS;
            motion_axis = ChVector.VECT_Z;
            last_rel_coord = new ChCoordsys(new ChVector(0, 0, 0), new ChQuaternion(1, 0, 0, 0));//ChCoordsys.CSYSNORM;
            last_rel_coord_dt = new ChCoordsys(new ChVector(0, 0, 0), new ChQuaternion(0, 0, 0, 0));// ChCoordsys.CSYSNULL;
            last_time = 0;
            motion_X = new ChFunction_Const(0);  // default: no motion
            motion_Y = new ChFunction_Const(0);
            motion_Z = new ChFunction_Const(0);
            motion_ang = new ChFunction_Const(0);

            UpdateState();
        }
        public ChMarker(string name,
             ChBody body,
             ChCoordsys rel_pos,
             ChCoordsys rel_pos_dt,
             ChCoordsys rel_pos_dtdt)
        {
            SetNameString(name);
            Body = body;

            motion_X = new ChFunction_Const(0);  // default: no motion
            motion_Y = new ChFunction_Const(0);
            motion_Z = new ChFunction_Const(0);
            motion_ang = new ChFunction_Const(0);
            motion_axis = ChVector.VECT_Z;

            rest_coord = new ChCoordsys(new ChVector(0, 0, 0), new ChQuaternion(1, 0, 0, 0));//ChCoordsys.CSYSNORM;

            motion_type = eChMarkerMotion.M_MOTION_FUNCTIONS;

            FrameMoving.SetCoord(rel_pos);
            FrameMoving.SetCoord_dt(rel_pos_dt);
            FrameMoving.SetCoord_dtdt(rel_pos_dtdt);

            last_rel_coord = new ChCoordsys(new ChVector(0, 0, 0), new ChQuaternion(1, 0, 0, 0));//ChCoordsys.CSYSNORM;
            last_rel_coord_dt = new ChCoordsys(new ChVector(0, 0, 0), new ChQuaternion(0, 0, 0, 0));// ChCoordsys.CSYSNULL;
            last_time = 0;

            UpdateState();
        }
        public ChMarker(ChMarker other): base(other) {

            FrameMoving = new ChFrameMoving<double>(other.FrameMoving);
            Body = null;

            motion_X = new ChFunction(other.motion_X.Clone());
            motion_Y = new ChFunction(other.motion_Y.Clone());
            motion_Z = new ChFunction(other.motion_Z.Clone());
            motion_ang = new ChFunction(other.motion_ang.Clone());

            motion_axis = other.motion_axis;

            rest_coord = other.rest_coord;

            motion_type = other.motion_type;

            abs_frame = other.abs_frame;

            last_rel_coord = other.last_rel_coord;
            last_rel_coord_dt = other.last_rel_coord_dt;
            last_time = other.last_time;
        }

        /// "Virtual" copy constructor (covariant return type).
        public override ChObj Clone(){ return new ChMarker(this);
    }

        /// Gets the address of the parent rigid body.
        public ChBody GetBody() { return Body; }
        /// Sets the parent rigid body.
        public void SetBody(ChBody newRB) { Body = newRB; }


        /// Set body-relative coord. and update auxiliary variables
        /// Also, current position becomes the 'resting position' coordinates
        /// for the current time.
        public void Impose_Rel_Coord(ChCoordsys m_coord) {
            ChQuaternion qtemp;// = new ChQuaternion(1, 0, 0, 0);
            // set the actual coordinates
            FrameMoving.SetCoord(m_coord);
            // set the resting position coordinates
            rest_coord.pos.x = m_coord.pos.x - motion_X.Get_y(ChTime);
            rest_coord.pos.y = m_coord.pos.y - motion_Y.Get_y(ChTime);
            rest_coord.pos.z = m_coord.pos.z - motion_Z.Get_y(ChTime);
            qtemp = ChQuaternion.Q_from_AngAxis2(-(motion_ang.Get_y(ChTime)), motion_axis);
            rest_coord.rot = ChQuaternion.Qcross(m_coord.rot, qtemp);  // ***%%% check
                                                          // set also the absolute positions, and other.
            UpdateState();
        }

        /// Set absolute coordinates  and update auxiliary variables
        /// Also, current position becomes the 'resting position' coordinates
        /// for the current time.
        public void Impose_Abs_Coord(ChCoordsys m_coord) {
            ChBody my_body;
            my_body = GetBody();

            ChCoordsys csys = new ChCoordsys(new ChVector(0, 0, 0), new ChQuaternion(1, 0, 0, 0));//ChCoordsys.CSYSNULL;
            // coordsys: transform the representation from the parent reference frame
            // to the local reference frame.
            csys.pos = ChTransform<double>.TransformParentToLocal(m_coord.pos, my_body.BodyFrame.GetCoord().pos, my_body.BodyFrame.GetA());
            csys.rot = ChQuaternion.Qcross(ChQuaternion.Qconjugate(my_body.BodyFrame.GetCoord().rot), m_coord.rot);
            // apply the imposition on local  coordinate and resting coordinate:
            Impose_Rel_Coord(csys);
        }

        /// Get the 'resting position' (that is, the position which the
        /// marker should have when the x,y,z motion laws are at time=0).
        public ChCoordsys GetRest_Coord() { return rest_coord; }

        //
        // Body-relative coordinates
        //

        // No functions here...
        //  In order to get/set  body-relative coordinates,
        // you can use the methods of the ChFrameMoving parent
        // class: for example use  my_marker->SetCoord(newpos) to
        // impose a new position&rotation, etc.
        //  NOTE!! after each modification of the frame position,
        // speed, acceleration etc., you should remember to call UpdateState()
        // if you want to kee updated also the absolute coordinates , ie. the
        // auxiliary structure to get with GetAbsFrame().

        //
        // Absolute coordinates (auxiliary data)
        //

        /// Get reference to the inner 'absolute frame' auxiliary
        /// coordinates. This object (coordinates/speeds/accel. of marker
        /// expressed in absolute coordinates) is useful for performance
        /// reasons. Note! it is updated only after each Update() function.
        public ChFrameMoving<double> GetAbsFrame() { return abs_frame; }

        /// Get the translation and rotation (as a ChCoordsys) of the marker
        /// respect to the absolute coordinates.
        public ChCoordsys GetAbsCoord() { return abs_frame.GetCoord(); }

        /// Get the speed of translation and rotation (as a derived ChCoordsys)
        /// of the marker respect to the absolute coordinates.
        public ChCoordsys GetAbsCoord_dt() { return abs_frame.GetCoord_dt(); }

        /// Get the acceleration of translation and rotation (as a derived ChCoordsys)
        /// of the marker respect to the absolute coordinates.
        public ChCoordsys GetAbsCoord_dtdt() { return abs_frame.GetCoord_dtdt(); }

        /// Set the translation and rotation (as a ChCoordsys) of the marker
        /// respect to the absolute coordinates.
        /// NOTE! inner use only, for the moment. Use  Impose_Abs_Coord() if needed.
        public void SetAbsCoord(ChCoordsys newpos) { abs_frame.SetCoord(newpos); }
        /// Set the speed of translation and rotation (as a ChCoordsys) of the marker
        /// respect to the absolute coordinates.
        /// NOTE! inner use only, for the moment.
        public void SetAbsCoord_dt(ChCoordsys newpos_dt) { abs_frame.SetCoord(newpos_dt); }
        /// Set the speed of translation and rotation (as a ChCoordsys) of the marker
        /// respect to the absolute coordinates.
        /// NOTE! inner use only, for the moment.
        public void SetAbsCoord_dtdt(ChCoordsys newpos_dtdt) { abs_frame.SetCoord(newpos_dtdt); }

        /// Get the angular speed respect to absolute coordinates,
        /// expressed in  absolute coordinates.
        public ChVector GetAbsWvel() { return abs_frame.GetWvel_par(); }

        /// Get the angular acceleration respect to absolute coordinates,
        /// expressed in  absolute coordinates.
        public ChVector GetAbsWacc() { return abs_frame.GetWacc_par(); }

        //
        // Imposed motion
        //

        /// Set the imposed motion law, for translation on X body axis
        public void SetMotion_X(ChFunction m_funct) {
            motion_X = m_funct;
        }
        /// Set the imposed motion law, for translation on Y body axis
        public void SetMotion_Y(ChFunction m_funct) {
            motion_Y = m_funct;
        }
        /// Set the imposed motion law, for translation on Z body axis
        public void SetMotion_Z(ChFunction m_funct) {
            motion_Z = m_funct;
        }
        /// Set the imposed motion law, for rotation about an axis
        public void SetMotion_ang(ChFunction m_funct) {
            motion_ang = m_funct;
        }
        /// Set the axis of rotation, if rotation motion law is used.
        public void SetMotion_axis(ChVector m_axis) {
            motion_axis = m_axis;
        }

        /// The imposed motion law, for translation on X body axis
        public ChFunction GetMotion_X() { return motion_X; }
        /// The imposed motion law, for translation on Y body axis
        public ChFunction GetMotion_Y() { return motion_Y; }
        /// The imposed motion law, for translation on Z body axis
        public ChFunction GetMotion_Z() { return motion_Z; }
        /// The imposed motion law, for rotation about an axis
        public ChFunction GetMotion_ang() { return motion_ang; }
        /// Get the axis of rotation, if rotation motion law is used.
        public ChVector GetMotion_axis() { return motion_axis; }

        /// Sets the way the motion of this marker (if any) is handled (see
        /// the eChMarkerMotion enum options).
        public void SetMotionType(eChMarkerMotion m_motion) { motion_type = m_motion; }

        /// Gets the way the motion of this marker (if any) is handled (see
        /// the eChMarkerMotion enum options).
        public eChMarkerMotion GetMotionType() { return motion_type; }

        //
        // UPDATING
        //

        /// Updates the time.dependant variables (ex: ChFunction objects
        /// which impose the body-relative motion, etc.)
        public void UpdateTime(double mytime) {
            ChCoordsys csys = new ChCoordsys(new ChVector(0, 0, 0), new ChQuaternion(1, 0, 0, 0));
            ChCoordsys csys_dt = new ChCoordsys(new ChVector(0, 0, 0), new ChQuaternion(1, 0, 0, 0));
            ChCoordsys csys_dtdt = new ChCoordsys(new ChVector(0, 0, 0), new ChQuaternion(1, 0, 0, 0));
            ChQuaternion qtemp;// = new ChQuaternion(1, 0, 0, 0);
            double ang, ang_dt, ang_dtdt;

            ChTime = mytime;

            // if a imposed motion (keyframed movement) affects the marker position (example,from R3D animation system),
            // compute the speed and acceleration values by BDF (example,see the UpdatedExternalTime() function, later)
            // so the updating via motion laws can be skipped!
            if (motion_type == eChMarkerMotion.M_MOTION_KEYFRAMED)
                return;

            // skip relative-position-functions evaluation also if
            // someone is already handling this from outside..
            if (motion_type == eChMarkerMotion.M_MOTION_EXTERNAL)
                return;

            // positions:
            // update positions:    rel_pos
            csys.pos.x = motion_X.Get_y(mytime);
            csys.pos.y = motion_Y.Get_y(mytime);
            csys.pos.z = motion_Z.Get_y(mytime);
            if (motion_X.Get_Type() != ChFunction.FunctionType.FUNCT_MOCAP)
                csys.pos += rest_coord.pos;

            // update speeds:		rel_pos_dt
            csys_dt.pos.x = motion_X.Get_y_dx(mytime);
            csys_dt.pos.y = motion_Y.Get_y_dx(mytime);
            csys_dt.pos.z = motion_Z.Get_y_dx(mytime);

            // update accelerations
            csys_dtdt.pos.x = motion_X.Get_y_dxdx(mytime);
            csys_dtdt.pos.y = motion_Y.Get_y_dxdx(mytime);
            csys_dtdt.pos.z = motion_Z.Get_y_dxdx(mytime);

            // rotations:

            ang = motion_ang.Get_y(mytime);
            ang_dt = motion_ang.Get_y_dx(mytime);
            ang_dtdt = motion_ang.Get_y_dxdx(mytime);

            if ((ang != 0) || (ang_dt != 0) || (ang_dtdt != 0))
            {
                // update q
                ChVector motion_axis_versor = ChVector.Vnorm(motion_axis);
                qtemp = ChQuaternion.Q_from_AngAxis2(ang, motion_axis_versor);
                csys.rot = ChQuaternion.Qcross(qtemp, rest_coord.rot);
                // update q_dt
                csys_dt.rot = ChQuaternion.Qdt_from_AngAxis(csys.rot, ang_dt, motion_axis_versor);
                // update q_dtdt
                csys_dtdt.rot = ChQuaternion.Qdtdt_from_AngAxis(ang_dtdt, motion_axis_versor, csys.rot, csys_dt.rot);
            }
            else
            {
                csys.rot = FrameMoving.coord.rot;  // rel_pos.rot;
                csys_dt.rot = ChQuaternion.QNULL;
                csys_dtdt.rot = ChQuaternion.QNULL;
            }

            // Set the position, speed and acceleration in relative space,
            // automatically getting also the absolute values,
            if (!(csys == this.FrameMoving.coord))
                FrameMoving.SetCoord(csys);

            if (!(csys_dt == this.FrameMoving.coord_dt) || !(csys_dt.rot == new ChQuaternion(0, 0, 0, 0)))
                FrameMoving.SetCoord_dt(csys_dt);

            if (!(csys_dtdt == this.FrameMoving.coord_dtdt) || !(csys_dtdt.rot == new ChQuaternion(0, 0, 0, 0)))
                FrameMoving.SetCoord_dtdt(csys_dtdt);
        }

        /// Given current state, updates auxiliary variables (for example
        /// the abs_frame data, containing the absolute pos/speed/acc of
        /// the marker.
        public void UpdateState() {
            if (GetBody() == null)
                return;

            GetBody().BodyFrame.TransformLocalToParent(this.FrameMoving, ref abs_frame);
           // Debug.Log("data " + abs_frame.GetPos().z);
        }

        /// Both UpdateTime() and UpdateState() at once.
        public void update(double mytime) {
            UpdateTime(mytime);
            UpdateState();
        }

        /// Someone (ex. an ChExternalObject() ) may send this message to
        /// the marker to tell that time has changed (even if simulation is
        /// not running! - so it is different from the usual UpdateTime() -)
        public void UpdatedExternalTime(double prevtime, double mtime) {
            double mstep = mtime - prevtime;

            ChCoordsys m_rel_pos_dt = new ChCoordsys(new ChVector(0, 0, 0), new ChQuaternion(1, 0, 0, 0));
            ChCoordsys m_rel_pos_dtdt = new ChCoordsys(new ChVector(0, 0, 0), new ChQuaternion(1, 0, 0, 0));

            // do not try to switch on the M_MOTION_KEYFRAMED mode if
            // we are already in the M_MOTION_EXTERNAL mode, maybe because
            // a link point-surface is already moving the marker and
            // it will handle the accelerations by itself
            if (this.motion_type == eChMarkerMotion.M_MOTION_EXTERNAL)
                return;

            // otherwise see if a BDF is needed, cause an external 3rd party is moving the marker
            this.motion_type = eChMarkerMotion.M_MOTION_FUNCTIONS;

            // if POSITION or ROTATION ("rel_pos") has been changed in acceptable time step...
            if ((!(ChVector.Vequal(FrameMoving.coord.pos, last_rel_coord.pos)) || !(ChQuaternion.Qequal(FrameMoving.coord.rot, last_rel_coord.rot))) && (Math.Abs(mstep) < 0.1) &&
                (mstep != 0))
            {
                // ... and if motion wasn't caused by motion laws, then it was a keyframed movement!
                if ((motion_X.Get_y(mtime) == 0) && (motion_Y.Get_y(mtime) == 0) && (motion_Z.Get_y(mtime) == 0) &&
                    (motion_ang.Get_y(mtime) == 0) && (motion_X.Get_Type() == ChFunction.FunctionType.FUNCT_CONST) &&
                    (motion_Y.Get_Type() == ChFunction.FunctionType.FUNCT_CONST) && (motion_Z.Get_Type() == ChFunction.FunctionType.FUNCT_CONST) &&
                    (motion_ang.Get_Type() == ChFunction.FunctionType.FUNCT_CONST))
                {
                    // compute the relative speed by BDF !
                    m_rel_pos_dt.pos = ChVector.Vmul(ChVector.Vsub(FrameMoving.coord.pos, last_rel_coord.pos), 1 / mstep);
                    m_rel_pos_dt.rot = ChQuaternion.Qscale(ChQuaternion.Qsub(FrameMoving.coord.rot, last_rel_coord.rot), 1 / mstep);

                    // compute the relative acceleration by BDF !
                    m_rel_pos_dtdt.pos = ChVector.Vmul(ChVector.Vsub(m_rel_pos_dt.pos, last_rel_coord_dt.pos), 1 / mstep);
                    m_rel_pos_dtdt.rot = ChQuaternion.Qscale(ChQuaternion.Qsub(m_rel_pos_dt.rot, last_rel_coord_dt.rot), 1 / mstep);

                    // Set the position, speed and acceleration in relative space,
                    // automatically getting also the absolute values,
                    FrameMoving.SetCoord_dt(m_rel_pos_dt);
                    FrameMoving.SetCoord_dtdt(m_rel_pos_dtdt);

                    // update the remaining state variables
                    this.UpdateState();

                    // remember that the movement of this guy won't need further update
                    // of speed and acc. via motion laws!
                    this.motion_type = eChMarkerMotion.M_MOTION_KEYFRAMED;
                }
            }

            // restore state buffers and that's all.
            last_time = ChTime;
            last_rel_coord = (ChCoordsys)FrameMoving.coord;
            last_rel_coord_dt = (ChCoordsys)FrameMoving.coord_dt;
        }

        //
        // UTILITIES
        //

        public ChVector Point_World2Ref(ChVector mpoint) {
            return (abs_frame / mpoint);
        }
        public ChVector Point_Ref2World(ChVector mpoint) {
            return (abs_frame * mpoint);
        }
        public ChVector Dir_World2Ref(ChVector mpoint) {
            return abs_frame.GetA().MatrT_x_Vect(mpoint);
        }
        public ChVector Dir_Ref2World(ChVector mpoint) {
            return abs_frame.GetA().Matr_x_Vect(mpoint);
        }
    }
}
