using System.Collections;
using System.Collections.Generic;
using UnityEngine;


namespace chrono
{

    /// Class for links which connect two 'markers'. The markers are two
    /// ChMarker objects each belonging to the two linked ChBody parts.
    /// Many specialized classes are based on this ChLinkMarkers class, for example
    /// the ChLinkSpring and all the family of the ChLinkLock classes - see them-.
    /// Also, ChLinkMarkers class allows an optional force vector and torque vector
    /// to be set between the two connected markers.

    public class ChLinkMarkers : ChLink
    {

        protected ChMarker marker1 = new ChMarker();  //< slave coordsys
        protected ChMarker marker2 = new ChMarker();  //< master coordsys, =0 if liked to ground

        protected int markID1;  //< unique identifier for markers 1 & 2,
        protected int markID2;  //< when using plugin dynamic hierarchies

        protected ChCoordsys<double> relM = new ChCoordsys<double>();       //< relative marker position 2-1
        protected ChCoordsys<double> relM_dt = new ChCoordsys<double>();    //< relative marker speed
        protected ChCoordsys<double> relM_dtdt = new ChCoordsys<double>();  //< relative marker acceleration

        protected double relAngle;        //< relative angle of rotation
        protected ChVector relAxis = new ChVector();     //< relative axis of rotation
        protected ChVector relRotaxis = new ChVector();  //< relative rotaion vector =angle*axis
        protected ChVector relWvel = new ChVector();     //< relative angular speed
        protected ChVector relWacc = new ChVector();     //< relative angular acceleration
        protected double dist;            //< the distance between the two origins of markers,
        protected double dist_dt;         //< the speed between the two  origins of markers

        protected ChVector Scr_force = new ChVector();   //< internal force  set by script only (just added to C_force)
        protected ChVector Scr_torque = new ChVector();  //< internal torque set by script only (just added to C_force)
        protected ChVector C_force = new ChVector();     //< internal force  applied by springs/dampers/actuators
        protected ChVector C_torque = new ChVector();    //< internal torque applied by springs/dampers/actuators

        public ChLinkMarkers()
        {
            marker1 = null;
            marker2 = null;
            markID1 = 0;
            markID2 = 0;
            relM = new ChCoordsys<double>(new ChVector(), new ChQuaternion(1, 0, 0, 0));//Coordsys.CSYSNORM;
            relM_dt = new ChCoordsys<double>(new ChVector(), new ChQuaternion());//ChCoordsys<double>.CSYSNULL;
            relM_dtdt = new ChCoordsys<double>(new ChVector(), new ChQuaternion());// ChCoordsys<double>.CSYSNULL;
            relAngle = 0;
            relRotaxis = new ChVector();
            relWvel = new ChVector();
            relWacc = new ChVector();
            C_force = new ChVector();
            C_torque = new ChVector();
            Scr_force = new ChVector();
            Scr_torque = new ChVector();
        }

        public ChLinkMarkers(ChLinkMarkers other) : base(other)
        {
            marker1 = null;
            marker2 = null;

            markID1 = other.markID1;
            markID2 = other.markID2;

            relM = other.relM;  // copy vars
            relM_dt = other.relM_dt;
            relM_dtdt = other.relM_dtdt;
            relAngle = other.relAngle;
            relRotaxis = other.relRotaxis;
            relAxis = other.relAxis;
            relWvel = other.relWvel;
            relWacc = other.relWacc;
            dist = other.dist;
            dist_dt = other.dist_dt;

            C_force = other.C_force;
            C_torque = other.C_torque;

            Scr_force = other.Scr_force;
            Scr_torque = other.Scr_torque;
        }

        /// "Virtual" copy constructor (covariant return type).
        public override ChObj Clone() { return new ChLinkMarkers(this); }

        /// Return the 1st referenced marker (the 'slave' marker, owned by 1st body)
        public ChMarker GetMarker1() { return marker1; }

        /// Return the 2nd referenced marker (the 'master' marker, owned by 2nd body)
        public ChMarker GetMarker2() { return marker2; }

        /// set the two markers associated with this link
        public virtual void SetUpMarkers(ChMarker mark1, ChMarker mark2)
        {
            // take care of the first link marker

            marker1 = mark1;

            if (mark1)
                Body1 = mark1.GetBody().BodyFrame;
            else
                Body1 = null;

            // take care of the second link marker
            marker2 = mark2;
            if (mark2)
                Body2 = mark2.GetBody().BodyFrame;
            else
                Body2 = null;
        }

        public void SetMarkID1(int mid) { markID1 = mid; }
        public void SetMarkID2(int mid) { markID2 = mid; }
        public int GetMarkID1() { return markID1; }
        public int GetMarkID2() { return markID2; }

        /// Shortcut: set markers and marker IDs at once.
        public bool ReferenceMarkers(ChMarker mark1, ChMarker mark2)
        {
            this.SetUpMarkers(mark1, mark2);

            if (mark1)
                SetMarkID1(mark1.GetIdentifier());
            else
                SetMarkID1(0);
            if (mark2)
                SetMarkID2(mark2.GetIdentifier());
            else
                SetMarkID2(0);

            return mark1 && mark2;
        }

        /// Use this function after link creation, to initialize the link from
        /// two markers to join.
        /// Each marker must belong to a rigid body, and both rigid bodies
        /// must belong to the same ChSystem.
        /// The position of mark2 is used as link's position and main reference.
        public virtual void Initialize(ChMarker mmark1,  //< first  marker to join
                                       ChMarker mmark2   //< second marker to join (master)
                                       )
        {
            ChMarker mm1 = mmark1;
            ChMarker mm2 = mmark2;
            //Debug.Assert(mm1 && mm2);
            //Debug.Assert(mm1 != mm2);
            //Debug.Assert(mm1.GetBody() && mm2.GetBody());
            //Debug.Assert(mm1.GetBody().GetSystem() == mm2.GetBody().GetSystem());

            ReferenceMarkers(mm1, mm2);
        }

        /// Use this function after link creation, to initialize the link from
        /// two joined rigid bodies.
        /// Both rigid bodies must belong to the same ChSystem.
        /// Two markers will be created and added to the rigid bodies (later,
        /// you can use GetMarker1() and GetMarker2() to access them.
        /// To specify the (absolute) position of link and markers, use 'mpos'.
        public virtual void Initialize(ChBody mbody1,  //< first  body to join
                            ChBody mbody2,              //< second body to join
                            ChCoordsys<double> mpos         //< the current absolute pos.& alignment.
                            )
        {
             Initialize(mbody1, mbody2, false, mpos, mpos);
        }

        /// Use this function after link creation, to initialize the link from
        /// two joined rigid bodies.
        /// Both rigid bodies must belong to the same ChSystem.
        /// Two markers will be created and added to the rigid bodies (later,
        /// you can use GetMarker1() and GetMarker2() to access them.
        /// To specify the (absolute) position of link and markers, use 'mpos'.
        public virtual void Initialize(
                    ChBody mbody1,  //< first  body to join
                    ChBody mbody2,  //< second body to join
                    bool pos_are_relative,      //< if =true, following two positions are relative to bodies. If false, are absolute.
                    ChCoordsys<double> mpos1,  //< the position & alignment of 1st marker (relative to body1 cords, or absolute)
                    ChCoordsys<double> mpos2   //< the position & alignment of 2nd marker (relative to body2 cords, or absolute)
        )
        {
            Debug.Assert(mbody1 != mbody2);
            Debug.Assert(mbody1.GetSystem() == mbody2.GetSystem());

            // create markers to add to the two bodies
            //ChMarker mmark1 = new ChMarker(new ChMarker());
            //ChMarker mmark2 = new ChMarker(new ChMarker());
            ChMarker mmark1 = gameObject.AddComponent<ChMarker>();
            ChMarker mmark2 = gameObject.AddComponent<ChMarker>();
            mbody1.AddMarker(mmark1);
            mbody2.AddMarker(mmark2);

            ChMarker mm1 = mmark1;
            ChMarker mm2 = mmark2;
            ReferenceMarkers(mm1, mm2);

            if (pos_are_relative)
            {
                mmark1.Impose_Rel_Coord(mpos1);
                mmark2.Impose_Rel_Coord(mpos2);
            }
            else
            {
                mmark1.Impose_Abs_Coord(mpos1);
                mmark2.Impose_Abs_Coord(mpos2);
            }
        }

        /// Get the link coordinate system, expressed relative to Body2 (the 'master'
        /// body). This represents the 'main' reference of the link: reaction forces
        /// and torques are expressed in this coordinate system.
        /// (It is the coordinate system of the 'master' marker2 relative to Body2)
        public override ChCoordsys<double> GetLinkRelativeCoords() { return marker2.FrameMoving.GetCoord(); }

        /// Get the master coordinate system for the assets (this will return the
        /// absolute coordinate system of the 'master' marker2)
        // public override ChFrame<double> GetAssetsFrame(int nclone = 0) { return marker2.GetAbsFrame(); }

        //
        // UPDATING FUNCTIONS
        //

        /// Updates auxiliary vars relM, relM_dt, relM_dtdt,
        /// dist, dist_dt et simila.
        public virtual void UpdateRelMarkerCoords()
        {
            // FOR ALL THE 6(or3) COORDINATES OF RELATIVE MOTION OF THE TWO MARKERS:
            // COMPUTE THE relM, relM_dt relM_dtdt COORDINATES, AND AUXILIARY DATA (distance,etc.)

            ChVector PQw = ChVector.Vsub(marker1.GetAbsCoord().pos, marker2.GetAbsCoord().pos);
            ChVector PQw_dt = ChVector.Vsub(marker1.GetAbsCoord_dt().pos, marker2.GetAbsCoord_dt().pos);
            ChVector PQw_dtdt = ChVector.Vsub(marker1.GetAbsCoord_dtdt().pos, marker2.GetAbsCoord_dtdt().pos);

            dist = ChVector.Vlength(PQw);                 // distance between origins, modulus
            dist_dt = ChVector.Vdot(ChVector.Vnorm(PQw), PQw_dt);  // speed between origins, modulus.

            ChVector vtemp1; // for intermediate calculus
            ChVector vtemp2;
            ChQuaternion qtemp1;

            ChMatrixNM<IntInterface.Three, IntInterface.Four> relGw = new ChMatrixNM<IntInterface.Three, IntInterface.Four>();
            ChQuaternion q_AD;
            ChQuaternion q_BC;
            ChQuaternion q_8;
            ChVector q_4;

             ChQuaternion temp1 = marker1.FrameMoving.GetCoord_dt().rot;
             ChQuaternion temp2 = marker2.FrameMoving.GetCoord_dt().rot;

            if ( ChQuaternion.Qnotnull(temp1) ||  ChQuaternion.Qnotnull(temp2))
            {
                q_AD =  //  q'qqq + qqqq'
                     ChQuaternion.Qadd( ChQuaternion.Qcross( ChQuaternion.Qconjugate(marker2.FrameMoving.GetCoord_dt().rot),
                                 ChQuaternion.Qcross( ChQuaternion.Qconjugate(marker2.GetBody().BodyFrame.GetCoord().rot),
                                        ChQuaternion.Qcross((marker1.GetBody().BodyFrame.GetCoord().rot), (marker1.FrameMoving.GetCoord().rot)))),
                          ChQuaternion.Qcross( ChQuaternion.Qconjugate(marker2.FrameMoving.GetCoord().rot),
                                 ChQuaternion.Qcross( ChQuaternion.Qconjugate(marker2.GetBody().BodyFrame.GetCoord().rot),
                                        ChQuaternion.Qcross((marker1.GetBody().BodyFrame.GetCoord().rot), (marker1.FrameMoving.GetCoord_dt().rot)))));
            }
            else
                q_AD = new ChQuaternion(0, 0, 0, 0);

            q_BC =  // qq'qq + qqq'q
                 ChQuaternion.Qadd( ChQuaternion.Qcross( ChQuaternion.Qconjugate(marker2.FrameMoving.GetCoord().rot),
                             ChQuaternion.Qcross( ChQuaternion.Qconjugate(marker2.GetBody().BodyFrame.GetCoord_dt().rot),
                                    ChQuaternion.Qcross((marker1.GetBody().BodyFrame.GetCoord().rot), (marker1.FrameMoving.GetCoord().rot)))),
                      ChQuaternion.Qcross( ChQuaternion.Qconjugate(marker2.FrameMoving.GetCoord().rot),
                             ChQuaternion.Qcross( ChQuaternion.Qconjugate(marker2.GetBody().BodyFrame.GetCoord().rot),
                                    ChQuaternion.Qcross((marker1.GetBody().BodyFrame.GetCoord_dt().rot), (marker1.FrameMoving.GetCoord().rot)))));

            // q_8 = q''qqq + 2q'q'qq + 2q'qq'q + 2q'qqq'
            //     + 2qq'q'q + 2qq'qq' + 2qqq'q' + qqqq''
            temp2 = marker2.FrameMoving.GetCoord_dtdt().rot;
            if ( ChQuaternion.Qnotnull(temp2))
                q_8 =  ChQuaternion.Qcross( ChQuaternion.Qconjugate(marker2.FrameMoving.GetCoord_dtdt().rot),
                              ChQuaternion.Qcross( ChQuaternion.Qconjugate(Body2.GetCoord().rot),
                                     ChQuaternion.Qcross(Body1.GetCoord().rot,
                                           marker1.FrameMoving.GetCoord().rot)));  // q_dtdt'm2 * q'o2 * q,o1 * q,m1
            else
                q_8 = new ChQuaternion(0, 0, 0, 0);
            temp1 = marker1.FrameMoving.GetCoord_dtdt().rot;
            if ( ChQuaternion.Qnotnull(temp1))
            {
                qtemp1 =  ChQuaternion.Qcross( ChQuaternion.Qconjugate(marker2.FrameMoving.GetCoord().rot),
                                 ChQuaternion.Qcross( ChQuaternion.Qconjugate(Body2.GetCoord().rot),
                                        ChQuaternion.Qcross(Body1.GetCoord().rot,
                                              marker1.FrameMoving.GetCoord_dtdt().rot)));  // q'm2 * q'o2 * q,o1 * q_dtdt,m1
                q_8 =  ChQuaternion.Qadd(q_8, qtemp1);
            }
            temp2 = marker2.FrameMoving.GetCoord_dt().rot;
            if ( ChQuaternion.Qnotnull(temp2))
            {
                qtemp1 =  ChQuaternion.Qcross(
                     ChQuaternion.Qconjugate(marker2.FrameMoving.GetCoord_dt().rot),
                     ChQuaternion.Qcross( ChQuaternion.Qconjugate(Body2.GetCoord_dt().rot),  ChQuaternion.Qcross(Body1.GetCoord().rot, marker1.FrameMoving.GetCoord().rot)));
                qtemp1 =  ChQuaternion.Qscale(qtemp1, 2);  // 2( q_dt'm2 * q_dt'o2 * q,o1 * q,m1)
                q_8 =  ChQuaternion.Qadd(q_8, qtemp1);
            }
            temp2 = marker2.FrameMoving.GetCoord_dt().rot;
            if ( ChQuaternion.Qnotnull(temp2))
            {
                qtemp1 =  ChQuaternion.Qcross(
                     ChQuaternion.Qconjugate(marker2.FrameMoving.GetCoord_dt().rot),
                     ChQuaternion.Qcross( ChQuaternion.Qconjugate(Body2.GetCoord().rot),  ChQuaternion.Qcross(Body1.GetCoord_dt().rot, marker1.FrameMoving.GetCoord().rot)));
                qtemp1 =  ChQuaternion.Qscale(qtemp1, 2);  // 2( q_dt'm2 * q'o2 * q_dt,o1 * q,m1)
                q_8 =  ChQuaternion.Qadd(q_8, qtemp1);
            }
            temp1 = marker1.FrameMoving.GetCoord_dt().rot;
            temp2 = marker2.FrameMoving.GetCoord_dt().rot;
            if ( ChQuaternion.Qnotnull(temp2) &&  ChQuaternion.Qnotnull(temp1))
            {
                qtemp1 =  ChQuaternion.Qcross(
                     ChQuaternion.Qconjugate(marker2.FrameMoving.GetCoord_dt().rot),
                     ChQuaternion.Qcross( ChQuaternion.Qconjugate(Body2.GetCoord().rot),  ChQuaternion.Qcross(Body1.GetCoord().rot, marker1.FrameMoving.GetCoord_dt().rot)));
                qtemp1 =  ChQuaternion.Qscale(qtemp1, 2);  // 2( q_dt'm2 * q'o2 * q,o1 * q_dt,m1)
                q_8 =  ChQuaternion.Qadd(q_8, qtemp1);
            }

            qtemp1 =
                 ChQuaternion.Qcross( ChQuaternion.Qconjugate(marker2.FrameMoving.GetCoord().rot),
                        ChQuaternion.Qcross( ChQuaternion.Qconjugate(Body2.GetCoord_dt().rot),  ChQuaternion.Qcross(Body1.GetCoord_dt().rot, marker1.FrameMoving.GetCoord().rot)));
            qtemp1 =  ChQuaternion.Qscale(qtemp1, 2);  // 2( q'm2 * q_dt'o2 * q_dt,o1 * q,m1)
            q_8 =  ChQuaternion.Qadd(q_8, qtemp1);
            temp1 = marker1.FrameMoving.GetCoord_dt().rot;
            if ( ChQuaternion.Qnotnull(temp1))
            {
                qtemp1 =  ChQuaternion.Qcross(
                     ChQuaternion.Qconjugate(marker2.FrameMoving.GetCoord().rot),
                     ChQuaternion.Qcross( ChQuaternion.Qconjugate(Body2.GetCoord_dt().rot),  ChQuaternion.Qcross(Body1.GetCoord().rot, marker1.FrameMoving.GetCoord_dt().rot)));
                qtemp1 =  ChQuaternion.Qscale(qtemp1, 2);  // 2( q'm2 * q_dt'o2 * q,o1 * q_dt,m1)
                q_8 =  ChQuaternion.Qadd(q_8, qtemp1);
            }
            temp1 = marker1.FrameMoving.GetCoord_dt().rot;
            if ( ChQuaternion.Qnotnull(temp1))
            {
                qtemp1 =  ChQuaternion.Qcross(
                     ChQuaternion.Qconjugate(marker2.FrameMoving.GetCoord().rot),
                     ChQuaternion.Qcross( ChQuaternion.Qconjugate(Body2.GetCoord().rot),  ChQuaternion.Qcross(Body1.GetCoord_dt().rot, marker1.FrameMoving.GetCoord_dt().rot)));
                qtemp1 =  ChQuaternion.Qscale(qtemp1, 2);  // 2( q'm2 * q'o2 * q_dt,o1 * q_dt,m1)
                q_8 =  ChQuaternion.Qadd(q_8, qtemp1);
            }

            // q_4 = [Adtdt]'[A]'q + 2[Adt]'[Adt]'q
            //       + 2[Adt]'[A]'qdt + 2[A]'[Adt]'qdt
            ChMatrix33<double> m2_Rel_A_dt = new ChMatrix33<double>();
            marker2.FrameMoving.Compute_Adt(ref m2_Rel_A_dt);
            ChMatrix33<double> m2_Rel_A_dtdt = new ChMatrix33<double>();
            marker2.FrameMoving.Compute_Adtdt(ref m2_Rel_A_dtdt);

            vtemp1 = Body2.GetA_dt().MatrT_x_Vect(PQw);
            vtemp2 = m2_Rel_A_dt.MatrT_x_Vect(vtemp1);
            q_4 = ChVector.Vmul(vtemp2, 2);  // 2[Aq_dt]'[Ao2_dt]'*Qpq,w

            vtemp1 = Body2.GetA().MatrT_x_Vect(PQw_dt);
            vtemp2 = m2_Rel_A_dt.MatrT_x_Vect(vtemp1);
            vtemp2 = ChVector.Vmul(vtemp2, 2);  // 2[Aq_dt]'[Ao2]'*Qpq,w_dt
            q_4 = ChVector.Vadd(q_4, vtemp2);

            vtemp1 = Body2.GetA_dt().MatrT_x_Vect(PQw_dt);
            vtemp2 = marker2.FrameMoving.GetA().MatrT_x_Vect(vtemp1);
            vtemp2 = ChVector.Vmul(vtemp2, 2);  // 2[Aq]'[Ao2_dt]'*Qpq,w_dt
            q_4 = ChVector.Vadd(q_4, vtemp2);

            vtemp1 = Body2.GetA().MatrT_x_Vect(PQw);
            vtemp2 = m2_Rel_A_dtdt.MatrT_x_Vect(vtemp1);
            q_4 = ChVector.Vadd(q_4, vtemp2);  //  [Aq_dtdt]'[Ao2]'*Qpq,w

            // ----------- RELATIVE MARKER COORDINATES

            // relM.pos
            relM.pos = marker2.FrameMoving.GetA().MatrT_x_Vect(Body2.GetA().MatrT_x_Vect(PQw));

            // relM.rot
            relM.rot =  ChQuaternion.Qcross( ChQuaternion.Qconjugate(marker2.FrameMoving.GetCoord().rot),
                               ChQuaternion.Qcross( ChQuaternion.Qconjugate(marker2.GetBody().BodyFrame.GetCoord().rot),
                                      ChQuaternion.Qcross((marker1.GetBody().BodyFrame.GetCoord().rot), (marker1.FrameMoving.GetCoord().rot))));

            // relM_dt.pos
            relM_dt.pos = ChVector.Vadd(ChVector.Vadd(m2_Rel_A_dt.MatrT_x_Vect(Body2.GetA().MatrT_x_Vect(PQw)),
                                    marker2.FrameMoving.GetA().MatrT_x_Vect(Body2.GetA_dt().MatrT_x_Vect(PQw))),
                               marker2.FrameMoving.GetA().MatrT_x_Vect(Body2.GetA().MatrT_x_Vect(PQw_dt)));

            // relM_dt.rot
            relM_dt.rot =  ChQuaternion.Qadd(q_AD, q_BC);

            // relM_dtdt.pos
            relM_dtdt.pos = ChVector.Vadd(ChVector.Vadd(marker2.FrameMoving.GetA().MatrT_x_Vect(Body2.GetA_dtdt().MatrT_x_Vect(PQw)),
                                      marker2.FrameMoving.GetA().MatrT_x_Vect(Body2.GetA().MatrT_x_Vect(PQw_dtdt))),
                                 q_4);

            // relM_dtdt.rot
            qtemp1 =  ChQuaternion.Qcross( ChQuaternion.Qconjugate(marker2.FrameMoving.GetCoord().rot),
                             ChQuaternion.Qcross( ChQuaternion.Qconjugate(Body2.GetCoord_dtdt().rot),
                                    ChQuaternion.Qcross(Body1.GetCoord().rot,
                                          marker1.FrameMoving.GetCoord().rot)));  // ( q'm2 * q_dtdt'o2 * q,o1 * q,m1)
            relM_dtdt.rot =  ChQuaternion.Qadd(q_8, qtemp1);
            qtemp1 =  ChQuaternion.Qcross( ChQuaternion.Qconjugate(marker2.FrameMoving.GetCoord().rot),
                             ChQuaternion.Qcross( ChQuaternion.Qconjugate(Body2.GetCoord().rot),
                                    ChQuaternion.Qcross(Body1.GetCoord_dtdt().rot,
                                          marker1.FrameMoving.GetCoord().rot)));  // ( q'm2 * q'o2 * q_dtdt,o1 * q,m1)
            relM_dtdt.rot =  ChQuaternion.Qadd(relM_dtdt.rot, qtemp1);               // = q_8 + qq''qq + qqq''q

            // ... and also "user-friendly" relative coordinates:

            // relAngle and relAxis
             ChQuaternion.Q_to_AngAxis(relM.rot, ref relAngle, ref relAxis);
            // flip rel rotation axis if jerky sign
            if (relAxis.z < 0)
            {
                relAxis = ChVector.Vmul(relAxis, -1);
                relAngle = -relAngle;
            }
            // rotation axis
            relRotaxis = ChVector.Vmul(relAxis, relAngle);
            // relWvel
            ChFrame<double>.SetMatrix_Gw(ref relGw, relM.rot);  // relGw.Set_Gw_matrix(relM.rot);
            relWvel = relGw.Matr34_x_Quat(relM_dt.rot);
            // relWacc
            relWacc = relGw.Matr34_x_Quat(relM_dtdt.rot);
        }

        ///  Updates auxiliary forces caused by springs/dampers/etc. which may
        /// be connected between the two bodies of the link.
        /// By default, it adds the forces which might have been added by the
        /// user using Set_Scr_force() and Set_Scr_torque(). Note, these forces
        /// are considered in the reference coordsystem of marker2 (the MAIN marker),
        /// and their application point is the origin of marker1 (the SLAVE marker).
        public virtual void UpdateForces(double mytime)
        {
            C_force = new ChVector();  // initialize int.forces accumulators
            C_torque = new ChVector();

            // First and only operation: add the 'externally set' script forces (torques)
            C_force = ChVector.Vadd(C_force, Scr_force);
            C_torque = ChVector.Vadd(C_torque, Scr_torque);
        }

        // -----------COMPLETE UPDATE.
        // sequence:
        //			UpdateTime;
        //          UpdateRelMarkerCoords;
        //			UpdateForces;

        public override void update(double time, bool update_assets = true)
        {
            // 1 -
            UpdateTime(time);

            // 2 -
            UpdateRelMarkerCoords();

            // 3 -
            UpdateForces(time);

            // Inherit time changes of parent class (ChLink)
            base.update(time, update_assets);
        }

        //
        // STATE FUNCTIONS
        //

        /// Adds force to residual R, as R*= F*c
        /// NOTE: here the off offset in R is NOT used because add F at the TWO offsets of the two connected bodies,
        /// so it is assumed that offsets for Body1 and Body2 variables have been already set properly!
        public override void IntLoadResidual_F(int off, ref ChVectorDynamic<double> R, double c)
        {
            if (Body1 == null || Body2 == null)
                return;

            ChVector mbody_force = new ChVector();
            ChVector mbody_torque = new ChVector();
            if (ChVector.Vnotnull(C_force))
            {
                ChVector m_abs_force = Body2.GetA().Matr_x_Vect(marker2.FrameMoving.GetA().Matr_x_Vect(C_force));

                if (Body2.Variables().IsActive())
                {
                    Body2.To_abs_forcetorque(m_abs_force,
                                              marker1.GetAbsCoord().pos,  // absolute application point is always marker1
                                              false,                       // from abs. space
                                              ref mbody_force, ref mbody_torque);  // resulting force-torque, both in abs coords
                    R.PasteSumVector(mbody_force * -c, Body2.Variables().GetOffset(), 0);
                    R.PasteSumVector(Body2.TransformDirectionParentToLocal(mbody_torque) * -c,
                                     Body2.Variables().GetOffset() + 3, 0);
                }

                if (Body1.Variables().IsActive())
                {
                    Body1.To_abs_forcetorque(m_abs_force,
                                              marker1.GetAbsCoord().pos,  // absolute application point is always marker1
                                              false,                       // from abs. space
                                              ref mbody_force, ref mbody_torque);  // resulting force-torque, both in abs coords
                    R.PasteSumVector(mbody_force * c, Body1.Variables().GetOffset(), 0);
                    R.PasteSumVector(Body1.TransformDirectionParentToLocal(mbody_torque) * c,
                                     Body1.Variables().GetOffset() + 3, 0);
                }
            }
            if (ChVector.Vnotnull(C_torque))
            {
                ChVector m_abs_torque = Body2.GetA().Matr_x_Vect(marker2.FrameMoving.GetA().Matr_x_Vect(C_torque));
                // load torques in 'fb' vector accumulator of body variables (torques in local coords)
                if (Body1.Variables().IsActive())
                {
                    R.PasteSumVector(Body1.TransformDirectionParentToLocal(m_abs_torque) * c,
                                     Body1.Variables().GetOffset() + 3, 0);
                }
                if (Body2.Variables().IsActive())
                {
                    R.PasteSumVector(Body2.TransformDirectionParentToLocal(m_abs_torque) * -c,
                                     Body2.Variables().GetOffset() + 3, 0);
                }
            }
        }

        //
        // SOLVER INTERFACE
        //

        /// Overrides the empty behaviour of the parent ChLink implementation, which
        /// does not consider any user-imposed force between the two bodies.
        /// It adds the current link-forces, if any, (caused by springs, etc.) to the 'fb' vectors
        /// of the ChVariables referenced by encapsulated ChConstraints.
        /// In details, it adds the effect caused by C_force and C_torque.
        /// Both C_force and C_torque these forces are considered expressed in the
        /// reference coordsystem of marker2 (the MAIN marker),
        /// and their application point is the origin of marker1 (the SLAVE marker).
        public override void ConstraintsFbLoadForces(double factor = 1)
        {
            if (Body1 == null || Body2 == null)
                return;

            ChVector mbody_force = new ChVector();
            ChVector mbody_torque = new ChVector();
            if (ChVector.Vnotnull(C_force))
            {
                ChVector m_abs_force = Body2.GetA().Matr_x_Vect(marker2.FrameMoving.GetA().Matr_x_Vect(C_force));
                Body2.To_abs_forcetorque(m_abs_force,
                                          marker1.GetAbsCoord().pos,  // absolute application point is always marker1
                                          false,                       // from abs. space
                                          ref mbody_force, ref mbody_torque);  // resulting force-torque, both in abs coords
                Body2.Variables().Get_fb().PasteSumVector(mbody_force * -factor, 0, 0);
                Body2.Variables().Get_fb().PasteSumVector(Body2.TransformDirectionParentToLocal(mbody_torque) * -factor, 3,
                                                           0);

                Body1.To_abs_forcetorque(m_abs_force,
                                          marker1.GetAbsCoord().pos,  // absolute application point is always marker1
                                          false,                       // from abs. space
                                          ref mbody_force, ref mbody_torque);  // resulting force-torque, both in abs coords
                Body1.Variables().Get_fb().PasteSumVector(mbody_force * factor, 0, 0);
                Body1.Variables().Get_fb().PasteSumVector(Body1.TransformDirectionParentToLocal(mbody_torque) * factor, 3, 0);
            }
            if (ChVector.Vnotnull(C_torque))
            {
                ChVector m_abs_torque = Body2.GetA().Matr_x_Vect(marker2.FrameMoving.GetA().Matr_x_Vect(C_torque));
                // load torques in 'fb' vector accumulator of body variables (torques in local coords)
                Body1.Variables().Get_fb().PasteSumVector(Body1.TransformDirectionParentToLocal(m_abs_torque) * factor, 3, 0);
                Body2.Variables().Get_fb().PasteSumVector(Body2.TransformDirectionParentToLocal(m_abs_torque) * -factor, 3,
                                                           0);
            }
        }

        //
        // LINK COORDINATES and other functions:
        //

        /// Relative position of marker 1 respect to marker 2.
        public ChCoordsys<double> GetRelM() { return relM; }

        /// Relative speed of marker 1 respect to marker 2.
        public ChCoordsys<double> GetRelM_dt() { return relM_dt; }
        /// Relative acceleration of marker 1 respect to marker 2.
        public ChCoordsys<double> GetRelM_dtdt() { return relM_dtdt; }
        /// Relative rotation angle of marker 1 respect to marker 2 (best with revolute joints..).
        public double GetRelAngle() { return relAngle; }
        /// Relative finite rotation axis of marker 1 respect to marker 2.
        public ChVector GetRelAxis() { return relAxis; }
        public ChVector GetRelRotaxis() { return relRotaxis; }
        /// Relative angular speed of marker 1 respect to marker 2.
        public ChVector GetRelWvel() { return relWvel; }
        /// Relative angular acceleration of marker 1 respect to marker 2.
        public ChVector GetRelWacc() { return relWacc; }
        /// Relative 'polar' distance of marker 1 respect to marker 2.
        public double GetDist() { return dist; }
        /// Relative speed of marker 1 respect to marker 2, along the polar distance vector.
        public double GetDist_dt() { return dist_dt; }

        /// To get & set the 'script' force buffers(only accessed by
        /// external scripts, so it's up to the script to remember
        /// to set& reset them -link class just add them to C_force etc.
        public ChVector Get_Scr_force() { return Scr_force; }
        public ChVector Get_Scr_torque() { return Scr_torque; }
        public void Set_Scr_force(ChVector mf) { Scr_force = mf; }
        public void Set_Scr_torque(ChVector mf) { Scr_torque = mf; }

        /// Get the total applied force accumulators (force, momentum) in link coords.
        /// These forces might be affected by additional springs, dampers, etc. but they do not
        /// include the reaction forces.
        public ChVector GetC_force() { return C_force; }
        public ChVector GetC_torque() { return C_torque; }

    }
}
