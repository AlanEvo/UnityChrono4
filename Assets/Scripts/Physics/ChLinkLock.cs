using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;


namespace chrono
{


    /// ChLinkLock class.
    /// This class implements lot of sub types like the revolute
    /// joint, the linear guide, the spherical joint, etc. using
    /// the 'lock formulation'.
    /// Also, it optionally allows the adoption of 'limits' over
    /// upper-lower motions on all the 6 degrees of freedom,
    /// thank to the ChLinkLimit objects.
    [System.Serializable]
    public class ChLinkLock : ChLinkMasked
    {

        protected ChCoordsys relC;       //< relative constraint position: relC = (relM-deltaC)

        protected ChCoordsys relC_dt;    //< relative constraint speed
        protected ChCoordsys relC_dtdt;  //< relative constraint acceleration

        protected ChCoordsys deltaC;       //< user-imposed rel. position
        protected ChCoordsys deltaC_dt;    //< user-imposed rel. speed
        protected ChCoordsys deltaC_dtdt;  //< user-imposed rel. acceleration

        //(only for intermediate calculus)
        protected ChMatrix Cq1_temp = new ChMatrix();  //<
        protected ChMatrix Cq2_temp = new ChMatrix();  //<   the temporary "lock" jacobians,
        protected ChMatrix Qc_temp = new ChMatrix();   //<   i.e. the full x,y,z,r0,r1,r2,r3 joint
        protected ChCoordsys Ct_temp = new ChCoordsys();


        protected ChVector PQw = new ChVector(0, 0, 0);  //< for intermediate calculus (here, for speed reasons)
        protected ChVector PQw_dt = new ChVector(0, 0, 0);
        protected ChVector PQw_dtdt = new ChVector(0, 0, 0);
        protected ChQuaternion q_AD = new ChQuaternion(1, 0, 0, 0);
        protected ChQuaternion q_BC = new ChQuaternion(1, 0, 0, 0);
        protected ChQuaternion q_8 = new ChQuaternion(1, 0, 0, 0);
        protected ChVector q_4 = new ChVector(0, 0, 0);

        // imposed motion
        protected ChFunction motion_X = new ChFunction();     //< user imposed motion for X coord, marker relative
        protected ChFunction motion_Y = new ChFunction();     //< user imposed motion for Y coord, marker relative
        protected ChFunction motion_Z = new ChFunction();     //< user imposed motion for Z coord, marker relative
        protected ChFunction motion_ang = new ChFunction();   //< user imposed angle rotation about axis
        protected ChFunction motion_ang2 = new ChFunction();  //< user imposed angle rotation if three-angles rot.
        protected ChFunction motion_ang3 = new ChFunction();  //< user imposed angle rotation if three-angles rot.
        protected ChVector motion_axis = new ChVector(0, 0, 0);       //< this is the axis for the user imposed rotation
        protected AngleSet angleset;             //< type of rotation (3 Eul angles, angle/axis, etc.)

        // limits
        protected ChLinkLimit limit_X;//= new ChLinkLimit();   //< the upper/lower limits for X dof
        protected ChLinkLimit limit_Y;// = new ChLinkLimit();   //< the upper/lower limits for Y dof
        protected ChLinkLimit limit_Z;// = new ChLinkLimit();   //< the upper/lower limits for Z dof
        protected ChLinkLimit limit_Rx;// = new ChLinkLimit();  //< the upper/lower limits for Rx dof
        protected ChLinkLimit limit_Ry;// = new ChLinkLimit();  //< the upper/lower limits for Ry dof
        protected ChLinkLimit limit_Rz;// = new ChLinkLimit();  //< the upper/lower limits for Rz dof
        protected ChLinkLimit limit_Rp;// = new ChLinkLimit();  //< the polar (conical) limit for "shoulder"rotation
        protected ChLinkLimit limit_D;// = new ChLinkLimit();   //< the polar (conical) limit for "shoulder"rotation


        public ChBody body1;
        public ChBody body2;

        public bool enableLimits = false;
        public bool showLimits = false;
        public double minAngle = 0;
        public double maxAngle = 0;
        public double minDisplacement = 0;
        public double maxDisplacement = 0;

        public double angle;  // current joint relative angle

        // TEST
        ChMatrixNM<IntInterface.Four, IntInterface.Four> mtempQ2 = new ChMatrixNM<IntInterface.Four, IntInterface.Four>();
        ChMatrixNM<IntInterface.Four, IntInterface.Four> CqrR = new ChMatrixNM<IntInterface.Four, IntInterface.Four>();
        ChMatrixNM<IntInterface.Three, IntInterface.Four> body2Gl = new ChMatrixNM<IntInterface.Three, IntInterface.Four>();

        /*  ChVector vtemp1;  // for intermediate calculus
          ChVector vtemp2;
          ChQuaternion qtemp1;
          ChMatrixNM<IntInterface.Three, IntInterface.Four> relGw = new ChMatrixNM<IntInterface.Three, IntInterface.Four>();
          ChQuaternion temp1;// = marker1.FrameMoving.GetCoord_dt().rot;
          ChQuaternion temp2;//
          ChMatrix33<double> m2_Rel_A_dt = new ChMatrix33<double>();
          ChMatrix33<double> m2_Rel_A_dtdt = new ChMatrix33<double>();
          ChMatrix33<double> mtemp1 = new ChMatrix33<double>();
          ChMatrix33<double> mtemp2 = new ChMatrix33<double>();
          ChMatrix33<double> mtemp3 = new ChMatrix33<double>();
          ChMatrixNM<IntInterface.Four, IntInterface.Four> mtempQ1 = new ChMatrixNM<IntInterface.Four, IntInterface.Four>();
          ChMatrixNM<IntInterface.Four, IntInterface.Four> mtempQ2 = new ChMatrixNM<IntInterface.Four, IntInterface.Four>();
          ChMatrix33<double> CqxT = new ChMatrix33<double>();              // the 3x3 piece of Cq_temp for trasl. link, trasl.coords,
          ChMatrixNM<IntInterface.Three, IntInterface.Four> CqxR = new ChMatrixNM<IntInterface.Three, IntInterface.Four>();  // the 3x4 piece of Cq_temp for trasl. link,   rotat. coords,
          ChMatrixNM<IntInterface.Four, IntInterface.Four> CqrR = new ChMatrixNM<IntInterface.Four, IntInterface.Four>();  // the 4x4 piece of Cq_temp for rotat..link,   rotat. coords,
          ChVector Qcx;                 // the 3x1 vector of Qc     for trasl. link
          ChQuaternion Qcr;             // the 4x1 quaternion of Qc for rotat. link
          ChMatrix33<double> P1star = new ChMatrix33<double>();  // [P] star matrix of rel pos of mark1
          ChMatrix33<double> Q2star = new ChMatrix33<double>();  // [Q] star matrix of rel pos of mark2
          ChMatrixNM<IntInterface.Three, IntInterface.Four> body1Gl = new ChMatrixNM<IntInterface.Three, IntInterface.Four>();
          ChMatrixNM<IntInterface.Three, IntInterface.Four> body2Gl = new ChMatrixNM<IntInterface.Three, IntInterface.Four>();
          ChMatrixNM<IntInterface.Three, IntInterface.Four> mGl = new ChMatrixNM<IntInterface.Three, IntInterface.Four>();*/

        /// <summary>
        /// Type of link-lock
        /// </summary>
        public enum LinkType
        {
            LOCK = 0,
            SPHERICAL = 1,
            POINTPLANE = 2,
            POINTLINE = 3,
            CYLINDRICAL = 4,
            PRISMATIC = 5,
            PLANEPLANE = 6,
            OLDHAM = 7,
            REVOLUTE = 8,
            FREE = 9,
            ALIGN = 10,
            PARALLEL = 11,
            PERPEND = 12,
            TRAJECTORY = 13,
            CLEARANCE = 14,
            REVOLUTEPRISMATIC = 15
        };

        /// <summary>
        /// Type of link-lock
        /// </summary>
       // [SerializeField]
        public LinkType type;


        public ChLinkLock()
        {

            type = LinkType.SPHERICAL;
            relC = new ChCoordsys(new ChVector(0, 0, 0), new ChQuaternion(1, 0, 0, 0));// ChCoordsys.CSYSNORM;
            relC_dt = new ChCoordsys(new ChVector(0, 0, 0), new ChQuaternion(0, 0, 0, 0));// ChCoordsys.CSYSNULL;
            relC_dtdt = new ChCoordsys(new ChVector(0, 0, 0), new ChQuaternion(0, 0, 0, 0)); // ChCoordsys.CSYSNULL;
            deltaC = new ChCoordsys(new ChVector(0, 0, 0), new ChQuaternion(1, 0, 0, 0));// ChCoordsys.CSYSNORM;
            deltaC_dt = new ChCoordsys(new ChVector(0, 0, 0), new ChQuaternion(0, 0, 0, 0));//ChCoordsys.CSYSNULL;
            deltaC_dtdt = new ChCoordsys(new ChVector(0, 0, 0), new ChQuaternion(0, 0, 0, 0)); //ChCoordsys.CSYSNULL;
            motion_axis = ChVector.VECT_Z;
            angleset = AngleSet.ANGLE_AXIS;
            // matrices used by lock formulation
            Cq1_temp = new ChMatrixDynamic<double>(7, ChBody.BODY_QDOF);
            Cq2_temp = new ChMatrixDynamic<double>(7, ChBody.BODY_QDOF);
            Qc_temp = new ChMatrixDynamic<double>(7, 1);

            motion_X = new ChFunction_Const(0);  // default: no motion
            motion_Y = new ChFunction_Const(0);
            motion_Z = new ChFunction_Const(0);
            motion_ang = new ChFunction_Const(0);
            motion_ang2 = new ChFunction_Const(0);
            motion_ang3 = new ChFunction_Const(0);

            limit_X = new ChLinkLimit();  // default: inactive limits
            limit_Y = new ChLinkLimit();
            limit_Z = new ChLinkLimit();
            limit_Rx = new ChLinkLimit();
            limit_Ry = new ChLinkLimit();
            limit_Rz = new ChLinkLimit();
            limit_D = new ChLinkLimit();
            limit_Rp = new ChLinkLimit();  // the polar limit;
            limit_Rp.Set_polar(true);

            // delete the class mask created by base constructor
            if (mask != null)
                mask = null;
            // create instead the LF-mask (the extended version, for lock-formulation)
            mask = new ChLinkMaskLF();

            // default type: spherical link
            // Sets the mask, all the matrices, and number of DOC and DOF
            BuildLinkType(LinkType.SPHERICAL);
        }

        public virtual void Awake()
        {
            //lineRenderer = gameObject.AddComponent<LineRenderer>();
            //lineRenderer.startWidth = 0.02f; //thickness of line
            //lineRenderer.endWidth = 0.02f;
            // lineRenderer.positionCount = size;

            ChCoordsys csys = new ChCoordsys(Utils.ToChrono(transform.position), Utils.ToChrono(transform.rotation));
            ChangeLinkType(type);
            // Initialize(body1, body2, csys);
            ChSystem msystem = FindObjectOfType<ChSystem>();
            msystem.AddLink(this);

            if (enableLimits)
            {
                switch (type)
                {
                    case LinkType.FREE:

                        break;
                    case LinkType.LOCK:

                        break;
                    case LinkType.SPHERICAL:

                        break;
                    case LinkType.POINTPLANE:

                        break;
                    case LinkType.POINTLINE:

                        break;
                    case LinkType.REVOLUTE:
                        GetLimit_Rz().Set_active(true);
                        GetLimit_Rz().Set_min(minAngle);
                        GetLimit_Rz().Set_max(maxAngle);
                        break;
                    case LinkType.CYLINDRICAL:

                        break;
                    case LinkType.PRISMATIC:
                        GetLimit_Z().Set_active(true);
                        GetLimit_Z().Set_min(minDisplacement);
                        GetLimit_Z().Set_max(maxDisplacement);
                        break;
                    case LinkType.PLANEPLANE:

                        break;
                    case LinkType.OLDHAM:

                        break;
                    case LinkType.ALIGN:

                        break;
                    case LinkType.PARALLEL:

                        break;
                    case LinkType.PERPEND:

                        break;
                    case LinkType.REVOLUTEPRISMATIC:

                        break;
                    default:

                        break;
                }
            }

            Initialize(body1, body2, csys);
        }
        private static void DrawEllipse(Vector3 pos, Vector3 forward, Vector3 up, float radiusX, float radiusY, int segments, Color color, float duration = 0)
        {
            float angle = 0f;
            UnityEngine.Quaternion rot = UnityEngine.Quaternion.LookRotation(forward, up);
            Vector3 lastPoint = Vector3.zero;
            Vector3 thisPoint = Vector3.zero;

            for (int i = 0; i < segments + 1; i++)
            {
                thisPoint.x = Mathf.Sin(Mathf.Deg2Rad * angle) * radiusX;
                thisPoint.y = Mathf.Cos(Mathf.Deg2Rad * angle) * radiusY;

                if (i > 0)
                {
                    Debug.DrawLine(rot * lastPoint + pos, rot * thisPoint + pos, color, duration);
                }

                lastPoint = thisPoint;
                angle += 360f / segments;
            }
        }


        protected virtual void OnDrawGizmos()
        {
            if (type == LinkType.CYLINDRICAL)
            {
                Color color = Color.green;
                DrawEllipse(transform.position, transform.forward, transform.up, 0.25f * transform.localScale.x, 0.25f * transform.localScale.y, 32, color);
            }


            if (enableLimits)
            {
                if (showLimits)
                {
                    switch (type)
                    {
                        case LinkType.FREE:

                            break;
                        case LinkType.LOCK:

                            break;
                        case LinkType.SPHERICAL:

                            break;
                        case LinkType.POINTPLANE:

                            break;
                        case LinkType.POINTLINE:

                            break;
                        case LinkType.REVOLUTE:
                            float distance = 1.0f;

                            Vector3 r0 = new Vector3(transform.position.x, transform.position.y, transform.position.z);
                            Vector3 r1 = new Vector3(Mathf.Cos((float)minAngle), -Mathf.Sin((float)minAngle), 0);
                            Vector3 r2 = new Vector3(Mathf.Cos((float)maxAngle), -Mathf.Sin((float)maxAngle), 0);
                            Gizmos.color = new Color(0, 255, 0);
                            Gizmos.DrawLine(r0, r0 + distance * r1);
                            Gizmos.color = new Color(255, 0, 0);
                            Gizmos.DrawLine(r0, r0 + distance * r2);
                            break;
                        case LinkType.CYLINDRICAL:

                            break;
                        case LinkType.PRISMATIC:

                            Vector3 p0 = new Vector3(transform.position.x, transform.position.y, transform.position.z);
                            Vector3 p1 = new Vector3(transform.position.x, transform.position.y, transform.position.z + (float)minDisplacement);
                            Vector3 p2 = new Vector3(transform.position.x, transform.position.y, transform.position.z + (float)maxDisplacement);
                            Gizmos.color = new Color(0, 255, 0);
                            Gizmos.DrawLine(p0, p1);
                            Gizmos.DrawLine(p0, p2);

                            break;
                        case LinkType.PLANEPLANE:

                            break;
                        case LinkType.OLDHAM:

                            break;
                        case LinkType.ALIGN:

                            break;
                        case LinkType.PARALLEL:

                            break;
                        case LinkType.PERPEND:

                            break;
                        case LinkType.REVOLUTEPRISMATIC:

                            break;
                        default:

                            break;
                    }
                }
            }
        }

        public void BuildLinkType(LinkType link_type)
        {
            type = link_type;

            ChLinkMaskLF m_mask = new ChLinkMaskLF();

            // SetLockMask() sets the constraints for the link coordinates: (X,Y,Z, E0,E1,E2,E3)
            switch (type)
            {
                case LinkType.FREE:
                    m_mask.SetLockMask(false, false, false, false, false, false, false);
                    break;
                case LinkType.LOCK:
                    m_mask.SetLockMask(true, true, true, false, true, true, true);
                    break;
                case LinkType.SPHERICAL:
                    m_mask.SetLockMask(true, true, true, false, false, false, false);
                    break;
                case LinkType.POINTPLANE:
                    m_mask.SetLockMask(false, false, true, false, false, false, false);
                    break;
                case LinkType.POINTLINE:
                    m_mask.SetLockMask(false, true, true, false, false, false, false);
                    break;
                case LinkType.REVOLUTE:
                    m_mask.SetLockMask(true, true, true, false, true, true, false);
                    break;
                case LinkType.CYLINDRICAL:
                    m_mask.SetLockMask(true, true, false, false, true, true, false);
                    break;
                case LinkType.PRISMATIC:
                    m_mask.SetLockMask(true, true, false, false, true, true, true);
                    break;
                case LinkType.PLANEPLANE:
                    m_mask.SetLockMask(false, false, true, false, true, true, false);
                    break;
                case LinkType.OLDHAM:
                    m_mask.SetLockMask(false, false, true, false, true, true, true);
                    break;
                case LinkType.ALIGN:
                    m_mask.SetLockMask(false, false, false, false, true, true, true);
                    break;
                case LinkType.PARALLEL:
                    m_mask.SetLockMask(false, false, false, false, true, true, false);
                    break;
                case LinkType.PERPEND:
                    m_mask.SetLockMask(false, false, false, false, true, false, true);
                    break;
                case LinkType.REVOLUTEPRISMATIC:
                    m_mask.SetLockMask(false, true, true, false, true, true, false);
                    break;
                default:
                    m_mask.SetLockMask(false, false, false, false, false, false, false);
                    break;
            }

            BuildLink(m_mask);
        }

        public void ChangeLinkType(LinkType new_link_type)
        {
            DestroyLink();
            BuildLinkType(new_link_type);

            // reset all motions and limits!

            motion_X = new ChFunction_Const(0);  // default: no motion
            motion_Y = new ChFunction_Const(0);
            motion_Z = new ChFunction_Const(0);
            motion_ang = new ChFunction_Const(0);
            motion_ang2 = new ChFunction_Const(0);
            motion_ang3 = new ChFunction_Const(0);
            motion_axis = ChVector.VECT_Z;
            angleset = AngleSet.ANGLE_AXIS;

            if (limit_X != null)
                limit_X = null;
            if (limit_Y != null)
                limit_Y = null;
            if (limit_Z != null)
                limit_Z = null;
            if (limit_Rx != null)
                limit_Rx = null;
            if (limit_Ry != null)
                limit_Ry = null;
            if (limit_Rz != null)
                limit_Rz = null;
            if (limit_Rp != null)
                limit_Rp = null;
            if (limit_D != null)
                limit_D = null;

            limit_X = new ChLinkLimit();  // default: inactive limits
            limit_Y = new ChLinkLimit();
            limit_Z = new ChLinkLimit();
            limit_Rx = new ChLinkLimit();
            limit_Ry = new ChLinkLimit();
            limit_Rz = new ChLinkLimit();
            limit_D = new ChLinkLimit();
            limit_Rp = new ChLinkLimit();  // the polar limit;
            limit_Rp.Set_polar(true);
        }

        //
        // UPDATING FUNCTIONS
        //

        // Inherits, and also updates motion laws: deltaC, deltaC_dt, deltaC_dtdt
        public override void UpdateTime(double time)
        {
            base.UpdateTime(time);

            /* double ang, ang_dt, ang_dtdt;
             // If some limit is provided, the delta values may have been
             // changed by limits themselves, so no further modifications by motion laws..
             if (limit_X.Get_active() || limit_Y.Get_active() || limit_Z.Get_active() || limit_Rx.Get_active() ||
                 limit_Ry.Get_active() || limit_Rz.Get_active())
                 return;
             // Update motion position/speed/acceleration by motion laws
             // as expressed by specific link CH functions
             deltaC.pos.x = motion_X.Get_y(time);
             deltaC_dt.pos.x = motion_X.Get_y_dx(time);
             deltaC_dtdt.pos.x= motion_X.Get_y_dxdx(time);
             deltaC.pos.y = motion_Y.Get_y(time);
             deltaC_dt.pos.y = motion_Y.Get_y_dx(time);
             deltaC_dtdt.pos.y = motion_Y.Get_y_dxdx(time);
             deltaC.pos.z = motion_Z.Get_y(time);
             deltaC_dt.pos.z = motion_Z.Get_y_dx(time);
             deltaC_dtdt.pos.z = motion_Z.Get_y_dxdx(time);
             switch (angleset)
             {
                 case AngleSet.ANGLE_AXIS:
                     ang = motion_ang.Get_y(time);
                     ang_dt = motion_ang.Get_y_dx(time);
                     ang_dtdt = motion_ang.Get_y_dxdx(time);
                     if ((ang != 0) || (ang_dt != 0) || (ang_dtdt != 0))
                     {
                         deltaC.rot =  ChQuaternion.Q_from_AngAxis2(ang, motion_axis);
                         deltaC_dt.rot =  ChQuaternion.Qdt_from_AngAxis(deltaC.rot, ang_dt, motion_axis);
                         deltaC_dtdt.rot = ChQuaternion.Qdtdt_from_AngAxis(ang_dtdt, motion_axis, deltaC.rot, deltaC_dt.rot);
                     }
                     else
                     {
                         deltaC.rot =  new ChQuaternion(1, 0, 0, 0); // QUNIT;
                         deltaC_dt.rot =  new ChQuaternion(0, 0, 0, 0); // QNULL;
                         deltaC_dtdt.rot = new ChQuaternion(0, 0, 0, 0); // QNULL;
                     }
                     break;
                 case AngleSet.EULERO:
                 case AngleSet.CARDANO:
                 case AngleSet.HPB:
                 case AngleSet.RXYZ:
                     {
                         ChVector vangles = new ChVector(0, 0, 0);
                         ChVector vangles_dt = new ChVector(0, 0, 0);
                         ChVector vangles_dtdt = new ChVector(0, 0, 0);
                         vangles.x = motion_ang.Get_y(time);
                         vangles.y = motion_ang2.Get_y(time);
                         vangles.z = motion_ang3.Get_y(time);
                         vangles_dt.x = motion_ang.Get_y_dx(time);
                         vangles_dt.y = motion_ang2.Get_y_dx(time);
                         vangles_dt.z = motion_ang3.Get_y_dx(time);
                         vangles_dtdt.x = motion_ang.Get_y_dxdx(time);
                         vangles_dtdt.y = motion_ang2.Get_y_dxdx(time);
                         vangles_dtdt.z= motion_ang3.Get_y_dxdx(time);
                       //  deltaC.rot =  ChQuaternion.Angle_to_Quat(angleset, vangles);
                       //  deltaC_dt.rot =  ChQuaternion.AngleDT_to_QuatDT(angleset, vangles_dt, deltaC.rot);
                       //  deltaC_dtdt.rot =  ChQuaternion.AngleDTDT_to_QuatDTDT(angleset, vangles_dtdt, deltaC.rot);
                         break;
                     }
                 default:
                     break;
             }*/
        }

        // Updates coords relM, relM_dt, relM_dtdt;
        // dist, dist_dt et simila, just like in parent class, but
        // overrides parent implementation of ChLinkMarkers because it can save some
        // temporary vectors (q_4, q_8 etc.) which can be useful in UpdateState(),
        // for speed reasons.
        public override void UpdateRelMarkerCoords()
        {
            // FOR ALL THE 6(or3) COORDINATES OF RELATIVE MOTION OF THE TWO MARKERS.
            //  Also set some static vectors/quaternions which will be used later in the
            // UpdateState function for the Lock-Formulation method (this customization,
            // happens only for speed reasons, otherwise the base UpdateRelMarkerCoords()
            // could be sufficient)

            PQw = ChVector.Vsub(marker1.GetAbsCoord().pos, marker2.GetAbsCoord().pos);
            PQw_dt = ChVector.Vsub(marker1.GetAbsCoord_dt().pos, marker2.GetAbsCoord_dt().pos);
            PQw_dtdt = ChVector.Vsub(marker1.GetAbsCoord_dtdt().pos, marker2.GetAbsCoord_dtdt().pos);

            dist = ChVector.Vlength(PQw);                 // distance between origins, modulus
            dist_dt = ChVector.Vdot(ChVector.Vnorm(PQw), PQw_dt);  // speed between origins, modulus.

            ChVector vtemp1;  // for intermediate calculus
            ChVector vtemp2;
            ChQuaternion qtemp1;
            ChMatrixNM<IntInterface.Three, IntInterface.Four> relGw = new ChMatrixNM<IntInterface.Three, IntInterface.Four>();
            ChQuaternion temp1 = marker1.FrameMoving.GetCoord_dt().rot;
            ChQuaternion temp2 = marker2.FrameMoving.GetCoord_dt().rot;

            if (ChQuaternion.Qnotnull(temp2) || ChQuaternion.Qnotnull(temp1))
            {
                q_AD =  //  q'qqq + qqqq'
                     ChQuaternion.Qadd(ChQuaternion.Qcross(ChQuaternion.Qconjugate(marker2.FrameMoving.GetCoord_dt().rot),
                                 ChQuaternion.Qcross(ChQuaternion.Qconjugate(marker2.GetBody().BodyFrame.GetCoord().rot),
                                        ChQuaternion.Qcross((marker1.GetBody().BodyFrame.GetCoord().rot), (marker1.FrameMoving.GetCoord().rot)))),
                          ChQuaternion.Qcross(ChQuaternion.Qconjugate(marker2.FrameMoving.GetCoord().rot),
                                 ChQuaternion.Qcross(ChQuaternion.Qconjugate(marker2.GetBody().BodyFrame.GetCoord().rot),
                                        ChQuaternion.Qcross((marker1.GetBody().BodyFrame.GetCoord().rot), (marker1.FrameMoving.GetCoord_dt().rot)))));
            }
            else
               // q_AD = new ChQuaternion(0, 0, 0, 0);
             q_AD = ChQuaternion.QNULL;

            q_BC =  // qq'qq + qqq'q
                 ChQuaternion.Qadd(ChQuaternion.Qcross(ChQuaternion.Qconjugate(marker2.FrameMoving.GetCoord().rot),
                             ChQuaternion.Qcross(ChQuaternion.Qconjugate(marker2.GetBody().BodyFrame.GetCoord_dt().rot),
                                    ChQuaternion.Qcross((marker1.GetBody().BodyFrame.GetCoord().rot), (marker1.FrameMoving.GetCoord().rot)))),
                      ChQuaternion.Qcross(ChQuaternion.Qconjugate(marker2.FrameMoving.GetCoord().rot),
                             ChQuaternion.Qcross(ChQuaternion.Qconjugate(marker2.GetBody().BodyFrame.GetCoord().rot),
                                    ChQuaternion.Qcross((marker1.GetBody().BodyFrame.GetCoord_dt().rot), (marker1.FrameMoving.GetCoord().rot)))));

            // q_8 = q''qqq + 2q'q'qq + 2q'qq'q + 2q'qqq'
            //     + 2qq'q'q + 2qq'qq' + 2qqq'q' + qqqq''
            temp2 = marker2.FrameMoving.GetCoord_dtdt().rot;
            if (ChQuaternion.Qnotnull(temp2))
                q_8 = ChQuaternion.Qcross(ChQuaternion.Qconjugate(marker2.FrameMoving.GetCoord_dtdt().rot),
                              ChQuaternion.Qcross(ChQuaternion.Qconjugate(Body2.GetCoord().rot),
                                     ChQuaternion.Qcross(Body1.GetCoord().rot,
                                           marker1.FrameMoving.GetCoord().rot)));  // q_dtdt'm2 * q'o2 * q,o1 * q,m1
            else
               // q_8 = new ChQuaternion(0, 0, 0, 0);
             q_AD = ChQuaternion.QNULL;
            temp1 = marker1.FrameMoving.GetCoord_dtdt().rot;
            if (ChQuaternion.Qnotnull(temp1))
            {
                qtemp1 = ChQuaternion.Qcross(ChQuaternion.Qconjugate(marker2.FrameMoving.GetCoord().rot),
                                 ChQuaternion.Qcross(ChQuaternion.Qconjugate(Body2.GetCoord().rot),
                                        ChQuaternion.Qcross(Body1.GetCoord().rot,
                                              marker1.FrameMoving.GetCoord_dtdt().rot)));  // q'm2 * q'o2 * q,o1 * q_dtdt,m1
                q_8 = ChQuaternion.Qadd(q_8, qtemp1);
            }
            temp2 = marker2.FrameMoving.GetCoord_dt().rot;
            if (ChQuaternion.Qnotnull(temp2))
            {
                qtemp1 = ChQuaternion.Qcross(
                     ChQuaternion.Qconjugate(marker2.FrameMoving.GetCoord_dt().rot),
                     ChQuaternion.Qcross(ChQuaternion.Qconjugate(Body2.GetCoord_dt().rot), ChQuaternion.Qcross(Body1.GetCoord().rot, marker1.FrameMoving.GetCoord().rot)));
                qtemp1 = ChQuaternion.Qscale(qtemp1, 2);  // 2( q_dt'm2 * q_dt'o2 * q,o1 * q,m1)
                q_8 = ChQuaternion.Qadd(q_8, qtemp1);
            }
            temp2 = marker2.FrameMoving.GetCoord_dt().rot;
            if (ChQuaternion.Qnotnull(temp2))
            {
                qtemp1 = ChQuaternion.Qcross(
                     ChQuaternion.Qconjugate(marker2.FrameMoving.GetCoord_dt().rot),
                     ChQuaternion.Qcross(ChQuaternion.Qconjugate(Body2.GetCoord().rot), ChQuaternion.Qcross(Body1.GetCoord_dt().rot, marker1.FrameMoving.GetCoord().rot)));
                qtemp1 = ChQuaternion.Qscale(qtemp1, 2);  // 2( q_dt'm2 * q'o2 * q_dt,o1 * q,m1)
                q_8 = ChQuaternion.Qadd(q_8, qtemp1);
            }
            temp1 = marker1.FrameMoving.GetCoord_dt().rot;
            temp2 = marker2.FrameMoving.GetCoord_dt().rot;
            if (ChQuaternion.Qnotnull(temp2) && ChQuaternion.Qnotnull(temp1))
            {
                qtemp1 = ChQuaternion.Qcross(
                     ChQuaternion.Qconjugate(marker2.FrameMoving.GetCoord_dt().rot),
                     ChQuaternion.Qcross(ChQuaternion.Qconjugate(Body2.GetCoord().rot), ChQuaternion.Qcross(Body1.GetCoord().rot, marker1.FrameMoving.GetCoord_dt().rot)));
                qtemp1 = ChQuaternion.Qscale(qtemp1, 2);  // 2( q_dt'm2 * q'o2 * q,o1 * q_dt,m1)
                q_8 = ChQuaternion.Qadd(q_8, qtemp1);
            }

            qtemp1 =
                 ChQuaternion.Qcross(ChQuaternion.Qconjugate(marker2.FrameMoving.GetCoord().rot),
                        ChQuaternion.Qcross(ChQuaternion.Qconjugate(Body2.GetCoord_dt().rot), ChQuaternion.Qcross(Body1.GetCoord_dt().rot, marker1.FrameMoving.GetCoord().rot)));
            qtemp1 = ChQuaternion.Qscale(qtemp1, 2);  // 2( q'm2 * q_dt'o2 * q_dt,o1 * q,m1)
            q_8 = ChQuaternion.Qadd(q_8, qtemp1);
            temp1 = marker1.FrameMoving.GetCoord_dt().rot;
            if (ChQuaternion.Qnotnull(temp1))
            {
                qtemp1 = ChQuaternion.Qcross(
                     ChQuaternion.Qconjugate(marker2.FrameMoving.GetCoord().rot),
                     ChQuaternion.Qcross(ChQuaternion.Qconjugate(Body2.GetCoord_dt().rot), ChQuaternion.Qcross(Body1.GetCoord().rot, marker1.FrameMoving.GetCoord_dt().rot)));
                qtemp1 = ChQuaternion.Qscale(qtemp1, 2);  // 2( q'm2 * q_dt'o2 * q,o1 * q_dt,m1)
                q_8 = ChQuaternion.Qadd(q_8, qtemp1);
            }
            temp1 = marker1.FrameMoving.GetCoord_dt().rot;
            if (ChQuaternion.Qnotnull(temp1))
            {
                qtemp1 = ChQuaternion.Qcross(
                     ChQuaternion.Qconjugate(marker2.FrameMoving.GetCoord().rot),
                     ChQuaternion.Qcross(ChQuaternion.Qconjugate(Body2.GetCoord().rot), ChQuaternion.Qcross(Body1.GetCoord_dt().rot, marker1.FrameMoving.GetCoord_dt().rot)));
                qtemp1 = ChQuaternion.Qscale(qtemp1, 2);  // 2( q'm2 * q'o2 * q_dt,o1 * q_dt,m1)
                q_8 = ChQuaternion.Qadd(q_8, qtemp1);
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
            relM.rot = ChQuaternion.Qcross(ChQuaternion.Qconjugate(marker2.FrameMoving.GetCoord().rot),
                               ChQuaternion.Qcross(ChQuaternion.Qconjugate(marker2.GetBody().BodyFrame.GetCoord().rot),
                                      ChQuaternion.Qcross((marker1.GetBody().BodyFrame.GetCoord().rot), (marker1.FrameMoving.GetCoord().rot))));

            // relM_dt.pos
            relM_dt.pos = ChVector.Vadd(ChVector.Vadd(m2_Rel_A_dt.MatrT_x_Vect(Body2.GetA().MatrT_x_Vect(PQw)),
                                    marker2.FrameMoving.GetA().MatrT_x_Vect(Body2.GetA_dt().MatrT_x_Vect(PQw))),
                               marker2.FrameMoving.GetA().MatrT_x_Vect(Body2.GetA().MatrT_x_Vect(PQw_dt)));

            // relM_dt.rot
            relM_dt.rot = ChQuaternion.Qadd(q_AD, q_BC);

            // relM_dtdt.pos
            relM_dtdt.pos = ChVector.Vadd(ChVector.Vadd(marker2.FrameMoving.GetA().MatrT_x_Vect(Body2.GetA_dtdt().MatrT_x_Vect(PQw)),
                                      marker2.FrameMoving.GetA().MatrT_x_Vect(Body2.GetA().MatrT_x_Vect(PQw_dtdt))), q_4);

            // relM_dtdt.rot
            qtemp1 = ChQuaternion.Qcross(ChQuaternion.Qconjugate(marker2.FrameMoving.GetCoord().rot),
                             ChQuaternion.Qcross(ChQuaternion.Qconjugate(Body2.GetCoord_dtdt().rot),
                                    ChQuaternion.Qcross(Body1.GetCoord().rot,
                                          marker1.FrameMoving.GetCoord().rot)));  // ( q'm2 * q_dtdt'o2 * q,o1 * q,m1)
                                                                                  // relM_dtdt.rot =  ChQuaternion.Qadd(q_8, qtemp1);
            qtemp1 = ChQuaternion.Qcross(ChQuaternion.Qconjugate(marker2.FrameMoving.GetCoord().rot),
                             ChQuaternion.Qcross(ChQuaternion.Qconjugate(Body2.GetCoord().rot),
                                    ChQuaternion.Qcross(Body1.GetCoord_dtdt().rot,
                                          marker1.FrameMoving.GetCoord().rot)));  // ( q'm2 * q'o2 * q_dtdt,o1 * q,m1)
            relM_dtdt.rot = ChQuaternion.Qadd(relM_dtdt.rot, qtemp1);               // = q_8 + qq''qq + qqq''q


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

        // Given current time and body state, computes
        // the constraint differentiation to get the
        // the state matrices     Cq1,  Cq2,  Qc,  Ct , and also
        // C, C_dt, C_dtd.   ie. the JACOBIAN matrices and friends.
        //  NOTE!! this function uses the fast analytical approach
        // of the "lock formulation".
        public override void UpdateState()
        {
            // ---------------------
            // Updates Cq1_temp, Cq2_temp, Qc_temp,
            // etc., i.e. all LOCK-FORMULATION temp.matrices
            // ---------------------

            ChVector vtemp1;  // for intermediate calculus
            ChVector vtemp2;

            ChMatrix33<double> mtemp1 = new ChMatrix33<double>();
            ChMatrix33<double> mtemp2 = new ChMatrix33<double>();
            ChMatrix33<double> mtemp3 = new ChMatrix33<double>();
            ChMatrixNM<IntInterface.Four, IntInterface.Four> mtempQ1 = new ChMatrixNM<IntInterface.Four, IntInterface.Four>();
            ChMatrixNM<IntInterface.Four, IntInterface.Four> mtempQ2 = new ChMatrixNM<IntInterface.Four, IntInterface.Four>();

            ChMatrix33<double> CqxT = new ChMatrix33<double>();              // the 3x3 piece of Cq_temp for trasl. link, trasl.coords,
            ChMatrixNM<IntInterface.Three, IntInterface.Four> CqxR = new ChMatrixNM<IntInterface.Three, IntInterface.Four>();  // the 3x4 piece of Cq_temp for trasl. link,   rotat. coords,
            ChMatrixNM<IntInterface.Four, IntInterface.Four> CqrR = new ChMatrixNM<IntInterface.Four, IntInterface.Four>();  // the 4x4 piece of Cq_temp for rotat..link,   rotat. coords,
            ChVector Qcx;                 // the 3x1 vector of Qc     for trasl. link
            ChQuaternion Qcr;             // the 4x1 quaternion of Qc for rotat. link

            // [Cq_temp]= [[CqxT] [CqxR]]     {Qc_temp} ={[Qcx]}
            //            [[ 0  ] [CqrR]]                {[Qcr]}

            // ----------- SOME PRECALCULATED VARIABLES, to optimize speed

            ChMatrix33<double> P1star = new ChMatrix33<double>();  // [P] star matrix of rel pos of mark1
            P1star.Set_X_matrix(marker1.FrameMoving.GetCoord().pos);
            ChMatrix33<double> Q2star = new ChMatrix33<double>();  // [Q] star matrix of rel pos of mark2
            Q2star.Set_X_matrix(marker2.FrameMoving.GetCoord().pos);

            ChMatrixNM<IntInterface.Three, IntInterface.Four> body1Gl = new ChMatrixNM<IntInterface.Three, IntInterface.Four>();
            ChMatrixNM<IntInterface.Three, IntInterface.Four> body2Gl = new ChMatrixNM<IntInterface.Three, IntInterface.Four>();

            ChFrame<double>.SetMatrix_Gl(ref body1Gl, Body1.GetCoord().rot);
            ChFrame<double>.SetMatrix_Gl(ref body2Gl, Body2.GetCoord().rot);

            // ----------- RELATIVE LINK-LOCK COORDINATES (violations)

            // relC.pos
            relC.pos = ChVector.Vsub(relM.pos, deltaC.pos);

            // relC.rot
            relC.rot = ChQuaternion.Qcross(ChQuaternion.Qconjugate(deltaC.rot), relM.rot);

            // relC_dt.pos
            relC_dt.pos = ChVector.Vsub(relM_dt.pos, deltaC_dt.pos);

            // relC_dt.rot
            relC_dt.rot = ChQuaternion.Qadd(ChQuaternion.Qcross(ChQuaternion.Qconjugate(deltaC_dt.rot), relM.rot), ChQuaternion.Qcross(ChQuaternion.Qconjugate(deltaC.rot), relM_dt.rot));

            // relC_dtdt.pos
            relC_dtdt.pos = ChVector.Vsub(relM_dtdt.pos, deltaC_dtdt.pos);

            // relC_dtdt.rot
            relC_dtdt.rot =
                 ChQuaternion.Qadd(ChQuaternion.Qadd(ChQuaternion.Qcross(ChQuaternion.Qconjugate(deltaC_dtdt.rot), relM.rot), ChQuaternion.Qcross(ChQuaternion.Qconjugate(deltaC.rot), relM_dtdt.rot)),
                      ChQuaternion.Qscale(ChQuaternion.Qcross(ChQuaternion.Qconjugate(deltaC_dt.rot), relM_dt.rot), 2));

            // +++++++++ COMPUTE THE  Cq Ct Qc    matrices (temporary, for complete lock
            // constraint)

            ChMatrix33<double> m2_Rel_A_dt = new ChMatrix33<double>();
            marker2.FrameMoving.Compute_Adt(ref m2_Rel_A_dt);
            ChMatrix33<double> m2_Rel_A_dtdt = new ChMatrix33<double>();
            marker2.FrameMoving.Compute_Adtdt(ref m2_Rel_A_dtdt);

            // ----------- PARTIAL DERIVATIVE Ct OF CONSTRAINT
            Ct_temp.pos = ChVector.Vadd(m2_Rel_A_dt.MatrT_x_Vect(Body2.GetA().MatrT_x_Vect(PQw)),
                               marker2.FrameMoving.GetA().MatrT_x_Vect(
                                   ChVector.Vsub(Body2.GetA().MatrT_x_Vect(Body1.GetA().Matr_x_Vect(marker1.FrameMoving.GetCoord_dt().pos)),
                                        marker2.FrameMoving.GetCoord_dt().pos)));
            Ct_temp.pos = ChVector.Vsub(Ct_temp.pos, deltaC_dt.pos);  // the deltaC contribute

            Ct_temp.rot =  // deltaC^*(q_AD) + deltaC_dt^*q_pq
                  ChQuaternion.Qadd(ChQuaternion.Qcross(ChQuaternion.Qconjugate(deltaC.rot), q_AD), ChQuaternion.Qcross(ChQuaternion.Qconjugate(deltaC_dt.rot), relM.rot));

            //------------ COMPLETE JACOBIANS Cq1_temp AND Cq2_temp AND Qc_temp VECTOR.

            //  JACOBIANS Cq1_temp, Cq2_temp:

            mtemp1.CopyFromMatrixT(marker2.FrameMoving.GetA());
            CqxT.MatrMultiplyT(mtemp1, Body2.GetA());  // [CqxT]=[Aq]'[Ao2]'

            Cq1_temp.PasteMatrix(CqxT, 0, 0);  // *- -- Cq1_temp(1-3)  =[Aqo2]

            CqxT.MatrNeg();
            Cq2_temp.PasteMatrix(CqxT, 0, 0);  // -- *- Cq2_temp(1-3)  =-[Aqo2]

            mtemp1.MatrMultiply(CqxT, Body1.GetA());
            mtemp2.MatrMultiply(mtemp1, P1star);

            CqxR.MatrMultiply(mtemp2, body1Gl);

            Cq1_temp.PasteMatrix(CqxR, 0, 3);  // -* -- Cq1_temp(4-7)

            CqxT.MatrNeg();
            mtemp1.MatrMultiply(CqxT, Body2.GetA());
            mtemp2.MatrMultiply(mtemp1, Q2star);
            CqxR.MatrMultiply(mtemp2, body2Gl);
            Cq2_temp.PasteMatrix(CqxR, 0, 3);

            mtemp1.CopyFromMatrixT(marker2.FrameMoving.GetA());
            mtemp2.Set_X_matrix(Body2.GetA().MatrT_x_Vect(PQw));
            mtemp3.MatrMultiply(mtemp1, mtemp2);
            CqxR.MatrMultiply(mtemp3, body2Gl);

            Cq2_temp.PasteSumMatrix(CqxR, 0, 3);  // -- -* Cq1_temp(4-7)

            mtempQ1.Set_Xq_matrix(ChQuaternion.Qcross(ChQuaternion.Qconjugate(marker2.FrameMoving.GetCoord().rot), ChQuaternion.Qconjugate(Body2.GetCoord().rot)));
            CqrR.Set_Xq_matrix(marker1.FrameMoving.GetCoord().rot);
            CqrR.MatrXq_SemiTranspose();
            mtempQ2.MatrMultiply(mtempQ1, CqrR);
            mtempQ1.Set_Xq_matrix(ChQuaternion.Qconjugate(deltaC.rot));
            CqrR.MatrMultiply(mtempQ1, mtempQ2);

            Cq1_temp.PasteMatrix(CqrR, 3, 3);  // =* == Cq1_temp(col 4-7, row 4-7)

            mtempQ1.Set_Xq_matrix(ChQuaternion.Qconjugate(marker2.FrameMoving.GetCoord().rot));
            CqrR.Set_Xq_matrix(ChQuaternion.Qcross(Body1.GetCoord().rot, marker1.FrameMoving.GetCoord().rot));
            CqrR.MatrXq_SemiTranspose();
            CqrR.MatrXq_SemiNeg();
            mtempQ2.MatrMultiply(mtempQ1, CqrR);
            mtempQ1.Set_Xq_matrix(ChQuaternion.Qconjugate(deltaC.rot));
            CqrR.MatrMultiply(mtempQ1, mtempQ2);

            Cq2_temp.PasteMatrix(CqrR, 3, 3);  // == =* Cq2_temp(col 4-7, row 4-7)

            //--------- COMPLETE Qc VECTOR

            vtemp1 = ChVector.Vcross(Body1.GetWvel_loc(), ChVector.Vcross(Body1.GetWvel_loc(), marker1.FrameMoving.GetCoord().pos));
            vtemp1 = ChVector.Vadd(vtemp1, marker1.FrameMoving.GetCoord_dtdt().pos);
            vtemp1 = ChVector.Vadd(vtemp1, ChVector.Vmul(ChVector.Vcross(Body1.GetWvel_loc(), marker1.FrameMoving.GetCoord_dt().pos), 2));
            vtemp1 = Body1.GetA().Matr_x_Vect(vtemp1);

            vtemp2 = ChVector.Vcross(Body2.GetWvel_loc(), ChVector.Vcross(Body2.GetWvel_loc(), marker2.FrameMoving.GetCoord().pos));
            vtemp2 = ChVector.Vadd(vtemp2, marker2.FrameMoving.GetCoord_dtdt().pos);
            vtemp2 = ChVector.Vadd(vtemp2, ChVector.Vmul(ChVector.Vcross(Body2.GetWvel_loc(), marker2.FrameMoving.GetCoord_dt().pos), 2));
            vtemp2 = Body2.GetA().Matr_x_Vect(vtemp2);

            vtemp1 = ChVector.Vsub(vtemp1, vtemp2);
            Qcx = CqxT.Matr_x_Vect(vtemp1);

            mtemp1.Set_X_matrix(Body2.GetWvel_loc());
            mtemp2.MatrMultiply(mtemp1, mtemp1);
            mtemp3.MatrMultiply(Body2.GetA(), mtemp2);
            mtemp3.MatrTranspose();
            vtemp1 = mtemp3.Matr_x_Vect(PQw);
            vtemp2 = marker2.FrameMoving.GetA().MatrT_x_Vect(vtemp1);  // [Aq]'[[A2][w2][w2]]'*Qpq,w
            Qcx = ChVector.Vadd(Qcx, vtemp2);

            Qcx = ChVector.Vadd(Qcx, q_4);  // [Adtdt]'[A]'q + 2[Adt]'[Adt]'q + 2[Adt]'[A]'qdt + 2[A]'[Adt]'qdt

            Qcx = ChVector.Vsub(Qcx, deltaC_dtdt.pos);  // ... - deltaC_dtdt

            Qc_temp.PasteVector(Qcx, 0, 0);  // * Qc_temp, for all translational coords

            Qcr = ChQuaternion.Qcross(ChQuaternion.Qconjugate(deltaC.rot), q_8);
            Qcr = ChQuaternion.Qadd(Qcr, ChQuaternion.Qscale(ChQuaternion.Qcross(ChQuaternion.Qconjugate(deltaC_dt.rot), relM_dt.rot), 2));
            Qcr = ChQuaternion.Qadd(Qcr, ChQuaternion.Qcross(ChQuaternion.Qconjugate(deltaC_dtdt.rot), relM.rot));  // = deltaC'*q_8 + 2*deltaC_dt'*q_dt,po +
                                                                                                                    // deltaC_dtdt'*q,po

            Qc_temp.PasteQuaternion(Qcr, 3, 0);  // * Qc_temp, for all rotational coords

            // *** NOTE! The definitive  Qc must change sign, to be used in
            // lagrangian equation:    [Cq]*q_dtdt = Qc
            // because until now we have computed it as [Cq]*q_dtdt + "Qc" = 0,
            // but the most used form is the previous, so let's change sign!!

            Qc_temp.MatrNeg();

            // FINALLY.....
            // ---------------------
            // Updates Cq1, Cq2, Qc,
            // C, C_dt, C_dtdt, Ct.
            // ---------------------
            int index = 0;

            ChLinkMaskLF mmask = (ChLinkMaskLF)this.mask;

            if (mmask.Constr_X().IsActive())  // for X constraint...
            {
                Cq1.PasteClippedMatrix(Cq1_temp, 0, 0, 1, 7, index, 0);
                Cq2.PasteClippedMatrix(Cq2_temp, 0, 0, 1, 7, index, 0);

                Qc.SetElement(index, 0, Qc_temp.GetElement(0, 0));

                C.SetElement(index, 0, relC.pos.x);
                C_dt.SetElement(index, 0, relC_dt.pos.x);
                C_dtdt.SetElement(index, 0, relC_dtdt.pos.x);

                Ct.SetElement(index, 0, Ct_temp.pos.x);

                index++;
            }

            if (mmask.Constr_Y().IsActive())  // for Y constraint...
            {
                Cq1.PasteClippedMatrix(Cq1_temp, 1, 0, 1, 7, index, 0);
                Cq2.PasteClippedMatrix(Cq2_temp, 1, 0, 1, 7, index, 0);

                Qc.SetElement(index, 0, Qc_temp.GetElement(1, 0));

                C.SetElement(index, 0, relC.pos.y);
                C_dt.SetElement(index, 0, relC_dt.pos.y);
                C_dtdt.SetElement(index, 0, relC_dtdt.pos.y);

                Ct.SetElement(index, 0, Ct_temp.pos.y);

                index++;
            }

            if (mmask.Constr_Z().IsActive())  // for Z constraint...
            {
                Cq1.PasteClippedMatrix(Cq1_temp, 2, 0, 1, 7, index, 0);
                Cq2.PasteClippedMatrix(Cq2_temp, 2, 0, 1, 7, index, 0);

                Qc.SetElement(index, 0, Qc_temp.GetElement(2, 0));

                C.SetElement(index, 0, relC.pos.z);
                C_dt.SetElement(index, 0, relC_dt.pos.z);
                C_dtdt.SetElement(index, 0, relC_dtdt.pos.z);

                Ct.SetElement(index, 0, Ct_temp.pos.z);

                index++;
            }

            if (mmask.Constr_E0().IsActive())  // for E0 constraint...
            {
                Cq1.PasteClippedMatrix(Cq1_temp, 3, 3, 1, 4, index, 3);
                Cq2.PasteClippedMatrix(Cq2_temp, 3, 3, 1, 4, index, 3);

                Qc.SetElement(index, 0, Qc_temp.GetElement(3, 0));

                C.SetElement(index, 0, relC.rot.e0);
                C_dt.SetElement(index, 0, relC_dt.rot.e0);
                C_dtdt.SetElement(index, 0, relC_dtdt.rot.e0);

                Ct.SetElement(index, 0, Ct_temp.rot.e0);

                index++;
            }

            if (mmask.Constr_E1().IsActive())  // for E1 constraint...
            {
                Cq1.PasteClippedMatrix(Cq1_temp, 4, 3, 1, 4, index, 3);
                Cq2.PasteClippedMatrix(Cq2_temp, 4, 3, 1, 4, index, 3);

                Qc.SetElement(index, 0, Qc_temp.GetElement(4, 0));

                C.SetElement(index, 0, relC.rot.e1);
                C_dt.SetElement(index, 0, relC_dt.rot.e1);
                C_dtdt.SetElement(index, 0, relC_dtdt.rot.e1);

                Ct.SetElement(index, 0, Ct_temp.rot.e1);

                index++;
            }

            if (mmask.Constr_E2().IsActive())  // for E2 constraint...
            {
                Cq1.PasteClippedMatrix(Cq1_temp, 5, 3, 1, 4, index, 3);
                Cq2.PasteClippedMatrix(Cq2_temp, 5, 3, 1, 4, index, 3);

                Qc.SetElement(index, 0, Qc_temp.GetElement(5, 0));

                C.SetElement(index, 0, relC.rot.e2);
                C_dt.SetElement(index, 0, relC_dt.rot.e2);
                C_dtdt.SetElement(index, 0, relC_dtdt.rot.e2);

                Ct.SetElement(index, 0, Ct_temp.rot.e2);

                index++;
            }

            if (mmask.Constr_E3().IsActive())  // for E3 constraint...
            {
                Cq1.PasteClippedMatrix(Cq1_temp, 6, 3, 1, 4, index, 3);
                Cq2.PasteClippedMatrix(Cq2_temp, 6, 3, 1, 4, index, 3);

                Qc.SetElement(index, 0, Qc_temp.GetElement(6, 0));

                C.SetElement(index, 0, relC.rot.e3);
                C_dt.SetElement(index, 0, relC_dt.rot.e3);
                C_dtdt.SetElement(index, 0, relC_dtdt.rot.e3);

                Ct.SetElement(index, 0, Ct_temp.rot.e3);

                index++;
            }
        }

        // Inherits, and also updates the local F,M forces adding penalties from
        // the contained link ChLinkLimit objects, if any.
        public override void UpdateForces(double mytime)
        {
            // Inherit force computation:
            // also base class can add its own forces.
            base.UpdateForces(mytime);

            // now add:

            // ========== the link-limits "cushion forces"

            /* ChVector m_force = new ChVector(0, 0, 0);
             ChVector m_torque = new ChVector(0, 0, 0);
             if (limit_X.Get_active())
             {
                 m_force.x = limit_X.GetForce(relM.pos.x, relM_dt.pos.x);
             }
             if (limit_Y.Get_active())
             {
                 m_force.y = limit_Y.GetForce(relM.pos.y, relM_dt.pos.y);
             }
             if (limit_Z.Get_active())
             {
                 m_force.z = limit_Z.GetForce(relM.pos.z, relM_dt.pos.z);
             }
             if (limit_D.Get_active())
             {
                 m_force = ChVector.Vadd(m_force, ChVector.Vmul(ChVector.Vnorm(relM.pos), limit_D.GetForce(dist, dist_dt)));
             }
             if (limit_Rx.Get_active())
             {
                 m_torque.x = limit_Rx.GetForce(relRotaxis.x, relWvel.x);
             }
             if (limit_Ry.Get_active())
             {
                 m_torque.y = limit_Ry.GetForce(relRotaxis.y, relWvel.y);
             }
             if (limit_Rz.Get_active())
             {
                 m_torque.z = limit_Rz.GetForce(relRotaxis.z, relWvel.z);
             }
             if (limit_Rp.Get_active())
             {
                 ChVector arm_xaxis =  ChQuaternion.VaxisXfromQuat(relM.rot);  // the X axis of the marker1, respect to m2.
                 double zenith = ChVector.VangleYZplaneNorm(arm_xaxis);     // the angle of m1 Xaxis about normal to YZ plane
                 double polar = ChVector.VangleRX(arm_xaxis);               // the polar angle of m1 Xaxis spinning about m2 Xaxis
                 ChVector projected_arm = new ChVector(0, arm_xaxis.y, arm_xaxis.z);
                 ChVector torq_axis;
                 torq_axis = ChVector.Vcross(ChVector.VECT_X, projected_arm);
                 torq_axis = ChVector. Vnorm(torq_axis);  // the axis of torque, laying on YZ plane.
                 double zenithspeed = ChVector.Vdot(torq_axis, relWvel);  // the speed of zenith rotation toward cone.
                 m_torque = ChVector.Vadd(m_torque, ChVector.Vmul(torq_axis, limit_Rp.GetPolarForce(zenith, zenithspeed, polar)));
             }
             C_force = ChVector.Vadd(C_force, m_force);     // +++
             C_torque = ChVector.Vadd(C_torque, m_torque);  // +++*/

            // ========== other forces??
        }

        //
        // OTHER FUNCTIONS
        //

        // constraint violations in pos/rot coordinates
        public ChCoordsys GetRelC() { return relC; }
        public ChCoordsys GetRelC_dt() { return relC_dt; }
        public ChCoordsys GetRelC_dtdt() { return relC_dtdt; }

        // to get the imposed clearances
        public ChCoordsys GetDeltaC() { return deltaC; }
        public ChCoordsys GetDeltaC_dt() { return deltaC_dt; }
        public ChCoordsys GetDeltaC_dtdt() { return deltaC_dtdt; }
        // to set the imposed clearances (best use SetMotion() if you can..)
        public void SetDeltaC(ChCoordsys mc) { deltaC = mc; }
        public void SetDeltaC_dt(ChCoordsys mc) { deltaC_dt = mc; }
        public void SetDeltaC_dtdt(ChCoordsys mc) { deltaC_dtdt = mc; }

        // for the imposed motion functions
        public ChFunction GetMotion_X() { return motion_X; }
        public ChFunction GetMotion_Y() { return motion_Y; }
        public ChFunction GetMotion_Z() { return motion_Z; }
        public ChFunction GetMotion_ang() { return motion_ang; }
        public ChFunction GetMotion_ang2() { return motion_ang2; }
        public ChFunction GetMotion_ang3() { return motion_ang3; }
        public ChVector GetMotion_axis() { return motion_axis; }
        public void SetMotion_X(ChFunction m_funct)
        {
            motion_X = m_funct;
        }
        public void SetMotion_Y(ChFunction m_funct)
        {
            motion_Y = m_funct;
        }
        public void SetMotion_Z(ChFunction m_funct)
        {
            motion_Z = m_funct;
        }
        public void SetMotion_ang(ChFunction m_funct)
        {
            motion_ang = m_funct;
        }
        public void SetMotion_ang2(ChFunction m_funct)
        {
            motion_ang2 = m_funct;
        }
        public void SetMotion_ang3(ChFunction m_funct)
        {
            motion_ang3 = m_funct;
        }
        public void SetMotion_axis(ChVector m_axis)
        {
            motion_axis = m_axis;
        }

        public AngleSet Get_angleset() { return angleset; }
        public void Set_angleset(AngleSet mset) { angleset = mset; }

        // for the limits on free degrees
        public ChLinkLimit GetLimit_X() { return limit_X; }
        public ChLinkLimit GetLimit_Y() { return limit_Y; }
        public ChLinkLimit GetLimit_Z() { return limit_Z; }
        public ChLinkLimit GetLimit_Rx() { return limit_Rx; }
        public ChLinkLimit GetLimit_Ry() { return limit_Ry; }
        public ChLinkLimit GetLimit_Rz() { return limit_Rz; }
        public ChLinkLimit GetLimit_Rp() { return limit_Rp; }
        public ChLinkLimit GetLimit_D() { return limit_D; }
        public void SetLimit_X(ChLinkLimit m_limit_X)
        {
            if (limit_X == null)
                limit_X = null;
            limit_X = m_limit_X;
        }
        public void SetLimit_Y(ChLinkLimit m_limit_Y)
        {
            if (limit_Y == null)
                limit_Y = null;
            limit_Y = m_limit_Y;
        }
        public void SetLimit_Z(ChLinkLimit m_limit_Z)
        {
            if (limit_Z == null)
                limit_Z = null;
            limit_Z = m_limit_Z;
        }
        public void SetLimit_Rx(ChLinkLimit m_limit_Rx)
        {
            if (limit_Rx == null)
                limit_Rx = null;
            limit_Rx = m_limit_Rx;
        }
        public void SetLimit_Ry(ChLinkLimit m_limit_Ry)
        {
            if (limit_Ry == null)
                limit_Ry = null;
            limit_Ry = m_limit_Ry;
        }
        public void SetLimit_Rz(ChLinkLimit m_limit_Rz)
        {
            if (limit_Rz == null)
                limit_Rz = null;
            limit_Rz = m_limit_Rz;
        }
        public void SetLimit_Rp(ChLinkLimit m_limit_Rp)
        {
            if (limit_Rp == null)
                limit_Rp = null;
            limit_Rp = m_limit_Rp;
        }
        public void SetLimit_D(ChLinkLimit m_limit_D)
        {
            if (limit_D == null)
                limit_D = null;
            limit_D = m_limit_D;
        }

        //
        // STATE FUNCTIONS
        //

        /// Get the number of scalar constraints, if any, in this item
        public override int GetDOC()
        {
            return GetDOC_c() + GetDOC_d();
        }
        /// Get the number of scalar constraints, if any, in this item (only bilateral constr.)
        // virtual int GetDOC_c  () {return 0;} // use parent ChLinkMasked ndof
        /// Get the number of scalar constraints, if any, in this item (only unilateral constr.)
        public override int GetDOC_d()   // customized because there might be some active ChLinkLimit
        {
            int mdocd = base.GetDOC_d();

            if (limit_X != null && limit_X.Get_active())
            {
                if (limit_X.constr_lower.IsActive())
                    ++mdocd;
                if (limit_X.constr_upper.IsActive())
                    ++mdocd;
            }
            if (limit_Y != null && limit_Y.Get_active())
            {
                if (limit_Y.constr_lower.IsActive())
                    ++mdocd;
                if (limit_Y.constr_upper.IsActive())
                    ++mdocd;
            }
            if (limit_Z != null && limit_Z.Get_active())
            {
                if (limit_Z.constr_lower.IsActive())
                    ++mdocd;
                if (limit_Z.constr_upper.IsActive())
                    ++mdocd;
            }
            if (limit_Rx != null && limit_Rx.Get_active())
            {
                if (limit_Rx.constr_lower.IsActive())
                    ++mdocd;
                if (limit_Rx.constr_upper.IsActive())
                    ++mdocd;
            }
            if (limit_Ry != null && limit_Ry.Get_active())
            {
                if (limit_Ry.constr_lower.IsActive())
                    ++mdocd;
                if (limit_Ry.constr_upper.IsActive())
                    ++mdocd;
            }
            if (limit_Rz != null && limit_Rz.Get_active())
            {
                if (limit_Rz.constr_lower.IsActive())
                    ++mdocd;
                if (limit_Rz.constr_upper.IsActive())
                    ++mdocd;
            }

            return mdocd;
        }

        /// Specialize the following respect to ChLinkMasked base ,in order to update intuitive react_torque and react_force
        public override void IntStateScatterReactions(int off_L, ChVectorDynamic<double> L)
        {
            // parent (from ChConstraint objects to react vector)
            base.IntStateScatterReactions(off_L, L);

            // From react vector to the 'intuitive' react_force and react_torque
            /* ChQuaternion q2 = Body2.GetRot();
             ChQuaternion q1p = marker1.GetAbsCoord().rot;
             ChQuaternion qs = marker2.FrameMoving.GetCoord().rot;
             ChMatrix33<double> Cs = marker2.FrameMoving.GetA();
             ChMatrixNM<IntInterface.Three, IntInterface.Four> Gl_q2 = new ChMatrixNM<IntInterface.Three, IntInterface.Four>();
             Body1.SetMatrix_G(ref Gl_q2, q2);
             ChMatrixNM<IntInterface.Four, IntInterface.Four> Chi__q1p_barT = new ChMatrixNM<IntInterface.Four, IntInterface.Four>();  //[Chi] * [transpose(bar(q1p))]
             Chi__q1p_barT[0, 0] = q1p.e0;
             Chi__q1p_barT[0, 1] = q1p.e1;
             Chi__q1p_barT[0, 2] = q1p.e2;
             Chi__q1p_barT[0, 3] = q1p.e3;
             Chi__q1p_barT[1, 0] = q1p.e1;
             Chi__q1p_barT[1, 1] = -q1p.e0;
             Chi__q1p_barT[1, 2] = q1p.e3;
             Chi__q1p_barT[1, 3] = -q1p.e2;
             Chi__q1p_barT[2, 0] = q1p.e2;
             Chi__q1p_barT[2, 1] = -q1p.e3;
             Chi__q1p_barT[2, 2] = -q1p.e0;
             Chi__q1p_barT[2, 3] = q1p.e1;
             Chi__q1p_barT[3, 0] = q1p.e3;
             Chi__q1p_barT[3, 1] = q1p.e2;
             Chi__q1p_barT[3, 2] = -q1p.e1;
             Chi__q1p_barT[3, 3] = -q1p.e0;
             ChMatrixNM<double, IntInterface.Four, IntInterface.Four> qs_tilde = new ChMatrixNM<double, IntInterface.Four, IntInterface.Four>();
             qs_tilde[0, 0] = qs.e0;
             qs_tilde[0, 1] = -qs.e1;
             qs_tilde[0, 2] = -qs.e2;
             qs_tilde[0, 3] = -qs.e3;
             qs_tilde[1, 0] = qs.e1;
             qs_tilde[1, 1] = qs.e0;
             qs_tilde[1, 2] = -qs.e3;
             qs_tilde[1, 3] = qs.e2;
             qs_tilde[2, 0] = qs.e2;
             qs_tilde[2, 1] = qs.e3;
             qs_tilde[2, 2] = qs.e0;
             qs_tilde[2, 3] = -qs.e1;
             qs_tilde[3, 0] = qs.e3;
             qs_tilde[3, 1] = -qs.e2;
             qs_tilde[3, 2] = qs.e1;
             qs_tilde[3, 3] = qs.e0;
             // Ts = 0.5*CsT*G(q2)*Chi*(q1 qp)_barT*qs~*KT*lambda
             ChMatrixNM<double, IntInterface.Three, IntInterface.Four> Ts = new ChMatrixNM<double, IntInterface.Three, IntInterface.Four>();
             ChMatrixNM<double, IntInterface.Three, IntInterface.Four> Temp = new ChMatrixNM<double, IntInterface.Three, IntInterface.Four>();  // temp matrix since MatrMultiply overwrites
                                                // "this" during the calculation.  i.e.
                                                // Ts.MatrMultiply(Ts,A) ~= Ts=[Ts]*[A]
             Ts.MatrTMultiply(Cs, Gl_q2);
             Ts.MatrScale(0.25);
             Temp.MatrMultiply(Ts, Chi__q1p_barT);
             Ts.MatrMultiply(Temp, qs_tilde);
             // Translational constraint reaction force = -lambda_translational
             // Translational constraint reaction torque = -d~''(t)*lambda_translational
             // No reaction force from the rotational constraints
             ChLinkMaskLF mmask = (ChLinkMaskLF)(mask);
             int local_off = 0;
             if (mmask.Constr_X().IsActive())
             {
                 react_force.x = -react.GetElement(local_off, 0);
                 react_torque.y = -relM.pos.z * react.GetElement(local_off, 0);
                 react_torque.z = relM.pos.y * react.GetElement(local_off, 0);
                 local_off++;
             }
             if (mmask.Constr_Y().IsActive())
             {
                 react_force.y = -react.GetElement(local_off, 0);
                 react_torque.x = relM.pos.z * react.GetElement(local_off, 0);
                 react_torque.z += -relM.pos.x * react.GetElement(local_off, 0);
                 local_off++;
             }
             if (mmask.Constr_Z().IsActive())
             {
                 react_force.z = -react.GetElement(local_off, 0);
                 react_torque.x += -relM.pos.y * react.GetElement(local_off, 0);
                 react_torque.y += relM.pos.x * react.GetElement(local_off, 0);
                 local_off++;
             }
             if (mmask.Constr_E1().IsActive())
             {
                 react_torque.x += Ts[0, 1] * (react.GetElement(local_off, 0));
                 react_torque.y += Ts[1, 1] * (react.GetElement(local_off, 0));
                 react_torque.z += Ts[2, 1] * (react.GetElement(local_off, 0));
                 local_off++;
             }
             if (mmask.Constr_E2().IsActive())
             {
                 react_torque.x += Ts[0, 2] * (react.GetElement(local_off, 0));
                 react_torque.y += Ts[1, 2] * (react.GetElement(local_off, 0));
                 react_torque.z += Ts[2, 2] * (react.GetElement(local_off, 0));
                 local_off++;
             }
             if (mmask.Constr_E3().IsActive())
             {
                 react_torque.x += Ts[0, 3] * (react.GetElement(local_off, 0));
                 react_torque.y += Ts[1, 3] * (react.GetElement(local_off, 0));
                 react_torque.z += Ts[2, 3] * (react.GetElement(local_off, 0));
                 local_off++;
             }
             // ***TO DO***?: TRASFORMATION FROM delta COORDS TO LINK COORDS, if
             // non-default delta
             // if delta rotation?
             // add also the contribution from link limits to the react_force and
             // react_torque.
             if (limit_X != null && limit_X.Get_active())
             {
                 if (limit_X.constr_lower.IsActive())
                 {
                     react_force.x -= L[off_L + local_off];
                     local_off++;
                 }
                 if (limit_X.constr_upper.IsActive())
                 {
                     react_force.x += L[off_L + local_off];
                     local_off++;
                 }
             }
             if (limit_Y != null && limit_Y.Get_active())
             {
                 if (limit_Y.constr_lower.IsActive())
                 {
                     react_force.y -= L[off_L + local_off];
                     local_off++;
                 }
                 if (limit_Y.constr_upper.IsActive())
                 {
                     react_force.y += L[off_L + local_off];
                     local_off++;
                 }
             }
             if (limit_Z != null && limit_Z.Get_active())
             {
                 if (limit_Z.constr_lower.IsActive())
                 {
                     react_force.z -= L[off_L + local_off];
                     local_off++;
                 }
                 if (limit_Z.constr_upper.IsActive())
                 {
                     react_force.z += L[off_L + local_off];
                     local_off++;
                 }
             }
             if (limit_Rx != null && limit_Rx.Get_active())
             {
                 if (limit_Rx.constr_lower.IsActive())
                 {
                     react_torque.x -= 0.5 * L[off_L + local_off];
                     local_off++;
                 }
                 if (limit_Rx.constr_upper.IsActive())
                 {
                     react_torque.x += 0.5 * L[off_L + local_off];
                     local_off++;
                 }
             }
             if (limit_Ry != null && limit_Ry.Get_active())
             {
                 if (limit_Ry.constr_lower.IsActive())
                 {
                     react_torque.y -= 0.5 * L[off_L + local_off];
                     local_off++;
                 }
                 if (limit_Ry.constr_upper.IsActive())
                 {
                     react_torque.y += 0.5 * L[off_L + local_off];
                     local_off++;
                 }
             }
             if (limit_Rz != null && limit_Rz.Get_active())
             {
                 if (limit_Rz.constr_lower.IsActive())
                 {
                     react_torque.z -= 0.5 * L[off_L + local_off];
                     local_off++;
                 }
                 if (limit_Rz.constr_upper.IsActive())
                 {
                     react_torque.z += 0.5 * L[off_L + local_off];
                     local_off++;
                 }
             }*/

            // the internal forces add their contribute to the reactions
            // NOT NEEDED?, since C_force and react_force must stay separated???
            // react_force  = Vadd(react_force, C_force);
            // react_torque = Vadd(react_torque, C_torque);
        }

        /// Specialize the following respect to ChLinkMasked base because there might be some active ChLinkLimit
        // virtual void IntLoadResidual_F(const unsigned int off,	ChVectorDynamic<>& R, const double c );

        public override void IntStateGatherReactions(int off_L, ref ChVectorDynamic<double> L)
        {
            // parent (from ChConstraint objects to react vector)
            base.IntStateGatherReactions(off_L, ref L);

            int local_off = this.GetDOC_c();

            // gather also the contribution from link limits
            // TODO not yet implemented
        }

        public override void IntLoadResidual_CqL(int off_L,
                                     ref ChVectorDynamic<double> R,
                                     ChVectorDynamic<double> L,
                                     double c)
        {
            // parent class:
             base.IntLoadResidual_CqL(off_L, ref R, L, c);

            /* int local_offset = this.GetDOC_c();
             if (limit_X != null && limit_X.Get_active())
             {
                 if (limit_X.constr_lower.IsActive())
                 {
                     limit_X.constr_lower.MultiplyTandAdd(R, L[off_L + local_offset] * c);
                     ++local_offset;
                 }
                 if (limit_X.constr_upper.IsActive())
                 {
                     limit_X.constr_upper.MultiplyTandAdd(R, L[off_L + local_offset] * c);
                     ++local_offset;
                 }
             }
             if (limit_Y != null && limit_Y.Get_active())
             {
                 if (limit_Y.constr_lower.IsActive())
                 {
                     limit_Y.constr_lower.MultiplyTandAdd(R, L[off_L + local_offset] * c);
                     ++local_offset;
                 }
                 if (limit_Y.constr_upper.IsActive())
                 {
                     limit_Y.constr_upper.MultiplyTandAdd(R, L[off_L + local_offset] * c);
                     ++local_offset;
                 }
             }
             if (limit_Z != null && limit_Z.Get_active())
             {
                 if (limit_Z.constr_lower.IsActive())
                 {
                     limit_Z.constr_lower.MultiplyTandAdd(R, L[off_L + local_offset] * c);
                     ++local_offset;
                 }
                 if (limit_Z.constr_upper.IsActive())
                 {
                     limit_Z.constr_upper.MultiplyTandAdd(R, L[off_L + local_offset] * c);
                     ++local_offset;
                 }
             }
             if (limit_Rx != null && limit_Rx.Get_active())
             {
                 if (limit_Rx.constr_lower.IsActive())
                 {
                     limit_Rx.constr_lower.MultiplyTandAdd(R, L[off_L + local_offset] * c);
                     ++local_offset;
                 }
                 if (limit_Rx.constr_upper.IsActive())
                 {
                     limit_Rx.constr_upper.MultiplyTandAdd(R, L[off_L + local_offset] * c);
                     ++local_offset;
                 }
             }
             if (limit_Ry != null && limit_Ry.Get_active())
             {
                 if (limit_Ry.constr_lower.IsActive())
                 {
                     limit_Ry.constr_lower.MultiplyTandAdd(R, L[off_L + local_offset] * c);
                     ++local_offset;
                 }
                 if (limit_Ry.constr_upper.IsActive())
                 {
                     limit_Ry.constr_upper.MultiplyTandAdd(R, L[off_L + local_offset] * c);
                     ++local_offset;
                 }
             }
             if (limit_Rz != null && limit_Rz.Get_active())
             {
                 if (limit_Rz.constr_lower.IsActive())
                 {
                     limit_Rz.constr_lower.MultiplyTandAdd(R, L[off_L + local_offset] * c);
                     ++local_offset;
                 }
                 if (limit_Rz.constr_upper.IsActive())
                 {
                     limit_Rz.constr_upper.MultiplyTandAdd(R, L[off_L + local_offset] * c);
                     ++local_offset;
                 }
             }*/
        }
        public override void IntLoadConstraint_C(int off_L,
                                     ref ChVectorDynamic<double> Qc,
                                     double c,
                                     bool do_clamp,
                                     double recovery_clamp)
        {
            // parent class:
            base.IntLoadConstraint_C(off_L, ref Qc, c, do_clamp, recovery_clamp);

           /* if (!do_clamp)
                recovery_clamp = 1e24;

            int local_offset = this.GetDOC_c();

            if (limit_X != null && limit_X.Get_active())
            {
                if (limit_X.constr_lower.IsActive())
                {
                    Qc[off_L + local_offset] += ChMaths.ChMax(c * (-limit_X.Get_min() + relM.pos.x), -recovery_clamp);
                    ++local_offset;
                }
                if (limit_X.constr_upper.IsActive())
                {
                    Qc[off_L + local_offset] += ChMaths.ChMax(c * (limit_X.Get_max() - relM.pos.x), -recovery_clamp);
                    ++local_offset;
                }
            }
            if (limit_Y != null && limit_Y.Get_active())
            {
                if (limit_Y.constr_lower.IsActive())
                {
                    Qc[off_L + local_offset] += ChMaths.ChMax(c * (-limit_Y.Get_min() + relM.pos.y), -recovery_clamp);
                    ++local_offset;
                }
                if (limit_Y.constr_upper.IsActive())
                {
                    Qc[off_L + local_offset] += ChMaths.ChMax(c * (limit_Y.Get_max() - relM.pos.y), -recovery_clamp);
                    ++local_offset;
                }
            }
            if (limit_Z != null && limit_Z.Get_active())
            {
                if (limit_Z.constr_lower.IsActive())
                {
                    Qc[off_L + local_offset] += ChMaths.ChMax(c * (-limit_Z.Get_min() + relM.pos.z), -recovery_clamp);
                    ++local_offset;
                }
                if (limit_Z.constr_upper.IsActive())
                {
                    Qc[off_L + local_offset] += ChMaths.ChMax(c * (limit_Z.Get_max() - relM.pos.z), -recovery_clamp);
                    ++local_offset;
                }
            }
            if (limit_Rx != null && limit_Rx.Get_active())
            {
                if (limit_Rx.constr_lower.IsActive())
                {
                    Qc[off_L + local_offset] += ChMaths.ChMax(c * (-Math.Sin(0.5 * limit_Rx.Get_min()) + relM.rot.e1), -recovery_clamp);
                    ++local_offset;
                }
                if (limit_Rx.constr_upper.IsActive())
                {
                    Qc[off_L + local_offset] += ChMaths.ChMax(c * (Math.Sin(0.5 * limit_Rx.Get_max()) - relM.rot.e1), -recovery_clamp);
                    ++local_offset;
                }
            }
            if (limit_Ry != null && limit_Ry.Get_active())
            {
                if (limit_Ry.constr_lower.IsActive())
                {
                    Qc[off_L + local_offset] += ChMaths.ChMax(c * (-Math.Sin(0.5 * limit_Ry.Get_min()) + relM.rot.e2), -recovery_clamp);
                    ++local_offset;
                }
                if (limit_Ry.constr_upper.IsActive())
                {
                    Qc[off_L + local_offset] += ChMaths.ChMax(c * (Math.Sin(0.5 * limit_Ry.Get_max()) - relM.rot.e2), -recovery_clamp);
                    ++local_offset;
                }
            }
            if (limit_Rz != null && limit_Rz.Get_active())
            {
                if (limit_Rz.constr_lower.IsActive())
                {
                    Qc[off_L + local_offset] += ChMaths.ChMax(c * (-Math.Sin(0.5 * limit_Rz.Get_min()) + relM.rot.e3), -recovery_clamp);
                    ++local_offset;
                }
                if (limit_Rz.constr_upper.IsActive())
                {
                    Qc[off_L + local_offset] += ChMaths.ChMax(c * (Math.Sin(0.5 * limit_Rz.Get_max()) - relM.rot.e3), -recovery_clamp);
                    ++local_offset;
                }
            }*/
        }
        public override void IntLoadConstraint_Ct(int off_L, ref ChVectorDynamic<double> Qc, double c)
        {
            // parent class:
            base.IntLoadConstraint_Ct(off_L, ref Qc, c);

            // nothing to do for ChLinkLimit
        }
        public override void IntToDescriptor(int off_v,
                                 ChStateDelta v,
                                 ChVectorDynamic<double> R,
                                 int off_L,
                                 ChVectorDynamic<double> L,
                                 ChVectorDynamic<double> Qc)
        {
            // parent class:
            base.IntToDescriptor(off_v, v, R, off_L, L, Qc);

          /*  int local_offset = this.GetDOC_c();

            if (limit_X != null && limit_X.Get_active())
            {
                if (limit_X.constr_lower.IsActive())
                {
                    limit_X.constr_lower.Set_l_i(L[off_L + local_offset]);
                    limit_X.constr_lower.Set_b_i(Qc[off_L + local_offset]);
                    ++local_offset;
                }
                if (limit_X.constr_upper.IsActive())
                {
                    limit_X.constr_upper.Set_l_i(L[off_L + local_offset]);
                    limit_X.constr_upper.Set_b_i(Qc[off_L + local_offset]);
                    ++local_offset;
                }
            }
            if (limit_Y != null && limit_Y.Get_active())
            {
                if (limit_Y.constr_lower.IsActive())
                {
                    limit_Y.constr_lower.Set_l_i(L[off_L + local_offset]);
                    limit_Y.constr_lower.Set_b_i(Qc[off_L + local_offset]);
                    ++local_offset;
                }
                if (limit_Y.constr_upper.IsActive())
                {
                    limit_Y.constr_upper.Set_l_i(L[off_L + local_offset]);
                    limit_Y.constr_upper.Set_b_i(Qc[off_L + local_offset]);
                    ++local_offset;
                }
            }
            if (limit_Z != null && limit_Z.Get_active())
            {
                if (limit_Z.constr_lower.IsActive())
                {
                    limit_Z.constr_lower.Set_l_i(L[off_L + local_offset]);
                    limit_Z.constr_lower.Set_b_i(Qc[off_L + local_offset]);
                    ++local_offset;
                }
                if (limit_Z.constr_upper.IsActive())
                {
                    limit_Z.constr_upper.Set_l_i(L[off_L + local_offset]);
                    limit_Z.constr_upper.Set_b_i(Qc[off_L + local_offset]);
                    ++local_offset;
                }
            }
            if (limit_Rx != null && limit_Rx.Get_active())
            {
                if (limit_Rx.constr_lower.IsActive())
                {
                    limit_Rx.constr_lower.Set_l_i(L[off_L + local_offset]);
                    limit_Rx.constr_lower.Set_b_i(Qc[off_L + local_offset]);
                    ++local_offset;
                }
                if (limit_Rx.constr_upper.IsActive())
                {
                    limit_Rx.constr_upper.Set_l_i(L[off_L + local_offset]);
                    limit_Rx.constr_upper.Set_b_i(Qc[off_L + local_offset]);
                    ++local_offset;
                }
            }
            if (limit_Ry != null && limit_Ry.Get_active())
            {
                if (limit_Ry.constr_lower.IsActive())
                {
                    limit_Ry.constr_lower.Set_l_i(L[off_L + local_offset]);
                    limit_Ry.constr_lower.Set_b_i(Qc[off_L + local_offset]);
                    ++local_offset;
                }
                if (limit_Ry.constr_upper.IsActive())
                {
                    limit_Ry.constr_upper.Set_l_i(L[off_L + local_offset]);
                    limit_Ry.constr_upper.Set_b_i(Qc[off_L + local_offset]);
                    ++local_offset;
                }
            }
            if (limit_Rz != null && limit_Rz.Get_active())
            {
                if (limit_Rz.constr_lower.IsActive())
                {
                    limit_Rz.constr_lower.Set_l_i(L[off_L + local_offset]);
                    limit_Rz.constr_lower.Set_b_i(Qc[off_L + local_offset]);
                    ++local_offset;
                }
                if (limit_Rz.constr_upper.IsActive())
                {
                    limit_Rz.constr_upper.Set_l_i(L[off_L + local_offset]);
                    limit_Rz.constr_upper.Set_b_i(Qc[off_L + local_offset]);
                    ++local_offset;
                }
            }*/
        }
        public override void IntFromDescriptor(int off_v,
                                   ref ChStateDelta v,
                                   int off_L,
                                   ref ChVectorDynamic<double> L)
        {
            // parent class:
            base.IntFromDescriptor(off_L, ref v, off_L, ref L);

             /* int local_offset = this.GetDOC_c();
              if (limit_X != null && limit_X.Get_active())
              {
                  if (limit_X.constr_lower.IsActive())
                  {
                      L[off_L + local_offset] = limit_X.constr_lower.Get_l_i();
                      ++local_offset;
                  }
                  if (limit_X.constr_upper.IsActive())
                  {
                      L[off_L + local_offset] = limit_X.constr_upper.Get_l_i();
                      ++local_offset;
                  }
              }
              if (limit_Y != null && limit_Y.Get_active())
              {
                  if (limit_Y.constr_lower.IsActive())
                  {
                      L[off_L + local_offset] = limit_Y.constr_lower.Get_l_i();
                      ++local_offset;
                  }
                  if (limit_Y.constr_upper.IsActive())
                  {
                      L[off_L + local_offset] = limit_Y.constr_upper.Get_l_i();
                      ++local_offset;
                  }
              }
              if (limit_Z != null && limit_Z.Get_active())
              {
                  if (limit_Z.constr_lower.IsActive())
                  {
                      L[off_L + local_offset] = limit_Z.constr_lower.Get_l_i();
                      ++local_offset;
                  }
                  if (limit_Z.constr_upper.IsActive())
                  {
                      L[off_L + local_offset] = limit_Z.constr_upper.Get_l_i();
                      ++local_offset;
                  }
              }
              if (limit_Rx != null && limit_Rx.Get_active())
              {
                  if (limit_Rx.constr_lower.IsActive())
                  {
                      L[off_L + local_offset] = limit_Rx.constr_lower.Get_l_i();
                      ++local_offset;
                  }
                  if (limit_Rx.constr_upper.IsActive())
                  {
                      L[off_L + local_offset] = limit_Rx.constr_upper.Get_l_i();
                      ++local_offset;
                  }
              }
              if (limit_Ry != null && limit_Ry.Get_active())
              {
                  if (limit_Ry.constr_lower.IsActive())
                  {
                      L[off_L + local_offset] = limit_Ry.constr_lower.Get_l_i();
                      ++local_offset;
                  }
                  if (limit_Ry.constr_upper.IsActive())
                  {
                      L[off_L + local_offset] = limit_Ry.constr_upper.Get_l_i();
                      ++local_offset;
                  }
              }
              if (limit_Rz != null && limit_Rz.Get_active())
              {
                  if (limit_Rz.constr_lower.IsActive())
                  {
                      L[off_L + local_offset] = limit_Rz.constr_lower.Get_l_i();
                      ++local_offset;
                  }
                  if (limit_Rz.constr_upper.IsActive())
                  {
                      L[off_L + local_offset] = limit_Rz.constr_upper.Get_l_i();
                      ++local_offset;
                  }
              }*/
        }

        //
        // SOLVER SYSTEM FUNCTIONS
        //

        // expand parent constraint stuff from ChLinkMasked because here
        // it may also consider the	constraints caused by 'limits'..
        public override void InjectConstraints(ref ChSystemDescriptor mdescriptor)
        {
            // parent
            base.InjectConstraints(ref mdescriptor);

            if (limit_X != null && limit_X.Get_active())
            {
                if (limit_X.constr_lower.IsActive())
                {
                    limit_X.constr_lower.SetVariables(Body1.Variables(), Body2.Variables());
                    mdescriptor.InsertConstraint(limit_X.constr_lower);
                }
                if (limit_X.constr_upper.IsActive())
                {
                    limit_X.constr_upper.SetVariables(Body1.Variables(), Body2.Variables());
                    mdescriptor.InsertConstraint(limit_X.constr_upper);
                }
            }
            if (limit_Y != null && limit_Y.Get_active())
            {
                if (limit_Y.constr_lower.IsActive())
                {
                    limit_Y.constr_lower.SetVariables(Body1.Variables(), Body2.Variables());
                    mdescriptor.InsertConstraint(limit_Y.constr_lower);
                }
                if (limit_Y.constr_upper.IsActive())
                {
                    limit_Y.constr_upper.SetVariables(Body1.Variables(), Body2.Variables());
                    mdescriptor.InsertConstraint(limit_Y.constr_upper);
                }
            }
            if (limit_Z != null && limit_Z.Get_active())
            {
                if (limit_Z.constr_lower.IsActive())
                {
                    limit_Z.constr_lower.SetVariables(Body1.Variables(), Body2.Variables());
                    mdescriptor.InsertConstraint(limit_Z.constr_lower);
                }
                if (limit_Z.constr_upper.IsActive())
                {
                    limit_Z.constr_upper.SetVariables(Body1.Variables(), Body2.Variables());
                    mdescriptor.InsertConstraint(limit_Z.constr_upper);
                }
            }
            if (limit_Rx != null && limit_Rx.Get_active())
            {
                if (limit_Rx.constr_lower.IsActive())
                {
                    limit_Rx.constr_lower.SetVariables(Body1.Variables(), Body2.Variables());
                    mdescriptor.InsertConstraint(limit_Rx.constr_lower);
                }
                if (limit_Rx.constr_upper.IsActive())
                {
                    limit_Rx.constr_upper.SetVariables(Body1.Variables(), Body2.Variables());
                    mdescriptor.InsertConstraint(limit_Rx.constr_upper);
                }
            }
            if (limit_Ry != null && limit_Ry.Get_active())
            {
                if (limit_Ry.constr_lower.IsActive())
                {
                    limit_Ry.constr_lower.SetVariables(Body1.Variables(), Body2.Variables());
                    mdescriptor.InsertConstraint(limit_Ry.constr_lower);
                }
                if (limit_Ry.constr_upper.IsActive())
                {
                    limit_Ry.constr_upper.SetVariables(Body1.Variables(), Body2.Variables());
                    mdescriptor.InsertConstraint(limit_Ry.constr_upper);
                }
            }
            if (limit_Rz != null && limit_Rz.Get_active())
            {
                if (limit_Rz.constr_lower.IsActive())
                {
                    limit_Rz.constr_lower.SetVariables(Body1.Variables(), Body2.Variables());
                    mdescriptor.InsertConstraint(limit_Rz.constr_lower);
                }
                if (limit_Rz.constr_upper.IsActive())
                {
                    limit_Rz.constr_upper.SetVariables(Body1.Variables(), Body2.Variables());
                    mdescriptor.InsertConstraint(limit_Rz.constr_upper);
                }
            }
        }
        public override void ConstraintsBiReset()
        {
            // parent
            base.ConstraintsBiReset();

             /* if (limit_X != null && limit_X.Get_active())
              {
                  if (limit_X.constr_lower.IsActive())
                  {
                      limit_X.constr_lower.Set_b_i(0.0);
                  }
                  if (limit_X.constr_upper.IsActive())
                  {
                      limit_X.constr_upper.Set_b_i(0.0);
                  }
              }
              if (limit_Y != null && limit_Y.Get_active())
              {
                  if (limit_Y.constr_lower.IsActive())
                  {
                      limit_Y.constr_lower.Set_b_i(0.0);
                  }
                  if (limit_Y.constr_upper.IsActive())
                  {
                      limit_Y.constr_upper.Set_b_i(0.0);
                  }
              }
              if (limit_Z != null && limit_Z.Get_active())
              {
                  if (limit_Z.constr_lower.IsActive())
                  {
                      limit_Z.constr_lower.Set_b_i(0.0);
                  }
                  if (limit_Z.constr_upper.IsActive())
                  {
                      limit_Z.constr_upper.Set_b_i(0.0);
                  }
              }
              if (limit_Rx != null && limit_Rx.Get_active())
              {
                  if (limit_Rx.constr_lower.IsActive())
                  {
                      limit_Rx.constr_lower.Set_b_i(0.0);
                  }
                  if (limit_Rx.constr_upper.IsActive())
                  {
                      limit_Rx.constr_upper.Set_b_i(0.0);
                  }
              }
              if (limit_Ry != null && limit_Ry.Get_active())
              {
                  if (limit_Ry.constr_lower.IsActive())
                  {
                      limit_Ry.constr_lower.Set_b_i(0.0);
                  }
                  if (limit_Ry.constr_upper.IsActive())
                  {
                      limit_Ry.constr_upper.Set_b_i(0.0);
                  }
              }
              if (limit_Rz != null && limit_Rz.Get_active())
              {
                  if (limit_Rz.constr_lower.IsActive())
                  {
                      limit_Rz.constr_lower.Set_b_i(0.0);
                  }
                  if (limit_Rz.constr_upper.IsActive())
                  {
                      limit_Rz.constr_upper.Set_b_i(0.0);
                  }
              }*/
        }
        public override void ConstraintsBiLoad_C(double factor = 1, double recovery_clamp = 0.1, bool do_clamp = false)
        {
            // parent
            base.ConstraintsBiLoad_C(factor, recovery_clamp, do_clamp);

             /* if (limit_X != null && limit_X.Get_active())
              {
                  if (limit_X.constr_lower.IsActive())
                  {
                      if (!do_clamp)
                      {
                          limit_X.constr_lower.Set_b_i(limit_X.constr_lower.Get_b_i() +
                                                        factor * (-limit_X.Get_min() + relM.pos.x));
                      }
                      else
                      {
                          limit_X.constr_lower.Set_b_i(limit_X.constr_lower.Get_b_i() +
                                                       ChMaths.ChMax(factor * (-limit_X.Get_min() + relM.pos.x), -recovery_clamp));
                      }
                  }
                  if (limit_X.constr_upper.IsActive())
                  {
                      if (!do_clamp)
                      {
                          limit_X.constr_upper.Set_b_i(limit_X.constr_upper.Get_b_i() +
                                                        factor * (limit_X.Get_max() - relM.pos.x));
                      }
                      else
                      {
                          limit_X.constr_upper.Set_b_i(limit_X.constr_upper.Get_b_i() +
                                                        ChMaths.ChMax(factor * (limit_X.Get_max() - relM.pos.x), -recovery_clamp));
                      }
                  }
              }
              if (limit_Y != null && limit_Y.Get_active())
              {
                  if (limit_Y.constr_lower.IsActive())
                  {
                      if (!do_clamp)
                      {
                          limit_Y.constr_lower.Set_b_i(limit_Y.constr_lower.Get_b_i() +
                                                        factor * (-limit_Y.Get_min() + relM.pos.y));
                      }
                      else
                      {
                          limit_Y.constr_lower.Set_b_i(limit_Y.constr_lower.Get_b_i() +
                                                        ChMaths.ChMax(factor * (-limit_Y.Get_min() + relM.pos.y), -recovery_clamp));
                      }
                  }
                  if (limit_Y.constr_upper.IsActive())
                  {
                      if (!do_clamp)
                      {
                          limit_Y.constr_upper.Set_b_i(limit_Y.constr_upper.Get_b_i() +
                                                        factor * (limit_Y.Get_max() - relM.pos.y));
                      }
                      else
                      {
                          limit_Y.constr_upper.Set_b_i(limit_Y.constr_upper.Get_b_i() +
                                                        ChMaths.ChMax(factor * (limit_Y.Get_max() - relM.pos.y), -recovery_clamp));
                      }
                  }
              }
              if (limit_Z != null && limit_Z.Get_active())
              {
                  if (limit_Z.constr_lower.IsActive())
                  {
                      if (!do_clamp)
                      {
                          limit_Z.constr_lower.Set_b_i(limit_Z.constr_lower.Get_b_i() +
                                                        factor * (-limit_Z.Get_min() + relM.pos.z));
                      }
                      else
                      {
                          limit_Z.constr_lower.Set_b_i(limit_Z.constr_lower.Get_b_i() +
                                                        ChMaths.ChMax(factor * (-limit_Z.Get_min() + relM.pos.z), -recovery_clamp));
                      }
                  }
                  if (limit_Z.constr_upper.IsActive())
                  {
                      if (!do_clamp)
                      {
                          limit_Z.constr_upper.Set_b_i(limit_Z.constr_upper.Get_b_i() +
                                                        factor * (limit_Z.Get_max() - relM.pos.z));
                      }
                      else
                      {
                          limit_Z.constr_upper.Set_b_i(limit_Z.constr_upper.Get_b_i() +
                                                        ChMaths.ChMax(factor * (limit_Z.Get_max() - relM.pos.z), -recovery_clamp));
                      }
                  }
              }
              if (limit_Rx != null && limit_Rx.Get_active())
              {
                  if (limit_Rx.constr_lower.IsActive())
                  {
                      if (!do_clamp)
                      {
                          limit_Rx.constr_lower.Set_b_i(limit_Rx.constr_lower.Get_b_i() +
                                                         factor * (-Math.Sin(0.5 * limit_Rx.Get_min()) + relM.rot.e1));
                      }
                      else
                      {
                          limit_Rx.constr_lower.Set_b_i(
                              limit_Rx.constr_lower.Get_b_i() +
                              ChMaths.ChMax(factor * (-Math.Sin(0.5 * limit_Rx.Get_min()) + relM.rot.e1), -recovery_clamp));
                      }
                  }
                  if (limit_Rx.constr_upper.IsActive())
                  {
                      if (!do_clamp)
                      {
                          limit_Rx.constr_upper.Set_b_i(limit_Rx.constr_upper.Get_b_i() +
                                                         factor * (Math.Sin(0.5 * limit_Rx.Get_max()) - relM.rot.e1));
                      }
                      else
                      {
                          limit_Rx.constr_upper.Set_b_i(
                              limit_Rx.constr_upper.Get_b_i() +
                              ChMaths.ChMax(factor * (Math.Sin(0.5 * limit_Rx.Get_max()) - relM.rot.e1), -recovery_clamp));
                      }
                  }
              }
              if (limit_Ry != null&& limit_Ry.Get_active())
              {
                  if (limit_Ry.constr_lower.IsActive())
                  {
                      if (!do_clamp)
                      {
                          limit_Ry.constr_lower.Set_b_i(limit_Ry.constr_lower.Get_b_i() +
                                                         factor * (-Math.Sin(0.5 * limit_Ry.Get_min()) + relM.rot.e2));
                      }
                      else
                      {
                          limit_Ry.constr_lower.Set_b_i(
                              limit_Ry.constr_lower.Get_b_i() +
                              ChMaths.ChMax(factor * (-Math.Sin(0.5 * limit_Ry.Get_min()) + relM.rot.e2), -recovery_clamp));
                      }
                  }
                  if (limit_Ry.constr_upper.IsActive())
                  {
                      if (!do_clamp)
                      {
                          limit_Ry.constr_upper.Set_b_i(limit_Ry.constr_upper.Get_b_i() +
                                                         factor * (Math.Sin(0.5 * limit_Ry.Get_max()) - relM.rot.e2));
                      }
                      else
                      {
                          limit_Ry.constr_upper.Set_b_i(
                              limit_Ry.constr_upper.Get_b_i() +
                              ChMaths.ChMax(factor * (Math.Sin(0.5 * limit_Ry.Get_max()) - relM.rot.e2), -recovery_clamp));
                      }
                  }
              }
              if (limit_Rz != null && limit_Rz.Get_active())
              {
                  if (limit_Rz.constr_lower.IsActive())
                  {
                      if (!do_clamp)
                      {
                          limit_Rz.constr_lower.Set_b_i(limit_Rz.constr_lower.Get_b_i() +
                                                         factor * (-Math.Sin(0.5 * limit_Rz.Get_min()) + relM.rot.e3));
                      }
                      else
                      {
                          limit_Rz.constr_lower.Set_b_i(
                              limit_Rz.constr_lower.Get_b_i() +
                              ChMaths.ChMax(factor * (-Math.Sin(0.5 * limit_Rz.Get_min()) + relM.rot.e3), -recovery_clamp));
                      }
                  }
                  if (limit_Rz.constr_upper.IsActive())
                  {
                      if (!do_clamp)
                      {
                          limit_Rz.constr_upper.Set_b_i(limit_Rz.constr_upper.Get_b_i() +
                                                         factor * (Math.Sin(0.5 * limit_Rz.Get_max()) - relM.rot.e3));
                      }
                      else
                      {
                          limit_Rz.constr_upper.Set_b_i(
                              limit_Rz.constr_upper.Get_b_i() +
                              ChMaths.ChMax(factor * (Math.Sin(0.5 * limit_Rz.Get_max()) - relM.rot.e3), -recovery_clamp));
                      }
                  }
              }*/
        }
        public override void ConstraintsBiLoad_Ct(double factor = 1)
        {
            // parent
            base.ConstraintsBiLoad_Ct(factor);
        }
        public override void ConstraintsBiLoad_Qc(double factor = 1)
        {
            // parent
            base.ConstraintsBiLoad_Qc(factor);
        }

        public void Transform_Cq_to_Cqw_row(ChMatrix mCq, int qrow, ChMatrix mCqw, int qwrow, ChBodyFrame mbody)
        {
            // translational part - not changed
          /*  mCqw.PasteClippedMatrix(mCq, qrow, 0, 1, 3, qwrow, 0);

            // rotational part [Cq_w] = [Cq_q]*[Gl]'*1/4
            int col, colres;
            double sum;
            ChMatrixNM<IntInterface.Three, IntInterface.Four> mGl = new ChMatrixNM<IntInterface.Three, IntInterface.Four>();
            ChFrame<double>.SetMatrix_Gl(ref mGl, mbody.GetCoord().rot);
            for (colres = 0; colres < 3; colres++)
            {
                sum = 0;
                for (col = 0; col < 4; col++)
                {
                    sum += ((mCq.GetElement(qrow, col + 3)) * (mGl.GetElement(colres, col)));
                }
                mCqw.SetElement(qwrow, colres + 3, sum * 0.25);
            }*/
        }

        public override void ConstraintsLoadJacobians()
        {
            // parent
            base.ConstraintsLoadJacobians();

           /* if (limit_X != null && limit_X.Get_active())
            {
                if (limit_X.constr_lower.IsActive())
                {
                    limit_X.constr_lower.SetVariables(Body1.Variables(), Body2.Variables());
                    Transform_Cq_to_Cqw_row(Cq1_temp, 0, limit_X.constr_lower.Get_Cq_a(), 0, Body1);
                    Transform_Cq_to_Cqw_row(Cq2_temp, 0, limit_X.constr_lower.Get_Cq_b(), 0, Body2);
                }
                if (limit_X.constr_upper.IsActive())
                {
                    limit_X.constr_upper.SetVariables(Body1.Variables(), Body2.Variables());
                    Transform_Cq_to_Cqw_row(Cq1_temp, 0, limit_X.constr_upper.Get_Cq_a(), 0, Body1);
                    Transform_Cq_to_Cqw_row(Cq2_temp, 0, limit_X.constr_upper.Get_Cq_b(), 0, Body2);
                    limit_X.constr_upper.Get_Cq_a().MatrNeg();
                    limit_X.constr_upper.Get_Cq_b().MatrNeg();
                }
            }
            if (limit_Y != null && limit_Y.Get_active())
            {
                if (limit_Y.constr_lower.IsActive())
                {
                    limit_Y.constr_lower.SetVariables(Body1.Variables(), Body2.Variables());
                    Transform_Cq_to_Cqw_row(Cq1_temp, 1, limit_Y.constr_lower.Get_Cq_a(), 0, Body1);
                    Transform_Cq_to_Cqw_row(Cq2_temp, 1, limit_Y.constr_lower.Get_Cq_b(), 0, Body2);
                }
                if (limit_Y.constr_upper.IsActive())
                {
                    limit_Y.constr_upper.SetVariables(Body1.Variables(), Body2.Variables());
                    Transform_Cq_to_Cqw_row(Cq1_temp, 1, limit_Y.constr_upper.Get_Cq_a(), 0, Body1);
                    Transform_Cq_to_Cqw_row(Cq2_temp, 1, limit_Y.constr_upper.Get_Cq_b(), 0, Body2);
                    limit_Y.constr_upper.Get_Cq_a().MatrNeg();
                    limit_Y.constr_upper.Get_Cq_b().MatrNeg();
                }
            }
            if (limit_Z != null && limit_Z.Get_active())
            {
                if (limit_Z.constr_lower.IsActive())
                {
                    limit_Z.constr_lower.SetVariables(Body1.Variables(), Body2.Variables());
                    Transform_Cq_to_Cqw_row(Cq1_temp, 2, limit_Z.constr_lower.Get_Cq_a(), 0, Body1);
                    Transform_Cq_to_Cqw_row(Cq2_temp, 2, limit_Z.constr_lower.Get_Cq_b(), 0, Body2);
                }
                if (limit_Z.constr_upper.IsActive())
                {
                    limit_Z.constr_upper.SetVariables(Body1.Variables(), Body2.Variables());
                    Transform_Cq_to_Cqw_row(Cq1_temp, 2, limit_Z.constr_upper.Get_Cq_a(), 0, Body1);
                    Transform_Cq_to_Cqw_row(Cq2_temp, 2, limit_Z.constr_upper.Get_Cq_b(), 0, Body2);
                    limit_Z.constr_upper.Get_Cq_a().MatrNeg();
                    limit_Z.constr_upper.Get_Cq_b().MatrNeg();
                }
            }
            if (limit_Rx != null && limit_Rx.Get_active())
            {
                if (limit_Rx.constr_lower.IsActive())
                {
                    limit_Rx.constr_lower.SetVariables(Body1.Variables(), Body2.Variables());
                    Transform_Cq_to_Cqw_row(Cq1_temp, 4, limit_Rx.constr_lower.Get_Cq_a(), 0, Body1);
                    Transform_Cq_to_Cqw_row(Cq2_temp, 4, limit_Rx.constr_lower.Get_Cq_b(), 0, Body2);
                }
                if (limit_Rx.constr_upper.IsActive())
                {
                    limit_Rx.constr_upper.SetVariables(Body1.Variables(), Body2.Variables());
                    Transform_Cq_to_Cqw_row(Cq1_temp, 4, limit_Rx.constr_upper.Get_Cq_a(), 0, Body1);
                    Transform_Cq_to_Cqw_row(Cq2_temp, 4, limit_Rx.constr_upper.Get_Cq_b(), 0, Body2);
                    limit_Rx.constr_upper.Get_Cq_a().MatrNeg();
                    limit_Rx.constr_upper.Get_Cq_b().MatrNeg();
                }
            }
            if (limit_Ry != null && limit_Ry.Get_active())
            {
                if (limit_Ry.constr_lower.IsActive())
                {
                    limit_Ry.constr_lower.SetVariables(Body1.Variables(), Body2.Variables());
                    Transform_Cq_to_Cqw_row(Cq1_temp, 5, limit_Ry.constr_lower.Get_Cq_a(), 0, Body1);
                    Transform_Cq_to_Cqw_row(Cq2_temp, 5, limit_Ry.constr_lower.Get_Cq_b(), 0, Body2);
                }
                if (limit_Ry.constr_upper.IsActive())
                {
                    limit_Ry.constr_upper.SetVariables(Body1.Variables(), Body2.Variables());
                    Transform_Cq_to_Cqw_row(Cq1_temp, 5, limit_Ry.constr_upper.Get_Cq_a(), 0, Body1);
                    Transform_Cq_to_Cqw_row(Cq2_temp, 5, limit_Ry.constr_upper.Get_Cq_b(), 0, Body2);
                    limit_Ry.constr_upper.Get_Cq_a().MatrNeg();
                    limit_Ry.constr_upper.Get_Cq_b().MatrNeg();
                }
            }
            if (limit_Rz != null && limit_Rz.Get_active())
            {
                if (limit_Rz.constr_lower.IsActive())
                {
                    limit_Rz.constr_lower.SetVariables(Body1.Variables(), Body2.Variables());
                    Transform_Cq_to_Cqw_row(Cq1_temp, 6, limit_Rz.constr_lower.Get_Cq_a(), 0, Body1);
                    Transform_Cq_to_Cqw_row(Cq2_temp, 6, limit_Rz.constr_lower.Get_Cq_b(), 0, Body2);
                }
                if (limit_Rz.constr_upper.IsActive())
                {
                    limit_Rz.constr_upper.SetVariables(Body1.Variables(), Body2.Variables());
                    Transform_Cq_to_Cqw_row(Cq1_temp, 6, limit_Rz.constr_upper.Get_Cq_a(), 0, Body1);
                    Transform_Cq_to_Cqw_row(Cq2_temp, 6, limit_Rz.constr_upper.Get_Cq_b(), 0, Body2);
                    limit_Rz.constr_upper.Get_Cq_a().MatrNeg();
                    limit_Rz.constr_upper.Get_Cq_b().MatrNeg();
                }
            }*/
        }
        public override void ConstraintsFetch_react(double factor = 1)
        {
            // parent (from ChConstraint objects to react vector)
            base.ConstraintsFetch_react(factor);

            // From react vector to the 'intuitive' react_force and react_torque
            /* ChQuaternion q2 = Body2.GetRot();
             ChQuaternion q1p = marker1.GetAbsCoord().rot;
             ChQuaternion qs = marker2.FrameMoving.GetCoord().rot;
             ChMatrix33<double> Cs = marker2.FrameMoving.GetA();
             ChMatrixNM<IntInterface.Three, IntInterface.Four> Gl_q2 = new ChMatrixNM<IntInterface.Three, IntInterface.Four>();
             Body1.SetMatrix_Gl(ref Gl_q2, q2);
             ChMatrixNM<IntInterface.Four, IntInterface.Four> Chi__q1p_barT = new ChMatrixNM<IntInterface.Four, IntInterface.Four>();  //[Chi] * [transpose(bar(q1p))]
             Chi__q1p_barT[0, 0] = q1p.e0;
             Chi__q1p_barT[0, 1] = q1p.e1;
             Chi__q1p_barT[0, 2] = q1p.e2;
             Chi__q1p_barT[0, 3] = q1p.e3;
             Chi__q1p_barT[1, 0] = q1p.e1;
             Chi__q1p_barT[1, 1] = -q1p.e0;
             Chi__q1p_barT[1, 2] = q1p.e3;
             Chi__q1p_barT[1, 3] = -q1p.e2;
             Chi__q1p_barT[2, 0] = q1p.e2;
             Chi__q1p_barT[2, 1] = -q1p.e3;
             Chi__q1p_barT[2, 2] = -q1p.e0;
             Chi__q1p_barT[2, 3] = q1p.e1;
             Chi__q1p_barT[3, 0] = q1p.e3;
             Chi__q1p_barT[3, 1] = q1p.e2;
             Chi__q1p_barT[3, 2] = -q1p.e1;
             Chi__q1p_barT[3, 3] = -q1p.e0;
             ChMatrixNM<IntInterface.Four, IntInterface.Four> qs_tilde = new ChMatrixNM<IntInterface.Four, IntInterface.Four>();
             qs_tilde[0, 0] = qs.e0;
             qs_tilde[0, 1] = -qs.e1;
             qs_tilde[0, 2] = -qs.e2;
             qs_tilde[0, 3] = -qs.e3;
             qs_tilde[1, 0] = qs.e1;
             qs_tilde[1, 1] = qs.e0;
             qs_tilde[1, 2] = -qs.e3;
             qs_tilde[1, 3] = qs.e2;
             qs_tilde[2, 0] = qs.e2;
             qs_tilde[2, 1] = qs.e3;
             qs_tilde[2, 2] = qs.e0;
             qs_tilde[2, 3] = -qs.e1;
             qs_tilde[3, 0] = qs.e3;
             qs_tilde[3, 1] = -qs.e2;
             qs_tilde[3, 2] = qs.e1;
             qs_tilde[3, 3] = qs.e0;
             // Ts = 0.5*CsT*G(q2)*Chi*(q1 qp)_barT*qs~*KT*lambda
             ChMatrixNM<IntInterface.Three, IntInterface.Four> Ts = new ChMatrixNM<IntInterface.Three, IntInterface.Four>();
             ChMatrixNM<IntInterface.Three, IntInterface.Four> Temp = new ChMatrixNM<IntInterface.Three, IntInterface.Four>();  // temp matrix since MatrMultiply overwrites
                                                // "this" during the calculation.  i.e.
                                                // Ts.MatrMultiply(Ts,A) ~= Ts=[Ts]*[A]
             Ts.MatrTMultiply(Cs, Gl_q2);
             Ts.MatrScale(0.25);
             Temp.MatrMultiply(Ts, Chi__q1p_barT);
             Ts.MatrMultiply(Temp, qs_tilde);
             // Translational constraint reaction force = -lambda_translational
             // Translational constraint reaction torque = -d~''(t)*lambda_translational
             // No reaction force from the rotational constraints
             ChLinkMaskLF mmask = (ChLinkMaskLF)(mask);
             int n_constraint = 0;
             if (mmask.Constr_X().IsActive())
             {
                 react_force.x = -react.GetElement(n_constraint, 0);
                 react_torque.y = -relM.pos.z * react.GetElement(n_constraint, 0);
                 react_torque.z = relM.pos.y * react.GetElement(n_constraint, 0);
                 n_constraint++;
             }
             if (mmask.Constr_Y().IsActive())
             {
                 react_force.y = -react.GetElement(n_constraint, 0);
                 react_torque.x = relM.pos.z * react.GetElement(n_constraint, 0);
                 react_torque.z += -relM.pos.x * react.GetElement(n_constraint, 0);
                 n_constraint++;
             }
             if (mmask.Constr_Z().IsActive())
             {
                 react_force.z = -react.GetElement(n_constraint, 0);
                 react_torque.x += -relM.pos.y * react.GetElement(n_constraint, 0);
                 react_torque.y += relM.pos.x * react.GetElement(n_constraint, 0);
                 n_constraint++;
             }
             if (mmask.Constr_E1().IsActive())
             {
                 react_torque.x += Ts[0, 1] * (react.GetElement(n_constraint, 0));
                 react_torque.y += Ts[1, 1] * (react.GetElement(n_constraint, 0));
                 react_torque.z += Ts[2, 1] * (react.GetElement(n_constraint, 0));
                 n_constraint++;
             }
             if (mmask.Constr_E2().IsActive())
             {
                 react_torque.x += Ts[0, 2] * (react.GetElement(n_constraint, 0));
                 react_torque.y += Ts[1, 2] * (react.GetElement(n_constraint, 0));
                 react_torque.z += Ts[2, 2] * (react.GetElement(n_constraint, 0));
                 n_constraint++;
             }
             if (mmask.Constr_E3().IsActive())
             {
                 react_torque.x += Ts[0, 3] * (react.GetElement(n_constraint, 0));
                 react_torque.y += Ts[1, 3] * (react.GetElement(n_constraint, 0));
                 react_torque.z += Ts[2, 3] * (react.GetElement(n_constraint, 0));
                 n_constraint++;
             }
             // ***TO DO***?: TRASFORMATION FROM delta COORDS TO LINK COORDS, if
             // non-default delta
             // if delta rotation?
             // add also the contribution from link limits to the react_force and
             // react_torque.
             if (limit_X != null && limit_X.Get_active())
             {
                 if (limit_X.constr_lower.IsActive())
                 {
                     react_force.x -= factor * limit_X.constr_lower.Get_l_i();
                 }
                 if (limit_X.constr_upper.IsActive())
                 {
                     react_force.x += factor * limit_X.constr_upper.Get_l_i();
                 }
             }
             if (limit_Y != null && limit_Y.Get_active())
             {
                 if (limit_Y.constr_lower.IsActive())
                 {
                     react_force.y -= factor * limit_Y.constr_lower.Get_l_i();
                 }
                 if (limit_Y.constr_upper.IsActive())
                 {
                     react_force.y += factor * limit_Y.constr_upper.Get_l_i();
                 }
             }
             if (limit_Z != null && limit_Z.Get_active())
             {
                 if (limit_Z.constr_lower.IsActive())
                 {
                     react_force.z -= factor * limit_Z.constr_lower.Get_l_i();
                 }
                 if (limit_Z.constr_upper.IsActive())
                 {
                     react_force.z += factor * limit_Z.constr_upper.Get_l_i();
                 }
             }
             if (limit_Rx != null && limit_Rx.Get_active())
             {
                 if (limit_Rx.constr_lower.IsActive())
                 {
                     react_torque.x -= 0.5 * factor * limit_Rx.constr_lower.Get_l_i();
                 }
                 if (limit_Rx.constr_upper.IsActive())
                 {
                     react_torque.x += 0.5 * factor * limit_Rx.constr_upper.Get_l_i();
                 }
             }
             if (limit_Ry != null && limit_Ry.Get_active())
             {
                 if (limit_Ry.constr_lower.IsActive())
                 {
                     react_torque.y -= 0.5 * factor * limit_Ry.constr_lower.Get_l_i();
                 }
                 if (limit_Ry.constr_upper.IsActive())
                 {
                     react_torque.y += 0.5 * factor * limit_Ry.constr_upper.Get_l_i();
                 }
             }
             if (limit_Rz != null && limit_Rz.Get_active())
             {
                 if (limit_Rz.constr_lower.IsActive())
                 {
                     react_torque.z -= 0.5 * factor * limit_Rz.constr_lower.Get_l_i();
                 }
                 if (limit_Rz.constr_upper.IsActive())
                 {
                     react_torque.z += 0.5 * factor * limit_Rz.constr_upper.Get_l_i();
                 }
             }*/

            // the internal forces add their contribute to the reactions
            // NOT NEEDED?, since C_force and react_force must stay separated???
            // react_force  = Vadd(react_force, C_force);
            // react_torque = Vadd(react_torque, C_torque);
        }

        /* public void SetForce_Ry()
         {
             ChLinkLock_SetForce_Ry(m_ChLink, force_RY.m_ChLinkForce);
         }
         public void SetForce_Rz()
         {
             ChLinkLock_SetForce_Rz(m_ChLink, force_RZ.m_ChLinkForce);
         }*/

        // protected virtual void OnDrawGizmos()
        // {
        /*  if (minAngle > 0 || maxAngle > 0)
          {
              float distance = 0.04f;
              Vector3 p0 = new Vector3(transform.position.x, transform.position.y, transform.position.z);
              Vector3 p1 = new Vector3(Mathf.Cos((float)minAngle), -Mathf.Sin((float)minAngle), 0);
              Vector3 p2 = new Vector3(Mathf.Cos((float)maxAngle), -Mathf.Sin((float)maxAngle), 0);
              Gizmos.color = new Color(0, 255, 0);
              Gizmos.DrawLine(p0, p0 + distance * p1);
              Gizmos.color = new Color(255, 0, 0);
              Gizmos.DrawLine(p0, p0 + distance * p2);
              Gizmos.color = new Color(0, 255, 0);
              Gizmos.DrawLine(transform.position, transform.position + (transform.forward * distance));
          }*/

        // if (Application.isPlaying)
        // {
        /* float scale = 1;
        // float distance = 5;
         ChFrame<double> frAabs = new ChFrame<double>();
         ChFrame<double> frBabs = new ChFrame<double>();
         frAabs = GetMarker1().FrameMoving;
         frBabs = GetMarker2().FrameMoving;
         ChVector p0 = frAabs.GetPos();
         ChVector px = p0 + frAabs.GetA().Get_A_Xaxis() * 0.7 * scale;
         ChVector py = p0 + frAabs.GetA().Get_A_Yaxis() * 0.7 * scale;
         ChVector pz = p0 + frAabs.GetA().Get_A_Zaxis() * 0.7 * scale;
         Gizmos.color = new Color(70, 125, 0);
         Gizmos.DrawLine(new Vector3((float)p0.x, (float)p0.y, (float)p0.z), new Vector3((float)px.x, (float)px.y, (float)px.z));
         Gizmos.color = new Color(70, 0, 125);
         Gizmos.DrawLine(new Vector3((float)p0.x, (float)p0.y, (float)p0.z), new Vector3((float)py.x, (float)py.y, (float)py.z));
         Gizmos.color = new Color(70, 0, 0);
         Gizmos.DrawLine(new Vector3((float)p0.x, (float)p0.y, (float)p0.z), new Vector3((float)pz.x, (float)pz.y, (float)pz.z));
         p0 = frBabs.GetPos();
         px = p0 + frBabs.GetA().Get_A_Xaxis() * scale;
         py = p0 + frBabs.GetA().Get_A_Yaxis() * scale;
         pz = p0 + frBabs.GetA().Get_A_Zaxis() * scale;
         Gizmos.color = new Color(70, 255, 0);
         Gizmos.DrawLine(new Vector3((float)p0.x, (float)p0.y, (float)p0.z), new Vector3((float)px.x, (float)px.y, (float)px.z));
         Gizmos.color = new Color(70, 0, 255);
         Gizmos.DrawLine(new Vector3((float)p0.x, (float)p0.y, (float)p0.z), new Vector3((float)py.x, (float)py.y, (float)py.z));
         Gizmos.color = new Color(70, 0, 0);
         Gizmos.DrawLine(new Vector3((float)p0.x, (float)p0.y, (float)p0.z), new Vector3((float)pz.x, (float)pz.y, (float)pz.z));*/

        // }

        /* float distance = 0.2f;
         Gizmos.color = new Color(0, 255, 0);
         Gizmos.DrawLine(transform.position, transform.position + (transform.forward * distance));
         if (type == LinkType.SPHERICAL)
         {
             Gizmos.color = new Color(0, 255, 180);
             Gizmos.DrawSphere(transform.position, 0.03f);
         }
         if (type == LinkType.REVOLUTE)
         {
             Gizmos.color = new Color(0, 255, 180);
         }*/

        //  }

        /* public ChBody GetBody1()
         {
             ChBody bod = new ChBody();
             bod.m_ChBody = ChLinkLock_GetBody1(m_ChLink);
             return bodyA;
         }
         public ChBody GetBody2()
         {
             ChBody bod = new ChBody();
             bod.m_ChBody = ChLinkLock_GetBody2(m_ChLink);
             return bodyA;
         }*/

    }

}