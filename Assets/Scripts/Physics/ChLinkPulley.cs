using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;

namespace chrono
{

    /// Class to create pulleys on two rigid bodies, connected by a belt.
    /// The two bodies must be already connected to a truss by other
    /// links, for example two revolute joints (ChLinkLockRevolute), because
    /// this link constraints only the rotation.

    public class ChLinkPulley : ChLinkLock {


        public double tau;       //< transmission coeff.
        protected double r1;        //< radius of pulley in body1
        protected double r2;        //< radius of pulley in body2
        public double phase;     //< mounting phase angle
        public bool checkphase;  //< keep pulleys always on phase

        protected double a1;  //< auxiliary
        protected double a2;  //< auxiliary

        protected double shaft_dist;  //< distance between shafts

        protected ChVector belt_up1;   //< upper segment of belt - end on body1.
        protected ChVector belt_up2;   //< upper segment of belt - end on body2.
        protected ChVector belt_low1;  //< lower segment of belt - end on body1.
        protected ChVector belt_low2;  //< lower segment of belt - end on body2.

        protected ChFrame<double> local_shaft1 = new ChFrame<double>();  //< shaft1 pos & dir (as Z axis), relative to body1
        protected ChFrame<double> local_shaft2 = new ChFrame<double>();  //< shaft2 pos & dir (as Z axis), relative to body2

        public Vector3 shaft1Origin;
        public Vector3 shaft1Direction;

        public Vector3 shaft2Origin;
        public Vector3 shaft2Direction;

        public Material lineMat;
        private LineRenderer line1;
        private LineRenderer line2;

        Vector3 up1 = new Vector3();
        Vector3 up2 = new Vector3();

        public ChLinkPulley()
        {
            a1 = 0;
            a2 = 0;
            r1 = 1;
            r2 = 1;
            tau = 1;
            phase = 0;
            checkphase = false;
            shaft_dist = 0;
            belt_up1 = ChVector.VNULL;
            belt_up2 = ChVector.VNULL;
            belt_low1 = ChVector.VNULL;
            belt_low2 = ChVector.VNULL;
            // initializes type
            local_shaft1.SetIdentity();
            local_shaft2.SetIdentity();

            // Mask: initialize our LinkMaskLF (lock formulation mask)
            // to X  only. It was a LinkMaskLF because this class inherited from LinkLock.
            ((ChLinkMaskLF)mask).SetLockMask(true, false, false, false, false, false, false);
            ChangedLinkMask();
        }
        public ChLinkPulley(ChLinkPulley other) {
            tau = other.tau;
            phase = other.phase;
            a1 = other.a1;
            a2 = other.a2;
            checkphase = other.checkphase;
            r1 = other.r1;
            r2 = other.r2;
            belt_up1 = other.belt_up1;
            belt_up2 = other.belt_up2;
            belt_low1 = other.belt_low1;
            belt_low2 = other.belt_low2;
            local_shaft1 = other.local_shaft1;
            local_shaft2 = other.local_shaft2;
            shaft_dist = other.shaft_dist;
        }

        /// "Virtual" copy constructor (covariant return type).
        public override ChObj Clone() { return new ChLinkPulley(this); }

        public override void Start()
        {
            // shaft1Direction = new Vector3(0, 0, 1);
            // shaft2Direction = new Vector3(0, 0, 1);

           // line1 = new GameObject().AddComponent<LineRenderer>();
           // line2 = new GameObject().AddComponent<LineRenderer>();

            ChCoordsys csys = new ChCoordsys(Utils.ToChrono(transform.position), Utils.ToChrono(transform.rotation));
            Initialize(body1, body2, csys);

            //// TODO: Check that this is correct.
            var rot1 = UnityEngine.Quaternion.Euler(shaft1Direction);
            //var rot1 = Quaternion.LookRotation(shaft1Direction.normalized);
            Set_local_shaft1(new ChFrame<double>(Utils.ToChrono(shaft1Origin), Utils.ToChrono(rot1)));
            var rot2 = UnityEngine.Quaternion.Euler(shaft2Direction);
            //var rot2 = Quaternion.LookRotation(shaft2Direction.normalized);
            Set_local_shaft2(new ChFrame<double>(Utils.ToChrono(shaft2Origin), Utils.ToChrono(rot2)));

            var height = 2 * transform.localScale.y;
            var pulley2height = Body2.GetType();
            Set_r1(4);
            Set_r2(2);

            //ChSystem msystem = FindObjectOfType<ChSystem>();
            // msystem.AddLink(this);
            ChSystem.system.AddLink(this);
        }

        public void Update()
        {
            DrawPulleyLines();
        }


        void DrawPulleyLines()
         {
            up1.x = (float)Get_belt_up1().x;
            up1.y = (float)Get_belt_up1().y;
            up1.z = (float)Get_belt_up1().z;
            //Vector3 up2 = new Vector3();
            up2.x = (float)Get_belt_up2().x;
            up2.y = (float)Get_belt_up2().y;
            up2.z = (float)Get_belt_up2().z;

            Vector3 low1 = new Vector3();
            low1.x = (float)Get_belt_low1().x;
            low1.y = (float)Get_belt_low1().y;
            low1.z = (float)Get_belt_low1().z;
            Vector3 low2 = new Vector3();
            low2.x = (float)Get_belt_low2().x;
            low2.y = (float)Get_belt_low2().y;
            low2.z = (float)Get_belt_low2().z;

           /* line1.material = lineMat;
            line1.SetWidth(0.1f, 0.1f);
            line1.SetPosition(0, up1);
            line1.SetPosition(1, up2);
            // line2.material = lineMat;
            // line2.SetWidth(0.1f, 0.1f);
            line1.SetPosition(0, low1);
            line1.SetPosition(1, low2);*/

        }


        /// Updates motion laws, marker positions, etc.
        public override void UpdateTime(double mytime) {
            // First, inherit to parent class
            base.UpdateTime(mytime);

            ChFrame<double> abs_shaft1 = ChFrame<double>.FNULL; //new ChFrame<double>();
            ChFrame<double> abs_shaft2 = ChFrame<double>.FNULL; //new ChFrame<double>();

            ((ChFrame<double>)Body1).TransformLocalToParent(local_shaft1, abs_shaft1);
            ((ChFrame<double>)Body2).TransformLocalToParent(local_shaft2, abs_shaft2);

            ChVector dcc_w = ChVector.Vsub(Get_shaft_pos2(), Get_shaft_pos1());

            // compute actual rotation of the two wheels (relative to truss).
            ChVector md1 = abs_shaft1.GetA().MatrT_x_Vect(dcc_w);
            md1.z = 0;
            md1 = ChVector.Vnorm(md1);
            ChVector md2 = abs_shaft2.GetA().MatrT_x_Vect(dcc_w);
            md2.z = 0;
            md2 = ChVector.Vnorm(md2);

            double periodic_a1 = ChMaths.ChAtan2(md1.x, md1.y);
            double periodic_a2 = ChMaths.ChAtan2(md2.x, md2.y);
            double old_a1 = a1;
            double old_a2 = a2;
            double turns_a1 = Math.Floor(old_a1 / ChMaths.CH_C_2PI);
            double turns_a2 = Math.Floor(old_a2 / ChMaths.CH_C_2PI);
            double a1U = turns_a1 * ChMaths.CH_C_2PI + periodic_a1 + ChMaths.CH_C_2PI;
            double a1M = turns_a1 * ChMaths.CH_C_2PI + periodic_a1;
            double a1L = turns_a1 * ChMaths.CH_C_2PI + periodic_a1 - ChMaths.CH_C_2PI;
            a1 = a1M;
            if (Math.Abs(a1U - old_a1) < Math.Abs(a1M - old_a1))
                a1 = a1U;
            if (Math.Abs(a1L - a1) < Math.Abs(a1M - a1))
                a1 = a1L;
            double a2U = turns_a2 * ChMaths.CH_C_2PI + periodic_a2 + ChMaths.CH_C_2PI;
            double a2M = turns_a2 * ChMaths.CH_C_2PI + periodic_a2;
            double a2L = turns_a2 * ChMaths.CH_C_2PI + periodic_a2 - ChMaths.CH_C_2PI;
            a2 = a2M;
            if (Math.Abs(a2U - old_a2) < Math.Abs(a2M - old_a2))
                a2 = a2U;
            if (Math.Abs(a2L - a2) < Math.Abs(a2M - a2))
                a2 = a2L;

            // correct marker positions if phasing is not correct
            double m_delta = 0;
            if (checkphase)
            {
                double realtau = tau;

                m_delta = a1 - phase - (a2 / realtau);

                if (m_delta > ChMaths.CH_C_PI)
                    m_delta -= (ChMaths.CH_C_2PI);  // range -180..+180 is better than 0...360
                if (m_delta > (ChMaths.CH_C_PI / 4.0))
                    m_delta = (ChMaths.CH_C_PI / 4.0);  // phase correction only in +/- 45°
                if (m_delta < -(ChMaths.CH_C_PI / 4.0))
                    m_delta = -(ChMaths.CH_C_PI / 4.0);
                //***TODO***
            }

            // Move markers 1 and 2 to align them as pulley ends

            ChVector d21_w = dcc_w - Get_shaft_dir1() * ChVector.Vdot(Get_shaft_dir1(), dcc_w);
            ChVector D21_w = ChVector.Vnorm(d21_w);

            shaft_dist = d21_w.Length();

            ChVector U1_w = ChVector.Vcross(Get_shaft_dir1(), D21_w);

            double gamma1 = Math.Acos((r1 - r2) / shaft_dist);

            ChVector Ru_w = D21_w * Math.Cos(gamma1) + U1_w * Math.Sin(gamma1);
            ChVector Rl_w = D21_w * Math.Cos(gamma1) - U1_w * Math.Sin(gamma1);

            belt_up1 = Get_shaft_pos1() + Ru_w * r1;
            belt_low1 = Get_shaft_pos1() + Rl_w * r1;
            belt_up2 = Get_shaft_pos1() + d21_w + Ru_w * r2;
            belt_low2 = Get_shaft_pos1() + d21_w + Rl_w * r2;

            // marker alignment
            ChMatrix33<double> maU = new ChMatrix33<double>(0);
            ChMatrix33<double> maL = new ChMatrix33<double>(0);

            ChVector Dxu = ChVector.Vnorm(belt_up2 - belt_up1);
            ChVector Dyu = Ru_w;
            ChVector Dzu = ChVector.Vnorm(ChVector.Vcross(Dxu, Dyu));
            Dyu = ChVector.Vnorm(ChVector.Vcross(Dzu, Dxu));
            maU.Set_A_axis(Dxu, Dyu, Dzu);

            // ! Require that the BDF routine of marker won't handle speed and acc.calculus of the moved marker 2!
            marker2.SetMotionType(ChMarker.eChMarkerMotion.M_MOTION_EXTERNAL);
            marker1.SetMotionType(ChMarker.eChMarkerMotion.M_MOTION_EXTERNAL);

            ChCoordsys newmarkpos = new ChCoordsys();

            // move marker1 in proper positions
            newmarkpos.pos = this.belt_up1;
            newmarkpos.rot = maU.Get_A_quaternion();
            marker1.Impose_Abs_Coord(newmarkpos);  // move marker1 into teeth position
                                                    // move marker2 in proper positions
            newmarkpos.pos = this.belt_up2;
            newmarkpos.rot = maU.Get_A_quaternion();
            marker2.Impose_Abs_Coord(newmarkpos);  // move marker2 into teeth position

            double phase_correction_up = m_delta * r1;
            double phase_correction_low = -phase_correction_up;
            double hU = ChVector.Vlength(belt_up2 - belt_up1) + phase_correction_up;
            double hL = ChVector.Vlength(belt_low2 - belt_low1) + phase_correction_low;

            // imposed relative positions/speeds
            deltaC.pos = new ChVector(-hU, 0, 0);
            deltaC_dt.pos = ChVector.VNULL;
            deltaC_dtdt.pos = ChVector.VNULL;

            deltaC.rot = ChQuaternion.QUNIT;  // no relative rotations imposed!
            deltaC_dt.rot = ChQuaternion.QNULL;
            deltaC_dtdt.rot = ChQuaternion.QNULL;
        }

        /// Set radius of 1st pulley.
        public void Set_r1(double mr) {
            r1 = mr;
            tau = r1 / r2;
        }

        /// Set radius of 2nd pulley.
        public void Set_r2(double mr) {
            r2 = mr;
            tau = r1 / r2;
        }

        /// Get radius of 1st pulley.        
        public double Get_r1() { return r1; }
        /// Get radius of 2nd pulley.
        public double Get_r2() { return r2; }

        /// Get the transmission ratio. Its value is assumed always positive.
        public double Get_tau() { return tau; }

        /// Get the initial phase of rotation of pulley A.
        public double Get_phase() { return phase; }
        /// Set the initial phase of rotation of pulley A.
        public void Set_phase(double mset) { phase = mset; }

        /// If true, enforce check on exact phase between pulleys
        /// (otherwise after many simulation steps the phasing
        /// may be affected by numerical error accumulation).
        ///  By default, it is turned off, but for the simulation of
        /// synchro belts, this should be better turned on.
        ///  Note that, to ensure the correct phasing during the many
        /// rotations, an algorithm will use the a1 and a2 total rotation
        /// values, which might be affected by loss of numerical precision
        /// after few thousands of revolutions, so this is NOT suited to
        /// real-time simulators which must run for many hours.
        public void Set_checkphase(bool mset) { checkphase = mset; }
        public bool Get_checkphase() { return checkphase; }

        /// Get total rotation of 1st pulley, respect to interaxis, in radians
        public double Get_a1() { return a1; }
        /// Get total rotation of 1st pulley, respect to interaxis, in radians
        public double Get_a2() { return a2; }
        /// Reset the total rotations of a1 and a2.
        public void Reset_a1a2() { a1 = a2 = 0; }

        /// Get shaft position and direction, for 1st pulley, in body1-relative reference.
        /// The shaft direction is the Z axis of that frame.
        public ChFrame<double> Get_local_shaft1() { return local_shaft1; }
        /// Set shaft position and direction, for 1st pulley, in body1-relative reference.
        /// The shaft direction is the Z axis of that frame.  It should be parallel to shaft 2.
        /// Note that the origin of shaft position will be automatically shifted along
        /// shaft direction in order to have both pulleys on same plane.
        public void Set_local_shaft1(ChFrame<double> mf) { local_shaft1 = mf; }

        /// Get shaft position and direction, for 2nd pulley, in body2-relative reference.
        /// The shaft direction is the Z axis of that frame.
        public ChFrame<double> Get_local_shaft2() { return local_shaft2; }
        /// Set shaft position and direction, for 2nd pulley, in body2-relative reference.
        /// The shaft direction is the Z axis of that frame.  It should be parallel to shaft 1.
        public void Set_local_shaft2(ChFrame<double> mf) { local_shaft2 = mf; }

        /// Get shaft direction, for 1st pulley, in absolute reference
        public ChVector Get_shaft_dir1() {
            if (Body1 != null)
            {
                ChFrame<double> absframe = ChFrame<double>.FNULL; //new ChFrame<double>();
                ((ChFrame<double>)Body1).TransformLocalToParent(local_shaft1, absframe);
                return absframe.GetA().Get_A_Zaxis();
            }
            else
                return ChVector.VECT_Z;
        }
        /// Get shaft direction, for 2nd pulley, in absolute reference
        public ChVector Get_shaft_dir2() {
            if (Body1 != null)
            {
                ChFrame<double> absframe = ChFrame<double>.FNULL;// new ChFrame<double>();
                ((ChFrame<double>)Body2).TransformLocalToParent(local_shaft2, absframe);
                return absframe.GetA().Get_A_Zaxis();
            }
            else
                return ChVector.VECT_Z;
        }

        /// Get shaft position, for 1st pulley, in absolute reference
        public ChVector Get_shaft_pos1() {
            if (Body1 != null)
            {
                ChFrame<double> absframe = ChFrame<double>.FNULL;// new ChFrame<double>();
                ((ChFrame<double>)Body1).TransformLocalToParent(local_shaft1, absframe);
                return absframe.GetPos();
            }
            else
                return ChVector.VNULL;
        }
        /// Get shaft position, for 2nd pulley, in absolute reference
        public ChVector Get_shaft_pos2() {
            if (Body1 != null)
            {
                ChFrame<double> absframe = ChFrame<double>.FNULL;// new ChFrame<double>();
                ((ChFrame<double>)Body2).TransformLocalToParent(local_shaft2, absframe);
                return absframe.GetPos();
            }
            else
                return ChVector.VNULL;
        }

        /// Get the endpoint of belt, on pulley of body1, for the 'upper' segment,
        /// in absolute coordinates.
        public ChVector Get_belt_up1() { return belt_up1; }
        /// Get the endpoint of belt, on pulley of body2, for the 'upper' segment,
        /// in absolute coordinates.
        public ChVector Get_belt_up2() { return belt_up2; }
        /// Get the endpoint of belt, on pulley of body1, for the 'lower' segment,
        /// in absolute coordinates.
        public ChVector Get_belt_low1() { return belt_low1; }
        /// Get the endpoint of belt, on pulley of body1, for the 'lower' segment,
        /// in absolute coordinates.
        public ChVector Get_belt_low2() { return belt_low2; }
    }

}
