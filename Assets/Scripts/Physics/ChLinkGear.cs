using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;


namespace chrono
{

    /// Gear link between two rigid bodies. This can
    /// also be used to represent spur and bevel gears, and
    /// it correctly handles the direction of transmitted force
    /// given the teeth pressure angle.

    public class ChLinkGear : ChLinkLock
    {

        protected double tau;       //< transmission coeff.
        protected double alpha;     //< inclination of action line
        protected double beta;      //< helix angle
        protected double phase;     //< mounting phase angle
        public bool checkphase;  //< keep gear always on phase
        public bool epicyclic;   //< epiciclyc (gear 1 is internal to gear2)  if true.

        protected double a1;  //< auxiliary
        protected double a2;  //< auxiliary
        protected double r1;  //< auxiliary
        protected double r2;  //< auxiliary

        protected ChVector contact_pt;

        protected ChFrame<double> local_shaft1 = new ChFrame<double>();  //< shaft1 pos & dir (as Z axis), relative to body1
        protected ChFrame<double> local_shaft2 = new ChFrame<double>();  //< shaft2 pos & dir (as Z axis), relative to body2

        public Vector3 shaft1Origin;
        public Vector3 shaft1Direction;

        public Vector3 shaft2Origin;
        public Vector3 shaft2Direction;

        public double transmissionRatio = 1;
        // public bool enforcePhase = true;
        // public bool epicyclic = false;

        //Debug reasons
        public double debug;

        public ChLinkGear()
        {
            tau = 1;
            alpha = 0;
            beta = 0;
            phase = 0;
            checkphase = false;
            epicyclic = false;
            a1 = 0;
            a2 = 0;
            r1 = 0;
            r2 = 0;
            contact_pt = ChVector.VNULL;
            local_shaft1.SetIdentity();
            local_shaft2.SetIdentity();

            // Mask: initialize our LinkMaskLF (lock formulation mask)
            // to X  only. It was a LinkMaskLF because this class inherited from LinkLock.
            ((ChLinkMaskLF)mask).SetLockMask(true, false, false, false, false, false, false);
            ChangedLinkMask();
        }

        public ChLinkGear(ChLinkGear other) {
            tau = other.tau;
            alpha = other.alpha;
            beta = other.beta;
            phase = other.phase;
            a1 = other.a1;
            a2 = other.a2;
            epicyclic = other.epicyclic;
            checkphase = other.checkphase;
            r1 = other.r1;
            r2 = other.r2;
            contact_pt = other.contact_pt;
            local_shaft1 = other.local_shaft1;
            local_shaft2 = other.local_shaft2;
        }

        /// "Virtual" copy constructor (covariant return type).
        public override ChObj Clone() { return new ChLinkGear(this);
    }

        public override void Start()
        {
           // shaft1Direction = new Vector3(0, 0, 1);
           // shaft2Direction = new Vector3(0, 0, 1);

            ChCoordsys csys = new ChCoordsys(Utils.ToChrono(transform.position), Utils.ToChrono(transform.rotation));
            Initialize(body1, body2, csys);

            //// TODO: Check that this is correct.
            var rot1 = UnityEngine.Quaternion.Euler(shaft1Direction);
            //var rot1 = Quaternion.LookRotation(shaft1Direction.normalized);
            Set_local_shaft1(new ChFrame<double>(Utils.ToChrono(shaft1Origin), Utils.ToChrono(rot1)));
            var rot2 = UnityEngine.Quaternion.Euler(shaft2Direction);
            //var rot2 = Quaternion.LookRotation(shaft2Direction.normalized);
            Set_local_shaft2(new ChFrame<double>(Utils.ToChrono(shaft2Origin), Utils.ToChrono(rot2)));

            Set_tau(transmissionRatio);
            Set_epicyclic(epicyclic);
            Set_checkphase(checkphase);

            //ChSystem msystem = FindObjectOfType<ChSystem>();
            // msystem.AddLink(this);
            ChSystem.system.AddLink(this);
        }

        void Update()
        {
           // ChVector pos = GetMarker2().GetAbsCoord().pos;
           // transform.position = Utils.FromChrono(pos);
        }

        protected void OnDrawGizmos()
        {
            Utils.DrawEllipse(new Vector3((float)GetBody2().GetCoord().pos.x, (float)GetBody2().GetCoord().pos.y, (float)GetBody2().GetCoord().pos.z), transform.forward, transform.up, (float)Get_r1(), (float)Get_r1(), 30, new Color(255, 0, 0));
            Utils.DrawEllipse(new Vector3((float)GetBody2().GetCoord().pos.x, (float)GetBody2().GetCoord().pos.y, (float)GetBody2().GetCoord().pos.z), transform.forward, transform.up, (float)Get_r2(), (float)Get_r2(), 50, new Color(255, 0, 0));
        }

        // Updates motion laws, marker positions, etc.
        public override void UpdateTime(double mytime)
        {
            // First, inherit to parent class
            base.UpdateTime(mytime);

            // Move markers 1 and 2 to align them as gear teeth

            ChMatrix33<double> ma1 = new ChMatrix33<double>(0);
            ChMatrix33<double> ma2 = new ChMatrix33<double>(0);
            ChMatrix33<double> mrotma = new ChMatrix33<double>(0);
            ChMatrix33<double> marot_beta = new ChMatrix33<double>(0);
            ChVector mx;
            ChVector my;
            ChVector mz;
            ChVector mr;
            ChVector mmark1;
            ChVector mmark2;
            ChVector lastX;
            ChVector vrota;
            ChCoordsys newmarkpos = new ChCoordsys(new ChVector(0, 0, 0), new ChQuaternion(1, 0, 0, 0));

            ChFrame<double> abs_shaft1 = ChFrame<double>.FNULL;// new ChFrame<double>();
            ChFrame<double> abs_shaft2 = ChFrame<double>.FNULL; //new ChFrame<double>();

            ((ChFrame<double>)Body1).TransformLocalToParent(local_shaft1, abs_shaft1);
            ((ChFrame<double>)Body2).TransformLocalToParent(local_shaft2, abs_shaft2);

            ChVector vbdist = ChVector.Vsub(Get_shaft_pos1(), Get_shaft_pos2());
            // ChVector Trad1 = ChVector.Vnorm(ChVector.Vcross(Get_shaft_dir1(), ChVector.Vnorm(ChVector.Vcross(Get_shaft_dir1(), vbdist))));
            // ChVector Trad2 = ChVector.Vnorm(ChVector.Vcross(ChVector.Vnorm(ChVector.Vcross(Get_shaft_dir2(), vbdist)), Get_shaft_dir2()));

            double dist = ChVector.Vlength(vbdist);

            // compute actual rotation of the two wheels (relative to truss).
            ChVector md1 = abs_shaft1.GetA().MatrT_x_Vect(-vbdist);
            md1.z = 0;
            md1 = ChVector.Vnorm(md1);
            ChVector md2 = abs_shaft2.GetA().MatrT_x_Vect(-vbdist);
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

            // compute new markers coordsystem alignment
            my = ChVector.Vnorm(vbdist);
            mz = Get_shaft_dir1();
            mx = ChVector.Vnorm(ChVector.Vcross(my, mz));
            mr = ChVector.Vnorm(ChVector.Vcross(mz, mx));
            mz = ChVector.Vnorm(ChVector.Vcross(mx, my));
            ChVector mz2, mx2, mr2, my2;
            my2 = my;
            mz2 = Get_shaft_dir2();
            mx2 = ChVector.Vnorm(ChVector.Vcross(my2, mz2));
            mr2 = ChVector.Vnorm(ChVector.Vcross(mz2, mx2));

            ma1.Set_A_axis(mx, my, mz);

            // rotate csys because of beta
            vrota.x = 0.0;
            vrota.y = beta;
            vrota.z = 0.0;
            mrotma.Set_A_Rxyz(vrota);
            marot_beta.nm.matrix.MatrMultiply(ma1.nm.matrix, mrotma.nm.matrix);
            // rotate csys because of alpha
            vrota.x = 0.0;
            vrota.y = 0.0;
            vrota.z = alpha;
            if (react_force.x < 0)
                vrota.z = alpha;
            else
                vrota.z = -alpha;
            mrotma.Set_A_Rxyz(vrota);
            ma1.nm.matrix.MatrMultiply(marot_beta.nm.matrix, mrotma.nm.matrix);

            ma2.nm.matrix.CopyFromMatrix(ma1.nm.matrix);

            // is a bevel gear?
            double be = Math.Acos(ChVector.Vdot(Get_shaft_dir1(), Get_shaft_dir2()));
            bool is_bevel = true;
            if (Math.Abs(ChVector.Vdot(Get_shaft_dir1(), Get_shaft_dir2())) > 0.96)
                is_bevel = false;

            // compute wheel radii so that:
            //            w2 = - tau * w1
            if (!is_bevel)
            {
                double pardist = ChVector.Vdot(mr, vbdist);
                double inv_tau = 1.0 / tau;
                if (!epicyclic)
                {
                    r2 = pardist - pardist / (inv_tau + 1.0);
                }
                else
                {
                    r2 = pardist - (tau * pardist) / (tau - 1.0);
                }
                r1 = r2 * tau;
            }
            else
            {
                double gamma2;
                if (!epicyclic)
                {
                    gamma2 = be / (tau + 1.0);
                }
                else
                {
                    gamma2 = be / (-tau + 1.0);
                }
                double al = ChMaths.CH_C_PI - Math.Acos(ChVector.Vdot(Get_shaft_dir2(), my));
                double te = ChMaths.CH_C_PI - al - be;
                double fd = Math.Sin(te) * (dist / Math.Sin(be));
                r2 = fd * Math.Tan(gamma2);
                r1 = r2 * tau;
            }

            // compute markers positions, supposing they
            // stay on the ideal wheel contact point
            mmark1 = ChVector.Vadd(Get_shaft_pos2(), ChVector.Vmul(mr2, r2));
            mmark2 = mmark1;
            contact_pt = mmark1;

            // correct marker 1 position if phasing is not correct
            if (checkphase)
            {
                double realtau = tau;
                if (epicyclic)
                    realtau = -tau;
                double m_delta;
                m_delta = -(a2 / realtau) - a1 - phase;

                if (m_delta > ChMaths.CH_C_PI)
                    m_delta -= (ChMaths.CH_C_2PI);  // range -180..+180 is better than 0...360
                if (m_delta > (ChMaths.CH_C_PI / 4.0))
                    m_delta = (ChMaths.CH_C_PI / 4.0);  // phase correction only in +/- 45°
                if (m_delta < -(ChMaths.CH_C_PI / 4.0))
                    m_delta = -(ChMaths.CH_C_PI / 4.0);

                vrota.x = vrota.y = 0.0;
                vrota.z = -m_delta;
                mrotma.Set_A_Rxyz(vrota);  // rotate about Z of shaft to correct
                mmark1 = abs_shaft1.GetA().MatrT_x_Vect(ChVector.Vsub(mmark1, Get_shaft_pos1()));
                mmark1 = mrotma.Matr_x_Vect(mmark1);
                mmark1 = ChVector.Vadd(abs_shaft1.GetA().Matr_x_Vect(mmark1), Get_shaft_pos1());
            }
            // Move Shaft 1 along its direction if not aligned to wheel
            double offset = ChVector.Vdot(Get_shaft_dir1(), (contact_pt - Get_shaft_pos1()));
            ChVector moff = Get_shaft_dir1() * offset;
            if (Math.Abs(offset) > 0.0001)
                local_shaft1.SetPos(local_shaft1.GetPos() + Body1.TransformDirectionParentToLocal(moff));

            // ! Require that the BDF routine of marker won't handle speed and acc.calculus of the moved marker 2!
            marker2.SetMotionType(ChMarker.eChMarkerMotion.M_MOTION_EXTERNAL);
            marker1.SetMotionType(ChMarker.eChMarkerMotion.M_MOTION_EXTERNAL);

            // move marker1 in proper positions
            newmarkpos.pos = mmark1;
            newmarkpos.rot = ma1.Get_A_quaternion();
            marker1.Impose_Abs_Coord(newmarkpos);  // move marker1 into teeth position
                                                   // move marker2 in proper positions
            newmarkpos.pos = mmark2;
            newmarkpos.rot = ma2.Get_A_quaternion();
            marker2.Impose_Abs_Coord(newmarkpos);  // move marker2 into teeth position

            // imposed relative positions/speeds
            deltaC.pos = ChVector.VNULL;
            deltaC_dt.pos = ChVector.VNULL;
            deltaC_dtdt.pos = ChVector.VNULL;

            deltaC.rot = ChQuaternion.QUNIT;  // no relative rotations imposed!
            deltaC_dt.rot = ChQuaternion.QNULL;
            deltaC_dtdt.rot = ChQuaternion.QNULL;

        }

        /// Get the transmission ratio. Its value is assumed always positive,
        /// both for inner and outer gears (so use Get_epicyclic() to distinguish)
        public double Get_tau() { return tau; }
        /// Set the transmission ratio. Its value is assumed always positive,
        /// both for inner and outer gears (so use Set_epicyclic() to distinguish)
        public void Set_tau(double mset) { tau = Math.Abs(mset); }
        /// Set the transmission ratio given the number of teeth (or radius) of 1st gear
        /// and the number of teeth (or radius) of 2nd gear
        public void Set_tau(double mz1, double mz2) { tau = Math.Abs(mz1 / mz2); }

        /// Get the pressure angle (usually 20° for typical gears)
        public double Get_alpha() { return alpha; }
        /// Set the pressure angle (usually 20° for typical gears)
        public void Set_alpha(double mset) { alpha = mset; }

        /// Get the angle of teeth in bevel gears (0° for spur gears)
        public double Get_beta() { return beta; }
        /// Set the angle of teeth in bevel gears (0° for spur gears)
        public void Set_beta(double mset) { beta = mset; }

        /// Get the initial phase of rotation of gear A respect to gear B
        public double Get_phase() { return phase; }
        /// Set the initial phase of rotation of gear A respect to gear B
        public void Set_phase(double mset) { phase = mset; }

        /// If true, the bigger wheel has inner (internal) teeth
        public bool Get_epicyclic() { return epicyclic; }
        /// If true, the bigger wheel has inner (internal) teeth
        public void Set_epicyclic(bool mset) { epicyclic = mset; }

        /// If true, enforce check on exact phase between gears
        /// (otherwise after many simulation steps the phasing
        /// may be affected by numerical error accumulation).
        /// By default, it is turned off.
        /// Note that, to ensure the correct phasing during the many
        /// rotations, an algorithm will use the a1 and a2 total rotation
        /// values, which might be affected by loss of numerical precision
        /// after few thousands of revolutions, so this is NOT suited to
        /// real-time simulators which must run for many hours.
        public void Set_checkphase(bool mset) { checkphase = mset; }
        public bool Get_checkphase() { return checkphase; }

        /// Get total rotation of 1st gear, respect to interaxis, in radians
        public double Get_a1() { return a1; }

        /// Get total rotation of 1st gear, respect to interaxis, in radians
        public double Get_a2() { return a2; }

        /// Reset the total rotations of a1 and a2.
        public void Reset_a1a2() { a1 = a2 = 0; }

        /// Get radius of 1st gear (depends on axis position and t.ratio)
        public double Get_r1() { return r1; }
        /// Get radius of 2nd gear (depends on axis position and t.ratio)
        public double Get_r2() { return r2; }

        /// Get shaft position and direction, for 1st gear, in body1-relative reference.
        /// The shaft direction is the Z axis of that frame.
        public ChFrame<double> Get_local_shaft1() { return local_shaft1; }

        /// Set shaft position and direction, for 1st gear, in body1-relative reference.
        /// The shaft direction is the Z axis of that frame.
        /// Note that the origin of shaft position may be automatically shifted along
        /// shaft direction in order to have both wheels on same plane (for spur gears) -
        /// same sphere (for bevel gears).
        public void Set_local_shaft1(ChFrame<double> mf) { 
            local_shaft1 = mf; 
        }

        /// Get shaft position and direction, for 2nd gear, in body2-relative reference.
        /// The shaft direction is the Z axis of that frame.
        public ChFrame<double> Get_local_shaft2() { return local_shaft2; }

        /// Set shaft position and direction, for 2nd gear, in body2-relative reference.
        /// The shaft direction is the Z axis of that frame.
        public void Set_local_shaft2(ChFrame<double> mf) { 
            local_shaft2 = mf; 
        }

        /// Get shaft direction, for 1st gear, in absolute reference
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

        /// Get shaft direction, for 2nd gear, in absolute reference
        public ChVector Get_shaft_dir2() {
            if (Body1 != null)
            {
                ChFrame<double> absframe = ChFrame<double>.FNULL; //new ChFrame<double>();
                ((ChFrame<double>)Body2).TransformLocalToParent(local_shaft2, absframe);
                return absframe.GetA().Get_A_Zaxis();
            }
            else
                return ChVector.VECT_Z;
        }

        /// Get shaft position, for 1st gear, in absolute reference
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

        /// Get shaft position, for 2nd gear, in absolute reference
        public ChVector Get_shaft_pos2() {
            if (Body1 != null)
            {
                ChFrame<double> absframe = ChFrame<double>.FNULL; //new ChFrame<double>();
                ((ChFrame<double>)Body2).TransformLocalToParent(local_shaft2, absframe);
                return absframe.GetPos();
            }
            else
                return ChVector.VNULL;
        }
    }

}
