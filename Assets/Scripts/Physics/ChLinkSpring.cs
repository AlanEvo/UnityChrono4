using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
using UnityEditor;


namespace chrono
{

    /// Class for spring-damper systems with the force specified through a functor object.
    public class ChLinkSpring : ChLinkMarkers
    {

        protected double spr_restlength;                    //< spring rest (undeformed) length
        protected double spr_k;                             //< spring coefficient
        protected double spr_r;                             //< damping coefficient
        protected double spr_f;                             //< actuator force
        protected ChFunction mod_f_time;   //< f(t)
        protected ChFunction mod_k_d;      //< k(d)
        protected ChFunction mod_r_d;      //< r(d)
        protected ChFunction mod_r_speed;  //< k(speed)
        protected ChFunction mod_k_speed;  //< r(speed)
        protected double spr_react;                         //< resulting force in dist. coord / readonly

        public int springResolution = 8;
        public double springTurns = 2.0;
        public double radius = 0.1f;

        public ChBody body1;
        public ChBody body2;

        public bool useRelativePos = true;
       // public Vector3 position1;
        public Transform position2;

        public double springCoefficient = 0;
        public double dampingCoefficient = 0;
        public double actuatorForce = 0;

        public bool autoRestLength = false;
        public double restLength = 0;
        private ChVector sPosition1 = new ChVector(0, 0, 0);
        private ChVector sPosition2 = new ChVector(0, 0, 0);


        public ChLinkSpring() {
            spr_restlength = 0;
            spr_k = 100;
            spr_r = 5;
            spr_f = 0;

            mod_f_time = new ChFunction_Const(1);
            mod_k_d = new ChFunction_Const(1);
            mod_k_speed = new ChFunction_Const(1);
            mod_r_d = new ChFunction_Const(1);
            mod_r_speed = new ChFunction_Const(1);

            spr_react = 0.0;
        }
        public ChLinkSpring(ChLinkSpring other) :base(other){
            spr_restlength = other.spr_restlength;
            spr_f = other.spr_f;
            spr_k = other.spr_k;
            spr_r = other.spr_r;
            spr_react = other.spr_react;

            mod_f_time = new ChFunction(other.mod_f_time.Clone());
            mod_k_d = new ChFunction(other.mod_k_d.Clone());
            mod_k_speed = new ChFunction(other.mod_k_speed.Clone());
            mod_r_d = new ChFunction(other.mod_r_d.Clone());
            mod_r_speed = new ChFunction(other.mod_r_speed.Clone());
        }

        public void Start()
        {
            Initialize(body1, body2, useRelativePos, Utils.ToChrono(transform.position), Utils.ToChrono(position2.position), autoRestLength, restLength);
            Set_SpringK(springCoefficient);
            Set_SpringR(dampingCoefficient);
            Set_SpringF(actuatorForce);

            ChSystem msystem = FindObjectOfType<ChSystem>();
            msystem.AddLink(this);
        }

        void FixedUpdate()
        {
             sPosition1 = GetEndPoint1Abs();
             sPosition2 = GetEndPoint2Abs();
        }

        void OnDrawGizmos()
        {
            if (Application.isPlaying)
            {
                Utils.drawSpring(radius, sPosition1, sPosition2, springResolution, springTurns);
               // Debug.Log("pos " + sPosition1.y);
            }
            else
            {
                Gizmos.color = new Color(255, 255, 0);
                Utils.drawSpring(radius, new ChVector(transform.position.x, transform.position.y, transform.position.z), new ChVector(position2.position.x, position2.position.y, position2.position.z), springResolution, springTurns);
            }
        }

        /// "Virtual" copy constructor (covariant return type).
        public override ChObj Clone() { return new ChLinkSpring(this); }

        // data fetch/store
        public double Get_SpringRestLength() { return spr_restlength; }
        public double Get_SpringDeform() { return (dist - spr_restlength); }
        public double Get_SpringLength() { return dist; }
        public double Get_SpringVelocity() { return dist_dt; }
        public double Get_SpringK() { return spr_k; }
        public double Get_SpringR() { return spr_r; }
        public double Get_SpringF() { return spr_f; }
        public double Get_SpringReact() { return spr_react; }

        public void Set_SpringRestLength(double m_r) { spr_restlength = m_r; }
        public void Set_SpringK(double m_r) { spr_k = m_r; }
        public void Set_SpringR(double m_r) { spr_r = m_r; }
        public void Set_SpringF(double m_r) { spr_f = m_r; }

        public ChFunction Get_mod_f_time() { return mod_f_time; }
        public ChFunction Get_mod_k_d() { return mod_k_d; }
        public ChFunction Get_mod_r_d() { return mod_r_d; }
        public ChFunction Get_mod_k_speed() { return mod_k_speed; }
        public ChFunction Get_mod_r_speed() { return mod_r_speed; }

        public void Set_mod_f_time(ChFunction mf) { mod_f_time = mf; }
        public void Set_mod_k_d(ChFunction mf) { mod_k_d = mf; }
        public void Set_mod_r_d(ChFunction mf) { mod_r_d = mf; }
        public void Set_mod_k_speed(ChFunction mf) { mod_k_speed = mf; }
        public void Set_mod_r_speed(ChFunction mf) { mod_r_speed = mf; }

        /// Specialized initialization for springs, given the two bodies to be connected,
        /// the positions of the two anchor endpoints of the spring (each expressed
        /// in body or abs. coordinates) and the imposed rest length of the spring.
        /// NOTE! As in ChLinkMarkers::Initialize(), the two markers are automatically
        /// created and placed inside the two connected bodies.
        public void Initialize(
                ChBody mbody1,  //< first body to link
                ChBody mbody2,  //< second body to link
                bool pos_are_relative,           //< true: following pos. are relative to bodies
                ChVector mpos1,                //< position of spring endpoint, for 1st body (rel. or abs., see flag above)
                ChVector mpos2,                //< position of spring endpoint, for 2nd body (rel. or abs., see flag above)
                bool auto_rest_length = true,    //< if true, initializes the rest-length as the distance between mpos1 and mpos2
                double mrest_length = 0          //< imposed rest_length (no need to define, if auto_rest_length=true.)
        )
        {
            // First, initialize as all constraint with markers.
            // In this case, create the two markers also!.
            base.Initialize(mbody1, mbody2, new ChCoordsys(new ChVector(0, 0, 0), new ChQuaternion(1, 0, 0, 0)));

            if (pos_are_relative)
            {
                marker1.Impose_Rel_Coord(new ChCoordsys(mpos1, new ChQuaternion(1, 0, 0, 0)));
                marker2.Impose_Rel_Coord(new ChCoordsys(mpos2, new ChQuaternion(1, 0, 0, 0)));
            }
            else
            {
                marker1.Impose_Abs_Coord(new ChCoordsys(mpos1, new ChQuaternion(1, 0, 0, 0)));
                marker2.Impose_Abs_Coord(new ChCoordsys(mpos2, new ChQuaternion(1, 0, 0, 0)));
            }

            ChVector AbsDist = marker1.GetAbsCoord().pos - marker2.GetAbsCoord().pos;
            dist = AbsDist.Length();

            spr_restlength = auto_rest_length ? dist : mrest_length;
        }

        /// Get the 1st spring endpoint (expressed in Body1 coordinate system)
        public ChVector GetEndPoint1Rel() { return marker1.FrameMoving.GetPos(); }
        /// Set the 1st spring endpoint (expressed in Body1 coordinate system)
        public void SetEndPoint1Rel(ChVector mset) { marker1.Impose_Rel_Coord(new ChCoordsys(mset, new ChQuaternion(0, 0, 0, 0))); }
        /// Get the 1st spring endpoint (expressed in absolute coordinate system)
        public ChVector GetEndPoint1Abs() { return marker1.GetAbsCoord().pos; }
        /// Set the 1st spring endpoint (expressed in absolute coordinate system)
        public void SetEndPoint1Abs(ChVector mset) { marker1.Impose_Abs_Coord(new ChCoordsys(mset, new ChQuaternion(0, 0, 0, 0))); }

        /// Get the 2nd spring endpoint (expressed in Body2 coordinate system)
        public ChVector GetEndPoint2Rel() { return marker2.FrameMoving.GetPos(); }
        /// Set the 2nd spring endpoint (expressed in Body2 coordinate system)
        public void SetEndPoint2Rel(ChVector mset) { marker2.Impose_Rel_Coord(new ChCoordsys(mset, new ChQuaternion(0, 0, 0, 0))); }
        /// Get the 1st spring endpoint (expressed in absolute coordinate system)
        public ChVector GetEndPoint2Abs() { return marker2.GetAbsCoord().pos; }
        /// Set the 1st spring endpoint (expressed in absolute coordinate system)
        public void SetEndPoint2Abs(ChVector mset) { marker2.Impose_Abs_Coord(new ChCoordsys(mset, new ChQuaternion(0, 0, 0, 0))); }

        /// Inherits, then also adds the spring custom forces to the C_force and C_torque.
        public override void UpdateForces(double mytime) {
            // Inherit force computation:
            // also base class can add its own forces.
            base.UpdateForces(mytime);

            spr_react = 0.0;
            ChVector m_force;
            double deform = Get_SpringDeform();

            spr_react = spr_f * mod_f_time.Get_y(ChTime);
            spr_react -= (spr_k * mod_k_d.Get_y(deform) * mod_k_speed.Get_y(dist_dt)) * (deform);
            spr_react -= (spr_r * mod_r_d.Get_y(deform) * mod_r_speed.Get_y(dist_dt)) * (dist_dt);

            m_force = ChVector.Vmul(ChVector.Vnorm(relM.pos), spr_react);

            C_force = ChVector.Vadd(C_force, m_force);
        }

       

       /* public ChVector GetEndPoint1Abs()
        {
            ChVector temp = new ChVector(0, 0, 0);
            temp.m_ChVector = ChLinkSpring_GetEndPoint1Abs(m_ChLink);
            return temp;
        }

        public ChVector GetEndPoint2Abs()
        {
            ChVector temp = new ChVector(0, 0, 0);
            temp.m_ChVector = ChLinkSpring_GetEndPoint2Abs(m_ChLink);
            return temp;
        }*/


    }
}
