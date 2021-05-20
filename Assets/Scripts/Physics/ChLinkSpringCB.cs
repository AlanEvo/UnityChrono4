using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
using UnityEditor;


namespace chrono
{

    /// Class for spring-damper systems with the force specified through a functor object.
    public class ChLinkSpringCB : ChLinkMarkers {

        public int springResolution = 8;
        public double springTurns = 2.0;
        public double radius = 0.1f;


        public ChBody body1;
        public ChBody body2;

        public bool useRelativePos = true;
        public bool autoRestLength = false;
        public double restLength = 0;
        //public Vector3 position1;
        //public Vector3 position2;
        public Transform position2;

        private ChVector sPosition1 = new ChVector(0, 0, 0);
        private ChVector sPosition2 = new ChVector(0, 0, 0);

        public enum Type
        {
            SPRING,
            SHOCK
        }

        // Purely for debug visual representaion only.  Test.  I'm thinking of using this
        // because if the project uses both shocks and springs, visually they will look the
        // same, which could look confusing.
        public Type type = Type.SPRING;

        public ChLinkSpringCB() {
            m_rest_length = 0;
            m_force = 0;
            m_force_fun = null;
        }
        public ChLinkSpringCB(ChLinkSpringCB other) {
            m_rest_length = other.m_rest_length;
            m_force = other.m_force;
            m_force_fun = other.m_force_fun;  //// do we need a deep copy?
        }

        /// "Virtual" copy constructor (covariant return type).
        public override ChObj Clone() { return new ChLinkSpringCB(this); }

        public void Start()
        {
            Initialize(body1, body2, useRelativePos, Utils.ToChrono(transform.position), Utils.ToChrono(position2.position), autoRestLength, restLength);

            // Get a handle to the associated function component and set the motor's function
            var fun_component = this.GetComponent<ForceFunctor>();
            if (fun_component != null)
            {
                RegisterForceFunctor(fun_component);
            }

            ChSystem msystem = FindObjectOfType<ChSystem>();
            msystem.AddLink(this);
        }

        // Update is called once per frame
        void Update()
        {
            sPosition1 = GetEndPoint1Abs();
            sPosition2 = GetEndPoint2Abs();
        }

        void OnDrawGizmos()
        {
            if (Application.isPlaying)
            {
                switch (type)
                {
                    case Type.SPRING:
                        Utils.drawSpring(radius, sPosition1, sPosition2, springResolution, springTurns);
                        break;
                    case Type.SHOCK:
                        Debug.DrawLine(new Vector3((float)sPosition1.x, (float)sPosition1.y, (float)sPosition1.z), new Vector3((float)sPosition2.x, (float)sPosition2.y, (float)sPosition2.z), new Color(255, 0, 255));
                        break;
                }
            }
            else
            {
                switch (type)
                {
                    case Type.SPRING:
                        Gizmos.color = new Color(255, 255, 0);
                        Utils.drawSpring(radius, new ChVector(transform.position.x, transform.position.y, transform.position.z), new ChVector(position2.position.x, position2.position.y, position2.position.z), springResolution, springTurns);
                        Gizmos.DrawSphere(transform.position, 0.01f);
                        break;
                    case Type.SHOCK:
                        Gizmos.color = new Color(255, 0, 255);
                        Gizmos.DrawLine(transform.position, position2.position);
                        break;
                }
            }
        }

        // data fetch/store
        public double GetSpringRestLength() { return m_rest_length; }
        public double GetSpringDeform() { return dist - m_rest_length; }
        public double GetSpringLength() { return dist; }
        public double GetSpringVelocity() { return dist_dt; }
        public double GetSpringReact() { return m_force; }
        public void SetSpringRestLength(double len) { m_rest_length = len; }

        /// Class to be used as a functor interface for calculating the general spring-damper force.
        /// A derived class must implement the virtual operator().
        public abstract class ForceFunctor : MonoBehaviour
        {
            /// Calculate and return the general spring-damper force at the specified configuration.
            public virtual double this[double time,          //< current time
                                  double rest_length,         //< undeformed length
                                  double length,              //< current length
                                  double vel,                 //< current velocity (positive when extending)
                                  ChLinkSpringCB link         //< back-pointer to associated link
                                  ]
            { get { return 0; } }            
        }


        /// Specify the functor object for calculating the force.
        public void RegisterForceFunctor(ForceFunctor functor) { m_force_fun = functor; }

        /// Specialized initialization for springs, given the two bodies to be connected,
        /// the positions of the two anchor endpoints of the spring (each expressed
        /// in body or abs. coordinates) and the imposed rest length of the spring.
        /// NOTE! As in ChLinkMarkers::Initialize(), the two markers are automatically
        /// created and placed inside the two connected bodies.
        public void Initialize(
                                ChBody body1,  //< first body to link
                                ChBody body2,  //< second body to link
                                bool pos_are_relative,          //< true: following pos. are relative to bodies
                                ChVector pos1,                //< pos. of spring endpoint for 1st body (rel. or abs., see flag above)
                                ChVector pos2,                //< pos. of spring endpoint for 2nd body (rel. or abs., see flag above)
                                bool auto_rest_length = true,   //< if true, initializes the rest length as the distance between pos1 and pos2
                                double rest_length = 0          //< rest length (no need to define if auto_rest_length=true.)
    )
        {
            // First, initialize as all constraint with markers.
            // In this case, create the two markers also!.
            base.Initialize(body1, body2, new ChCoordsys(new ChVector(0, 0, 0), new ChQuaternion(0, 0, 0, 0)));

            if (pos_are_relative)
            {
                marker1.Impose_Rel_Coord(new ChCoordsys(pos1, new ChQuaternion(0, 0, 0, 0)));
                marker2.Impose_Rel_Coord(new ChCoordsys(pos2, new ChQuaternion(0, 0, 0, 0)));
            }
            else
            {
                marker1.Impose_Abs_Coord(new ChCoordsys(pos1, new ChQuaternion(0, 0, 0, 0)));
                marker2.Impose_Abs_Coord(new ChCoordsys(pos2, new ChQuaternion(0, 0, 0, 0)));
            }

            ChVector AbsDist = marker1.GetAbsCoord().pos - marker2.GetAbsCoord().pos;
            dist = AbsDist.Length();

            m_rest_length = auto_rest_length ? dist : rest_length;
        }

        /// Get the 1st spring endpoint (expressed in Body1 coordinate system)
        public ChVector GetEndPoint1Rel() { return marker1.FrameMoving.GetPos(); }
        /// Set the 1st spring endpoint (expressed in Body1 coordinate system)
        public void SetEndPoint1Rel(ChVector mset) { marker1.Impose_Rel_Coord(new ChCoordsys(mset, new ChQuaternion(0, 0, 0, 0))); }
        /// Get the 1st spring endpoint (expressed in absolute coordinate system)
        public ChVector GetEndPoint1Abs() { return marker1.GetAbsCoord().pos; }
        /// Set the 1st spring endpoint (expressed in absolute coordinate system)
        public void SetEndPoint1Abs(ref ChVector mset) { marker1.Impose_Abs_Coord(new ChCoordsys(mset, new ChQuaternion(0, 0, 0, 0))); }

        /// Get the 2nd spring endpoint (expressed in Body2 coordinate system)
        public ChVector GetEndPoint2Rel() { return marker2.FrameMoving.GetPos(); }
        /// Set the 2nd spring endpoint (expressed in Body2 coordinate system)
        public void SetEndPoint2Rel(ChVector mset) { marker2.Impose_Rel_Coord(new ChCoordsys(mset, new ChQuaternion(0, 0, 0, 0))); }
        /// Get the 1st spring endpoint (expressed in absolute coordinate system)
        public ChVector GetEndPoint2Abs() { return marker2.GetAbsCoord().pos; }
        /// Set the 1st spring endpoint (expressed in absolute coordinate system)
        public void SetEndPoint2Abs(ref ChVector mset) { marker2.Impose_Abs_Coord(new ChCoordsys(mset, new ChQuaternion(0, 0, 0, 0))); }

        /// Inherits, then also adds the spring custom forces to the C_force and C_torque.
        public override void UpdateForces(double time)
        {
            // Allow the base class to update itself (possibly adding its own forces)
            base.UpdateForces(time);

            // Invoke the provided functor to evaluate force
            if (m_force_fun != null)
            {
                m_force = m_force_fun[time, m_rest_length, dist, dist_dt, this];
            }

            // Add to existing force.
            C_force += m_force * relM.pos.GetNormalized();
        }

        protected ForceFunctor m_force_fun;  //< functor for force calculation
        protected double m_rest_length;       //< undeform length
        protected double m_force;             //< resulting force in dist. coord
    };

}

