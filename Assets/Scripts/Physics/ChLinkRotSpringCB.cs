using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono
{

    /// Class for rotational spring-damper systems with the torque specified through a
    /// functor object.
    /// It is ASSUMED that the two bodies are joined such that they have a rotational
    /// degree of freedom about the z axis of the specified link reference frame (i.e.,
    /// they are connected through a revolute, cylindrical, or screw joint). The
    /// relative angle and relative angular speed of this link are about the common axis.
    public class ChLinkRotSpringCB : ChLinkMarkers
    {
        public ChBody body1;
        public ChBody body2;

        public ChLinkRotSpringCB()
        {
            m_torque = 0;
            m_torque_fun = null;
        }
        public ChLinkRotSpringCB(ChLinkRotSpringCB other)
        {
            m_torque = other.m_torque;
            m_torque_fun = other.m_torque_fun;  //// do we need a deep copy?
        }

        /// "Virtual" copy constructor (covariant return type).
        public override ChObj Clone() { return new ChLinkRotSpringCB(this); }

        public void Start()
        {
            ChCoordsys csys = new ChCoordsys(Utils.ToChrono(transform.position), Utils.ToChrono(transform.rotation));
            Initialize(body1, body2, csys);

            // Get a handle to the associated function component and set the motor's function
            var fun_component = this.GetComponent<TorqueFunctor>();
            if (fun_component != null)
            {
                RegisterTorqueFunctor(fun_component);
            }

            ChSystem msystem = FindObjectOfType<ChSystem>();
            msystem.AddLink(this);
        }

        /// Get the current relative angle about the common rotation axis.
        public double GetRotSpringAngle() { return relAngle; }

        /// Get the current relative angular speed about the common rotation axis.
        public double GetRotSpringSpeed() { return ChVector.Vdot(relWvel, relAxis); }

        /// Get the current generated torque.
        public double GetRotSpringTorque() { return m_torque; }

        /// Class to be used as a functor interface for calculating the general spring-damper torque.
        /// A derived class must implement the virtual operator().
        public class TorqueFunctor : MonoBehaviour
        {

            /// Calculate and return the general spring-damper torque at the specified configuration.
            public virtual double this[double time,             //< current time
                                  double angle,            //< relative angle of rotation
                                  double vel,              //< relative angular speed
                                  ChLinkRotSpringCB link  //< back-pointer to associated link
                                  ]
            { get { return 0; } }
        };

        /// Specify the functor object for calculating the torque.
        public void RegisterTorqueFunctor(TorqueFunctor functor) { m_torque_fun = functor; }

        /// Include the rotational spring custom torque.
        public override void UpdateForces(double time)
        {
            // Allow the base class to update itself (possibly adding its own forces).
            base.UpdateForces(time);

            // Invoke the provided functor to evaluate torque.
            // NOTE: we implicitly assume that the kinematics are CONSISTENT with this
            // type of link!
            double angle = relAngle;
            double angle_dt = ChVector.Vdot(relWvel, relAxis);

            if (m_torque_fun != null)
            {
                m_torque = (m_torque_fun)[time, relAngle, angle_dt, this];
            }

            // Add to existing torque.
            C_torque += ChVector.Vmul(relAxis, m_torque);
        }

        protected TorqueFunctor m_torque_fun;  //< functor for torque calculation
        protected double m_torque;              //< resulting torque along relative axis of rotation

    }
}