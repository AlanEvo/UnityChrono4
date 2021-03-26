using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;


namespace chrono
{
    /// Class for defining a thermal engine between two one-degree-of-freedom parts;
    /// i.e., shafts that can be used to build 1D models of power trains.
    /// The first shaft is the 'crankshaft' to whom the torque is applied, the second
    /// is the motor block, that receives the negative torque.
    public class ChShaftsThermalEngine : MonoBehaviour
    {


        private double torque;
        private ChFunction Tw;  //< torque as function of angular vel.
        private double throttle;

        private bool error_backward;

        public IntPtr m_ChShaftsThermalEngine = IntPtr.Zero;

        protected ChSystem m_system;

        public ChShaft shaft1;  //< first shaft
        public ChShaft shaft2;  //< second shaft

        public ChShaftsThermalEngine() {

        }

        // Use this for initialization
        void Start()
        {
            m_system = GameObject.FindObjectOfType<ChSystem>();



        }

        /// Set the torque curve T(w), function of angular speed between shaft1 and shaft2.
        /// Output units: [Nm]  , input units: [rad/s]
        public void SetTorqueCurve(ChFunction mf) {
            Tw = mf;
        }

        /// Set the current throttle value 's' in [0,1] range. If s=1,
        /// the torque is exactly T=T(w), otherwise it is linearly
        /// scaled as T=T(w)*s.
        /// This is a simplified model of real torque modulation, but
        /// enough for a basic model. An advanced approach would require a 2D map T(w,s)
        public void SetThrottle(double mt) {
            throttle = mt;

        }

        /// Get the reaction torque exchanged between the two shafts,
        /// considered as applied to the 1st axis.
        public double GetTorqueReactionOn1() {

            return torque;

        }


        // Update is called once per frame
        void FixedUpdate()
        {
           // SetThrottle(throttle);
        }
    }
}
