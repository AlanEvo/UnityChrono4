using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;


namespace chrono
{

    /// Class for defining a clutch or a brake (1D model) between two one-degree-of-freedom
    /// parts; i.e., shafts that can be used to build 1D models of powertrains.
   /* public class ChShaftsClutch : MonoBehaviour
    {

        public ChShaft shaft1;    //< first shaft
        public ChShaft shaft2;    //< second shaft

        public double maxT;                             //< clutch max transmissible torque (for forward direction
        public double minT;                             //< clutch min transmissible torque (for backward direction)
        public double modulation;                       //< 0...1  (default 1).
        public double torque_react;                     //< reaction torque
        //ChConstraintTwoGenericBoxed constraint;  ///< used as an interface to the solver

        public double m_differential_ratio;

        public IntPtr m_ChShaftsClutch = IntPtr.Zero;

        protected ChSystem m_system;

        // Use this for initialization
        void Start()
        {
            m_system = GameObject.FindObjectOfType<ChSystem>();

            m_ChShaftsClutch = ChShaftsClutch_Create(out m_ChShaftsClutch);

            Initialize(shaft1, shaft2);
            SetTorqueLimit(m_differential_ratio);
            SetModulation(modulation);

        }

        /// Use this function after gear creation, to initialize it, given
        /// two shafts to join.
        /// Each shaft must belong to the same ChSystem.
        public bool Initialize(ChShaft mshaft1,  //< first  shaft to join
                               ChShaft mshaft2   //< second shaft to join
                    )
        {
            return false;
        }

        /// Set the transmissible torque limit (the maximum torque that
        /// the clutch can transmit between the two shafts).
        /// You can specify two values for backward/forward directions: usually
        /// these are equal (ex. -100,100) in most commercial clutches, but
        /// if you define (0,100), for instance, you can create a so called
        /// freewheel or overrunning clutch that works only in one direction.
        public void SetTorqueLimit(double ml, double mu) {
            minT = ml;
            maxT = mu;
        }
        /// Set the transmissible torque limit (the maximum torque that
        /// the clutch can transmit between the two shafts), for both
        /// forward and backward direction.
        void SetTorqueLimit(double ml) {
            SetTorqueLimit(-Math.Abs(ml), Math.Abs(ml));
            ChShaftsClutch_SetTorqueLimit(m_ChShaftsClutch, ml);
        }

        /// Set the user modulation of the torque (or brake, if you use it between
        /// a fixed shaft and a free shaft). The modulation must range from
        /// 0 (switched off) to 1 (max torque). Default is 1, when clutch is created.
        /// You can update this during integration loop to simulate the pedal pushing by the driver.
        public void SetModulation(double mm) {
            modulation = ChMaths.ChMax(ChMaths.ChMin(mm, 1.0), 0.0);
            ChShaftsClutch_SetModulation(m_ChShaftsClutch, mm);
        }

        /// Get the reaction torque exchanged between the two shafts,
        /// considered as applied to the 1st axis.
        public double GetTorqueReactionOn1() {
            torque_react = ChShaftsClutch_GetTorqueReactionOn1(m_ChShaftsClutch);
            return torque_react;
        }

        /// Get the reaction torque exchanged between the two shafts,
        /// considered as applied to the 2nd axis.
        public double GetTorqueReactionOn2() {
            torque_react = ChShaftsClutch_GetTorqueReactionOn2(m_ChShaftsClutch);
            return torque_react;
        }

// Update is called once per frame
void Update()
        {

        }
    }*/
}
