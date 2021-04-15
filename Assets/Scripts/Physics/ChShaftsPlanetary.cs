using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
using System.Runtime.InteropServices;

namespace chrono
{

    /// Class for defining a planetary gear between three one-degree-of-freedom parts;
    /// i.e., shafts that can be used to build 1D models of powertrains; this is more
    /// efficient than simulating power trains modeled full 3D ChBody objects).
    /// Planetary gears can be used to make, for instance, the differentials of cars.
    /// While traditional gear reducers have one input and one output, the planetary
    /// gear have two inputs and one output (or, if you prefer, one input and two outputs).
    /// Note that you can use this class also to make a gearbox if you are interested
    /// in knowing the reaction torque transmitted to the truss (whereas the basic
    /// ChLinkGear cannot do this because it has only in and out); in this case you
    /// just use the shaft n.1 as truss and fix it.
    public class ChShaftsPlanetary : MonoBehaviour
    {
        [DllImport("ChronoEngine", EntryPoint = "ChShaftsPlanetary_Create")]
        public static extern IntPtr ChShaftsPlanetary_Create(out IntPtr shaft);
        [DllImport("ChronoEngine", EntryPoint = "ChShaftsPlanetary_Initialize")]
        public static extern bool ChShaftsPlanetary_Initialize(IntPtr shaft, IntPtr mshaft1, IntPtr mshaft2, IntPtr mshaft3);
        [DllImport("ChronoEngine", EntryPoint = "ChShaftsPlanetary_SetTransmissionRatioOrdinary")]
        public static extern void ChShaftsPlanetary_SetTransmissionRatioOrdinary(IntPtr shaft, double t0);
        [DllImport("ChronoEngine", EntryPoint = "ChShaftsPlanetary_GetTorqueReactionOn1")]
        public static extern double ChShaftsPlanetary_GetTorqueReactionOn1(IntPtr shaft);
        [DllImport("ChronoEngine", EntryPoint = "ChShaftsPlanetary_GetTorqueReactionOn2")]
        public static extern double ChShaftsPlanetary_GetTorqueReactionOn2(IntPtr shaft);
        [DllImport("ChronoEngine", EntryPoint = "ChShaftsPlanetary_GetTorqueReactionOn3")]
        public static extern double ChShaftsPlanetary_GetTorqueReactionOn3(IntPtr shaft);

      //  private double r1;  //< transmission ratios  as in   r1*w1 + r2*w2 + r3*w3 = 0
      //  private double r2;
        //private double r3;

        public double torque_react;  //< shaft reaction torque

        private bool avoid_phase_drift;
        private double phase1;
        private double phase2;
        private double phase3;

        //ChConstraintThreeGeneric constraint;  ///< used as an interface to the solver

        public ChShaft shaft1;  //< first connected shaft
        public ChShaft shaft2;  //< second connected shaft
        public ChShaft shaft3;  //< third connected shaft

        public double m_differential_ratio;

        public IntPtr m_ChShaftsPlanetary = IntPtr.Zero;

        protected ChSystem m_system;

        // Use this for initialization
        void Start()
        {
            m_system = GameObject.FindObjectOfType<ChSystem>();

            m_ChShaftsPlanetary = ChShaftsPlanetary_Create(out m_ChShaftsPlanetary);
            Initialize(shaft1, shaft2, shaft3);
            SetTransmissionRatioOrdinary(m_differential_ratio);
        }

        /// Use this function after planetary gear creation, to initialize it, given
        /// three shafts to join.
        /// Although there's no special requirement, you may think of the three
        /// typical moving parts of an epicycloidal reducer: the carrier, the
        /// input gear, and the gear with inner teeth that usually is kept fixed (but the
        /// ChShaftsPlanetary does not require that one shaft is fixed - it's up to you)
        /// Each shaft must belong to the same ChSystem.
        public bool Initialize(ChShaft mshaft1,  //< first  shaft to join (carrier wheel)
                               ChShaft mshaft2,  //< second shaft to join (wheel)
                               ChShaft mshaft3   //< third  shaft to join (wheel)
                    )
        {
            return false;
        }

        /// Setting the transmission ratios r1 r2 r3 for  r1*w1 + r2*w2 + r3*w3 = 0
        /// may be cumbersome, but when you deal with typical planetary devices, this
        /// function provides a shortcut to setting them for you, given a single
        /// parameter t0, that is the speed ratio t'=w3'/w2' of the inverted planetary.
        /// That ratio is simple to get: to invert the planetary, imagine to hold fixed
        /// the carrier of shaft 1 (that is w1' =0), move the shaft 2 and see which is
        /// the speed of shaft 3, to get the ratio t0=w3'/w2'. Generally, shaft 1 is
        /// called the 'carrier'. For example, in normal operation of an epicycloidal
        /// reducer, the carrier (shaft 1) is used as output, shaft 2 is the input, and
        /// shaft 3 is hold fixed to get one degree of freedom only; but in 'inverted' operation
        /// imagine the carrier is fixed, so t0 can be easily got as t0=-z2/z3, with z=n.of teeth.
        /// In a car differential, again with shaft 1 as carrier, one can see that t0=w3'/w2'
        /// so t0=-1.     See the Willis theory for more details on these formulas.
        /// Note that t0 should be different from 1 (singularity).
        /// Once you get t0, simply use this function and it will set r1 r2 r3 automatically.
        void SetTransmissionRatioOrdinary(double t0)
        {
          /*  r1 = (1.0 - t0);
            r2 = t0;
            r3 = -1.0;*/
            ChShaftsPlanetary_SetTransmissionRatioOrdinary(m_ChShaftsPlanetary, t0);
        }

        /// Get the reaction torque considered as applied to the 1st axis.
        public double GetTorqueReactionOn1() {
            torque_react = ChShaftsPlanetary_GetTorqueReactionOn1(m_ChShaftsPlanetary);
            return torque_react;
        }

        /// Get the reaction torque considered as applied to the 2nd axis.
        public double GetTorqueReactionOn2() {
            torque_react = ChShaftsPlanetary_GetTorqueReactionOn2(m_ChShaftsPlanetary);
            return torque_react;
        }

        /// Get the reaction torque considered as applied to the 3rd axis.
        public double GetTorqueReactionOn3() {
            torque_react = ChShaftsPlanetary_GetTorqueReactionOn3(m_ChShaftsPlanetary);
            return torque_react;
        }

        // Update is called once per frame
        void Update()
        {

        }
    }

}