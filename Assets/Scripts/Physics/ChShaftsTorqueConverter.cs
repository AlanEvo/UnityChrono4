using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
using System.Runtime.InteropServices;

namespace chrono
{

    /// Class for defining a torque converter between two one-degree-of-freedom parts;
    /// i.e., shafts that can be used to build 1D models of powertrains. Note that is
    /// not inherited from ChShaftsTorqueBase, because it requires a third part: the stator.
    /// The torque converter multiplies the input torque if there is slippage between input
    /// and output, then the multiplicative effect becomes closer to unity when the slippage
    /// is almost null; so it is similar to a variable-transmission-ratio gearbox, and just
    /// like any gearbox it requires a truss (the 'stator') that gets some torque.
    /// Note: it can work only in a given direction.

    public class ChShaftsTorqueConverter : MonoBehaviour
    {
        [DllImport("ChronoEngine", EntryPoint = "ChShaftsTorqueConverter_Create")]
        public static extern IntPtr ChShaftsTorqueConverter_Create(out IntPtr shaft);
        [DllImport("ChronoEngine", EntryPoint = "ChShaftsTorqueConverter_Initialize")]
        public static extern bool ChShaftsTorqueConverter_Initialize(IntPtr shaft, IntPtr mshaft1, IntPtr mshaft2, IntPtr stator);
        [DllImport("ChronoEngine", EntryPoint = "ChShaftsTorqueConverter_SetCurveCapacityFactor")]
        public static extern void ChShaftsTorqueConverter_SetCurveCapacityFactor(IntPtr shaft, IntPtr mf);
        [DllImport("ChronoEngine", EntryPoint = "ChShaftsTorqueConverter_SetCurveTorqueRatio")]
        public static extern void ChShaftsTorqueConverter_SetCurveTorqueRatio(IntPtr shaft, IntPtr mf);
        [DllImport("ChronoEngine", EntryPoint = "ChShaftsTorqueConverter_GetSpeedRatio")]
        public static extern double ChShaftsTorqueConverter_GetSpeedRatio(IntPtr shaft);
        [DllImport("ChronoEngine", EntryPoint = "ChShaftsTorqueConverter_GetTorqueReactionOnInput")]
        public static extern double ChShaftsTorqueConverter_GetTorqueReactionOnInput(IntPtr shaft);
        [DllImport("ChronoEngine", EntryPoint = "ChShaftsTorqueConverter_GetTorqueReactionOnOutput")]
        public static extern double ChShaftsTorqueConverter_GetTorqueReactionOnOutput(IntPtr shaft);

        public ChShaft shaft1;
        public ChShaft shaft2;
        public ChShaft shaft_stator;

        public double torque_in;
        public double torque_out;

        private ChFunction K;
        private ChFunction T;

        private bool state_warning_reverseflow;
        private bool state_warning_wrongimpellerdirection;

        public IntPtr m_ChShaftsTorqueConverter = IntPtr.Zero;

        protected ChSystem m_system;

        // Use this for initialization
        void Start()
        {
            m_system = GameObject.FindObjectOfType<ChSystem>();

            m_ChShaftsTorqueConverter = ChShaftsTorqueConverter_Create(out m_ChShaftsTorqueConverter);

            Initialize(shaft1, shaft2, shaft_stator);
        }

        /// Use this function after torque converter creation, to initialize it, given
        /// input and output shafts to join (plus the stator shaft, that should be fixed).
        /// Each shaft must belong to the same ChSystem.
        public bool Initialize(ChShaft mshaft1,       //< input shaft
                               ChShaft mshaft2,      //< output shaft
                               ChShaft mshaft_stator  //< stator shaft (often fixed)
                    )
        {
            return false;
        }

        /// Set the capacity factor curve, function of speed ratio R.
        /// It is K(R)= input speed / square root of the input torque.
        /// Units: (rad/s) / sqrt(Nm)
        public void SetCurveCapacityFactor(ChFunction mf) {
            K = mf;
        }

        /// Set the torque ratio curve, function of speed ratio R.
        /// It is T(R) = (output torque) / (input torque)
        public void SetCurveTorqueRatio(ChFunction mf) {
            T = mf;
        }

        /// Get the actual slippage, for slippage = 1 complete slippage,
        /// for slippage = 0 perfect locking.
        public double GetSlippage()
        {
            return ChShaftsTorqueConverter_GetSpeedRatio(m_ChShaftsTorqueConverter);
        }

        /// Get the torque applied to the input shaft
        public double GetTorqueReactionOnInput() {
            torque_in = ChShaftsTorqueConverter_GetTorqueReactionOnInput(m_ChShaftsTorqueConverter);
            return torque_in;
        }

        /// Get the torque applied to the output shaft
        public double GetTorqueReactionOnOutput() {
            torque_out = ChShaftsTorqueConverter_GetTorqueReactionOnOutput(m_ChShaftsTorqueConverter);
            return torque_out;
        }

// Update is called once per frame
void Update()
        {

        }
    };
}
