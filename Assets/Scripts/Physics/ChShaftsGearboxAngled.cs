using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
using System.Runtime.InteropServices;

namespace chrono
{

    ///  Class for defining a gearbox with 1D input and 1D output, but with
    ///  different directions in 3D space. Basically it defines a
    ///  transmission ratio between two 1D entities of ChShaft type, and
    ///  transmits the reaction of the gearbox to a 3D body that acts as the
    ///  support truss.
    ///  A typical example is the case of a gearbox with bevel gears,
    ///  where input shaft and output shaft are at 90°.
    ///  Note that the more basic ChShaftsGear can do the same, except
    ///  that it does not provide a way to transmit reaction
    ///  to a truss body.
    public class ChShaftsGearboxAngled : MonoBehaviour
    {
        [DllImport("ChronoEngine", EntryPoint = "ChShaftsGearboxAngled_Create")]
        public static extern IntPtr ChShaftsGearboxAngled_Create(out IntPtr shaft);
        [DllImport("ChronoEngine", EntryPoint = "ChShaftsGearboxAngled_Initialize")]
        public static extern bool ChShaftsGearboxAngled_Initialize(IntPtr shaft, IntPtr mshaft1, IntPtr mshaft2, IntPtr bodyframe, IntPtr mdir1, IntPtr mdir2);
        [DllImport("ChronoEngine", EntryPoint = "ChShaftsGearboxAngled_SetTransmissionRatio")]
        public static extern void ChShaftsGearboxAngled_SetTransmissionRatio(IntPtr shaft, double t0);

        private double t0;

        public double torque_react;  //< reaction torque

        //ChConstraintThreeGeneric constraint;  ///< used as an interface to the solver

        public ChShaft shaft1;    //< first connected shaft
        public ChShaft shaft2;    //< second connected shaft
        public ChBody body;  //< connected body

        public Vector3 shaft_dir1 = new Vector3();  //< direction of first shaft
        public Vector3 shaft_dir2 = new Vector3();  //< direction of second shaft

        public double m_conicalgear_ratio;
        public double m_differential_ratio;
        public double m_axle_differential_locking_limit;

        public IntPtr m_ChShaftsGearboxAngled = IntPtr.Zero;

        protected ChSystem m_system;

        // Use this for initialization
        void Start()
        {
            m_system = GameObject.FindObjectOfType<ChSystem>();

            m_ChShaftsGearboxAngled = ChShaftsGearboxAngled_Create(out m_ChShaftsGearboxAngled);

          /*  ChVector dir1 = new ChVector(shaft_dir1.x, shaft_dir1.y, shaft_dir1.z);
            ChVector dir2 = new ChVector(shaft_dir2.x, shaft_dir2.y, shaft_dir2.z);
            Initialize(shaft1, shaft2, body, dir1, dir2);
            SetTransmissionRatio(m_conicalgear_ratio);
            m_system.Add(m_ChShaftsGearboxAngled);*/
        }

        /// Use this function after gear creation, to initialize it, given
        /// two shafts to join, and the 3D body that acts as a truss and
        /// receives the reaction torque of the gearbox.
        /// Each shaft and body must belong to the same ChSystem.
        /// Shafts directions are considered in local body coordinates.
       /* public bool
        Initialize(ChShaft mshaft1,    //< first (input) shaft to join
               ChShaft mshaft2,    //< second  (output) shaft to join
               ChBody mbody,  //< 3D body to use as truss (also carrier, if rotates as in planetary gearboxes)
               ChVector mdir1,  //< the direction of the first shaft on 3D body defining the gearbox truss
               ChVector mdir2   //< the direction of the first shaft on 3D body defining the gearbox truss
               )
        {
            return ChShaftsGearboxAngled_Initialize(m_ChShaftsGearboxAngled, mshaft1.m_ChShaft, mshaft2.m_ChShaft, mbody.m_ChBody, mdir1.m_ChVector, mdir2.m_ChVector);
        }*/

        /// Set the transmission ratio t, as in w2=t*w1, or t=w2/w1 , or  t*w1 - w2 = 0.
        /// For example, t=1 for equal bevel gears, etc.
        public void SetTransmissionRatio(double mt0) {
            t0 = mt0;
            ChShaftsGearboxAngled_SetTransmissionRatio(m_ChShaftsGearboxAngled, t0);
        }

        // Update is called once per frame
        void Update()
        {

        }
    }
}
