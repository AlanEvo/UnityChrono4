using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
using System.Runtime.InteropServices;

namespace chrono
{

    /// Class for defining a gearbox. It defines a transmission ratio between two 1D
    /// entities of ChShaft type, and transmits the reaction of the gearbox to a 3D
    /// body that acts as the support truss.
    /// Note that the more basic ChShaftsGear can do the same, except that it does not
    /// provide a way to transmit reaction to a truss body.
    /// Also note that this can also be seen as a ChShaftPlanetary where one has joined
    /// the carrier shaft to a fixed body via a ChShaftsBody constraint.

    public class ChShaftsGearbox : MonoBehaviour
    {
        [DllImport("ChronoEngine", EntryPoint = "ChShaftsGearbox_Create")]
        public static extern IntPtr ChShaftsGearbox_Create(out IntPtr shaft);
        [DllImport("ChronoEngine", EntryPoint = "ChShaftsGearbox_Initialize")]
        public static extern bool ChShaftsGearbox_Initialize(IntPtr shaft, IntPtr mshaft1, IntPtr mshaft2, IntPtr bodyframe, IntPtr dir);
        [DllImport("ChronoEngine", EntryPoint = "ChShaftsGearbox_SetTransmissionRatio")]
        public static extern void ChShaftsGearbox_SetTransmissionRatio(IntPtr shaft, double t0);

       /* private double r1;  //< transmission ratios  as in   r1*w1 + r2*w2 + r3*w3 = 0
        private double r2;
        private double r3;*/


        public double torque_react;  //< reaction torque

        public double m_gearRatio;

      //  ChConstraintThreeGeneric constraint;  //< used as an interface to the solver

        public ChShaft shaft1;    //< first shaft
        public ChShaft shaft2;    //< second shaft
        public ChBodyAuxRef body;  //< support truss

        public Vector3 shaft_dir = new Vector3();  //< shaft direction

        public IntPtr m_ChShaftsGearbox = IntPtr.Zero;

        protected ChSystem m_system;

        // Use this for initialization
        void Start()
        {
            m_system = GameObject.FindObjectOfType<ChSystem>();

            m_ChShaftsGearbox = ChShaftsGearbox_Create(out m_ChShaftsGearbox);

          /*  ChVector dir = new ChVector(shaft_dir.x, shaft_dir.y, shaft_dir.z);
            Initialize(shaft1, shaft2, body, dir);
            SetTransmissionRatio(m_gearRatio);
            m_system.Add(m_ChShaftsGearbox);*/
        }

        /// Use this function after gear creation, to initialize it, given
        /// two shafts to join, and the 3D body that acts as a truss and
        /// receives the reaction torque of the gearbox.
        /// Each shaft and body must belong to the same ChSystem.
        /// Direction is expressed in the local coordinates of the body.
      /*  public bool Initialize(ChShaft mshaft1,  //< first (input) shaft to join
                               ChShaft mshaft2,  //< second  (output) shaft to join
                               ChBody mbody,  //< 3D body to use as truss (also carrier, if rotates as in planetary gearboxes)
                               ChVector mdir  //< the direction of the shaft on 3D body (applied on COG: pure torque)
                    )
        {
            return ChShaftsGearbox_Initialize(m_ChShaftsGearbox, mshaft1.m_ChShaft, mshaft2.m_ChShaft, mbody.m_ChBody, mdir.m_ChVector);
        }*/

        /// Set the transmission ratio t, as in w2=t*w1, or t=w2/w1 , or  t*w1 - w2 = 0.
        /// For example, t=1 for a rigid joint; t=-0.5 for representing
        /// a couple of spur gears with teeth z1=20 & z2=40; t=0.1 for
        /// a gear with inner teeth (or epicycloidal reducer), etc.
        /// Also the t0 ordinary equivalent ratio of the inverted planetary,
        /// if the 3D body is rotating as in planetary gears.
        public void SetTransmissionRatio(double t0)
        {
           /* r3 = (1.0 - t0);
            r1 = t0;
            r2 = -1.0;*/
            ChShaftsGearbox_SetTransmissionRatio(m_ChShaftsGearbox, t0);
        }

        // Update is called once per frame
        void Update()
        {

        }
    }
}
