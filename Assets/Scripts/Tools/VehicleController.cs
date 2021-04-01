using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;

namespace chrono
{
    namespace vehicle
    {

        public class VehicleController : MonoBehaviour
        {
            public ChBodyAuxRef chassisBody;
            public double speed;

            private ChSystem m_system;

           // [SerializeField]
           // public ChSimpleCustomPowertrain engine = new ChSimpleCustomPowertrain();

           // [SerializeField]
           // public ChCustomDriveline transmission = new ChCustomDriveline();

           // [SerializeField]
           // public List<ChAxle> axles = new List<ChAxle>();

           // [SerializeField]
           // public BrakeController brakeController = new BrakeController();

            // bool m_apply_drag = true;     //< enable aerodynamic drag force?
            // double m_Cd;           //< drag coefficient
            // double m_area;         //< reference area (m2)
            // double m_air_density;  //< air density (kg/m3)
            //ChBody body;

            public float vertical;
            public float handbrake;

            public Vector3 inertiaXX = new Vector3();
            protected ChMatrix33<double> m_inertia;
            protected ChVector m_inertiaXX = new ChVector(0, 0, 0);
            protected ChVector m_inertiaXY = new ChVector(0, 0, 0);

            public Vector3 airDrag = new Vector3();

            public float Unity_Time;
            public double Chrono_Time;

            public bool m_apply_drag;     //< enable aerodynamic drag force?
            public double m_Cd;           //< drag coefficient
            public double m_area;         //< reference area (m2)
            public double m_air_density;  //< air density (kg/m3)
            public float forwardVelocity;
            private float forwardAcceleration;
            private Vector3 velocity;
            private Vector3 prevVelocity;
            private bool wheelSpin;
            private bool wheelSkid;
            public bool canControl;

           

            // Use this for initialization
            void Start()
            {
                m_system = GameObject.FindObjectOfType<ChSystem>();

                m_inertia = new ChMatrix33<double>();

                chassisBody = GetComponent<ChBodyAuxRef>();
              /*  m_inertiaXX.data[0] = inertiaXX.x;
                m_inertiaXX.data[1] = inertiaXX.y;
                m_inertiaXX.data[2] = inertiaXX.z;
                m_inertia.SetElement(0, 0, m_inertiaXX.data[0]);
                m_inertia.SetElement(1, 1, m_inertiaXX.data[1]);
                m_inertia.SetElement(2, 2, m_inertiaXX.data[2]);

                m_inertia.SetElement(0, 1, m_inertiaXY.data[0]);
                m_inertia.SetElement(0, 2, m_inertiaXY.data[1]);
                m_inertia.SetElement(1, 2, m_inertiaXY.data[2]);
                m_inertia.SetElement(1, 0, m_inertiaXY.data[0]);
                m_inertia.SetElement(2, 0, m_inertiaXY.data[1]);
                m_inertia.SetElement(2, 1, m_inertiaXY.data[2]);*/

               // chassisBody.SetInertia(m_inertia);
                //SetAerodynamicDrag(0.5, 5.0, 1.2);

            }

            // -----------------------------------------------------------------------------
            // Simple model of aerodynamic drag forces.
            // The drag force, calculated based on the forward vehicle speed, is applied to
            // the center of mass of the chassis body.
            // -----------------------------------------------------------------------------
            public void SetAerodynamicDrag(double Cd, double area, double air_density)
            {
                 m_Cd = Cd;
                 m_area = area;
                 m_air_density = air_density;

                 m_apply_drag = true;
            }

            void FixedUpdate()
            {
                Unity_Time = Time.fixedDeltaTime;
                Chrono_Time = m_system.GetChTime();

                vertical = Input.GetAxis("Vertical");
                handbrake = Input.GetAxis("Handbrake");

              //  speed = GetSpeed();


                if (m_apply_drag)
                {

                    // Calculate aerodynamic drag force (in chassis local frame)
                  //  ChVector V = chassisBody.TransformDirectionParentToLocal(chassisBody.GetPos_dt());
                   // airDrag.x = (float)V.data[0];
                   // airDrag.y = (float)V.data[1];
                   // airDrag.z = (float)V.data[2];
                   // double Vx = V.data[0];
                   // double Fx = 0.5 * m_Cd * m_area * m_air_density * Vx * Vx;
                    //ChVector F = new ChVector(-Fx * Math.Sign(Vx), 0.0, 0.0);

                    // Apply aerodynamic drag force at COM
                    //chassisBody.Empty_forces_accumulators();
                   // chassisBody.Accumulate_force(F, new ChVector(0), true);
                }
            }


            /// Get the vehicle speed.
            /// Return the speed measured at the origin of the chassis reference frame.
           // public double GetSpeed() { return chassisBody.GetPos_dt().Length(); }
        }
    }
}
