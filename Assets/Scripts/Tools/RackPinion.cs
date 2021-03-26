using System.Collections;
using System.Collections.Generic;
using UnityEngine;


namespace chrono
{
    namespace vehicle
    {

        public class RackPinion : MonoBehaviour
        {
            private ChLinkLinActuator m_actuator;

            public ChBodyAuxRef m_chassis;
            public VehicleController vc;

            [Range(0f, 0.10f)]
            public float lowSpeedAngle = 0.03f;

            [Range(0f, 0.05f)]
            public float highSpeedAngle = 0.01f;

            [Range(0f, 1.0f)]
            public float degreesPerSecondLimit = 250f;

            [Tooltip("Steer angle will be multiplied by this value to get steering wheel angle.")]
            public float steeringWheelTurnRatio = 2f;

            /// <summary>
            /// Speed after which only high speed angle will be used. Also affects dynamic smoothing.
            /// </summary>
            [Tooltip("Speed after which only high speed angle will be used. Also affects dynamic smoothing.")]
            public float crossoverSpeed = 35f;

            public float angle = 0;
            //public float angle;
            //public float targetAngle;
            public float maxAngle;
            //[Range(-0,1)]
            public double steerInput;

            public float targetAngle;

            // Use this for initialization
            void Start()
            {

                m_actuator = GameObject.FindObjectOfType<ChLinkLinActuator>();
            }

            // Update is called once per frame
            void FixedUpdate()
            {

              /*  float maxAngle = Mathf.Abs(Mathf.Lerp(lowSpeedAngle, highSpeedAngle, (float)m_chassis.GetPos_dt().Length() / crossoverSpeed));
                targetAngle = maxAngle * Input.GetAxis("Horizontal");
                angle = Mathf.MoveTowards(angle, targetAngle, degreesPerSecondLimit * Time.fixedDeltaTime);            

                ChFunction_Const fun = (m_actuator.Get_dist_funct() as ChFunction_Const);
                fun.Set_yconst(angle);*/
            }
        }
    }
}
