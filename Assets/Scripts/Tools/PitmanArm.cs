using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono
{

    public class PitmanArm : MonoBehaviour
    {
        public ChBodyAuxRef m_chassis;
        public ChLinkMotorRotationAngle m_revolute;

        [Range(0f, 10f)]
        public float lowSpeedAngle = 10f;

        [Range(0f, 10f)]
        public float highSpeedAngle = 10f;

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

        // Start is called before the first frame update
        void Start()
        {

        }

        // Update is called once per frame
        void Update()
        {
            float maxAngle = Mathf.Abs(Mathf.Lerp(lowSpeedAngle, highSpeedAngle, (float)m_chassis.BodyFrame.GetPos_dt().Length() / crossoverSpeed));
            targetAngle = maxAngle * Input.GetAxis("Horizontal");
            angle = Mathf.MoveTowards(angle, targetAngle, degreesPerSecondLimit * Time.fixedDeltaTime);

            ChFunction_Setpoint fun = (m_revolute.GetAngleFunction() as ChFunction_Setpoint);
            fun.SetSetpoint(maxAngle * angle, ChSystem.system.GetChTime());
        }
    }

}
