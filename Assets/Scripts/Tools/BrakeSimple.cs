using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono
{
    namespace vehicle
    {
        [System.Serializable]
        public class BrakeSimple : MonoBehaviour
        {
            [SerializeField]
            private ChLinkBrake m_brake;// = new ChLinkBrake();
           // public ChLinkLock hub;

            public float brakeTorque;
            public bool active = false;
            public float modulation;

            public void Awake()
            {               
                m_brake = GameObject.FindObjectOfType<ChLinkBrake>();
            }

            public void AddBrakeTorque(float torque)
            {
                m_brake.Set_brake_torque(torque);
                active = true;
            }


            public void Update()
            {
                modulation = Input.GetAxis("Brake");                
                AddBrakeTorque(modulation * brakeTorque);                
                active = false;
            }
        }
    }
}
