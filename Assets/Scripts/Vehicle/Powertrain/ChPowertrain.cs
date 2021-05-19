using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono
{

    public class ChPowertrain : MonoBehaviour
    {
        /// Driving modes.
        public enum DriveMode
        {
            FORWARD,  //< vehicle moving forward
            NEUTRAL,  //< vehicle in neutral
            REVERSE   //< vehicle moving backward
        };

        public DriveMode m_drive_mode;

        // Use this for initialization
        void Start()
        {

        }

        // Update is called once per frame
        void Update()
        {

        }
    }
}
