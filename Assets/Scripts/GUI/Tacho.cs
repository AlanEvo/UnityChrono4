using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono
{
    namespace vehicle {

        public class Tacho : MonoBehaviour
        {
            public Transform needle;//The needle object of the meter

            public VehicleController vc;

            public float inputValue = 0f;// The value provided by the player
            public float inputMin = 0f;// The minimum expected value for the meter's lowest reading
            public float inputMax = 0f;// The maximum expected value for the meter's highest reading
            float inputTemp = 0f;   //Temporary number

            public float currentReading = 0f; // The angle of rotation for the needle object
            public float minReading = 0f;// The minimum angle of rotation for the needle object
            public float maxReading = 0f;// The maximum angle of rotation for the needle object

            float testNum = 0;// The number used to test the 
            bool testTurn = true;// Used to cycle the test number
            public bool testing = false;  //Are we testing?

            // Use this for initialization
            void Start()
            {
                //inputValue = 5000;

                if (testing)
                {
                    testNum = inputMin;
                }
                else
                {
                    inputValue = inputMin;
                }
            }

            // Update is called once per frame
            void Update()
            {
                if (testing)
                {
                    if (testTurn)
                    {
                        testNum = testNum + 0.1f;
                        if (testNum >= inputMax)
                        {
                            testTurn = false;
                        }
                    }
                    else
                    {
                        testNum = testNum - 0.1f;
                        if (testNum <= inputMin)
                        {
                            testTurn = true;

                        }
                    }
                    inputValue = testNum;
                }
            }

            void FixedUpdate()
            {
               // inputValue = vc.engine.RPM;

                inputTemp = (inputValue - inputMin) / (inputMax - inputMin);
                currentReading = inputTemp * (maxReading - minReading) + minReading;
                //needle.localRotation = Quaternion.Euler(new Vector3(0f, 0f, currentReading));
            }
        }
    }
}

