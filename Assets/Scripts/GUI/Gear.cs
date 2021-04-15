using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

namespace chrono
{
    namespace vehicle
    {

        public class Gear : MonoBehaviour
        {
            private Text text;
            private float getGear = 0;

            private VehicleController vc;

            // Use this for initialization
            void Start()
            {
                text = this.GetComponent<Text>();
                vc = GameObject.FindObjectOfType<VehicleController>();
            }

            // Update is called once per frame
            void FixedUpdate()
            {
              //  getGear = (float)vc.transmission.gear;
                text.text = Mathf.RoundToInt(getGear).ToString();
            }
        }
    }
}
