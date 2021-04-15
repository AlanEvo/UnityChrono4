using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

namespace chrono
{
    namespace vehicle
    {

        public class Speedo : MonoBehaviour
        {

            private Text text;
            private float getSpeed = 0;

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
              //  getSpeed = (float)vc.SpeedMPH;
                text.text = Mathf.RoundToInt(getSpeed).ToString();
            }
        }
    }
}
