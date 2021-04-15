using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

namespace racingsimulation
{
    namespace vehicle
    {

        public class DigitalGauge : MonoBehaviour
        {
            [Tooltip("Should string value or numerical value be used?")]
            public bool displayNumerical = true;

            [Tooltip("Numerical value. Will be auto converted to string.")]
            public float numericalValue;

            [Tooltip("Numerical value. Will be auto converted to string.")]
            public string stringValue;

            public string format = "0.0";
            [Range(0, 1)]
            public float numericalSmoothing = 0.5f;
            public string unit;
            public bool dynamicLine = false;
            public float maxValue;

            private Text readout;
            private Image line;
            private float fullLineWidth;
            private float prevNumericalValue;

            void Start()
            {
                // Find readout
                Transform readoutTransform = transform.Find("Readout");
                if (readoutTransform != null)
                {
                    readout = readoutTransform.gameObject.GetComponent<Text>();
                }

                // Find line
                Transform lineTransform = transform.Find("Line");
                if (lineTransform != null)
                {
                    line = lineTransform.gameObject.GetComponent<Image>();
                }

                // Disable dynamic line if non-numerical display
                if (!displayNumerical)
                {
                    dynamicLine = false;
                }

                // Remember initial line width
                if (line != null)
                {
                    fullLineWidth = line.rectTransform.sizeDelta.x;
                }
            }

            void Update()
            {
                // Update readout
                if (readout != null)
                {
                    readout.text = "";
                    if (displayNumerical)
                    {
                        numericalValue = Mathf.SmoothStep(prevNumericalValue, numericalValue, 1.05f - numericalSmoothing);
                        readout.text += numericalValue.ToString(format);
                        prevNumericalValue = numericalValue;
                    }

                    string space = "";
                    if (unit != "") space = " ";
                    readout.text += stringValue + space + unit;
                }

                // Update line
                if (line != null && dynamicLine)
                {
                    float percentage = Mathf.Clamp01(numericalValue / maxValue);
                    line.rectTransform.sizeDelta = new Vector2(percentage * fullLineWidth, line.rectTransform.sizeDelta.y);
                }
            }
        }

    }
}
