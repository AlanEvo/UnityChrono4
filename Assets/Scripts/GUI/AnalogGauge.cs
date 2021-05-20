using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace racingsimulation
{
    namespace vehicle
    {

        public class AnalogGauge : MonoBehaviour
        {
            /// <summary>
            /// Value at the end of needle travel, at the end angle.
            /// </summary>
            [Tooltip("Value at the end of needle travel, at the end angle.")]
            public float maxValue;

            /// <summary>
            /// Angle of the needle at the lowest value. You can use lock at start option to adjust this value while in play mode.
            /// </summary>
            [Tooltip("Angle of the needle at the lowest value. You can use lock at start option to adjust this value while in play mode.")]
            public float startAngle = 574;

            /// <summary>
            /// Angle of the needle at the highest value. You can use lock at end option to adjust this value while in play mode.
            /// </summary>
            [Tooltip("Angle of the needle at the highest value. You can use lock at end option to adjust this value while in play mode.")]
            public float endAngle = 330;

            /// <summary>
            /// Smooths the travel of the needle making it more inert, as if actually had some mass and resistance.
            /// </summary>
            [Tooltip("Smooths the travel of the needle making it more inert, as if actually had some mass and resistance.")]
            [Range(0, 1)]
            public float needleSmoothing;

            /// <summary>
            /// Locks the needle position at the start angle.
            /// </summary>
            [Tooltip("Locks the needle position at the start angle.")]
            public bool lockAtStart = false;

            /// <summary>
            /// Locks the needle position at the end angle.
            /// </summary>
            [Tooltip("Locks the needle position at the end angle.")]
            public bool lockAtEnd = false;

            private GameObject needle;
            private float currentValue;
            private float angle;
            private float prevAngle;
            private float percent;

            public float Value
            {
                get
                {
                    return currentValue;
                }
                set
                {
                    currentValue = Mathf.Clamp(value, 0, maxValue);
                }
            }

           /* private void Start()
            {
                needle = transform.Find("Needle").gameObject;
            }*/

            private void Start()
            {
                angle = startAngle;
            }

            void Update()
            {
                percent = Mathf.Clamp01(currentValue / maxValue);
                prevAngle = angle;
                angle = Mathf.Lerp(startAngle + (endAngle - startAngle) * percent, prevAngle, needleSmoothing);

                if (lockAtEnd) angle = endAngle;
                if (lockAtStart) angle = startAngle;

                needle.transform.eulerAngles = new Vector3(needle.transform.eulerAngles.x, needle.transform.eulerAngles.y, angle);
            }
        }
    }
}

