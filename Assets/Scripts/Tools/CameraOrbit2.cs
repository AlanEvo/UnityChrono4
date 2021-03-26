using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono {

    public class CameraOrbit2 : MonoBehaviour
    {
        public Transform target;

        public float turnSpeed;
        public float height;
        public float distance;

        private ChSystem m_system;

    /// <summary>
    /// Offset in the up direction from the target. Use this if you do not want to use camera baits.
    /// </summary>
    [Tooltip("Offset in the up direction from the target. Use this if you do not want to use camera baits.")]
        [Range(-5, 5f)]
        public float targetUpOffset = 1.25f;

        /// <summary>
        /// Offset in the forward direction from the target. Use this if you do not want to use camera baits.
        /// </summary>
        [Range(-10f, 10f)]
        public float targetForwardOffset = 0;

        private Vector3 offsetY;

        void Start()
        {
            m_system = GameObject.FindObjectOfType<ChSystem>();

           /* if (m_system.ISO)
            {
                offsetY = new Vector3(0, distance, height);
            }
            else {
                offsetY = new Vector3(distance, height, 0);
            }*/

        }

        void LateUpdate()
        {

           /* if (m_system.ISO)
            {
                offsetY = Quaternion.AngleAxis(Input.GetAxis("Mouse X") * turnSpeed, Vector3.forward) * offsetY;

                transform.position = target.position + offsetY;
                transform.LookAt(target.position + Vector3.forward * targetUpOffset + target.right * targetForwardOffset, new Vector3(0, 0, 1));
            }
            else {
                offsetY = Quaternion.AngleAxis(Input.GetAxis("Mouse X") * turnSpeed, Vector3.up) * offsetY;

                transform.position = target.position + offsetY;
                transform.LookAt(target.position + Vector3.up * targetUpOffset + target.right * targetForwardOffset);
            }*/
        }
    }
}
