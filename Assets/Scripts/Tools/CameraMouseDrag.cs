using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono
{

    public class CameraMouseDrag : MonoBehaviour
    {

        /// <summary>
        /// Target at which the camera will look.
        /// </summary>
        public Transform target;

        /// <summary>
        /// Distance from target at which camera will be positioned. Might vary depending on smoothing.
        /// </summary>
        [Range(0, 100f)]
        public float distance;

        /// <summary>
        /// Minimum distance that will be reached when zooming in.
        /// </summary>
        [Range(0, 100f)]
        public float minDistance = 5.0f;

        /// <summary>
        /// Maximum distance that will be reached when zooming out.
        /// </summary>
        [Range(0, 100f)]
        public float maxDistance = 13.0f;

        [Range(0, 15)]
        public float horizontalMouseSensitivity = 5.0f;

        [Range(0, 15)]
        public float verticalMouseSensitivity = 5.0f;

        [Range(0, 15)]
        public float mouseWheelSensitivity = 5.0f;

        [Range(-90, 90)]
        public float verticalMinAngle = -40.0f;

        [Range(-90, 90)]
        public float verticalMaxAngle = 80.0f;

        [Range(0, 1)]
        public float smoothing = 0.05f;

        [Range(0, 1)]
        public float distanceSmoothing = 0.05f;

        public bool followTargetsRotation = false;

        private Vector3 position;
        private float mouseX = 0.0f;
        private float mouseY = 0.0f;
        private float desiredDistance = 0.0f;
        private float velocityDistance = 0.0f;
        private Vector3 desiredPosition = Vector3.zero;
        private Vector3 velocity;
        private Quaternion rotation;
        private bool firstFrame = true;

        void Start()
        {
            distance = Mathf.Clamp(distance, minDistance, maxDistance);
            desiredDistance = distance;

            mouseY = 20f;
            mouseX = 0f;
        }

        private void OnEnable()
        {
            firstFrame = true;
        }

        // Running in fixed update due to rigidbody interpolation being none needed for WheelControllers
        void FixedUpdate()
        {
            if (target == null)
                return;

            // Handle input
            float deadZone = 0.02f;

            if (Input.GetMouseButton(0))
            {
                mouseX += Input.GetAxis("Mouse X") * horizontalMouseSensitivity;
                mouseY -= Input.GetAxis("Mouse Y") * verticalMouseSensitivity;
            }

            mouseY = ClampAngle(mouseY, verticalMinAngle, verticalMaxAngle);

            if (Mathf.Abs(Input.GetAxis("Mouse ScrollWheel")) > deadZone)
            {
                desiredDistance = Mathf.Clamp(distance - (Input.GetAxis("Mouse ScrollWheel") * mouseWheelSensitivity),
                                                                                 minDistance, maxDistance);
            }

            // Calculate position
            distance = Mathf.SmoothDamp(distance, desiredDistance, ref velocityDistance, distanceSmoothing);
            distance = Mathf.Clamp(distance, minDistance, maxDistance);

            Vector3 direction = new Vector3(0, -distance, 0);
           // rotation = Quaternion.Euler(0, 0, mouseY);

            if (followTargetsRotation)
            {
              //  desiredPosition = target.position + (target.transform.rotation * rotation * direction);
            }
            else
            {
               // desiredPosition = target.position + (rotation * direction);
            }


            if (!firstFrame)
            {
                position = Vector3.SmoothDamp(position, desiredPosition, ref velocity, smoothing);
            }
            else
            {
                position = desiredPosition;
                firstFrame = false;
            }


            // Check for ground
            RaycastHit hit;
            if (Physics.Raycast(position, -Vector3.forward, out hit, 1f))
            {
                position = hit.point + Vector3.forward * 1f;
            }

            // Set position and rotation
            transform.position = position;
            transform.LookAt(target.position, new Vector3(0, 0, 1));
        }

        public float ClampAngle(float angle, float min, float max)
        {
            while (angle < -360 || angle > 360)
            {
                if (angle < -360)
                    angle += 360;
                if (angle > 360)
                    angle -= 360;
            }

            return Mathf.Clamp(angle, min, max);
        }
    }
}
