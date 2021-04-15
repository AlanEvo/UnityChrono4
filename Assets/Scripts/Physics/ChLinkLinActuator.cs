using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
using UnityEditor;


namespace chrono
{
    /// Class for linear actuators between two markers,
    /// as the actuator were joined with two spherical
    /// bearing at the origin of the two markers.
    /// **NOTE! THIS IS OBSOLETE**. Prefer using the new classes 
    /// inherited from chrono::ChLinkMotor.
    public class ChLinkLinActuator : ChLinkLock
    {


        protected ChFunction dist_funct;  //< distance function


        public double linearOffset;
        public Transform pos2;           ////< second endpoint, in body rel.coords
        public Vector3 axis;

        public ChFunction Get_dist_funct() { return dist_funct; }
        public void Set_dist_funct(ChFunction mf) { dist_funct = mf; }

        // Use this for initialization
        void Start()
        {

        }

        public void Set_lin_offset(double offset)
        {

        }

        protected void OnDrawGizmos()
        {
            if (Application.isPlaying)
            {
              /*  Gizmos.color = new Color(0, 255, 0);
                Gizmos.DrawSphere(EndPoint1Abs, 0.01f);

                Gizmos.color = new Color(0, 0, 255);
                Gizmos.DrawSphere(EndPoint2Abs, 0.01f);

                Gizmos.color = new Color(255, 0, 20);
                Gizmos.DrawLine(EndPoint1Abs, EndPoint2Abs);*/
            }
            else
            {
                Gizmos.color = new Color(0, 255, 0);
                Gizmos.DrawSphere(transform.position, 0.01f);


                Gizmos.color = new Color(0, 0, 255);
                //double y = (float)pos2.position.y;
                Gizmos.DrawSphere(new Vector3(pos2.position.x, pos2.position.y, pos2.position.z), 0.01f);
                //Gizmos.DrawSphere(new Vector3(pos2.position.x, (float)y - (float)linearOffset, pos2.position.z), 0.01f);

                Gizmos.color = new Color(255, 0, 20);
                //Gizmos.DrawLine(pos2.position, new Vector3(pos2.position.x, (float)y - (float)linearOffset, pos2.position.z));
                //Gizmos.DrawLine(transform.position, new Vector3(pos2.position.x, pos2.position.y, pos2.position.z));
            }
        }

        // Update is called once per frame
        void Update()
        {

        }
    }
}
