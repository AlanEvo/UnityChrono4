using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;


namespace chrono
{
    /// Class for modeling a composite revolute-spherical joint between two
    /// two ChBodyFrame objects.  This joint is defined through a point and
    /// direction on the first body (the revolute side), a point on the second
    /// body (the spherical side), and a distance.  Kinematically, the two points
    /// are maintained at the prescribed distance while the vector between the
    /// two points is always perpendicular to the provided direction of the
    /// revolute joint.
    public class ChLinkRevoluteSpherical : ChLink
    {




        //private Vector3 p0;
        //private Vector3 p1;

        public ChLinkRevoluteSpherical()
        {
        }

        // Use this for initialization
        void Start()
        {

        }

        /// Initialize this joint by specifying the two bodies to be connected, a
        /// coordinate system specified in the absolute frame, and the distance of
        /// the massless connector.  The composite joint is constructed such that the
        /// direction of the revolute joint is aligned with the z axis of the specified
        /// coordinate system and the spherical joint is at the specified distance
        /// along the x axis.
        /*public void Initialize(ChBody body1,  //< first frame (revolute side)
                               ChBody body2,   //< second frame (spherical side)
                               ChCoordsys csys,  //< joint coordinate system (in absolute frame)
                    double distance            //< imposed distance
                    ) // Yes!

        {

        }*/

        public void Initialize(ChBody a, ChBody b)
        {
          //  ChLinkRevoluteSpherical_Initialize(this.m_ChLink, a.m_ChBody, b.m_ChBody); // no!
        }
    

        private void Update()
        {
            /*p0.x = (float)GetPoint1Abs().GetX();
            p0.y = (float)GetPoint1Abs().GetY();
            p0.z = (float)GetPoint1Abs().GetZ();
            p1.x = (float)GetPoint2Abs().GetX();
            p1.y = (float)GetPoint2Abs().GetY();
            p1.z = (float)GetPoint2Abs().GetZ();*/

            //p1 = (float)GetPoint2Abs().GetX(), (float)GetPoint2Abs().GetY(), (float)GetPoint2Abs().GetZ());
        }

        private void OnDrawGizmos()
        {
           // Gizmos.color = new Color(255, 0, 0);
           // Gizmos.DrawLine(p0, p1);
        }
    }
}
