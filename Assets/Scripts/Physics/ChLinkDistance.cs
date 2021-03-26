using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;


namespace chrono
{
    /// <summary>
    /// Class for enforcing a fixed polar distance
    /// between two points on two ChBodyFrame objects.
    /// The two points which are used to define the end points
    /// of the distance are assumed not to move respect to the
    /// two owner ChBody, as well as the amount of the distance
    /// is assumed not to change during the simulation. If you
    /// need to have a time-varying distance, or distance between
    /// two points which move respect to the bodies, please use
    /// the more advanced ChLinkLinActuator.
    /// </summary>
    public class ChLinkDistance : ChLink
    {


        public bool autoDistance;
        public double distance;           ////< the imposed distance
        //private Vector3 pos1;           ////< first endpoint, in body rel.coords
        public Transform pos2;           ////< second endpoint, in body rel.coords

       /* ChVector end_Position1 = new ChVector();
        ChVector end_Position2 = new ChVector();*/

        public Vector3 EndPoint1Abs;
        public Vector3 EndPoint2Abs;

        // Use this for initialization
        void Start()
        {
     

            //ChVector vec1 = new ChVector(transform.localPosition.x, transform.localPosition.y, transform.localPosition.z);
            //ChVector vec2 = new ChVector(bodyB.transform.localPosition.x, bodyB.transform.localPosition.y, bodyB.transform.localPosition.z);
           /* ChVector vec1 = new ChVector(transform.position.x, transform.position.y, transform.position.z);
            ChVector vec2 = new ChVector(pos2.position.x, pos2.position.y, pos2.position.z);
            Initialize(bodyA, bodyB, positionsAreRelative, vec1, vec2, autoDistance, distance);

            m_system.AddLink(m_ChLink);*/
        }

        /// Initialize this constraint, given the two bodies to be connected, the
        /// positions of the two anchor endpoints of the distance (each expressed
        /// in body or abs. coordinates) and the imposed distance.
       /* public int Initialize(ChBody mbody1,                        //< first frame to link
                              ChBody mbody2,                        //< second frame to link
                              bool pos_are_relative,                //< true: following pos. are relative to bodies
                              ChVector mpos1,                       //< pos. of distance endpoint, for 1st body (rel. or abs., see flag above)
                              ChVector mpos2,                       //< pos. of distance endpoint, for 2nd body (rel. or abs., see flag above)
                              bool auto_distance = true,            //< if true, initializes the imposed distance as the distance between mpos1 and mpos2
                              double mdistance = 0                  //< imposed distance (no need to define, if auto_distance=true.)
                              )
        {
            ChLinkDistance_Initialize(m_ChLink, mbody1.m_ChBody, mbody2.m_ChBody, pos_are_relative, mpos1.m_ChVector, mpos2.m_ChVector, auto_distance, mdistance);
            return 1;
        }*/

      /*  public ChVector GetEndPoint1Abs()
        {
            ChVector temp = new ChVector();
            temp.m_ChVector = ChLinkDistance_GetEndPoint1Abs(m_ChLink);
            temp.data[0] = temp.GetX();
            temp.data[1] = temp.GetY();
            temp.data[2] = temp.GetZ();
            return temp;
        }*/

       /* public ChVector GetEndPoint2Abs()
        {
            ChVector temp = new ChVector();
            temp.m_ChVector = ChLinkDistance_GetEndPoint2Abs(m_ChLink);
            temp.data[0] = temp.GetX();
            temp.data[1] = temp.GetY();
            temp.data[2] = temp.GetZ();
            return temp;
        }*/

      /*  public void SetEndPoint1Rel(ChVector mset) {
            ChLinkDistance_SetEndPoint1Rel(m_ChLink, mset.m_ChVector);
        }*/

        private void FixedUpdate()
        {
           /* end_Position1 = GetEndPoint1Abs();
            end_Position2 = GetEndPoint2Abs();

            EndPoint1Abs.x = (float)end_Position1.GetX();
            EndPoint1Abs.y = (float)end_Position1.GetY();
            EndPoint1Abs.z = (float)end_Position1.GetZ();
            EndPoint2Abs.x = (float)end_Position2.GetX();
            EndPoint2Abs.y = (float)end_Position2.GetY();
            EndPoint2Abs.z = (float)end_Position2.GetZ();*/

            //p1 = (float)GetPoint2Abs().GetX(), (float)GetPoint2Abs().GetY(), (float)GetPoint2Abs().GetZ());
        }

        void OnDrawGizmos()
        {            
            if (Application.isPlaying)
            {
                Gizmos.color = new Color(0, 255, 0);
                Gizmos.DrawSphere(EndPoint1Abs, 0.01f);

                Gizmos.color = new Color(0, 0, 255);
                Gizmos.DrawSphere(EndPoint2Abs, 0.01f);

                Gizmos.color = new Color(255, 0, 20);
                Gizmos.DrawLine(EndPoint1Abs, EndPoint2Abs);
            }
            else
            {
                Gizmos.color = new Color(0, 255, 0);
                Gizmos.DrawSphere(transform.position, 0.01f);

                Gizmos.color = new Color(0, 0, 255);
                Gizmos.DrawSphere(pos2.position, 0.01f);

                Gizmos.color = new Color(255, 0, 20);
                Gizmos.DrawLine(transform.position, pos2.position);
            }
        }

    };
}
