using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;

namespace chrono
{

    public static class Utils
    {
        public static Vector3 FromChrono(ChVector v)
        {
            return new Vector3((float)v.x, (float)v.y, (float)v.z);
        }

        public static UnityEngine.Quaternion FromChrono(ChQuaternion q)
        {
            return new UnityEngine.Quaternion((float)q.e1, (float)q.e2, (float)q.e3, (float)q.e0);
        }

        public static ChVector ToChrono(Vector3 v)
        {
            return new ChVector(v.x, v.y, v.z);
        }

        public static ChQuaternion ToChrono(UnityEngine.Quaternion q)
        {
            return new ChQuaternion(q.w, q.x, q.y, q.z);
        }

        // -----------------------------------------------------------------------------
        // Draw a spring in 3D space, with given color.
        // -----------------------------------------------------------------------------
        public static void drawSpring(double radius,
                               ChVector start,
                               ChVector end,
                               int mresolution,
                               double turns)
        {
            ChMatrix33<double> rel_matrix = new ChMatrix33<double>();
            ChVector dist = end - start;
            ChVector Vx = new ChVector();
            ChVector Vy = new ChVector();
            ChVector Vz = new ChVector();
            double length = dist.Length();
            ChVector dir = ChVector.Vnorm(dist);
            ChVector.XdirToDxDyDz(dir, ChVector.VECT_Y, ref Vx, ref Vy, ref Vz);
            rel_matrix.Set_A_axis(Vx, Vy, Vz);
            ChQuaternion Q12 = rel_matrix.Get_A_quaternion();
            ChCoordsys<double> mpos = new ChCoordsys<double>(start, Q12);

            double phaseA = 0;
            double phaseB = 0;
            double heightA = 0;
            double heightB = 0;

            for (int iu = 1; iu <= mresolution; iu++)
            {
                phaseB = turns * ChMaths.CH_C_2PI * (double)iu / (double)mresolution;
                heightB = length * ((double)iu / mresolution);
                ChVector V1 = new ChVector(heightA, radius * Math.Cos(phaseA), radius * Math.Sin(phaseA));
                ChVector V2 = new ChVector(heightB, radius * Math.Cos(phaseB), radius * Math.Sin(phaseB));
                Gizmos.color = new Color(255, 255, 0);
                Gizmos.DrawLine(new Vector3((float)mpos.TransformLocalToParent(V1).x, (float)mpos.TransformLocalToParent(V1).y, (float)mpos.TransformLocalToParent(V1).z),
                                new Vector3((float)mpos.TransformLocalToParent(V2).x, (float)mpos.TransformLocalToParent(V2).y, (float)mpos.TransformLocalToParent(V2).z));
                phaseA = phaseB;
                heightA = heightB;
            }
        }
    }
}
