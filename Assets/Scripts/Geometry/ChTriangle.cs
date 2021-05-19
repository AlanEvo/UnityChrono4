using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;

namespace chrono
{
    namespace geometry
    {

        /// A triangle geometric shape for collisions and visualization.
        public class ChTriangle : ChGeometry
        {

            public ChVector p1;  //< first triangle vertex
            public ChVector p2;  //< second triangle vertex
            public ChVector p3;  //< third triangle vertex

            public const double EPS_TRIDEGENERATE = 1e-20;

            public ChTriangle()
            {
                p1 = ChVector.VNULL;
                p2 = ChVector.VNULL;
                p3 = ChVector.VNULL;
            }
            public ChTriangle(ChVector mp1, ChVector mp2, ChVector mp3)
            {
                p1 = mp1;
                p2 = mp2;
                p3 = mp3;
            }

            public ChTriangle(ChTriangle source)
            {
                p1 = source.p1;
                p2 = source.p2;
                p3 = source.p3;
            }


            /// "Virtual" copy constructor (covariant return type).
            public override ChGeometry Clone()
            {
                return new ChTriangle(this);
            }

            /// Assignment operator: copy from another triangle
            // ChTriangle& operator=(const ChTriangle& source);

            public override GeometryType GetClassType() { return GeometryType.TRIANGLE; }

            public override void GetBoundingBox(ref double xmin,
                                                ref double xmax,
                                                ref double ymin,
                                                ref double ymax,
                                                ref double zmin,
                                                ref double zmax,
                                                ChMatrix33<double> Rot)
            {
                if (Rot == null)
                {
                    xmin = ChMaths.ChMin(ChMaths.ChMin(p1.x, p2.x), p3.x);
                    ymin = ChMaths.ChMin(ChMaths.ChMin(p1.y, p2.y), p3.y);
                    zmin = ChMaths.ChMin(ChMaths.ChMin(p1.z, p2.z), p3.z);
                    xmax = ChMaths.ChMax(ChMaths.ChMax(p1.x, p2.x), p3.x);
                    ymax = ChMaths.ChMax(ChMaths.ChMax(p1.y, p2.y), p3.y);
                    zmax = ChMaths.ChMax(ChMaths.ChMax(p1.z, p2.z), p3.z);
                }
                else
                {
                    ChVector trp1 = Rot.MatrT_x_Vect(p1);
                    ChVector trp2 = Rot.MatrT_x_Vect(p2);
                    ChVector trp3 = Rot.MatrT_x_Vect(p3);
                    xmin = ChMaths.ChMin(ChMaths.ChMin(trp1.x, trp2.x), trp3.x);
                    ymin = ChMaths.ChMin(ChMaths.ChMin(trp1.y, trp2.y), trp3.y);
                    zmin = ChMaths.ChMin(ChMaths.ChMin(trp1.z, trp2.z), trp3.z);
                    xmax = ChMaths.ChMax(ChMaths.ChMax(trp1.x, trp2.x), trp3.x);
                    ymax = ChMaths.ChMax(ChMaths.ChMax(trp1.y, trp2.y), trp3.y);
                    zmax = ChMaths.ChMax(ChMaths.ChMax(trp1.z, trp2.z), trp3.z);
                }
            }

            public override ChVector Baricenter()
            {
                ChVector mb = new ChVector();
                mb.x = (p1.x + p2.x + p3.x) / 3.0;
                mb.y = (p1.y + p2.y + p3.y) / 3.0;
                mb.z = (p1.z + p2.z + p3.z) / 3.0;
                return mb;
            }


            public override void CovarianceMatrix(ChMatrix33<double> C)
            {
                C.nm.matrix[0, 0] = p1.x * p1.x + p2.x * p2.x + p3.x * p3.x;
                C.nm.matrix[1, 1] = p1.y * p1.y + p2.y * p2.y + p3.y * p3.y;
                C.nm.matrix[2, 2] = p1.z * p1.z + p2.z * p2.z + p3.z * p3.z;
                C.nm.matrix[0, 1] = p1.x * p1.y + p2.x * p2.y + p3.x * p3.y;
                C.nm.matrix[0, 2] = p1.x * p1.z + p2.x * p2.z + p3.x * p3.z;
                C.nm.matrix[1, 2] = p1.y * p1.z + p2.y * p2.z + p3.y * p3.z;
            }

            /// This is a surface
            public override int GetManifoldDimension() { return 2; }

            // return false if triangle has almost zero area
            public bool IsDegenerated()
            {
                ChVector u = ChVector.Vsub(p2, p1);
                ChVector v = ChVector.Vsub(p3, p1);

                ChVector vcr = new ChVector();
                vcr = ChVector.Vcross(u, v);
                if (Mathfx.Abs(vcr.x) < EPS_TRIDEGENERATE && Math.Abs(vcr.y) < EPS_TRIDEGENERATE && Math.Abs(vcr.z) < EPS_TRIDEGENERATE)
                    return true;
                return false;
            }

            // compute triangle normal
            public bool Normal(ref ChVector N)
            {
                ChVector u;
                u = ChVector.Vsub(p2, p1);
                ChVector v;
                v = ChVector.Vsub(p3, p1);

                ChVector n;
                n = ChVector.Vcross(u, v);

                double len = ChVector.Vlength(n);

                if (Mathfx.Abs(len) > EPS_TRIDEGENERATE)
                    N = ChVector.Vmul(n, (1.0 / len));
                else
                    return false;

                return true;
            }

            public ChVector GetNormal()
            {
                ChVector mn = new ChVector();
                Normal(ref mn);
                return mn;
            }

            /// Given point B and a generic triangle, computes the distance from the triangle plane,
            /// returning also the projection of point on the plane and other infos
            ///			\return the signed distance
            public static double PointTriangleDistance(ChVector B,           //< point to be measured
                                        ref ChVector A1,         //< point of triangle
                                        ref ChVector A2,         //< point of triangle
                                        ref ChVector A3,         //< point of triangle
                                        ref double mu,             //< returns U parametric coord of projection
                                        ref double mv,             //< returns V parametric coord of projection
                                        ref bool is_into,          //< returns true if projection falls on the triangle
                                        ref ChVector Bprojected  //< returns the position of the projected point
    )
            {
                // defaults
                is_into = false;
                mu = mv = -1;
                double mdistance = 10e22;

                ChVector Dx, Dy, Dz, T1, T1p;

                Dx = ChVector.Vsub(A2, A1);
                Dz = ChVector.Vsub(A3, A1);
                Dy = ChVector.Vcross(Dz, Dx);

                double dylen = ChVector.Vlength(Dy);

                if (Mathfx.Abs(dylen) < EPS_TRIDEGENERATE)  // degenerate triangle
                    return mdistance;

                Dy = ChVector.Vmul(Dy, 1.0 / dylen);

                ChMatrix33<double> mA = new ChMatrix33<double>(0);
                ChMatrix33<double> mAi = new ChMatrix33<double>(0);
                mA.Set_A_axis(Dx, Dy, Dz);

                // invert triangle coordinate matrix -if singular matrix, was degenerate triangle-.
                if (Mathfx.Abs(mA.FastInvert(mAi)) < 0.000001)
                    return mdistance;

                T1 = mAi.Matr_x_Vect(ChVector.Vsub(B, A1));
                T1p = T1;
                T1p.y = 0;
                mu = T1.x;
                mv = T1.z;
                if (mu >= 0 && mv >= 0 && mv <= 1.0 - mu)
                {
                    is_into = true;
                    mdistance = Mathfx.Abs(T1.y);
                    Bprojected = ChVector.Vadd(A1, mA.Matr_x_Vect(T1p));
                }

                return mdistance;
            }

            /// Given point B, computes the distance from this triangle plane,
            /// returning also the projection of point on the plane and other infos
            ///			\return the signed distance
            public double PointTriangleDistance(ChVector B,           //< point to be measured
                                 ref double mu,             //< returns U parametric coord of projection
                                 ref double mv,             //< returns V parametric coord of projection
                                 ref bool is_into,          //< returns true if projection falls on the triangle
                                 ref ChVector Bprojected  //< returns the position of the projected point
                                            )
            {
                return PointTriangleDistance(B, ref this.p1, ref this.p2, ref this.p3, ref mu, ref mv, ref is_into, ref Bprojected);
            }

            /// Calculate distance between a point p and a line identified
            /// with segment dA,dB. Returns distance. Also, the mu value reference
            /// tells if the nearest projection of point on line falls into segment (for mu 0...1)
            ///			\return the distance
            public static double PointLineDistance(
                                                ref ChVector p,      //< point to be measured
                                                ref ChVector dA,     //< a point on the line
                                                ref ChVector dB,     //< another point on the line
                                                ref double mu,         //< parametric coord: if in 0..1 interval, projection is between dA and dB
                                                ref bool is_insegment  //< returns true if projected point is between dA and dB
                                                        )
            {
                mu = -1.0;
                is_insegment = false;
                double mdist = 10e34;

                ChVector vseg = ChVector.Vsub(dB, dA);
                ChVector vdir = ChVector.Vnorm(vseg);
                ChVector vray = ChVector.Vsub(p, dA);

                mdist = ChVector.Vlength(ChVector.Vcross(vray, vdir));
                mu = ChVector.Vdot(vray, vdir) / ChVector.Vlength(vseg);

                if ((mu >= 0) && (mu <= 1.0))
                    is_insegment = true;

                return mdist;
            }
        }

    }
}
