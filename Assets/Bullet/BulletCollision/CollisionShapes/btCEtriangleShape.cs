using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using BulletXNA.LinearMath;

/// btCEtriangleShape represents a triangle that is part of a collision mesh. 
/// This because the default Bullet or GImpact triangle mesh system is not flexible enough to
/// handle FEM problems where each triangle may have its collision model, and because
/// of other limitations related to robustness etc.
/// The idea is to use 'representative triangles' with additional infos on neighbours as in
/// "Fast Collision Detection for Deformable Models using Representative-Triangles"
/// S.Rasmus Tamstorf, D.Manocha1

namespace BulletXNA.BulletCollision
{

    public class btCEtriangleShape : ConvexInternalShape
    {
        public const double BT_LARGE_FLOAT = 1e18f;
        public const double CONVEX_DISTANCE_MARGIN = 0.04;

        private chrono.ChVector p1;
        private chrono.ChVector p2;
        private chrono.ChVector p3;
        private chrono.ChVector e1;
        private chrono.ChVector e2;
        private chrono.ChVector e3;
        private bool owns_vertex_1;
        private bool owns_vertex_2;
        private bool owns_vertex_3;
        private bool owns_edge_1;
        private bool owns_edge_2;
        private bool owns_edge_3;
        private double sphereswept_rad;

        public btCEtriangleShape(chrono.ChVector mp1,
                    chrono.ChVector mp2,
                    chrono.ChVector mp3,
                    chrono.ChVector me1,
                    chrono.ChVector me2,
                    chrono.ChVector me3,
                    bool mowns_vertex_1,
                    bool mowns_vertex_2,
                    bool mowns_vertex_3,
                    bool mowns_edge_1,
                    bool mowns_edge_2,
                    bool mowns_edge_3,
                    double msphereswept_rad = 0)
        {
            p1 = mp1;
            p2 = mp2;
            p3 = mp3;
            e1 = me1;
            e2 = me2;
            e3 = me3;
            owns_vertex_1 = mowns_vertex_1;
            owns_vertex_2 = mowns_vertex_2;
            owns_vertex_3 = mowns_vertex_3;
            owns_edge_1 = mowns_edge_1;
            owns_edge_2 = mowns_edge_2;
            owns_edge_3 = mowns_edge_3;
            sphereswept_rad = msphereswept_rad;

            //m_shapeType = CE_TRIANGLE_SHAPE_PROXYTYPE;
        }

        IndexedVector3 localGetSupportingVertexWithoutMargin(IndexedVector3 vec0)
        {
            IndexedVector3 supVec = new IndexedVector3(0.0f, 0.0f, 0.0f);
            float newDot, maxDot = -(float)BT_LARGE_FLOAT;
            IndexedVector3 vtx;
            vtx = new IndexedVector3((float)this.p1.x, (float)this.p1.y, (float)this.p1.z);
            newDot = vec0.Dot(vtx);
            if (newDot > maxDot) {
                maxDot = newDot;
                supVec = vtx;
            }
            vtx = new IndexedVector3((float)this.p2.x, (float)this.p2.y, (float)this.p2.z);
            newDot = vec0.Dot(vtx);
            if (newDot > maxDot) {
                maxDot = newDot;
                supVec = vtx;
            }
            vtx = new IndexedVector3((float)this.p3.x, (float)this.p3.y, (float)this.p3.z);
            newDot = vec0.Dot(vtx);
            if (newDot > maxDot) {
                maxDot = newDot;
                supVec = vtx;
            }

            return supVec;  //+ vec0.normalized()*this.sphereswept_rad; //***TODO*** add the sphereswept_rad layer (but gives seldom jittering.. why?)
        }

        ///CollisionShape Interface
        public virtual void calculateLocalInertia(float mass, ref IndexedVector3 inertia)
        {
            //***TO DO***
            //as an approximation, take the inertia of an average radius sphere

           // IndexedBasisMatrix ident = new IndexedBasisMatrix();
           // ident.setIdentity();

            IndexedVector3 halfExtents = new IndexedVector3();
            double radius = chrono.ChMaths.ChMax((p2 - p1).Length(), (p3 - p1).Length());
            halfExtents.X = ((float)radius);
            halfExtents.Y = ((float)radius);
            halfExtents.Z = ((float)radius);

            float margin = (float)CONVEX_DISTANCE_MARGIN;

            float lx = (float)2.0 * (halfExtents[0] + margin);
            float ly = (float)2.0 * (halfExtents[1] + margin);
            float lz = (float)2.0 * (halfExtents[2] + margin);
            float x2 = lx * lx;
            float y2 = ly * ly;
            float z2 = lz * lz;
            float scaledmass = mass * (float)0.08333333;

            inertia[0] = scaledmass * (y2 + z2);
            inertia[1] = scaledmass * (x2 + z2);
            inertia[2] = scaledmass * (x2 + y2);
        }

        public override void BatchedUnitVectorGetSupportingVertexWithoutMargin(IndexedVector3[] vectors, IndexedVector4[] supportVerticesOut, int numVectors)
        {
        }
        /// btConvexShape Interface
        /* public virtual float localGetSupportingVertexWithoutMargin(IndexedVector3 vec)
         {

         }*/


        /*  public virtual char getName()
              {
                  return "CEtriangleShape";
              }*/

        public virtual void getAabb(IndexedMatrix t, ref IndexedVector3 aabbMin, ref IndexedVector3 aabbMax)
        {
            IndexedVector3 p1_w = t._origin + t._basis * new IndexedVector3((float)this.p1.x, (float)this.p1.y, (float)this.p1.z);
            IndexedVector3 p2_w = t._origin + t._basis * new IndexedVector3((float)this.p2.x, (float)this.p2.y, (float)this.p2.z);
            IndexedVector3 p3_w = t._origin + t._basis * new IndexedVector3((float)this.p3.x, (float)this.p3.y, (float)this.p3.z);

            chrono.collision.ChModelBullet triModel = (chrono.collision.ChModelBullet)this.GetUserPointer();

            IndexedVector3 venvelope = new IndexedVector3(triModel.GetEnvelope(), triModel.GetEnvelope(), triModel.GetEnvelope());
            IndexedVector3 vsphereswept = new IndexedVector3((float)this.sphereswept_rad, (float)this.sphereswept_rad, (float)this.sphereswept_rad);

            aabbMin = new IndexedVector3((float)chrono.ChMaths.ChMin(chrono.ChMaths.ChMin(p1_w.X, p2_w.X), p3_w.X),
                                (float)chrono.ChMaths.ChMin(chrono.ChMaths.ChMin(p1_w.Y, p2_w.Y), p3_w.Y),
                                (float)chrono.ChMaths.ChMin(chrono.ChMaths.ChMin(p1_w.Z, p2_w.Z), p3_w.Z)) - venvelope - vsphereswept;

            aabbMax = new IndexedVector3((float)chrono.ChMaths.ChMax(chrono.ChMaths.ChMax(p1_w.X, p2_w.X), p3_w.X),
                                (float)chrono.ChMaths.ChMax(chrono.ChMaths.ChMax(p1_w.Y, p2_w.Y), p3_w.Y),
                                (float)chrono.ChMaths.ChMax(chrono.ChMaths.ChMax(p1_w.Z, p2_w.Z), p3_w.Z)) + venvelope + vsphereswept;
        }

        /// access vertex points  of triangle
        public chrono.ChVector get_p1() { return p1; }
        public chrono.ChVector get_p2() { return p2; }
        public chrono.ChVector get_p3() { return p3; }

        /// access points of neighbouring triangles at edges, if any (if no neighbour, is null ptr)
        public chrono.ChVector get_e1() { return e1; }
        public chrono.ChVector get_e2() { return e2; }
        public chrono.ChVector get_e3() { return e3; }

        /// tell if the representative triangle owns the vertex
        public bool owns_v1() { return owns_vertex_1; }
        public bool owns_v2() { return owns_vertex_2; }
        public bool owns_v3() { return owns_vertex_3; }

        /// tell if the representative triangle owns the edge
        public bool owns_e1() { return owns_edge_1; }
        public bool owns_e2() { return owns_edge_2; }
        public bool owns_e3() { return owns_edge_3; }

        /// thickness, for sphere-swept triangles.
        public double sphereswept_r() { return sphereswept_rad; }
    };
}