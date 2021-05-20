using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;

using BulletXNA.BulletCollision;
using BulletXNA.LinearMath;
using System.Linq;
//using BulletSharp;
//using BulletSharp.Math;
//using BulletUnity;

namespace chrono
{
    namespace collision
    {
        public class Pair<T, U>
        {
            public Pair()
            {
            }

            public Pair(T first, U second)
            {
                this.First = first;
                this.Second = second;
            }

            public T First { get; set; }
            public U Second { get; set; }
        };

        ///  A wrapper to use the Bullet collision detection  library
        public class ChModelBullet : ChCollisionModel
        {

            // The Bullet collision object containing Bullet geometries
            protected CollisionObject bt_collision_object;

            

            // Vector of shared pointers to geometric objects.
            protected List<CollisionShape> shapes = new List<CollisionShape>();

            public ChModelBullet()
            {
                bt_collision_object = new CollisionObject();
                bt_collision_object.SetCollisionShape(null);
                bt_collision_object.SetUserPointer(this);

                shapes.Clear();
            }

            public ChModelBullet(ChBody body)
            {
                
            }

            public override int ClearModel()
            {
                // delete previously added shapes, if collision shape(s) used by collision object
                if (shapes.Count > 0)
                {
                    // deletes shared pointers, so also deletes shapes if uniquely referenced
                    shapes.Clear();

                    // tell to the parent collision system to remove this from collision system,
                    // if still connected to a physical system
                    if (GetContactable() != null)
                        if (GetPhysicsItem())
                            if (GetPhysicsItem().GetSystem())
                                if (GetPhysicsItem().GetCollide())
                                    GetPhysicsItem().GetSystem().GetCollisionSystem().Remove(this);

                    // at the end, no collision shape
                    bt_collision_object.SetCollisionShape(null);
                }

                return 1;
            }



            /// Gets the pointer to the client owner ChPhysicsItem.
            /// Default: just casts GetContactable(). Just for backward compatibility.
            /// It might return null if contactable not inherited by  ChPhysicsItem.
            /// ***TODO*** remove the need of ChPhysicsItem*, just use ChContactable* in all code
            public virtual ChPhysicsItem GetPhysicsItem() {
                return mcontactable.GetPhysicsItem();
            }

            public override int BuildModel()
            {
                // insert again (we assume it was removed by ClearModel!!!)
                if (GetContactable() != null)
                    if (GetPhysicsItem())
                        if (GetPhysicsItem().GetSystem())
                            if (GetPhysicsItem().GetCollide())
                                GetPhysicsItem().GetSystem().GetCollisionSystem().Add(this);

                return 1;
            }

            /// Return the pointer to the Bullet model
            public CollisionObject GetBulletModel() { return this.bt_collision_object; }

            /// Return the collision family group of this model.
            /// The collision family of this model is the position of the single set bit
            /// in the return value.
            public virtual int GetFamilyGroup() { return family_group; }

            /// Set the collision family group of this model.
            /// This is an alternative way of specifying the collision family for this
            /// object.  The value family_group must have a single bit set (i.e. it must
            /// be a power of 2). The corresponding family is then the bit position.
            public virtual void SetFamilyGroup(int group) { }

            /// Return the collision mask for this model.
            /// Each bit of the return value indicates whether this model collides with
            /// the corresponding family (bit set) or not (bit unset).
            public virtual int GetFamilyMask() { return family_mask; }

            /// Set the collision mask for this model.
            /// Any set bit in the specified mask indicates that this model collides with
            /// all objects whose family is equal to the bit position.
            public virtual void SetFamilyMask(int mask) { }

            private void _injectShape(ChVector pos, ChMatrix33<double> rot, CollisionShape mshape)
            {
                bool centered = true;// (pos.IsNull() && rot.IsIdentity());  // FIX THIS !!!

                // This is needed so later one can access ChModelBullet::GetSafeMargin and ChModelBullet::GetEnvelope
                mshape.SetUserPointer(this);

                // start_vector = ||    -- description is still empty
                if (shapes.Count == 0) {
                    if (centered)
                    {
                        shapes.Add(mshape);
                        bt_collision_object.SetCollisionShape(mshape);
                        // end_vector=  | centered shape |
                        return;
                    } else {
                        CompoundShape mcompound = new CompoundShape();
                        shapes.Add(mcompound);
                        shapes.Add(mshape);
                        bt_collision_object.SetCollisionShape(mcompound);
                        IndexedMatrix mtransform = new IndexedMatrix();
                        ChPosMatrToBullet(pos, rot, ref mtransform);
                        mcompound.AddChildShape(ref mtransform, mshape);
                        // vector=  | compound | not centered shape |
                        return;
                    }
                }
                // start_vector = | centered shape |    ----just a single centered shape was added
                if (shapes.Count == 1) {
                    IndexedMatrix mtransform = new IndexedMatrix();
                    shapes.Add(shapes[0]);
                    shapes.Add(mshape);
                    CompoundShape mcompound = new CompoundShape(true);
                    shapes[0] = mcompound;
                    bt_collision_object.SetCollisionShape(mcompound);
                    //mtransform.setIdentity();
                    mcompound.AddChildShape(ref mtransform, shapes[1]);
                    ChPosMatrToBullet(pos, rot, ref mtransform);
                    mcompound.AddChildShape(ref mtransform, shapes[2]);
                    // vector=  | compound | old centered shape | new shape | ...
                    return;
                }
                // vector=  | compound | old | old.. |   ----already working with compounds..
                if (shapes.Count > 1)
                {
                    IndexedMatrix mtransform = new IndexedMatrix();
                    shapes.Add(mshape);
                    ChPosMatrToBullet(pos, rot, ref mtransform);
                    CollisionShape mcom = shapes[0];
                    ((CompoundShape)mcom).AddChildShape(ref mtransform, mshape);
                    // vector=  | compound | old | old.. | new shape | ...
                    return;
                }
            }

            public static void ChPosMatrToBullet(ChVector pos, ChMatrix33<double> rA, ref BulletXNA.LinearMath.IndexedMatrix mtransform) {
                IndexedBasisMatrix basisA = new IndexedBasisMatrix((float)rA.nm.matrix[0, 0], (float)rA.nm.matrix[0, 1], (float)rA.nm.matrix[0, 2], (float)rA.nm.matrix[1, 0],
                                                                    (float)rA.nm.matrix[1, 1], (float)rA.nm.matrix[1, 2], (float)rA.nm.matrix[2, 0], (float)rA.nm.matrix[2, 1],
                                                                    (float)rA.nm.matrix[2, 2]);

             

                mtransform._basis = basisA;
                mtransform._origin = new IndexedVector3((float)pos.x, (float)pos.y, (float)pos.z);
            }

            /// Add a sphere shape to this model, for collision purposes
            public override bool AddSphere(double radius, ChVector pos)
            {
                // adjust default inward 'safe' margin (always as radius)
                this.SetSafeMargin(radius);

                SphereShape mshape = new SphereShape((float)(radius + this.GetEnvelope()));

                mshape.SetMargin((float)this.GetSuggestedFullMargin());

                _injectShape(pos, new ChMatrix33<double>(1), mshape);

                return true;
            }


            /// Add a box shape to this model, for collision purposes
            public override bool AddBox(
                    double hx,                                 /// the halfsize on x axis
                    double hy,                                 /// the halfsize on y axis
                    double hz,                                  /// the halfsize on z axis
                    ChVector pos,      /// the position of the box COG
                    ChMatrix33<double> rot  /// the rotation of the box - matrix must be orthogonal
                    )
            {
                // adjust default inward margin (if object too thin)
                this.SetSafeMargin(ChMaths.ChMin(this.GetSafeMargin(), 0.2 * ChMaths.ChMin(ChMaths.ChMin(hx, hy), hz)));

                float ahx = (float)(hx + this.GetEnvelope());
                float ahy = (float)(hy + this.GetEnvelope());
                float ahz = (float)(hz + this.GetEnvelope());
                BoxShape mshape = new BoxShape(new IndexedVector3(ahx, ahy, ahz));

                mshape.SetMargin((float)this.GetSuggestedFullMargin());

                _injectShape(pos, rot, mshape);

                return true;
            }

            /// Add a cylinder to this model (default axis on Y direction), for collision purposes
            public override bool AddCylinder(double rx,
                                     double rz,
                                     double hy,
                                     ChVector pos,
                                     ChMatrix33<double> rot)
            {
                // adjust default inward margin (if object too thin)
                this.SetSafeMargin(ChMaths.ChMin(this.GetSafeMargin(), 0.2 * ChMaths.ChMin(ChMaths.ChMin(rx, rz), 0.5 * hy)));

                float arx = (float)(rx + this.GetEnvelope());
                float arz = (float)(rz + this.GetEnvelope());
                float ahy = (float)(hy + this.GetEnvelope());
                CylinderShape mshape = new CylinderShape(new IndexedVector3(arx, ahy, arz));

                mshape.SetMargin(this.GetSuggestedFullMargin());

                _injectShape(pos, rot, mshape);

                return true;
            }

            public static IndexedVector3 ChVectToBullet(ChVector pos)
            {
                return new IndexedVector3((float)pos.x, (float)pos.y, (float)pos.z);
            }

            /// Add a triangle from  mesh.
            /// For efficiency, points are stored as pointers. Thus, the user must
            /// take care of memory management and of dangling pointers.
            public virtual bool AddTriangleProxy(ChVector p1,                //< points to vertex1 coords
                                        ChVector p2,                 //< points to vertex2 coords
                                        ChVector p3,                 //< points to vertex3 coords
                                        ChVector ep1,                //< points to neighbouring vertex at edge1 if any
                                        ChVector ep2,                //< points to neighbouring vertex at edge1 if any
                                        ChVector ep3,             //< points to neighbouring vertex at edge1 if any
                                        bool mowns_vertex_1,         //< vertex is owned by this triangle (otherwise, owned by neighbour)
                                        bool mowns_vertex_2,         //< vertex is owned by this triangle (otherwise, owned by neighbour)
                                        bool mowns_vertex_3,         //< vertex is owned by this triangle (otherwise, owned by neighbour)
                                        bool mowns_edge_1,           //< edge is owned by this triangle (otherwise, owned by neighbour)
                                        bool mowns_edge_2,           //< edge is owned by this triangle (otherwise, owned by neighbour)
                                        bool mowns_edge_3,           //< edge is owned by this triangle (otherwise, owned by neighbour)
                                        double msphereswept_rad = 0  //< sphere swept triangle ('fat' triangle, improves robustness)
    )
            {
                // adjust default inward 'safe' margin (always as radius)
                this.SetSafeMargin(msphereswept_rad);


                btCEtriangleShape mshape = new btCEtriangleShape(p1, p2, p3, ep1, ep2, ep3,
                                                           mowns_vertex_1, mowns_vertex_2, mowns_vertex_3,
                                                           mowns_edge_1, mowns_edge_2, mowns_edge_3, msphereswept_rad);

                mshape.SetMargin((float)this.GetSuggestedFullMargin()); // this.GetSafeMargin());  // not this.GetSuggestedFullMargin() given the way that btCEtriangleShape works.

                _injectShape(ChVector.VNULL, new ChMatrix33<double>(1), mshape);

                return true;
            }

            /// Add a triangle mesh to this model, passing a triangle mesh.
            /// Note: if possible, for better performance, avoid triangle meshes and prefer simplified
            /// representations as compounds of primitive convex shapes (boxes, sphers, etc).
            public override bool AddTriangleMesh(                           //
                  Mesh trimesh,  //< the triangle mesh
                  bool is_static,                                     //< true if model doesn't move. May improve performance.
                  bool is_convex,                                     //< if true, a convex hull is used. May improve robustness.
                  ChVector pos,               //< displacement respect to COG
                  ChMatrix33<double> rot,          //< the rotation of the mesh
                  double sphereswept_thickness = 0.0                  //< outward sphere-swept layer (when supported)
                  )
            {

                for (int it = 0; it < trimesh.triangles.Length; it+=3)
                {
                    ChVector point1 = new ChVector(0, 0, 0);
                    ChVector point2 = new ChVector(0, 0, 0);
                    ChVector point3 = new ChVector(0, 0, 0);
                    Vector3 p1 = trimesh.vertices[trimesh.triangles[it + 0]];
                    Vector3 p2 = trimesh.vertices[trimesh.triangles[it + 1]];
                    Vector3 p3 = trimesh.vertices[trimesh.triangles[it + 2]];

                    point1.x = p1.x;
                    point1.y = p1.y;
                    point1.z = p1.z;
                    point2.x = p2.x;
                    point2.y = p2.y;
                    point2.z = p2.z;
                    point3.x = p3.x;
                    point3.y = p3.y;
                    point3.z = p3.z;

                    this.AddTriangleProxy(point1,
                                           point2,
                                           point3,
                            new ChVector(), new ChVector(), new ChVector(),
                                false, false, false, false, false, false, sphereswept_thickness);
                }
                // iterate on triangles
                /* for (int it = 0; it < trimesh.m_face_v_indices.Count; it++)
                 {

                     this.AddTriangleProxy(trimesh.m_vertices[(int)trimesh.m_face_v_indices[it].x],
                                            trimesh.m_vertices[(int)trimesh.m_face_v_indices[it].y],
                                            trimesh.m_vertices[(int)trimesh.m_face_v_indices[it].z],
                             new ChVector(), new ChVector(), new ChVector(),
                                 false, false, false, false, false, false, sphereswept_thickness);
                 }*/


                /* TriangleMesh bulletMesh = new TriangleMesh();
                 /// Store containing triangle indices for vertices
                 for (int i = 0; i < mesh.triangles.Length; i+=3)
                 {
                     ChVector point1 = new ChVector(0, 0, 0);
                     ChVector point2 = new ChVector(0, 0, 0);
                     ChVector point3 = new ChVector(0, 0, 0);
                     Vector3 p1 = mesh.vertices[mesh.triangles[i + 0]];
                     Vector3 p2 = mesh.vertices[mesh.triangles[i + 1]];
                     Vector3 p3 = mesh.vertices[mesh.triangles[i + 2]];

                     point1.x = p1.x;
                     point1.y = p1.y;
                     point1.z = p1.z;
                     point2.x = p2.x;
                     point2.y = p2.y;
                     point2.z = p2.z;
                     point3.x = p3.x;
                     point3.y = p3.y;
                     point3.z = p3.z;

                     // bulletMesh.m_weldingThreshold = ...
                     bulletMesh.AddTriangle(ChVectToBullet(point1), ChVectToBullet(point2),
                         ChVectToBullet(point3),
                         true);  // try to remove duplicate vertices
                 }

                  TriangleMesh bulletMesh = new TriangleMesh();
                  for (int i = 0; i < trimesh.triangles.Length; i++)
                  {
                      // bulletMesh.m_weldingThreshold = ...
                      bulletMesh.AddTriangle(ChVectToBullet(point1), ChVectToBullet(point2),
                          ChVectToBullet(point3),
                          false);  // try to remove duplicate vertices
                  }*/

                // CollisionShape pShape = (BvhTriangleMeshShape)new btBvhTriangleMeshShape_handlemesh(bulletMesh);
                // CollisionShape pShape = new TriangleMeshShape(bulletMesh);
                // pShape.setMargin((btScalar)this.GetSafeMargin());
                // ((btBvhTriangleMeshShape*)pShape).refitTree();
                // btCollisionShape* pShape = new btGImpactMeshShape_handlemesh(bulletMesh);
                // pShape.setMargin((btScalar) this.GetSafeMargin() );
                //((btGImpactMeshShape_handlemesh*)pShape).updateBound();
                // _injectShape(pos, rot, pShape);

                return true;
                }
            


            // These classes inherits the Bullet triangle mesh, but adds just a single feature:
            // when this shape is deleted, also the referenced triangle mesh interface is deleted.
            // Hence, when a btBvhTriangleMeshShape_handlemesh is added to the list of shapes of this ChModelBullet,
            // there's no need to remember to delete the mesh interface because it dies with the model, when shapes are deleted.
            // This is just to avoid adding a pointer to a triangle interface in all collision models, when not needed.
            public class btBvhTriangleMeshShape_handlemesh : BvhTriangleMeshShape
            {
                StridingMeshInterface minterface;


                public btBvhTriangleMeshShape_handlemesh(StridingMeshInterface meshInterface)
                : base(meshInterface, true, true)
                {
                    minterface = meshInterface;
                }
            };

            public override void SyncPosition()
            {
                ChCoordsys mcsys = this.mcontactable.GetCsysForCollisionModel();

                bt_collision_object.GetWorldTransform()._origin = new IndexedVector3(
                    (float)mcsys.pos.x, (float)mcsys.pos.y, (float)mcsys.pos.z);                               
                ChMatrix33<double> rA = new ChMatrix33<double>(mcsys.rot);             
                IndexedBasisMatrix basisA = new IndexedBasisMatrix((float)rA.nm.matrix[0, 0], (float)rA.nm.matrix[0, 1], (float)rA.nm.matrix[0, 2], (float)rA.nm.matrix[1, 0],
                                                                   (float)rA.nm.matrix[1, 1], (float)rA.nm.matrix[1, 2], (float)rA.nm.matrix[2, 0], (float)rA.nm.matrix[2, 1],
                                                                   (float)rA.nm.matrix[2, 2]);
                bt_collision_object.GetWorldTransform()._basis = basisA;
            }

            /// Returns the axis aligned bounding box (AABB) of the collision model,
            /// i.e. max-min along the x,y,z world axes. Remember that SyncPosition()
            /// should be invoked before calling this.
            /// MUST be implemented by child classes!
            public override void GetAABB(ref ChVector bbmin, ref ChVector bbmax) {
                IndexedVector3 btmin = new IndexedVector3();
                IndexedVector3 btmax = new IndexedVector3();
                if (bt_collision_object.GetCollisionShape() == null);
                    bt_collision_object.GetCollisionShape().GetAabb(bt_collision_object.GetWorldTransform(), out btmin, out btmax);
                bbmin.Set(btmin.X, btmin.Y, btmin.Z);
                bbmax.Set(btmax.X, btmax.Y, btmax.Z);
            }

            /// Deletes all inserted geometries.
            /// Also, if you begin the definition of a model, AFTER adding
            /// the geometric description, remember to call the ClearModel().
            /// MUST be inherited by child classes! (ex for resetting also BV hierarchies)
          /*  public override int ClearModel() {

            }*/

            /// Builds the BV hierarchy.
            /// Call this function AFTER adding the geometric description.
            /// MUST be inherited by child classes! (ex for building BV hierarchies)
           /* public override int BuildModel() {
               
            }
           */
            //
            // GEOMETRY DESCRIPTION
            //
            //  The following functions must be called inbetween
            //  the ClearModel() BuildModel() pair.

            /// Add a sphere shape to this model, for collision purposes
          /*  public override bool AddSphere(double radius)
            {

            }*/

            /// Add a box shape to this model, for collision purposes
          /*  public override bool AddBox(double hx,
                                double hy,
                                double hz,
                                ChVector<double> pos,
                                ChMatrix33<double> rot)
            {
                // adjust default inward margin (if object too thin)
               // this.SetSafeMargin(ChMin(this.GetSafeMargin(), 0.2 * ChMin(ChMin(hx, hy), hz)));

                float ahx = (hx + this.GetEnvelope());
                float ahy = (hy + this.GetEnvelope());
                float ahz = (hz + this.GetEnvelope());
                BBoxShape mshape = new BBoxShape();

               // mshape.setMargin((btScalar)this.GetSuggestedFullMargin());

                _injectShape(pos, rot, mshape);

                return true;
            }*/

            /// Add a cylinder to this model (default axis on Y direction), for collision purposes
           /* public override bool AddCylinder(double rx,
                                     double rz,
                                     double ry
                             //ChVector<>& pos = ChVector<>(),
                             //ChMatrix33<>& rot = ChMatrix33<>(1)) = 0;
                             )
            {

            }*/



        }
    }
}
