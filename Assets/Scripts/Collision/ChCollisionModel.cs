using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
using BulletXNA.LinearMath;
//using BulletSharp;
//using BulletSharp.Math;
//using BulletUnity;

namespace chrono
{
    namespace collision
    {
        /// Shape types that can be created.
        public enum ShapeType
        {
            SPHERE,
            ELLIPSOID,
            BOX,
            CYLINDER,
            CONVEXHULL,
            TRIANGLEMESH,
            BARREL,
            CAPSULE,      // Currently implemented in parallel only
            CONE,         // Currently implemented in parallel only
            ROUNDEDBOX,   // Currently implemented in parallel only
            ROUNDEDCYL,   // Currently implemented in parallel only
            ROUNDEDCONE,  // Currently implemented in parallel only
            CONVEX,       // Currently implemented in parallel only
            TETRAHEDRON   // Currently implemented in parallel only
        };

        /// Class containing the geometric model ready for collision detection.
        /// Each rigid body will have a ChCollisionModel.
        /// A ChCollisionModel will contain all the geometric description(s)
        /// of the shape of the rigid body, for collision purposes.
        public abstract class ChCollisionModel
        {
            private static double default_model_envelope = 0.03;
            private static double default_safe_margin = 0.01;

            public ChCollisionModel()
            {
                family_group = 1;
                family_mask = 0x7FFF;
                mcontactable = null;
                model_envelope = (float)default_model_envelope;
                model_safe_margin = (float)default_safe_margin;
            }


            /// Deletes all inserted geometries.
            /// Also, if you begin the definition of a model, AFTER adding
            /// the geometric description, remember to call the ClearModel().
            /// MUST be inherited by child classes! (ex for resetting also BV hierarchies)
            public abstract int ClearModel();

            /// Builds the BV hierarchy.
            /// Call this function AFTER adding the geometric description.
            /// MUST be inherited by child classes! (ex for building BV hierarchies)
            public abstract int BuildModel();

            //
            // GEOMETRY DESCRIPTION
            //
            //  The following functions must be called inbetween
            //  the ClearModel() BuildModel() pair.
            //  The class must implement automatic deletion of the created
            //  geometries at class destruction time and at ClearModel()
            //  Return value is true if the child class implements the
            //  corresponding type of geometry.

            /// Add a sphere shape to this model, for collision purposes
            public abstract bool AddSphere(double radius,                           //< the radius of the sphere
                                           ChVector pos                //< the position of the sphere in model coordinates
                           );

            /// Add a box shape to this model, for collision purposes
            public abstract bool AddBox(
                double hx,                                 /// the halfsize on x axis
                double hy,                                 /// the halfsize on y axis
                double hz,                                  /// the halfsize on z axis
                ChVector pos,      /// the position of the box COG
                ChMatrix33<double> rot  /// the rotation of the box - matrix must be orthogonal
                );

            /// Add a cylinder to this model (default axis on Y direction), for collision purposes
            public abstract bool AddCylinder(double rx,
                                     double rz,
                                     double hy,
                                     ChVector pos,
                                     ChMatrix33<double>rot);

            // TOLERANCES, ENVELOPES, THRESHOLDS

            /// Sets the suggested collision 'inward safe margin' for the
            /// shapes to be added from now on, using the AddBox,
            /// AddCylinder etc (where, if this margin is too high for some
            /// thin or small shapes, it may be clamped).
            /// If dist\<0 and interpretation occurs (ex.for numerical errors) within
            /// this 'safe margin' inward range, collision detection is still fast
            /// and reliable (beyond this, for deep penetrations, CD still works,
            /// but might be slower and less reliable)
            /// Call this BEFORE adding the shapes into the model.
            /// Side effect: think at the margin as a radius of a 'smoothing' fillet
            /// on all corners of the shapes - that's why you cannot exceed with this.
            public virtual void SetSafeMargin(double amargin) { model_safe_margin = (float)amargin; }
            /// Returns the inward safe margin (see SetSafeMargin() )
            public virtual float GetSafeMargin() { return model_safe_margin; }

            /// Sets the suggested collision outward 'envelope' (used from shapes
            /// added, from now on, to this collision model).  This 'envelope' is a
            /// surrounding invisible volume which extends outward from the
            /// surface, and it is used to detect contacts a bit before shapes
            /// come into contact, i.e. when dist\>0. However contact points will stay
            /// on the true surface of the geometry, not on the external surface of the
            /// envelope.
            /// Call this BEFORE adding the shapes into the model.
            /// Side effect: AABB are 'expanded' outward by this amount, so if you
            /// exaggerate with this value, CD might be slower and too sensible.
            /// On the other hand, if you set this value to 0, contacts are detected
            /// only for dist<=0, thus causing unstable simulation.
            public virtual void SetEnvelope(double amargin) { model_envelope = (float)amargin; }
            /// Returns the outward safe margin (see SetEnvelope() )
            public virtual float GetEnvelope() { return model_envelope; }

            /// Add a cylinder to this model (default axis on Y direction), for collision purposes
            /*public abstract bool AddCylinder(double rx,
                                     double rz,
                                     double hy
                             //ChVector<>& pos = ChVector<>(),
                             //ChMatrix33<>& rot = ChMatrix33<>(1)) = 0;
                             );*/

            /// Add a triangle mesh to this model, passing a triangle mesh.
            /// Note: if possible, for better performance, avoid triangle meshes and prefer simplified
            /// representations as compounds of primitive convex shapes (boxes, sphers, etc).
           /* public abstract bool AddTriangleMesh(                           //
                geometry.ChTriangleMesh trimesh,  //< the triangle mesh
                bool is_static,                                     //< true if model doesn't move. May improve performance.
                bool is_convex,                                     //< if true, a convex hull is used. May improve robustness.
                ChVector<double> pos,               //< displacement respect to COG
                ChMatrix33<double> rot,          //< the rotation of the mesh
                double sphereswept_thickness                  //< outward sphere-swept layer (when supported)
                );*/

            /// Sets the position and orientation of the collision
            /// model as the rigid body current position.
            /// By default it uses GetCsysForCollisionModel
            public abstract void SyncPosition();

            /// Returns the axis aligned bounding box (AABB) of the collision model,
            /// i.e. max-min along the x,y,z world axes. Remember that SyncPosition()
            /// should be invoked before calling this.
            /// MUST be implemented by child classes!
            public abstract void GetAABB(ref ChVector bbmin, ref ChVector bbmax);

            /// Gets the pointer to the contactable object
            public ChContactable GetContactable() { return mcontactable; }

            /// Sets the pointer to the contactable object.
            /// A derived class may override this, but should always invoke this base class implementation.
             public virtual void SetContactable(ChContactable mc) { mcontactable = mc; }

            /// Using this function BEFORE you start creating collision shapes,
            /// it will make all following collision shapes to take this collision
            /// envelope (safe outward layer) as default.
            /// Easier than calling SetEnvelope() all the times.
            public static void SetDefaultSuggestedEnvelope(double menv) {
                default_model_envelope = menv;
            }


            protected virtual float GetSuggestedFullMargin() { return model_envelope + model_safe_margin; }

            // Maximum envelope: surrounding volume from surface to the exterior
            protected float model_envelope;

            // This is the max.value to be used for fast penetration contact detection.
            protected float model_safe_margin;

            // Pointer to the contactable object
            protected ChContactable mcontactable;

            // Collision family group and mask
            protected int family_group;
            protected int family_mask;

            public IndexedVector3 collisionPosition;
        }
    }
}
