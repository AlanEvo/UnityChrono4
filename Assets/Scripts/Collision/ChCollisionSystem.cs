using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
using BulletXNA.BulletCollision;
using BulletXNA.BulletDynamics;
using BulletXNA.LinearMath;

namespace chrono
{
    namespace collision
    {


        /// Base class for generic collision engine.
        /// Most methods are 'pure virtual': they need to be implemented by derived classes.
        public abstract class ChCollisionSystem
        {

            public ChCollisionSystem(uint max_objects = 16000, double scene_size = 500)
            {
                narrow_callback = null;
                broad_callback = null;
            }


            /// Clears all data instanced by this algorithm
            /// if any (like persistent contact manifolds)
            public abstract void Clear(object o);

            /// Adds a collision model to the collision
            /// engine (custom data may be allocated).
            public abstract void Add(ChCollisionModel model);

            /// Removes a collision model from the collision
            /// engine (custom data may be deallocated).
            public abstract void Remove(ChCollisionModel model);

            /// Removes all collision models from the collision
            /// engine (custom data may be deallocated).
            // virtual void RemoveAll() = 0;

            /// Run the collision detection and finds the contacts.
            /// This function will be called at each simulation step.
            public abstract void Run();

            /// Return the time (in seconds) for broadphase collision detection.
            public abstract double GetTimerCollisionBroad();

            /// Return the time (in seconds) for narrowphase collision detection.
            public abstract double GetTimerCollisionNarrow();

            /// Reset any timers associated with collision detection.
            public virtual void ResetTimers() { }

            /// After the Run() has completed, you can call this function to
            /// fill a 'contact container', that is an object inherited from class
            /// ChContactContainer. For instance ChSystem, after each Run()
            /// collision detection, calls this method multiple times for all contact containers in the system,
            /// Children classes _must_ implement this.
            /// The basic behavior of the implementation should be the following: collision system
            /// will call in sequence the functions BeginAddContact(), AddContact() (x n times),
            /// EndAddContact() of the contact container.
            /// In case a specialized implementation (ex. a GPU parallel collision engine)
            /// finds that the contact container is a specialized one (ex with a GPU buffer)
            /// it can call more performant methods to add directly the contacts in batches, for instance
            /// by recognizing that he can call, say, some special AddAllContactsAsGpuBuffer() instead of many AddContact().
            public abstract void ReportContacts(ChContactContainer mcontactcontainer);

            /// After the Run() has completed, you can call this function to
            /// fill a 'proximity container' (container of narrow phase pairs), that is
            /// an object inherited from class ChProximityContainer. For instance ChSystem, after each Run()
            /// collision detection, calls this method multiple times for all proximity containers in the system,
            /// Children classes _must_ implement this.
            /// The basic behavior of the implementation should be the following: collision system
            /// will call in sequence the functions BeginAddProximities(), AddProximity() (x n times),
            /// EndAddProximities() of the proximity container.
            /// In case a specialized implementation (ex. a GPU parallel collision engine)
            /// finds that the proximity container is a specialized one (ex with a GPU buffer)
            /// it can call more performant methods to add directly the proximities in batches, for instance
            /// by recognizing that he can call, say, some special AddAllProximitiesAsGpuBuffer() instead of many
            /// AddProximity().
            public abstract void ReportProximities(ChProximityContainer mproximitycontainer);

            /// Class to be used as a callback interface for user-defined actions to be performed
            /// for each 'near enough' pair of collision shapes found by the broad-phase collision step.
            public abstract class BroadphaseCallback {


                /// Callback used to process 'near enough' pairs of collision models found by the
                /// broad-phase collision algorithm.
                /// Return false to skip narrow-phase contact generation for this pair of bodies.
                public abstract bool OnBroadphase(ChCollisionModel modelA,  ///< 1st model
                                  ChCollisionModel modelB   ///< 2nd model
                                  );
            };

            /// Specify a callback object to be used each time a pair of 'near enough' collision shapes
            /// is found by the broad-phase collision step. The OnBroadphase() method of the provided
            /// callback object will be called for each pair of 'near enough' shapes.
            public void RegisterBroadphaseCallback(BroadphaseCallback callback) { broad_callback = callback; }

            /// Class to be used as a callback interface for user-defined actions to be performed
            /// at each collision pair found during the narrow-phase collision step.
            /// It can be used to override the geometric information.
            public abstract class NarrowphaseCallback {


                /// Callback used to process collision pairs found by the narrow-phase collision step.
                /// Return true to generate a contact for this pair of overlapping bodies.
                public abstract bool OnNarrowphase(ChCollisionInfo contactinfo);
            };

            /// Specify a callback object to be used each time a collision pair is found during
            /// the narrow-phase collision detection step. The OnNarrowphase() method of the provided
            /// callback object will be called for each collision pair found during narrow phase.
            public void RegisterNarrowphaseCallback(NarrowphaseCallback callback) { narrow_callback = callback; }

            /// Recover results from RayHit() raycasting.
            public struct ChRayhitResult
            {
                public bool hit;                    //< if true, there was an hit
                public ChVector abs_hitPoint;     //< hit point in absolute space coordinates
                public ChVector abs_hitNormal;    //< normal to surface in absolute space coordinates
                public double dist_factor;          //< from 0 .. 1 means the distance of hit point along the segment
                public ChCollisionModel hitModel;  //< pointer to intersected model
            };

            /// Perform a ray-hit test with the collision models.
            public abstract bool RayHit(ChVector from, ChVector to, ChCollisionModel model, ref ChRayhitResult mresult);

            /// Perform a ray-hit test with the specified collision model.
            public abstract bool RayHit(ChVector from,
                        ChVector to,
                        ChCollisionModel model,
                        ref ChRayhitResult mresult,
                        CollisionFilterGroups filter_group,
                        CollisionFilterGroups filter_mask);

            public int debug;
            protected BroadphaseCallback broad_callback;    //< user callback for each near-enough pair of shapes
            protected NarrowphaseCallback narrow_callback;  //< user callback for each collision pair
        }
    }
}
