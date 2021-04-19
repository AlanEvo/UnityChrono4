using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
using BulletXNA.BulletCollision;
using BulletXNA.BulletDynamics;
using BulletXNA.LinearMath;
//using BulletSharp;
//using BulletSharp.Math;
//using BulletUnity;


namespace chrono
{
    namespace collision
    {
        ///
        /// Class for collision engine based on the 'Bullet' library.
        /// Contains either the broadphase and the narrow phase Bullet
        /// methods.
        ///
        public class ChCollisionSystemBullet : ChCollisionSystem
        {

            public ChCollisionSystemBullet(uint max_objects, double scene_size)
            {
                // btDefaultCollisionConstructionInfo conf_info(...); ***TODO***
                bt_collision_configuration = new DefaultCollisionConfiguration();

                bt_dispatcher = new CollisionDispatcher(bt_collision_configuration);
                //((btDefaultCollisionConfiguration*)bt_collision_configuration)->setConvexConvexMultipointIterations(4,4);

                //***OLD***
                float sscene_size = (float)scene_size;
                IndexedVector3 worldAabbMin = new IndexedVector3(-sscene_size, -sscene_size, -sscene_size);
                IndexedVector3 worldAabbMax = -worldAabbMin;//new IndexedVector3(sscene_size, sscene_size, sscene_size);
                // bt_broadphase = new bt32BitAxisSweep3(worldAabbMin,worldAabbMax, max_objects, 0, true); // true for disabling
                // raycast accelerator

                //***NEW***
                bt_broadphase = new AxisSweep3Internal(ref worldAabbMin, ref worldAabbMax, 0xfffe, 0xffff, 16384, null, false);
                //bt_broadphase = new DbvtBroadphase();

                bt_collision_world = new CollisionWorld(bt_dispatcher, bt_broadphase, bt_collision_configuration);

                // TO DO!!!

                // custom collision for 2D arc-segment case
                /* CollisionAlgorithmCreateFunc m_collision_arc_seg = new ArcSegmentCollisionAlgorithm.CreateFunc;
                 btCollisionAlgorithmCreateFunc* m_collision_seg_arc = new btArcSegmentCollisionAlgorithm::CreateFunc;
                 m_collision_seg_arc->m_swapped = true;
                 bt_dispatcher->registerCollisionCreateFunc(ARC_SHAPE_PROXYTYPE, SEGMENT_SHAPE_PROXYTYPE, m_collision_arc_seg);
                 bt_dispatcher->registerCollisionCreateFunc(SEGMENT_SHAPE_PROXYTYPE, ARC_SHAPE_PROXYTYPE, m_collision_seg_arc);

                 // custom collision for 2D arc-arc case
                 btCollisionAlgorithmCreateFunc* m_collision_arc_arc = new btArcArcCollisionAlgorithm::CreateFunc;
                 bt_dispatcher->registerCollisionCreateFunc(ARC_SHAPE_PROXYTYPE, ARC_SHAPE_PROXYTYPE, m_collision_arc_arc);*/

                // custom collision for C::E triangles:
                /* btCollisionAlgorithmCreateFunc* m_collision_cetri_cetri = new btCEtriangleShapeCollisionAlgorithm::CreateFunc;
                 bt_dispatcher->registerCollisionCreateFunc(CE_TRIANGLE_SHAPE_PROXYTYPE, CE_TRIANGLE_SHAPE_PROXYTYPE, m_collision_cetri_cetri);

                 // custom collision for point-point case (in point clouds, just never create point-point contacts)
                 //btCollisionAlgorithmCreateFunc* m_collision_point_point = new btPointPointCollisionAlgorithm::CreateFunc;
                 void* mem = btAlignedAlloc(sizeof(btEmptyAlgorithm::CreateFunc), 16);
                 btCollisionAlgorithmCreateFunc* m_emptyCreateFunc = new (mem) btEmptyAlgorithm::CreateFunc;
                 bt_dispatcher->registerCollisionCreateFunc(POINT_SHAPE_PROXYTYPE, POINT_SHAPE_PROXYTYPE, m_emptyCreateFunc);
                 bt_dispatcher->registerCollisionCreateFunc(POINT_SHAPE_PROXYTYPE, BOX_SHAPE_PROXYTYPE, bt_collision_configuration->getCollisionAlgorithmCreateFunc(SPHERE_SHAPE_PROXYTYPE, BOX_SHAPE_PROXYTYPE)); // just for speedup
                 bt_dispatcher->registerCollisionCreateFunc(BOX_SHAPE_PROXYTYPE, POINT_SHAPE_PROXYTYPE, bt_collision_configuration->getCollisionAlgorithmCreateFunc(BOX_SHAPE_PROXYTYPE, SPHERE_SHAPE_PROXYTYPE)); // just for speedup

                 // custom collision for GIMPACT mesh case too
                 btGImpactCollisionAlgorithm::registerAlgorithm(bt_dispatcher);*/

            }

            /// Clears all data instanced by this algorithm
            /// if any (like persistent contact manifolds)
            public override void Clear(object o)
            {
                int numManifolds = bt_collision_world.GetDispatcher().GetNumManifolds();
                for (int i = 0; i < numManifolds; i++)
                {
                    PersistentManifold contactManifold = bt_collision_world.GetDispatcher().GetManifoldByIndexInternal(i);
                    contactManifold.ClearManifold();
                }
            }

            /// Adds a collision model to the collision
            /// engine (custom data may be allocated).
            public override void Add(ChCollisionModel model)
            {
                if (((ChModelBullet)model).GetBulletModel().GetCollisionShape() != null)
                {
                    model.SyncPosition();
                    bt_collision_world.AddCollisionObject(((ChModelBullet)model).GetBulletModel(),
                                                           (CollisionFilterGroups)((ChModelBullet)model).GetFamilyGroup(),
                                                           (CollisionFilterGroups)((ChModelBullet)model).GetFamilyMask());
                }
            }

            /// Removes a collision model from the collision
            /// engine (custom data may be deallocated).
            public override void Remove(ChCollisionModel model)
            {
                if (((ChModelBullet)model).GetBulletModel().GetCollisionShape() != null)
                {
                    bt_collision_world.RemoveCollisionObject(((ChModelBullet)model).GetBulletModel());
                }
            }

            /// Removes all collision models from the collision
            /// engine (custom data may be deallocated).
            // virtual void RemoveAll() = 0;

            /// Run the collision detection and finds the contacts.
            /// This function will be called at each simulation step.
            public override void Run()
            {
                if (bt_collision_world != null)
                {
                    bt_collision_world.PerformDiscreteCollisionDetection();
                }
            }

            /// Return the time (in seconds) for broadphase collision detection.
            public override double GetTimerCollisionBroad()
            {
                return 1;// bt_collision_world.timer_collision_broad();
            }

            /// Return the time (in seconds) for narrowphase collision detection.
            public override double GetTimerCollisionNarrow()
            {
                return 1;// bt_collision_world.timer_collision_narrow();
            }

            public override void ReportContacts(ChContactContainer mcontactcontainer)
            {
                // This should remove all old contacts (or at least rewind the index)
                mcontactcontainer.BeginAddContact();

                // NOTE: Bullet does not provide information on radius of curvature at a contact point.
                // As such, for all Bullet-identified contacts, the default value will be used (SMC only). 
                ChCollisionInfo icontact = new ChCollisionInfo();

                int numManifolds = bt_collision_world.GetDispatcher().GetNumManifolds();
                for (int i = 0; i < numManifolds; i++)
                {
                    PersistentManifold contactManifold = bt_collision_world.GetDispatcher().GetManifoldByIndexInternal(i);
                    CollisionObject obA = (CollisionObject)(contactManifold.GetBody0());
                    CollisionObject obB = (CollisionObject)(contactManifold.GetBody1());

                    if (obA != null && obA != null) // Alan
                    {
                        contactManifold.RefreshContactPoints(ref obA.GetWorldTransform(), ref obB.GetWorldTransform());

                        icontact.modelA = (ChCollisionModel)obA.GetUserPointer();
                        icontact.modelB = (ChCollisionModel)obB.GetUserPointer();

                        double envelopeA = icontact.modelA.GetEnvelope();
                        double envelopeB = icontact.modelB.GetEnvelope();

                        double marginA = icontact.modelA.GetSafeMargin();
                        double marginB = icontact.modelB.GetSafeMargin();

                        // Execute custom broadphase callback, if any
                        bool do_narrow_contactgeneration = true;
                        if (this.broad_callback != null)
                            do_narrow_contactgeneration = this.broad_callback.OnBroadphase(icontact.modelA, icontact.modelB);

                        if (do_narrow_contactgeneration)
                        {
                            int numContacts = contactManifold.GetNumContacts();
                            //GetLog() << "numContacts=" << numContacts << "\n";
                            for (int j = 0; j < numContacts; j++)
                            {
                                // Debug.Log("contacts " + numContacts);
                                ManifoldPoint pt = contactManifold.GetContactPoint(j);

                                // Discard "too far" constraints (the Bullet engine also has its threshold)
                                if (pt.GetDistance() < marginA + marginB)
                                {
                                    IndexedVector3 ptA = pt.GetPositionWorldOnA();
                                    IndexedVector3 ptB = pt.GetPositionWorldOnB();

                                    icontact.vpA.Set(ptA.X, ptA.Y, ptA.Z);
                                    icontact.vpB.Set(ptB.X, ptB.Y, ptB.Z);

                                    icontact.vN.Set(-pt.GetNormalWorldOnB().X, -pt.GetNormalWorldOnB().Y,
                                                    -pt.GetNormalWorldOnB().Z);
                                    icontact.vN.Normalize();

                                    double ptdist = pt.GetDistance();

                                    icontact.vpA = icontact.vpA - icontact.vN * envelopeA;
                                    icontact.vpB = icontact.vpB + icontact.vN * envelopeB;
                                    icontact.distance = ptdist + envelopeA + envelopeB;

                                    icontact.reaction_cache = pt.reactions_cache;// reactions_cache;

                                    // Execute some user custom callback, if any
                                    bool add_contact = true;
                                    if (this.narrow_callback != null)
                                        add_contact = this.narrow_callback.OnNarrowphase(icontact);

                                    // Add to contact container
                                    if (add_contact)
                                        mcontactcontainer.AddContact(icontact);
                                }
                            }
                        }
                    }

                    // you can un-comment out this line, and then all points are removed
                    // contactManifold->clearManifold();
                }
                mcontactcontainer.EndAddContact();
            }

            public override void ReportProximities(ChProximityContainer mproximitycontainer)
            {
                mproximitycontainer.BeginAddProximities();
                /*
                int numManifolds = bt_collision_world->getDispatcher()->getNumManifolds(); 
                for (int i = 0; i < numManifolds; i++) {
                    btPersistentManifold* contactManifold = bt_collision_world->getDispatcher()->getManifoldByIndexInternal(i);
                    btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->getBody0());
                    btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->getBody1());
                    contactManifold->refreshContactPoints(obA->getWorldTransform(), obB->getWorldTransform());

                    ChCollisionModel* modelA = (ChCollisionModel*)obA->getUserPointer();
                    ChCollisionModel* modelB = (ChCollisionModel*)obB->getUserPointer();

                    // Add to proximity container
                    mproximitycontainer->AddProximity(modelA, modelB);
                }
                */
                int numPairs = bt_collision_world.GetBroadphase().GetOverlappingPairCache().GetNumOverlappingPairs();
                for (int i = 0; i < numPairs; i++)
                {
                    BroadphasePair mp = bt_collision_world.GetBroadphase().GetOverlappingPairCache().GetOverlappingPairArray()[i];

                    CollisionObject obA = (CollisionObject)(mp.m_pProxy0.GetClientObject());
                    CollisionObject obB = (CollisionObject)(mp.m_pProxy1.GetClientObject());

                    ChCollisionModel modelA = (ChCollisionModel)obA.GetUserPointer();
                    ChCollisionModel modelB = (ChCollisionModel)obB.GetUserPointer();

                    // Add to proximity container
                    mproximitycontainer.AddProximity(modelA, modelB);
                }
                mproximitycontainer.EndAddProximities();
            }


            /// Perform a ray-hit test with the collision models.
            public override bool RayHit(ChVector from, ChVector to, ref ChRayhitResult mresult) { return false; }

            /// Perform a ray-hit test with the specified collision model.
            public override bool RayHit(ChVector from,
                        ChVector to,

                        ChCollisionModel model,
                        ref ChRayhitResult mresult)
            {
                return false;
            }


            /*  private ICollisionConfiguration bt_collision_configuration;
              private CollisionDispatcher bt_dispatcher;
              private IBroadphaseInterface bt_broadphase;
              public CollisionWorld bt_collision_world;*/
            private ICollisionConfiguration bt_collision_configuration;
            private CollisionDispatcher bt_dispatcher;
            private IBroadphaseInterface bt_broadphase;
            public CollisionWorld bt_collision_world;

        }
    }
}
