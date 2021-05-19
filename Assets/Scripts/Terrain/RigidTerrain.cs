using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;


namespace chrono
{
    namespace vehicle
    {
        /// Class to be used as a functor interface for location-dependent coefficient of friction.
        public class FrictionFunctor
        {

        }

        public class RigidTerrain : MonoBehaviour
        {
            public enum Type { BOX, MESH, HEIGHT_MAP };
            public Type type = Type.BOX;

            [SerializeField]
            public List<ChBody> patches = new List<ChBody>(); // change to array for multiple terrain patches.

            public bool tiled;
            public double max_tile_size;

            // TEST
            collision.ChCollisionSystem.ChRayhitResult result = new collision.ChCollisionSystem.ChRayhitResult();

            // Use this for initialization
            void Start()
            {
               // m_system = GameObject.FindObjectOfType<ChSystem>();

                switch (type)
                {
                    case Type.BOX:
                        // patch = AddPatch();

                        break;
                    case Type.MESH:

                        break;
                }
            }

            public virtual float GetCoefficientFriction(double x, double y)
            {
                // TO DO, this needs to be implemented for localised friction changes on the terrain.
                //if (m_friction_fun != null) {
                //return (m_friction_fun)(x, y); }

                // double height = 0;
                // ChVector normal = new ChVector();
                float friction = 0;

                // bool hit = FindPoint(x, y, out height, out normal, friction);

                return friction;
            }

            public bool FindPoint(double x, double y, out double height, ref ChVector normal, float friction)
            {
                bool hit = false;
                height = -1000;
                normal = new ChVector(0, 1, 0);
                friction = 0.8f;

                ChVector from = new ChVector(x, 1000, y);
                ChVector to = new ChVector(x, -1000, y);

                for(int i = 0; i < patches.Count; i++)
                {
                   // ChBody body = patch.GetComponent<ChBody>();
                    //collision.ChCollisionSystem.ChRayhitResult result = new collision.ChCollisionSystem.ChRayhitResult();
                    ChSystem.system.GetCollisionSystem().RayHit(from, to, patches[i].GetCollisionModel(), ref result);
                    if (result.hit && result.abs_hitPoint.y > height)
                    {
                        hit = true;
                        height = result.abs_hitPoint.y;
                        normal = result.abs_hitNormal;
                        //friction = patch.m_friction;
                    }
                }

                return hit;
            }

            protected virtual void OnDrawGizmos()
            {

            }
            public virtual double GetHeight(double x, double y)
            {
                double height = 0;
                ChVector normal = new ChVector();
                float friction = 0;

                //bool hit = false; 
                bool hit = FindPoint(x, y, out height, ref normal, friction);

                return hit ? height : 0.0;
            }

            public virtual ChVector GetTest()
            {
                ChVector normal = new ChVector(300, 343, 654);
                return normal;
            }

            public virtual ChVector GetNormal(double x, double y)
            {
                double height = 0;
                ChVector normal = new ChVector();
                float friction = 0;

                //bool hit = false;
                bool hit = FindPoint(x, y, out height, ref normal, friction);
                if (hit)
                    return normal;
                else
                    return new ChVector();// null;
            }

            /// Add a terrain patch represented by a rigid box.
            /// If tiled = true, multiple side-by-side boxes are used.
            /*public Patch AddPatch(
                           ChCoordsys position,  //< [in] box location and orientation
                           ChVector size,        //< [in] box dimensions (x, y, z)
                           bool tiled = false,            //< [in] terrain created from multiple tiled boxes
                           double max_tile_size = 1      //< [in] maximum tile size
                           )
            {

            }*/

            // Update is called once per frame
            void Update()
            {

            }

            protected FrictionFunctor m_friction_fun;  //< functor for location-dependent coefficient of friction

            /// <summary>
            /// Ground entities - each represents a single ground surface.
            /// </summary>
          //  [Tooltip("Ground entities - each represents a single ground surface.")]
          //  [SerializeField]
          //  public List<GroundDetection.GroundEntity> groundEntities = new List<GroundDetection.GroundEntity>();

        }
    }
}
