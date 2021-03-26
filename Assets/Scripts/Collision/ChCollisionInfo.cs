using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
using UnityEngine.Serialization;
using System.Runtime.InteropServices;

namespace chrono
{
    namespace collision
    {

        /// @addtogroup chrono_collision
        /// @{

        ///   Class for passing basic data about contact pairs
        public class ChCollisionInfo
        {
            private static double default_eff_radius = 0.1;

            public ChCollisionModel modelA;  //< model A
            public ChCollisionModel modelB;  //< model B
            public ChVector vpA;            //< coll.point on A, in abs coords
            public ChVector vpB;            //< coll.point on B, in abs coords
            public ChVector vN;             //< coll.normal, respect to A, in abs coords
            public double distance;           //< distance (negative for penetration)
            public double eff_radius;         //< effective radius of curvature at contact (SMC only)
            public float[] reaction_cache;     //< pointer to some persistent user cache of reactions


            /// Basic default constructor.
            public ChCollisionInfo() {
                modelA = null;
                modelB = null;
                vpA = new ChVector();
                vpB = new ChVector();
                vN = new ChVector(1, 0, 0);
                distance = 0;
                eff_radius = default_eff_radius;
                reaction_cache = null;
            }

            /// Copy from other.
            public ChCollisionInfo(ChCollisionInfo other, bool swap = false) {
                if (!swap)
                {
                    modelA = other.modelA;
                    modelB = other.modelB;
                    vpA = other.vpA;
                    vpB = other.vpB;
                    vN = other.vN;
                }
                else
                {
                    // copy by swapping models
                    modelA = other.modelB;
                    modelB = other.modelA;
                    vpA = other.vpB;
                    vpB = other.vpA;
                    vN = -other.vN;
                }
                distance = other.distance;
                eff_radius = other.eff_radius;
                reaction_cache = other.reaction_cache;
            }

        }
    }
}
