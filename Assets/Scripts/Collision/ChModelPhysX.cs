using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono
{
    namespace collision
    {

      /*  public class ChModelPhysX : ChCollisionModel
        {
            // The Bullet collision object containing Bullet geometries
            protected Collider physx_collision_object;

            // Vector of shared pointers to geometric objects.
            protected List<Collider> shapes = new List<Collider>();

            private void _injectShape(ChVector pos, ChMatrix33<double> rot, Collider mshape)
            {
                bool centered = true;// (pos.IsNull() && rot.IsIdentity());  // FIX THIS !!!

                // start_vector = ||    -- description is still empty
                if (shapes.Count == 0)
                {
                    if (centered)
                    {
                        shapes.Add(mshape);
                        physx_collision_object = mshape;
                        // end_vector=  | centered shape |
                        return;
                    }
                    else
                    {
                       /* CompoundShape mcompound = new CompoundShape();
                        shapes.Add(mcompound);
                        shapes.Add(mshape);
                        bt_collision_object.SetCollisionShape(mcompound);
                        IndexedMatrix mtransform = new IndexedMatrix();
                        ChPosMatrToBullet(pos, rot, ref mtransform);
                        mcompound.AddChildShape(ref mtransform, mshape);*/
                        // vector=  | compound | not centered shape |
              /*          return;
                    }
                }
            }

            /// Add a box shape to this model, for collision purposes
            public bool AddBox(
                    double hx,                                 /// the halfsize on x axis
                    double hy,                                 /// the halfsize on y axis
                    double hz,                                  /// the halfsize on z axis
                    ChVector pos,      /// the position of the box COG
                    ChMatrix33<double> rot  /// the rotation of the box - matrix must be orthogonal
                    )
            {
                // adjust default inward margin (if object too thin)
               // this.SetSafeMargin(ChMaths.ChMin(this.GetSafeMargin(), 0.2 * ChMaths.ChMin(ChMaths.ChMin(hx, hy), hz)));

               /* float ahx = (float)(hx + this.GetEnvelope());
                float ahy = (float)(hy + this.GetEnvelope());
                float ahz = (float)(hz + this.GetEnvelope());*/
               // BoxCollider mshape = gameObject.AddComponent<BoxCollider>();
              /*  mshape.size = new Vector3((float)hx, (float)hy, (float)hz);​

               // mshape.SetMargin((float)this.GetSuggestedFullMargin());

                _injectShape(pos, rot, mshape);

                return true;
            }
        }*/
    }
}
