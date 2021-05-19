using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

namespace chrono
{


    public class ChContactTuple<Ta, Tb>
    {
       // public class typecarr_a : ChContactable_1vars<Ta>.type_variable_tuple_carrier { }
       // public class typecarr_b : ChContactable_1vars<Tb>.type_variable_tuple_carrier { }


        protected ChContactContainer container;  //< associated contact container

        protected Ta objA;  //< first ChContactable object in the pair
        protected Tb objB;  //< second ChContactable object in the pair

        protected ChVector p1;      //< max penetration point on geo1, after refining, in abs space
        protected ChVector p2;      //< max penetration point on geo2, after refining, in abs space
        protected ChVector normal;  //< normal, on surface of master reference (geo1)

        protected ChMatrix33<double> contact_plane = new ChMatrix33<double>(0);  //< the plane of contact (X is normal direction)

        protected double norm_dist;   //< penetration distance (negative if going inside) after refining
        protected double eff_radius;  //< effective radius of curvature at contact


        //
        // CONSTRUCTORS
        //

        public ChContactTuple() { }

        public ChContactTuple(ChContactContainer mcontainer,          //< contact container
                   Ta mobjA,                               //< ChContactable object A
                   Tb mobjB,                               //< ChContactable object B
                   collision.ChCollisionInfo cinfo  //< data for the contact pair
    )
        {
          //  Debug.Assert(mcontainer != null);
           // Debug.Assert(mobjA);
           // Debug.Assert(mobjB);

            container = mcontainer;

            Reset(mobjA, mobjB, cinfo);
        }

        //
        // FUNCTIONS
        //

        /// Initialize again this constraint.
        public virtual void Reset(Ta mobjA,                               //< ChContactable object A
                       Tb mobjB,                               //< ChContactable object B
                       collision.ChCollisionInfo cinfo  //< data for the contact pair
    ) 
        {
            //Debug.Assert(mobjA);
            //Debug.Assert(mobjB);

            this.objA = mobjA;
            this.objB = mobjB;

            this.p1 = cinfo.vpA;
            this.p2 = cinfo.vpB;
            this.normal = cinfo.vN;
            this.norm_dist = cinfo.distance;
            this.eff_radius = cinfo.eff_radius;

            // Contact plane
            ChVector Vx = new ChVector(0, 0, 0);
            ChVector Vy = new ChVector(0, 0, 0);
            ChVector Vz = new ChVector(0, 0, 0);
            ChVector.XdirToDxDyDz(normal, ChVector.VECT_Y, ref Vx, ref Vy, ref Vz);
            contact_plane.Set_A_axis(Vx, Vy, Vz);
        }

        /// Get the colliding object A, with point P1
        public Ta GetObjA() { return this.objA; }

        /// Get the colliding object B, with point P2
        public Tb GetObjB() { return this.objB; }

        /// Get the contact coordinate system, expressed in absolute frame.
        /// This represents the 'main' reference of the link: reaction forces
        /// are expressed in this coordinate system. Its origin is point P2.
        /// (It is the coordinate system of the contact plane and normal)
        public ChCoordsys GetContactCoords()
        {
            ChCoordsys mcsys = ChCoordsys.CSYSNULL;
            ChQuaternion mrot = this.contact_plane.Get_A_quaternion();
            mcsys.rot.Set(mrot.e0, mrot.e1, mrot.e2, mrot.e3);
            mcsys.pos = this.p2;
            return mcsys;
        }

        /// Returns the pointer to a contained 3x3 matrix representing the UV and normal
        /// directions of the contact. In detail, the X versor (the 1s column of the
        /// matrix) represents the direction of the contact normal.
        public ChMatrix33<double> GetContactPlane() { return contact_plane; }

        /// Get the contact point 1, in absolute coordinates
        public ChVector GetContactP1() { return p1; }

        /// Get the contact point 2, in absolute coordinates
        public ChVector GetContactP2() { return p2; }

        /// Get the contact normal, in absolute coordinates
        public ChVector GetContactNormal() { return normal; }

        /// Get the contact distance
        public double GetContactDistance() { return norm_dist; }

        /// Get the effective radius of curvature.
        public double GetEffectiveCurvatureRadius() { return eff_radius; }

        /// Get the contact force, if computed, in contact coordinate system
        public virtual ChVector GetContactForce() { return new ChVector(0); }

        //
        // UPDATING FUNCTIONS
        //

        public virtual void ContIntStateGatherReactions(int off_L, ref ChVectorDynamic<double> L) { }

        public virtual void ContIntStateScatterReactions(int off_L, ChVectorDynamic<double> L) { }

        public virtual void ContIntLoadResidual_CqL(int off_L,    //< offset in L multipliers
                                         ref ChVectorDynamic<double> R,        //< result: the R residual, R += c*Cq'*L
                                         ChVectorDynamic<double> L,  //< the L vector
                                         double c               //< a scaling factor
    )
        { }

        public virtual void ContIntLoadConstraint_C(int off_L,  //< offset in Qc residual
                                         ref ChVectorDynamic<double> Qc,     //< result: the Qc residual, Qc += c*C
                                         double c,            //< a scaling factor
                                         bool do_clamp,             //< apply clamping to c*C?
                                         double recovery_clamp      ///< value for min/max clamping of c*C
    )
        { }

        public virtual void ContIntLoadResidual_F(ref ChVectorDynamic<double> R, double c) { }

        public virtual void ContInjectKRMmatrices(ref ChSystemDescriptor mdescriptor) { }

        public virtual void ContKRMmatricesLoad(double Kfactor, double Rfactor) { }

        public virtual void ContIntToDescriptor(int off_L,    //< offset in L, Qc
                                     ChVectorDynamic<double> L,  //< the L vector
                                     ChVectorDynamic<double> Qc  //< the Qc vector
    )
        { }

        public virtual void ContIntFromDescriptor(int off_L,  //< offset in L
                                       ref ChVectorDynamic<double> L       //< the L vector
    )
        { }

        public virtual void InjectConstraints(ref ChSystemDescriptor mdescriptor) { }

        public virtual void ConstraintsBiReset() { }

        public virtual void ConstraintsBiLoad_C(double factor = 1.0, double recovery_clamp = 0.1, bool do_clamp = false) { }

        public virtual void ConstraintsFetch_react(double factor) { }
    }
}
