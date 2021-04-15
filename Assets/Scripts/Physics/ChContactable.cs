using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

namespace chrono
{

    //  using type_constraint_tuple = ChVariableTupleCarrier_1vars<int>;
    /* public enum EChContactableType
     {
         CONTACTABLE_UNKNOWN = 0,  //< unknown contactable type
         CONTACTABLE_6,            //< 1 variable with 6 DOFs (e.g., ChBody, ChNodeFEAxyzrot)
         CONTACTABLE_3,            //< 1 variable with 3 DOFS (e.g., ChNodeFEAxyz, ChAparticle)
         CONTACTABLE_333,          //< 3 variables, each with 3 DOFs (e.g., triangle between 3 ChNodeFEAxyz nodes)
         CONTACTABLE_666           //< 3 variables, each with 6 DOFs (e.g., triangle between 3 ChNodeFEAxyzrot nodes)
     };*/

    /// Interface for objects that generate contacts
    /// One should inherit from ChContactable_1vars, ChContactable_2vars  etc. depending
    /// on the number of ChVariable objects contained in the object (i.e. the variable chunks
    /// to whom the contact point position depends, also the variables affected by contact force).
    public interface ChContactable
    {
        /// Indicate whether or not the object must be considered in collision detection.
        public abstract bool IsContactActive();

        /// Get the number of DOFs affected by this object (position part)
        public abstract int ContactableGet_ndof_x();

        /// Get the number of DOFs affected by this object (speed part)
        public abstract int ContactableGet_ndof_w();

        /// Get all the DOFs packed in a single vector (position part)
        public abstract void ContactableGetStateBlock_x(ref ChState x);

        /// Get all the DOFs packed in a single vector (speed part)
        public abstract void ContactableGetStateBlock_w(ref ChStateDelta w);

        /// Increment the provided state of this object by the given state-delta increment.
        /// Compute: x_new = x + dw.
        public abstract void ContactableIncrementState(ChState x, ChStateDelta dw, ref ChState x_new);

        /// Return the pointer to the surface material.
        /// Use dynamic cast to understand if this is a ChMaterialSurfaceSMC, ChMaterialSurfaceNSC or others.
        /// This function returns a reference to the shared pointer member variable and is therefore THREAD SAFE.
        public abstract ChMaterialSurface GetMaterialSurfaceBase();

        /// Express the local point in absolute frame, for the given state position.
        public abstract ChVector GetContactPoint(ChVector loc_point, ChState state_x);

        /// Get the absolute speed of a local point attached to the contactable.
        /// The given point is assumed to be expressed in the local frame of this object.
        /// This function must use the provided states.  
        public abstract ChVector GetContactPointSpeed(ChVector loc_point, ChState state_x, ChStateDelta state_w);

        /// Get the absolute speed of point abs_point if attached to the surface.
        public abstract ChVector GetContactPointSpeed(ChVector abs_point);

        /// Return the coordinate system for the associated collision model.
        /// ChCollisionModel might call this to get the position of the
        /// contact model (when rigid) and sync it.
        public abstract ChCoordsys GetCsysForCollisionModel();

        /// Apply the given force at the given location and load into the global generalized force array.
        /// The force and its application point are given in the absolute reference frame. Each object must 
        /// update the entries in R corresponding to its variables. Force for example could come from a penalty model.
        public abstract void ContactForceLoadResidual_F(ChVector F, ChVector abs_point, ref ChVectorDynamic<double> R);

        /// Apply the given force at the given point and load the generalized force array.
        /// The force and its application point are specified in the global frame.
        /// Each object must set the entries in Q corresponding to its variables, starting at the specified offset.
        /// If needed, the object states must be extracted from the provided state position.
        public abstract void ContactForceLoadQ(ChVector F,
                                   ChVector point,
                                   ChState state_x,
                                   ref ChVectorDynamic<double> Q,
                                   int offset);

        /// This can be useful in some SMC code:
        public abstract double GetContactableMass();

        /// This is only for backward compatibility. Note that in recent code
        /// the reference to the ChPhysicsItem should disappear.
        /// The ChPhysicsItem could be the ChContactable itself (ex. see the ChBody) or
        /// a container (ex. the ChMEsh, for ChContactTriangle)
        public abstract ChPhysicsItem GetPhysicsItem();


    }    

    public interface ChContactable_1vars<Ta> : ChContactable, IntInterface.ChVariableTupleCarrier_1vars
    {

       //  public class type_variable_tuple_carrier : ChVariableTupleCarrier_1vars<T1> { public ChVariables GetVariables1() { return null; } }
        // public new class type_constraint_tuple : ChVariableTupleCarrier_1vars<T1>.type_constraint_tuple { }



        /* abstract void ComputeJacobianForContactPart<Ta, Tb>(ChVector abs_point,
                                             ref ChMatrix33<double> contact_plane,
                                             ref ChConstraintTuple_1vars<ChVariableTupleCarrier_1vars<ChContactNSC<Ta, Tb>.typecarr_a>> jacobian_tuple_N,
                                             ref ChConstraintTuple_1vars<ChVariableTupleCarrier_1vars<ChContactNSC<Ta, Tb>.typecarr_a>> jacobian_tuple_U,
                                             ref ChConstraintTuple_1vars<ChVariableTupleCarrier_1vars<ChContactNSC<Ta, Tb>.typecarr_a>> jacobian_tuple_V,
                                             bool second);*/
       

        abstract void ComputeJacobianForContactPart(ChVector abs_point,
                                               ref ChMatrix33<double> contact_plane,
                                               ref type_constraint_tuple jacobian_tuple_N,
                                               ref type_constraint_tuple jacobian_tuple_U,
                                               ref type_constraint_tuple jacobian_tuple_V,
                                               bool second);


        abstract void ComputeJacobianForRollingContactPart(ChVector abs_point,
                                              ref ChMatrix33<double> contact_plane,
                                              ref type_constraint_tuple jacobian_tuple_N,
                                              ref type_constraint_tuple jacobian_tuple_U,
                                              ref type_constraint_tuple jacobian_tuple_V,
                                              bool second);


        /// Compute the jacobian(s) part(s) for this contactable item. For example,
        /// if the contactable is a ChBody, this should update the three corresponding 1x6 jacobian rows.
         /*abstract void ComputeJacobianForContactPart(ChVector abs_point,
                                                ref ChMatrix33<double> contact_plane,
                                                ref type_constraint_tuple jacobian_tuple_N,
                                                ref type_constraint_tuple jacobian_tuple_U,
                                                ref type_constraint_tuple jacobian_tuple_V,
                                                bool second);*/



        /*abstract void ComputeJacobianForContactPart<T1, T2>(ChVector abs_point,
                                                ref ChMatrix33<double> contact_plane,
                                                ref ChConstraintTwoTuples<ChContactNSC<T1,T2>.typecarr_a, ChContactNSC<T1, T2>.typecarr_b>.type_constraint_tuple_a jacobian_tuple_N,
                                                ref ChConstraintTwoTuples<ChContactNSC<T1, T2>.typecarr_a, ChContactNSC<T1, T2>.typecarr_b>.type_constraint_tuple_a jacobian_tuple_U,
                                                ref ChConstraintTwoTuples<ChContactNSC<T1, T2>.typecarr_a, ChContactNSC<T1, T2>.typecarr_b>.type_constraint_tuple_a jacobian_tuple_V,
                                                bool second);*/

        /// Compute the jacobian(s) part(s) for this contactable item, for rolling about N,u,v
        /// (used only for rolling friction NSC contacts)



    }

    // Note that template T1 and T2 and T3 are the number of DOFs in the referenced ChVariable s, 
    // for instance 3 and 3 and 3 for a 'triangle face' betweeen two xyz nodes.
    public interface ChContactable_3vars<T1, T2, T3> : ChContactable, IntInterface.ChVariableTupleCarrier_3vars<T1, T2, T3>
    {

        /*public class type_variable_tuple_carrier : IntInterface.ChVariableTupleCarrier_3vars<T1, T2, T3> { 
            public override ChVariables GetVariables1() { return null; }
            public override ChVariables GetVariables2() { return null; }
            public override ChVariables GetVariables3() { return null; }
        }*/
        //public class type_constraint_tuple : ChVariableTupleCarrier_3vars<T1, T2, T3>.type_constraint_tuple { }

        /// Compute the jacobian(s) part(s) for this contactable item. For example,
        /// if the contactable is a ChBody, this should update the corresponding 1x6 jacobian.
        abstract void ComputeJacobianForContactPart(ChVector abs_point,
                                                   ref ChMatrix33<double> contact_plane,
                                                   ref type_constraint_tuple jacobian_tuple_N,
                                                   ref type_constraint_tuple jacobian_tuple_U,
                                                   ref type_constraint_tuple jacobian_tuple_V,
                                                   bool second);

        /// Compute the jacobian(s) part(s) for this contactable item, for rolling about N,u,v
        /// (used only for rolling friction NSC contacts)
        void ComputeJacobianForRollingContactPart(ChVector abs_point,
                                                      ref ChMatrix33<double> contact_plane,
                                                      ref type_constraint_tuple jacobian_tuple_N,
                                                      ref type_constraint_tuple jacobian_tuple_U,
                                                      ref type_constraint_tuple jacobian_tuple_V,
                                                      bool second);


    };

}

