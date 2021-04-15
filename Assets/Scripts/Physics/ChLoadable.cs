using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono
{

    /// Interface for objects that can be subject to loads (forces)
    /// Forces can be distributed on UV surfaces, or lines, etc.,so
    /// look also the more detailed children classes.
    /// 
    public interface IChLoadable
    {

        /// Gets the number of DOFs affected by this element (position part)
        int LoadableGet_ndof_x();

        /// Gets the number of DOFs affected by this element (speed part)
        int LoadableGet_ndof_w();

        /// Gets all the DOFs packed in a single vector (position part)
        void LoadableGetStateBlock_x(int block_offset, ChState mD);

        /// Gets all the DOFs packed in a single vector (speed part)
        void LoadableGetStateBlock_w(int block_offset, ChStateDelta mD);

        /// Increment all DOFs using a delta. Default is sum, but may override if 
        /// ndof_x is different than ndof_w, for example with rotation quaternions and angular w vel.
        /// This could be invoked, for example, by the BDF differentiation that computes the jacobians.
        void LoadableStateIncrement(int off_x,
                                       ChState x_new,
                                       ChState x,
                                       int off_v,
                                       ChStateDelta Dv);


        /// Number of coordinates in the interpolated field, ex=3 for a
        /// tetrahedron finite element or a cable, = 1 for a thermal problem, etc.
        int Get_field_ncoords();

        /// Tell the number of DOFs blocks (ex. =1 for a body, =4 for a tetrahedron, etc.)
        int GetSubBlocks();

        /// Get the offset of the i-th sub-block of DOFs in global vector
        int GetSubBlockOffset(int nblock);

        /// Get the size of the i-th sub-block of DOFs in global vector
        int GetSubBlockSize(int nblock);

        /// Get the pointers to the contained ChVariables, appending to the mvars vector.
        void LoadableGetVariables(List<ChVariables> mvars);

    }

    /// Interface for objects that can be subject to volume loads,
    /// distributed along UVW coordinates of the object.
    /// For instance finite elements like 3D bricks, ex.for gravitational loads.

    public interface IChLoadableUVW : IChLoadable {


        /// Evaluate N'*F , where N is some type of shape function
        /// evaluated at U,V,W coordinates of the volume, each ranging in -1..+1
        /// F is a load, N'*F is the resulting generalized load
        /// Returns also det[J] with J=[dx/du,..], that might be useful in gauss quadrature.
        void ComputeNF(double U,              //< parametric coordinate in volume
                           double V,              //< parametric coordinate in volume
                           double W,              //< parametric coordinate in volume
                           ChVectorDynamic<double> Qi,       //< Return result of N'*F  here, maybe with offset block_offset
                           double detJ,                //< Return det[J] here
                           ChVectorDynamic<double> F,  //< Input F vector, size is = n.field coords.
                           ChVectorDynamic<double> state_x,  //< if != 0, update state (pos. part) to this, then evaluate Q
                           ChVectorDynamic<double> state_w   //< if != 0, update state (speed part) to this, then evaluate Q
                           );

        /// This can be useful for loadable objects that has some density property, so it can be
        /// accessed by ChLoaderVolumeGravity. Return 0 if the element/nodes does not support xyz gravity.
        double GetDensity();

        /// If true, use quadrature over u,v,w in [0..1] range as tetrahedron volumetric coords, with z=1-u-v-w
        /// otherwise use quadrature over u,v,w in [-1..+1] as box isoparametric coords.
        bool IsTetrahedronIntegrationNeeded();

    }


}
