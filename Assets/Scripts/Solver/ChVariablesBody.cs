using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

namespace chrono
{

    /// Specialized class for representing a 6-DOF item for a
    /// system, that is a 3D rigid body, with mass matrix and
    /// associate variables (a 6 element vector, ex.speed)
    /// This is an abstract class, specialized for example in
    /// ChVariablesBodyOwnMass and ChVariablesBodySharedMass.
    public unsafe abstract class ChVariablesBody : ChVariables
    {
        private IntPtr user_data;


        public ChVariablesBody() : base(6) 
        { 
            user_data = IntPtr.Zero; 
        }

        /// Get the mass associated with translation of body
        public abstract double GetBodyMass();

        /// Access the 3x3 inertia matrix
        public abstract ChMatrix33<double> GetBodyInertia();

        /// Access the 3x3 inertia matrix inverted
        public abstract ChMatrix33<double> GetBodyInvInertia();

        /// The number of scalar variables in the vector qb
        /// (dof=degrees of freedom)
        public override int Get_ndof() { return 6; }

        public IntPtr GetUserData() { return this.user_data; }
        public void SetUserData(IntPtr mdata) { this.user_data = mdata; }
    }

}
