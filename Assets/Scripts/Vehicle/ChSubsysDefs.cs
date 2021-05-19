using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono
{

    public static class ChSubsysDefs
    {
        // -----------------------------------------------------------------------------
        // Utility classes and structures for data exchange
        // -----------------------------------------------------------------------------

        /// Enum for the side (left/right) of a vehicle.
        public enum VehicleSide
        {
            LEFT = 0,  //< left side of vehicle is always 0
            RIGHT = 1  //< right side of vehicle is always 1
        };

        /// Structure to communicate a full wheel body state.
        /// In addition to the quantities communicated for a generic body, the wheel
        /// state also includes the wheel angular speed about its axis of rotation.
        public struct WheelState
        {
            public ChVector pos;      //< global position
            public ChQuaternion rot;  //< orientation with respect to global frame
            public ChVector lin_vel;  //< linear velocity, expressed in the global frame
            public ChVector ang_vel;  //< angular velocity, expressed in the global frame
            public double omega;        //< wheel angular speed about its rotation axis
        };

        /// Structure to communicate a set of generalized terrain contact forces (tire or track shoe).
        public struct TerrainForce
        {
            public ChVector force;   //< force vector, epxressed in the global frame
            public ChVector point;   //< global location of the force application point
            public ChVector moment;  //< moment vector, expressed in the global frame
        };

    }

}
