using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono {


    /// @addtogroup vehicle_terrain
    /// @{

    /// Base class for a terrain system.
    public abstract class ChTerrain : MonoBehaviour
    {

        public ChTerrain() {
            m_friction_fun = null;
        }

        /// Update the state of the terrain system at the specified time.
        public virtual void Synchronize(double time) { }

        /// Advance the state of the terrain system by the specified duration.
        public virtual void Advance(double step) { }

        /// Get the terrain height at the specified (x,y) location.
        public abstract double GetHeight(double x, double y);

        /// Get the terrain normal at the specified (x,y) location.
        public abstract ChVector GetNormal(double x, double y);

        /// Get the terrain coefficient of friction at the specified (x,y) location.
        /// This coefficient of friction value may be used by certain tire models to modify
        /// the tire characteristics, but it will have no effect on the interaction of the terrain
        /// with other objects (including tire models that do not explicitly use it).
        public abstract float GetCoefficientFriction(double x, double y);

        /// Class to be used as a functor interface for location-dependent coefficient of friction.
        public class FrictionFunctor {

            /// Return the coefficient of friction at a given (x,y) location.
            public virtual float this[double x, double y] { get { return 0; } }
        }

        /// Specify the functor object to provide the coefficient of friction at given (x,y) locations.
        public void RegisterFrictionFunctor(FrictionFunctor functor) { m_friction_fun = functor; }


        protected FrictionFunctor m_friction_fun;  //< functor for location-dependent coefficient of friction
    };

}