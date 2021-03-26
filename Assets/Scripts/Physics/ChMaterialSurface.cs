using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;


namespace chrono
{
    /// Base class for specifying material properties for contact force generation.
    public abstract class ChMaterialSurface : MonoBehaviour
    {
        public enum ContactMethod
        {
            NSC = 0,  //< non-smooth, constraint-based (a.k.a. rigid-body) contact
            SMC = 1   //< smooth, penalty-based (a.k.a. soft-body) contact
        };

        /// "Virtual" copy constructor.
        public abstract ChMaterialSurface Clone();
        public abstract ContactMethod GetContactMethod();

    }

    /// Base class for composite material for a contact pair.
    public class ChMaterialComposite
    {

    };

    /// Base class for material composition strategy.
    /// Implements the default combination laws for coefficients of friction, cohesion, compliance, etc.
    /// Derived classes can override one or more of these combination laws.
    /// Enabling the use of a customized composition strategy is system type-dependent.
    public class ChMaterialCompositionStrategy<T>
    {
        public virtual T CombineFriction(T a1, T a2) { return Math.Min((dynamic)a1, (dynamic)a2); }
        public virtual T CombineCohesion(T a1, T a2) { return Math.Min((dynamic)a1, (dynamic)a2); }
        public virtual T CombineRestitution(T a1, T a2) { return Math.Min((dynamic)a1, (dynamic)a2); }
        public virtual T CombineDamping(T a1, T a2) { return Math.Min((dynamic)a1, (dynamic)a2); }
        public virtual T CombineCompliance(T a1, T a2) { return a1 + (dynamic)a2; }

        public virtual T CombineAdhesionMultiplier(T a1, T a2) { return Math.Min((dynamic)a1, (dynamic)a2); }
        public virtual T CombineStiffnessCoefficient(T a1, T a2) { return (a1 + (dynamic)a2) / 2; }
        public virtual T CombineDampingCoefficient(T a1, T a2) { return (a1 + (dynamic)a2) / 2; }
    };
}
