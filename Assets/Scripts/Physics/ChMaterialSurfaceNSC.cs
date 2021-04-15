using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
using UnityEngine.Serialization;


namespace chrono
{
    /// <summary>
    /// Material data for a surface for use with non-smooth (complementarity) contact method.
    /// This data is used to define surface properties owned by ChBody rigid bodies and
    /// similar objects; it carries information that is used to make contacts.
    /// </summary> 
    public class ChMaterialSurfaceNSC : ChMaterialSurface
    {
        public float static_friction;
        public float sliding_friction;
        public float rolling_friction;
        public float spinning_friction;
        public float restitution;
        public float cohesion;
        public float dampingf;
        public float compliance;
        public float complianceT;
        public float complianceRoll;
        public float complianceSpin;


        public ChMaterialSurfaceNSC() : base() {
            static_friction = 0.6f;
            sliding_friction = 0.6f;
            rolling_friction = 0;
            spinning_friction = 0;
            restitution = 0;
            cohesion = 0;
            dampingf = 0;
            compliance = 0;
            complianceT = 0;
            complianceRoll = 0;
            complianceSpin = 0;
        }

        public override ChMaterialSurface Clone() {
            return new ChMaterialSurfaceNSC();
        }
        public override ContactMethod GetContactMethod() {
            return new ContactMethod();
        }

        public void Awake()
        {

        }

      /*  public override void Create(ChBody body)
        {
           // contactMethod = ContactMethod.NSC;
          //  m_MaterialSurface = ChMaterialSurfaceNSC_Create(out m_MaterialSurface, body.m_ChBody);

            if(friction > 0) // When is friction ever going to be zero? not likely.
            {
                SetFriction(friction);
            }
            if (restitution > 0)
            {
                SetRestitution(restitution);
            }
            if (compliance > 0)
            {
                SetCompliance();
            }
            if (dampingf > 0)
            {
                SetDampingF();
            }
            if (spinning_friction > 0)
            {
                SetSpinningFriction();
            }
            if (rolling_friction > 0)
            {
                SetRollingFriction();
            }
        }*/

        /// The static friction coefficient.
        /// Usually in 0..1 range, rarely above. Default 0.6
        public void SetSfriction(float mval) {
            static_friction = mval;
        }

        /// The normal restitution coefficient, for collisions.
        /// Should be in 0..1 range. Default =0.
        void SetRestitution(float mval) {
            restitution = mval;
        }

        /// The sliding ('kinetic')friction coefficient. Default 0.6
        /// Usually in 0..1 range, rarely above.
        /// Note: currently the static friction will be used instead, anyway, because of an issue in the solver.
        public void SetKfriction(float mval) {
            sliding_friction = mval;
        }

        /// Set both static friction and kinetic friction at once, with same value.
        public void SetFriction(float mval) {
            SetSfriction(mval);
            SetKfriction(mval);
        }
        /// Compliance of the contact, in normal direction.
        /// It is the inverse of the stiffness [K] , so for zero
        /// value one has a perfectly rigid contact.
        /// Measuring unit: m/N
        /// Default =0.
        public void SetCompliance() {

        }

        /// The damping in contact, as a factor 'f': damping is a
        /// multiple of stiffness [K], that is: [R]=f*[K]
        /// Measuring unit: time, s. Default =0.
        public void SetDampingF() {

        }

        // The spinning friction (it has the dimension of a length).
        // Spinning resistant torque is Ts <= (normal force) * (this parameter)
        // Usually a very low value.  Measuring unit: m
        // Default =0.
        // Note! a non-zero value will make the simulation 2x slower! Also, the
        // GPU solver currently does not support spinning friction. Default: 0.
        public void SetSpinningFriction() {

        }

        // The rolling friction (rolling parameter, it has the dimension of a length).
        // Rolling resistant torque is Tr <= (normal force) * (this parameter)
        // Usually a very low value. Measuring unit: m
        // Default =0.
        // Note! a non-zero value will make the simulation 2x slower! Also, the
        // GPU solver currently does not support rolling friction. Default: 0.
        public void SetRollingFriction() {

        }


    }

    /// Composite NSC material data for a contact pair.
    public class ChMaterialCompositeNSC : ChMaterialComposite
    {
        public float static_friction;
        public float sliding_friction;
        public float rolling_friction;
        public float spinning_friction;
        public float restitution;
        public float cohesion;
        public float dampingf;
        public float compliance;
        public float complianceT;
        public float complianceRoll;
        public float complianceSpin;

        public ChMaterialCompositeNSC()
        {
            static_friction = 0;
            sliding_friction = 0;
            rolling_friction = 0;
            spinning_friction = 0;
            restitution = 0;
            cohesion = 0;
            dampingf = 0;
            compliance = 0;
            complianceT = 0;
            complianceRoll = 0;
            complianceSpin = 0;
        }

        public ChMaterialCompositeNSC(ChMaterialCompositionStrategy<float> strategy,
                           ChMaterialSurfaceNSC mat1,
                           ChMaterialSurfaceNSC mat2)
        {
            static_friction = strategy.CombineFriction(mat1.static_friction, mat2.static_friction);
            sliding_friction = strategy.CombineFriction(mat1.sliding_friction, mat2.sliding_friction);
            restitution = strategy.CombineRestitution(mat1.restitution, mat2.restitution);
            cohesion = strategy.CombineCohesion(mat1.cohesion, mat2.cohesion);
            dampingf = strategy.CombineDamping(mat1.dampingf, mat2.dampingf);
            compliance = strategy.CombineCompliance(mat1.compliance, mat2.compliance);
            complianceT = strategy.CombineCompliance(mat1.complianceT, mat2.complianceT);

            rolling_friction = strategy.CombineFriction(mat1.rolling_friction, mat2.rolling_friction);
            spinning_friction = strategy.CombineFriction(mat1.spinning_friction, mat2.spinning_friction);
            complianceRoll = strategy.CombineCompliance(mat1.complianceRoll, mat2.complianceRoll);
            complianceSpin = strategy.CombineCompliance(mat1.complianceSpin, mat2.complianceSpin);
        }

    };
}
