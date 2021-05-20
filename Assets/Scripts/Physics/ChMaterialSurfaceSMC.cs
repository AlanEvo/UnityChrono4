using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
using UnityEngine.Serialization;


namespace chrono
{

    /// Material data for a surface for use with smooth (penalty) contact method.
    /// This data is used to define surface properties owned by ChBody rigid bodies and
    /// similar objects; it carries information that is used to make contacts.
    public class ChMaterialSurfaceSMC : ChMaterialSurface
    {


        public float young_modulus;      /// Young's modulus (elastic modulus)
        public float poisson_ratio;      /// Poisson ratio
        public float static_friction;    /// Static coefficient of friction
        public float sliding_friction;   /// Kinetic coefficient of friction
        public float restitution;        /// Coefficient of restitution
        public float constant_adhesion;  /// Constant adhesion force, when constant adhesion model is used
        public float adhesionMultDMT;    /// Adhesion multiplier used in DMT model.

        // DMT adhesion model:
        //     adhesion = adhesionMultDMT * sqrt(R_eff).
        // Given the surface energy, w,
        //     adhesionMultDMT = 2 * CH_C_PI * w * sqrt(R_eff).
        // Given the equilibrium penetration distance, y_eq,
        //     adhesionMultDMT = 4.0 / 3.0 * E_eff * powf(y_eq, 1.5)

        public float kn;  //< user-specified normal stiffness coefficient
        public float kt;  //< user-specified tangential stiffness coefficient
        public float gn;  //< user-specified normal damping coefficient
        public float gt;  //< user-specified tangential damping coefficient

        public void Start()
        {

        }

        public ChMaterialSurfaceSMC() : base()
        {
            young_modulus = 2e5f;
            poisson_ratio = 0.3f;
            static_friction = 0.6f;
            sliding_friction = 0.6f;
            restitution = 0.4f;
            constant_adhesion = 0;
            adhesionMultDMT = 0;
            kn = 2e5f;
            kt = 2e5f;
            gn = 40;
            gt = 20;
        }

        public override ChMaterialSurface Clone()
        {
            return new ChMaterialSurfaceSMC();
        }
        public override ContactMethod GetContactMethod()
        {
            return new ContactMethod();
        }

        /* public override void Create(ChBody body)
         {
            // contactMethod = ContactMethod.SMC;
           //  m_MaterialSurface = ChMaterialSurfaceSMC_Create(out m_MaterialSurface, body.m_ChBody);

             if (static_friction > 0) // When is friction ever going to be zero? not likely.
             {
                 SetFriction(static_friction);
             }
             if (restitution > 0)
             {
                 SetRestitution(restitution);
             }
             SetYoungModulus(young_modulus);
             SetPoissonRatio(poisson_ratio);
             SetKn(kn);
             SetGn(gn);
             SetKt(kt);
             SetGt(gt);
         }*/
        /// Set both static friction and kinetic friction at once, with same value.
        public void SetFriction(float mval)
        {
            SetSfriction(mval);
            SetKfriction(mval);
        }

        /// Young's modulus.
        public void SetYoungModulus(float val)
        {
            young_modulus = val;

        }

        // Poisson ratio.
        public void SetPoissonRatio(float val)
        {

            poisson_ratio = val;

        }

        /// The sliding ('kinetic')friction coefficient. Default 0.6
        /// Usually in 0..1 range, rarely above.
        /// Note: currently the static friction will be used instead, anyway, because of an issue in the solver.
        public void SetKfriction(float mval)
        {
            sliding_friction = mval;

        }

        /// The static friction coefficient.
        /// Usually in 0..1 range, rarely above. Default 0.6
        public void SetSfriction(float mval)
        {
            static_friction = mval;

        }

        /// The normal restitution coefficient, for collisions.
        /// Should be in 0..1 range. Default =0.
        void SetRestitution(float mval)
        {
            restitution = mval;

        }

        public void SetKn(float val)
        {
            kn = val;
            // ChMaterialSurfaceSMC_SetKn(m_MaterialSurface, kn);
        }
        public void SetKt(float val)
        {

            kt = val;

        }
        public void SetGn(float val)
        {

            gn = val;

        }
        public void SetGt(float val)
        {

            gt = val;

        }
    }

    /// Composite SMC material data for a contact pair.
    public class ChMaterialCompositeSMC : ChMaterialComposite
    {

        public float E_eff;                //< Effective elasticity modulus
        public float G_eff;                //< Effective shear modulus
        public float mu_eff;               //< Effective coefficient of friction
        public float cr_eff;               //< Effective coefficient of restitution
        public float adhesion_eff;         //< Effective cohesion force
        public float adhesionMultDMT_eff;  //< Effective adhesion multiplier (DMT model)

        public float kn;  //< normal stiffness coefficient
        public float kt;  //< tangential stiffness coefficient
        public float gn;  //< normal viscous damping coefficient
        public float gt;  //< tangential viscuous damping coefficient

        public ChMaterialCompositeSMC()
        {
            E_eff = 0;
            G_eff = 0;
            mu_eff = 0;
            cr_eff = 0;
            adhesion_eff = 0;
            adhesionMultDMT_eff = 0;

            kn = 0;
            kt = 0;
            gn = 0;
            gt = 0;
        }

        public ChMaterialCompositeSMC(ChMaterialCompositionStrategy<float> strategy,
                           ChMaterialSurfaceSMC mat1,
                           ChMaterialSurfaceSMC mat2)
        {
            float inv_E = (1 - mat1.poisson_ratio * mat1.poisson_ratio) / mat1.young_modulus +
                  (1 - mat2.poisson_ratio * mat2.poisson_ratio) / mat2.young_modulus;
            float inv_G = 2 * (2 - mat1.poisson_ratio) * (1 + mat1.poisson_ratio) / mat1.young_modulus +
                          2 * (2 - mat2.poisson_ratio) * (1 + mat2.poisson_ratio) / mat2.young_modulus;

            E_eff = 1 / inv_E;
            G_eff = 1 / inv_G;

            mu_eff = strategy.CombineFriction(mat1.static_friction, mat2.static_friction);
            cr_eff = strategy.CombineRestitution(mat1.restitution, mat2.restitution);
            adhesion_eff = strategy.CombineCohesion(mat1.constant_adhesion, mat2.constant_adhesion);
            adhesionMultDMT_eff = strategy.CombineAdhesionMultiplier(mat1.adhesionMultDMT, mat2.adhesionMultDMT);

            kn = strategy.CombineStiffnessCoefficient(mat1.kn, mat2.kn);
            kt = strategy.CombineStiffnessCoefficient(mat1.kt, mat2.kt);
            gn = strategy.CombineDampingCoefficient(mat1.gn, mat2.gn);
            gt = strategy.CombineDampingCoefficient(mat1.gt, mat2.gt);
        }
    };
}
