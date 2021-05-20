using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
using UnityEngine.Serialization;


namespace chrono
{
    /// Class for a physical system in which contact is modeled using a smooth
    /// (penalty-based) method.
    [System.Serializable]
    public class ChSystemSMC : ChSystem
    {
        /// Enum for SMC contact type.
        public enum ContactForceModel
        {
            Hooke,        //< linear Hookean model
            Hertz,        //< nonlinear Hertzian model
            PlainCoulomb  //< basic tangential force definition for non-granular bodies
        };
        public ContactForceModel contactForceModel = ContactForceModel.Hertz;

        /// Enum for adhesion force model.
        public enum AdhesionForceModel
        {
            Constant,  //< constant adhesion force
            DMT        //< Derjagin-Muller-Toropov model
        };
        public AdhesionForceModel adhesionForceModel = AdhesionForceModel.Constant;

        /// Enum for tangential displacement model.
        public enum TangentialDisplacementModel
        {
            None,      //< no tangential force
            OneStep,   //< use only current relative tangential velocity
            MultiStep  //< use contact history (from contact initiation)
        };
        public TangentialDisplacementModel tangentialDisplacementModel = TangentialDisplacementModel.None;

        public int maxIterationsSolverSpeed;
        public int maxIterSolverStab;
        public double maxPenetrationRecoverySpeed;

        public int max_objects = 16000;
        public double scene_size = 500;
        public bool init_sys = true;

        // The soft-real-time cycle
        public double time = 0.0;
        public double out_time = 0.0;
        public double out_step;// = 2000 * step;

        /// Constructor for ChSystemSMC.
        /// Note that, in case you will use collision detection, the values of
        /// 'max_objects' and 'scene_size' can be used to initialize the broadphase
        /// collision algorithm in an optimal way. Scene size should be approximately
        /// the radius of the expected area where colliding objects will move.
        public ChSystemSMC()
        {            
        }

        // Use this for initialization
        public override void Start()
        {
            out_step = 2000 * step;

            base.Start();

            if (init_sys)
            {
                m_use_mat_props = true;
                m_contact_model = contactForceModel;
                m_adhesion_model = adhesionForceModel;
                m_tdispl_model = tangentialDisplacementModel;
                m_stiff_contact = false;

                descriptor = new ChSystemDescriptor();
                descriptor.SetNumThreads(parallel_thread_number);

                solver_speed = new ChSolverSMC();
                solver_stab = new ChSolverSMC();

                collision_system = new collision.ChCollisionSystemBullet((uint)max_objects, scene_size);

                // For default SMC there is no need to create contacts 'in advance'
                // when models are closer than the safety envelope, so set default envelope to 0
                collision.ChCollisionModel.SetDefaultSuggestedEnvelope(0);

                contact_container = gameObject.AddComponent<ChContactContainerSMC>();
                contact_container.SetSystem(this);

                m_minSlipVelocity = 1e-4;
                m_characteristicVelocity = 1;
            }

        }

        /// Enable/disable using physical contact material properties.
        /// If true, contact coefficients are estimated from physical material properties.
        /// Otherwise, explicit values of stiffness and damping coefficients are used.
        public void UseMaterialProperties(bool val) { m_use_mat_props = val; }
        /// Return true if contact coefficients are estimated from physical material properties.
        public bool UsingMaterialProperties() { return m_use_mat_props; }

        /// Set the normal contact force model.
        public void SetContactForceModel(ContactForceModel model) { m_contact_model = model; }
        /// Get the current normal contact force model.
        public ContactForceModel GetContactForceModel() { return m_contact_model; }

        /// Set the adhesion force model.
        public void SetAdhesionForceModel(AdhesionForceModel model) { m_adhesion_model = model; }
        /// Get the current adhesion force model.
        public AdhesionForceModel GetAdhesionForceModel() { return m_adhesion_model; }

        /// Set the tangential displacement model.
        /// Note that currently MultiStep falls back to OneStep.
        public void SetTangentialDisplacementModel(TangentialDisplacementModel model) { m_tdispl_model = model; }
        /// Get the current tangential displacement model.
        public TangentialDisplacementModel GetTangentialDisplacementModel() { return m_tdispl_model; }

        /// Slip velocity threshold.
        /// No tangential contact forces are generated if the magnitude of the tangential
        /// relative velocity is below this value.
        public void SetSlipVelocityThreshold(double vel) { m_minSlipVelocity = Math.Max(vel, double.Epsilon); }
        public double GetSlipVelocityThreshold() { return m_minSlipVelocity; }

    /// Characteristic impact velocity (Hooke contact force model).
    public void SetCharacteristicImpactVelocity(double vel) { m_characteristicVelocity = vel; }
        public double GetCharacteristicImpactVelocity() { return m_characteristicVelocity; }

    /// Declare the contact forces as stiff.
    /// If true, this enables calculation of contact force Jacobians.
    public void SetStiffContact(bool val) { m_stiff_contact = val; }
        public bool GetStiffContact() { return m_stiff_contact; }

    /// Return the contact method supported by this system.
    /// Bodies added to this system must be compatible.
    public override ChMaterialSurface.ContactMethod GetContactMethod() {
            return ChMaterialSurface.ContactMethod.SMC;
        }

        /// Create a new body, consistent with the contact method and collision model used by this system.
        /// The returned body is not added to the system.
        public override ChBody NewBody() { return new ChBody(ChMaterialSurface.ContactMethod.SMC);
    }

        /// Create a new body with non-centroidal reference frame, consistent with the contact method and
        /// collision model used by this system.  The returned body is not added to the system.
        public override ChBodyAuxRef NewBodyAuxRef()
        {
            return new ChBodyAuxRef(ChMaterialSurface.ContactMethod.SMC);
        }



        // Update is called once per frame
        public override void FixedUpdate()
        {
            Time.fixedDeltaTime = (float)step;

            DoStepDynamics(step);


            /* if (vehicleApplication)
             {
                 // Advance simulation for one timestep for all modules
                 m_softstep = enforce_soft_real_time ? realtime_timer.SuggestSimulationStep(step) : step;

                 Time.fixedDeltaTime = (float)m_softstep;

                 double t = 0;
                 while (t < m_softstep)
                 {
                     double h = Math.Min(step, m_softstep - t);
                     DoStepDynamics(h);
                     t += h;
                 }
             }
             else
             {
                 DoStepDynamics(step);
             }*/
        }


        private void OnGUI()
        {
           // GUI.Label(new Rect(100, 215, 400, 150), "SoftStep: " + m_softstep);
        }


        private bool m_use_mat_props;                        //< if true, derive contact parameters from mat. props.
        private ContactForceModel m_contact_model;           //< type of the contact force model
        private AdhesionForceModel m_adhesion_model;         //< type of the adhesion force model
        private TangentialDisplacementModel m_tdispl_model;  //< type of tangential displacement model
        private bool m_stiff_contact;                        //< flag indicating stiff contacts (triggers Jacobian calculation)
        private double m_minSlipVelocity;                    //< slip velocity below which no tangential forces are generated
        private double m_characteristicVelocity;             //< characteristic impact velocity (Hooke model)

    }
}
