using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
using UnityEngine.Serialization;



namespace chrono
{


    // Physical system in which contact is modeled using a non-smooth
    // (complementarity-based) method.
    //
    // =============================================================================

    /// Create a physical system.
    /// Note, in case you will use collision detection, the values of
    /// 'max_objects' and 'scene_size' can be used to initialize the broadphase
    /// collision algorithm in an optimal way. Scene size should be approximately
    /// the radius of the expected area where colliding objects will move.
    /// The default collision broadphase does not make use of max_objects and scene_size.
    /// If init_sys is false it does not initialize the collision system or solver
    /// assumes that the user will do so.
    //[System.Serializable]
    public class ChSystemNSC : ChSystem
    {
        public int max_objects = 16000;
        public double scene_size = 500;
        public bool init_sys = true;
        
        public int contacts;

        public double testTime = 0;
        
       // private ChRealtimeStepTimer realtime_Steptimer = new ChRealtimeStepTimer();

        /// Available types of solvers.
        public enum SolverType
        {
            SOR = 0,
            SYMMSOR,
            JACOBI,
            SOR_MULTITHREAD,
            PMINRES,
            BARZILAIBORWEIN,
            PCG,
            APGD,
            MINRES,
            SOLVER_SMC,
            CUSTOM,
        };
        public ChSolver.Type solverType = ChSolver.Type.SYMMSOR;

        /// Available methods for time integration (time steppers).
        public enum TimeStepperType
        {
            EULER_IMPLICIT_LINEARIZED = 0,
            EULER_IMPLICIT_PROJECTED = 1,
            EULER_IMPLICIT = 2,
            TRAPEZOIDAL = 3,
            TRAPEZOIDAL_LINEARIZED = 4,
            HHT = 5,
            HEUN = 6,
            RUNGEKUTTA45 = 7,
            EULER_EXPLICIT = 8,
            LEAPFROG = 9,
            NEWMARK = 10,
            CUSTOM = 20
        };
        public TimeStepperType timeStepperType = TimeStepperType.EULER_EXPLICIT;


        public int maxIterationsSolverSpeed;
        public int maxIterSolverStab;
        public double maxPenetrationRecoverySpeed;        

        public ChSystemNSC() : base()
        {
          
        }

        public override void Start()
        {
            base.Start();

            if (init_sys)
            {
                SetSolverWarmStarting(true);
                // Set default contact container
                contact_container = gameObject.AddComponent<ChContactContainerNSC>() as ChContactContainerNSC;//new ChContactContainerNSC();
                contact_container.SetSystem(this);

                // Set default collision engine
                collision_system = new collision.ChCollisionSystemBullet((uint)max_objects, scene_size);

                // Set the system descriptor
                descriptor = new ChSystemDescriptor();
                descriptor.SetNumThreads(parallel_thread_number);

                // Set default solver
                SetSolverType(solverType);

                // Set default collision envelope and margin.
                collision.ChCollisionModel.SetDefaultSuggestedEnvelope(0.03);
               // collision.ChCollisionModel.SetDefaultSuggestedMargin(0.01);
            }
        }

        /// Return the contact method supported by this system.
        /// Bodies added to this system must be compatible.
        public override ChMaterialSurface.ContactMethod GetContactMethod()
        {
            return ChMaterialSurface.ContactMethod.NSC;
        }

        /// Create a new body, consistent with the contact method and collision model used by this system.
        /// The returned body is not added to the system.
        public override ChBody NewBody()
        {
            return new ChBody(ChMaterialSurface.ContactMethod.NSC);
        }

        /// Create a new body with non-centroidal reference frame, consistent with the contact method and
        /// collision model used by this system.  The returned body is not added to the system.
        public override ChBodyAuxRef NewBodyAuxRef() { return new ChBodyAuxRef(); }

     

      /*  private void OnGUI()
        {
            GUI.Label(new Rect(100, 215, 400, 150), "Collision objects: " + debug);
        }*/

        // public virtual void SetSolverType(Type type);

    }
}
