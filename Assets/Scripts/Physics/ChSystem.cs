using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
using UnityEditor;
using UnityEngine.Serialization;
using System.Runtime.InteropServices;
using System.Linq;

namespace chrono
{
    public class ContactManager : ChContactContainer.ReportContactCallback
    {

        public ContactManager(ChSystem system)
        {
            m_system = system;
        }

        // Return the current total number of contacts experienced by the specified body.
        public int GetNcontacts(ChBody body)
        {
            //List<ChBody> search = m_bcontacts.find(body.get());
            var search = m_bcontacts.FirstOrDefault(pair => { return body; }).Value;
            return (search == m_bcontacts.LastOrDefault().Value) ? 0 : search;
        }

        
        // Process all contacts at current time.
        // Reset the hash map and invoke the callback for each collision.
        public void Process()
        {
            m_bcontacts.Clear();
            m_system.GetContactContainer().ReportAllContacts(this);
        }


        // Keep track of the number of contacts experienced by each body.
        // Maintain a hash map with body as key and current number of contacts as value.
        public override bool OnReportContact(ChVector pA,
                                 ChVector pB,
                                 ChMatrix33<double> plane_coord,
                                 double distance,
                                 double eff_radius,
                                 ChVector cforce,
                                 ChVector ctorque,
                                 ChContactable modA,
                                 ChContactable modB)
        {
            var bodyA = (ChBody)(modA);
            var searchA = m_bcontacts.FirstOrDefault(pair => { return bodyA; }).Value;
            if (searchA == m_bcontacts.LastOrDefault().Value)
                m_bcontacts.Add(bodyA, 1);
            else
                searchA++;

            var bodyB = (ChBody)(modB);
            var searchB = m_bcontacts.FirstOrDefault(pair => { return bodyB; }).Value; ;
            if (searchB == m_bcontacts.LastOrDefault().Value)
                m_bcontacts.Add(bodyB, 1);
            else
                searchB++;

            return true;
        }

        private ChSystem m_system;
        private Dictionary<ChBody, int> m_bcontacts = new Dictionary<ChBody, int>();
    };

   

        /// Physical system.
        ///
        /// This class is used to represent a multibody physical system,
        /// so it acts also as a database for most items involved in
        /// simulations, most noticeably objects of ChBody and ChLink
        /// classes, which are used to represent mechanisms.
        ///
        /// Moreover, it also owns some global settings and features,
        /// like the gravity acceleration, the global time and so on.
        ///
        /// This object will be responsible of performing the entire
        /// physical simulation (dynamics, kinematics, statics, etc.),
        /// so you need at least one ChSystem object in your program, in
        /// order to perform simulations (you'll insert rigid bodies and
        /// links into it..)
        ///
        /// Note that this is an abstract class, in your code you must
        /// create a system from one of the concrete classes: 
        ///   @ref chrono::ChSystemNSC (for non-smooth contacts) or
        ///   @ref chrono::ChSystemSMC (for smooth 'penalty' contacts).
        ///
        /// Further info at the @ref simulation_system  manual page.
        public abstract class ChSystem : ChAssembly, ChIntegrableIIorder
    {
        // private ChIntegrableIIorder m_intergral = new ChIntegrableIIorder();
        // public ContactManager contactManager;

        public ChSystem() : base()
        {

        }

        public virtual void Awake()
        {
            end_time = 1;
            // step = 0.04;
            step_min = 0.002;
            step_max = 0.04;
            tol = 2e-4;
            tol_force = 1e-3;
           // maxiter = 6;
          //  max_iter_solver_speed = 30;
          //  max_iter_solver_stab = 10;
            ncontacts = 0;
            min_bounce_speed = 0.15;
            max_penetration_recovery_speed = 0.6;
           // use_sleeping = false;

            SetSolverWarmStarting(false);

            G_acc.x = gravity.x;
            G_acc.y = gravity.y;
            G_acc.z = gravity.z;
            stepcount = 0;
            solvecount = 0;
            setupcount = 0;
            dump_matrices = false;
            last_err = false;
            composition_strategy = new ChMaterialCompositionStrategy<float>();

            // Required by ChAssembly
            system = this;

            // Set default number of threads to be equal to number of available cores
            parallel_thread_number = CHOMPfunctions.GetNumProcs();

            // Set default collision envelope and margin.
            // collision.ChCollisionModel.SetDefaultSuggestedEnvelope(0.03);
            // collision.ChCollisionModel.SetDefaultSuggestedMargin(0.01);

            // Set default timestepper.
            timestepper = new ChTimestepperEulerImplicitLinearized(this);
            /* end_time = 1;
             step = 0.04;
             step_min = 0.002;
             step_max = 0.04;
             tol = 2e-4;
             tol_force = 1e-3;
             maxiter = 6;
             max_iter_solver_speed = 30;
             max_iter_solver_stab = 10;
             ncontacts = 0;
             min_bounce_speed = 0.15;
             max_penetration_recovery_speed = 0.6;
             use_sleeping = false;
             G_acc = new ChVector(0, -9.8, 0);
             stepcount = 0;
             solvecount = 0;
             setupcount = 0;
             dump_matrices = false;
             last_err = false;
             composition_strategy = new ChMaterialCompositionStrategy<float>();

             // Required by ChAssembly
             system = this;

             // Set default number of threads to be equal to number of available cores
             parallel_thread_number = CHOMPfunctions.GetNumProcs();

             // Set default collision envelope and margin.
             // collision.ChCollisionModel.SetDefaultSuggestedEnvelope(0.03);
             // collision.ChCollisionModel.SetDefaultSuggestedMargin(0.01);

             // Set default timestepper.
             timestepper = new ChTimestepperEulerImplicitLinearized(this);*/
        }

        // public int GetNcoords_y() { return 0; }

        /// Return the number of coordinates in the state increment.
        /// This is a base implementation that works in many cases where dim(Y) = dim(dy),
        /// but it can be overridden in the case that y contains quaternions for rotations
        /// rather than simple y+dy
        // public int GetNcoords_dy() { return GetNcoords_y(); }

        /// Return the number of lagrangian multipliers (constraints).
        /// By default returns 0.
        // public int GetNconstr() { return 0; }

        /// Set up the system state.
       /* public void StateSetup(ChState y, ChStateDelta dy)
        {
            y.Resize(GetNcoords_y(), 1);
            dy.Resize(GetNcoords_dy(), 1);
        }*/

        /// Gather system state in specified array.
        /// Optionally, they will copy system private state, if any, to Y.
        public void StateGather(ref ChState y, ref double T) {
            ChState mx = new ChState(GetNcoords_x(), y.GetIntegrable());
            ChStateDelta mv = new ChStateDelta(GetNcoords_v(), y.GetIntegrable());
            this.StateGather(ref mx, ref mv, ref T);
            y.PasteMatrix(mx, 0, 0);
            y.PasteMatrix(mv, GetNcoords_x(), 0);
        }

        /// Scatter the states from the provided array to the system.
        /// This function is called by time integrators every time they modify the Y state.
        /// In some cases, the ChIntegrable object might contain dependent data structures
        /// that might need an update at each change of Y. If so, this function must be overridden.
        public void StateScatter(ChState y, double T) {
            ChState mx = new ChState(GetNcoords_x(), y.GetIntegrable());
            ChStateDelta mv = new ChStateDelta(GetNcoords_v(), y.GetIntegrable());
            mx.PasteClippedMatrix(y, 0, 0, GetNcoords_x(), 1, 0, 0);
            mv.PasteClippedMatrix(y, GetNcoords_x(), 0, GetNcoords_v(), 1, 0, 0);
            StateScatter(mx, mv, T);
        }

        /// Gather from the system the state derivatives in specified array.
        /// Optional: the integrable object might contain last computed state derivative, some integrators might reuse it.
        public void StateGatherDerivative(ref ChStateDelta Dydt) {
            ChStateDelta mv = new ChStateDelta(GetNcoords_v(), Dydt.GetIntegrable());
            ChStateDelta ma = new ChStateDelta(GetNcoords_v(), Dydt.GetIntegrable());
            StateGatherAcceleration(ref ma);
            Dydt.PasteMatrix(mv, 0, 0);
            Dydt.PasteMatrix(ma, GetNcoords_v(), 0);
        }

        public void StateScatterDerivative(ChStateDelta Dydt) {
            ChStateDelta ma = new ChStateDelta(GetNcoords_v(), Dydt.GetIntegrable());
            ma.PasteClippedMatrix(Dydt, GetNcoords_v(), 0, GetNcoords_v(), 1, 0, 0);
            StateScatterAcceleration(ma);
        }

        /// Gather from the system the Lagrange multipliers in specified array.
        /// Optional: the integrable object might contain Lagrange multipliers (reaction in constraints)
       // public void StateGatherReactions(ref ChVectorDynamic<double> L) { }

        /// Scatter the Lagrange multipliers from the provided array to the system.
        /// Optional: the integrable object might contain Lagrange multipliers (reaction in constraints)
        public void StateScatterReactions(ChVectorDynamic<double> L) {
            IntStateScatterReactions(0, L);
        }


        public void StateIncrement(ref ChState y_new,         //< resulting y_new = y + Dy
                               ChState y,       //< initial state y
                               ChStateDelta Dy  //< state increment Dy
                               )
        {
            double ynew;// = y_new[1]; // PROBLEM!  Huge cpu drain
            if (y.GetRows() == this.GetNcoords_x())
            {
                // Incrementing the x part only, user provided only x  in y={x, dx/dt}
                StateIncrementX(ref y_new, y, Dy);
                ynew = y_new[1];
                return;
            }

            if (y.GetRows() == this.GetNcoords_y())
            {
                // Incrementing y in y={x, dx/dt}.
                // PERFORMANCE WARNING! temporary vectors allocated on heap. This is only to support
                // compatibility with 1st order integrators.
                ChState mx = new ChState(this.GetNcoords_x(), y.GetIntegrable());
                ChStateDelta mv = new ChStateDelta(this.GetNcoords_v(), y.GetIntegrable());
                mx.PasteClippedMatrix(y, 0, 0, this.GetNcoords_x(), 1, 0, 0);
                mv.PasteClippedMatrix(y, this.GetNcoords_x(), 0, this.GetNcoords_v(), 1, 0, 0);
                ChStateDelta mDx = new ChStateDelta(this.GetNcoords_v(), y.GetIntegrable());
                ChStateDelta mDv = new ChStateDelta(this.GetNcoords_a(), y.GetIntegrable());
                mDx.PasteClippedMatrix(Dy, 0, 0, this.GetNcoords_v(), 1, 0, 0);
                mDv.PasteClippedMatrix(Dy, this.GetNcoords_v(), 0, this.GetNcoords_a(), 1, 0, 0);
                ChState mx_new = new ChState(this.GetNcoords_x(), y.GetIntegrable());
                ChStateDelta mv_new = new ChStateDelta(this.GetNcoords_v(), y.GetIntegrable());

                StateIncrementX(ref mx_new, mx, mDx);  // increment positions
                mv_new = mv + mDv;                 // increment speeds

                y_new.PasteMatrix(mx_new, 0, 0);
                y_new.PasteMatrix(mv_new, this.GetNcoords_x(), 0);
                return;
            }
          //  throw new ChException("StateIncrement() called with a wrong number of elements");
        }

        public void LoadResidual_Hv(ref ChVectorDynamic<double> R,        //< result: the R residual, R += c*M*v
                                 ChVectorDynamic<double> v,  //< the v vector
                                 double c               //< a scaling factor
                                 )
        {
            //throw new ChException("LoadResidual_Hv() not implemented, implicit integrators cannot be used. ");
        }

        public int GetNcoords_a() { return GetNcoords_v(); }

        public void StateSetup(ref ChState x, ref ChStateDelta v, ref ChStateDelta a) {
            x.Resize(GetNcoords_x());
            v.Resize(GetNcoords_v());
            a.Resize(GetNcoords_a());
        }

        /// Return the number of coordinates in the state Y.
        /// (overrides base - just a fallback to enable using with plain 1st order timesteppers)
        public int GetNcoords_y() { return GetNcoords_x() + GetNcoords_v(); }

        /// Return the number of coordinates in the state increment.
        /// (overrides base - just a fallback to enable using with plain 1st order timesteppers)
        public int GetNcoords_dy() { return GetNcoords_v() + GetNcoords_a(); }

        //  public void StateGatherDerivative(ref ChStateDelta Dydt) { }
        //  public void StateScatterDerivative(ChStateDelta Dydt) { }


        public ChSystem(ChSystem other) : base(other)
        {
            // Required by ChAssembly
            system = this;

            G_acc = other.G_acc;
            end_time = other.end_time;
            step = other.step;
            step_min = other.step_min;
            step_max = other.step_max;
            stepcount = other.stepcount;
            solvecount = other.solvecount;
            setupcount = other.setupcount;
            dump_matrices = other.dump_matrices;
            SetTimestepperType(other.GetTimestepperType());
            tol = other.tol;
            tol_force = other.tol_force;
            maxiter = other.maxiter;

            min_bounce_speed = other.min_bounce_speed;
            max_penetration_recovery_speed = other.max_penetration_recovery_speed;
            max_iter_solver_speed = other.max_iter_solver_speed;
            max_iter_solver_stab = other.max_iter_solver_stab;
            SetSolverType(GetSolverType());
            parallel_thread_number = other.parallel_thread_number;
            use_sleeping = other.use_sleeping;

            ncontacts = other.ncontacts;

            collision_callbacks = other.collision_callbacks;

            last_err = other.last_err;

            RemoveAllProbes();
            RemoveAllControls();
        }

        // Call the main Chrono update
        public virtual void FixedUpdate()
        {
            // TO DO This doesn't work yet
           /* bool try_realtime = true;

            double dt;
            if (try_realtime)
                dt = m_realtime_timer.SuggestSimulationStep(step);
            else
                dt = step;*/

            Time.fixedDeltaTime = (float)step;

            DoStepDynamics(step);
        }

        //
        // PROPERTIES
        //

        /// Sets the time step used for integration (dynamical simulation).
        /// The lower this value, the more precise the simulation. Usually, values
        /// about 0.01 s are enough for simple simulations. It may be modified automatically
        /// by integration methods, if they support automatic time adaption.
        public void SetStep(double m_step)
        {
            if (m_step > 0)
                step = m_step;
        }
        /// Gets the current time step used for the integration (dynamical simulation).
        public double GetStep() { return step; }

        /// Sets the end of simulation.
        public void SetEndTime(double m_end_time) { end_time = m_end_time; }
        /// Gets the end of the simulation
        public double GetEndTime() { return end_time; }

        /// Sets the lower limit for time step (only needed if using
        /// integration methods which support time step adaption).
        public void SetStepMin(double m_step_min)
        {
            if (m_step_min > 0.0)
                step_min = m_step_min;
        }
        /// Gets the lower limit for time step
        public double GetStepMin() { return step_min; }

        /// Sets the upper limit for time step (only needed if using
        /// integration methods which support time step adaption).
        public void SetStepMax(double m_step_max)
        {
            if (m_step_max > step_min)
                step_max = m_step_max;
        }
        /// Gets the upper limit for time step
        public double GetStepMax() { return step_max; }

        /// Set the method for time integration (time stepper type).
        ///   - Suggested for fast dynamics with hard (NSC) contacts: EULER_IMPLICIT_LINEARIZED
        ///   - Suggested for fast dynamics with hard (NSC) contacts and low inter-penetration: EULER_IMPLICIT_PROJECTED
        ///   - Suggested for finite element smooth dynamics: HHT, EULER_IMPLICIT_LINEARIZED
        ///
        /// *Notes*:
        ///   - for more advanced customization, use SetTimestepper()
        ///   - old methods ANITESCU and TASORA were replaced by EULER_IMPLICIT_LINEARIZED and EULER_IMPLICIT_PROJECTED,
        ///     respectively
        public void SetTimestepperType(ChTimestepper.Type type) {
            // Do nothing if changing to a CUSTOM timestepper.
            if (type == ChTimestepper.Type.CUSTOM)
                return;

            // Do nothing, if no change from current typestepper.
            if (type == GetTimestepperType())
                return;

            // Plug in the new required timestepper
            // (the previous will be automatically deallocated thanks to shared pointers)
            switch (type)
            {
                /*case ChTimestepper.Type.EULER_IMPLICIT:
                    timestepper = new ChTimestepperEulerImplicit(this);
                    std::static_pointer_cast<ChTimestepperEulerImplicit>(timestepper).SetMaxiters(4);
                    break;*/
                case ChTimestepper.Type.EULER_IMPLICIT_LINEARIZED:
                    timestepper = new ChTimestepperEulerImplicitLinearized(this);
                    break;
                /* case ChTimestepper::Type::EULER_IMPLICIT_PROJECTED:
                     timestepper = std::make_shared<ChTimestepperEulerImplicitProjected>(this);
                     break;
                 case ChTimestepper::Type::TRAPEZOIDAL:
                     timestepper = std::make_shared<ChTimestepperTrapezoidal>(this);
                     std::static_pointer_cast<ChTimestepperTrapezoidal>(timestepper).SetMaxiters(4);
                     break;
                 case ChTimestepper::Type::TRAPEZOIDAL_LINEARIZED:
                     timestepper = std::make_shared<ChTimestepperTrapezoidalLinearized>(this);
                     std::static_pointer_cast<ChTimestepperTrapezoidalLinearized>(timestepper).SetMaxiters(4);
                     break;
                 case ChTimestepper::Type::HHT:
                     timestepper = std::make_shared<ChTimestepperHHT>(this);
                     std::static_pointer_cast<ChTimestepperHHT>(timestepper).SetMaxiters(4);
                     break;
                 case ChTimestepper::Type::HEUN:
                     timestepper = std::make_shared<ChTimestepperHeun>(this);
                     break;
                 case ChTimestepper::Type::RUNGEKUTTA45:
                     timestepper = std::make_shared<ChTimestepperRungeKuttaExpl>(this);
                     break;
                 case ChTimestepper::Type::EULER_EXPLICIT:
                     timestepper = std::make_shared<ChTimestepperEulerExplIIorder>(this);
                     break;
                 case ChTimestepper::Type::LEAPFROG:
                     timestepper = std::make_shared<ChTimestepperLeapfrog>(this);
                     break;
                 case ChTimestepper::Type::NEWMARK:
                     timestepper = std::make_shared<ChTimestepperNewmark>(this);
                     break;*/
                default:
                    throw new ChException("SetTimestepperType: timestepper not supported");
            }

        }

        /// Get the current method for time integration (time stepper type).
        public ChTimestepper.Type GetTimestepperType()
        {
            return timestepper.GetType();
        }

        /// Set the timestepper object to be used for time integration.
        public void SetTimestepper(ChTimestepper mstepper) { timestepper = mstepper; }

        /// Get the timestepper currently used for time integration
        public ChTimestepper GetTimestepper() { return timestepper; }

        /// Sets outer iteration limit for assembly constraints. When trying to keep constraints together,
        /// the iterative process is stopped if this max.number of iterations (or tolerance) is reached.
        public void SetMaxiter(int m_maxiter) { maxiter = m_maxiter; }
        /// Gets iteration limit for assembly constraints.
        public int GetMaxiter() { return maxiter; }

        /// Sets tolerance (in m) for assembly constraints. When trying to keep constraints together,
        /// the iterative process is stopped if this tolerance (or max.number of iterations ) is reached
        public void SetTol(double tolerance) { tol = tolerance; }
        /// Gets current tolerance for assembly constraints.
        public double GetTol() { return tol; }

        /// Sets tolerance for satisfying constraints at the velocity level.
        /// The tolerance specified here is in fact a tolerance at the force level.
        /// this value is multiplied by the value of the current time step and then
        /// used as a stopping criteria for the iterative speed solver.
        public void SetTolForce(double tolerance) { tol_force = tolerance; }
        /// Return the current value of the tolerance used in the speed solver.
        public double GetTolForce() { return tol_force; }

        /// Change the default composition laws for contact surface materials
        /// (coefficient of friction, cohesion, compliance, etc.)
        public void SetMaterialCompositionStrategy(ChMaterialCompositionStrategy<float> strategy)
        {
            composition_strategy = strategy;
        }

        /// For elastic collisions, with objects that have nonzero
        /// restitution coefficient: objects will rebounce only if their
        /// relative colliding speed is above this threshold. Default 0.15 m/s.
        /// If this is too low, aliasing problems can happen with small high frequency
        /// rebounces, and settling to static stacking might be more difficult.
        public void SetMinBounceSpeed(double mval) { min_bounce_speed = mval; }
        /// Objects will rebounce only if their relative colliding speed is above this threshold.
        public double GetMinBounceSpeed() { return min_bounce_speed; }

        /// For the default stepper, you can limit the speed of exiting from penetration
        /// situations. Usually set a positive value, about 0.1 .. 2 . (as exiting speed, in m/s)
        public void SetMaxPenetrationRecoverySpeed(double mval) { max_penetration_recovery_speed = mval; }
        /// Get the limit on the speed for exiting from penetration situations (for Anitescu stepper)
        public double GetMaxPenetrationRecoverySpeed() { return max_penetration_recovery_speed; }

        /// Choose the solver type, to be used for the simultaneous solution of the constraints
        /// in dynamical simulations (as well as in kinematics, statics, etc.)
        ///   - Suggested solver for speed, but lower precision: SOR
        ///   - Suggested solver for higher precision: BARZILAIBORWEIN or APGD
        ///   - For problems that involve a stiffness matrix: MINRES
        ///
        /// *Notes*:
        ///   - Do not use CUSTOM type, as this type is reserved for external solvers
        ///     (set using SetSolver() and/or SetStabSolver())
        ///   - This function is a shortcut, internally equivalent to two calls to
        ///     SetSolver() and SetStabSolve()
        public virtual void SetSolverType(ChSolver.Type type) {
            // Do nothing if changing to a CUSTOM solver.
            if (type == ChSolver.Type.CUSTOM)
                return;

            descriptor = new ChSystemDescriptor();
            descriptor.SetNumThreads(parallel_thread_number);

            switch (type)
            {
                case ChSolver.Type.SOR:
                    solver_speed = new ChSolverSOR();
                    solver_stab = new ChSolverSOR();
                    break;
                case ChSolver.Type.SYMMSOR:
                    solver_speed = new ChSolverSymmSOR();
                    solver_stab =new ChSolverSymmSOR();
                    break;
                     /*case ChSolver::Type::JACOBI:
                         solver_speed = std::make_shared<ChSolverJacobi>();
                         solver_stab = std::make_shared<ChSolverJacobi>();
                         break;*/
                case ChSolver.Type.SOR_MULTITHREAD:
                    solver_speed = new ChSolverSORmultithread("speedSolver", parallel_thread_number);
                    solver_stab = new ChSolverSORmultithread("posSolver", parallel_thread_number);
                    break;
                     /*case ChSolver::Type::PMINRES:
                         solver_speed = std::make_shared<ChSolverPMINRES>();
                         solver_stab = std::make_shared<ChSolverPMINRES>();
                         break;
                     case ChSolver::Type::BARZILAIBORWEIN:
                         solver_speed = std::make_shared<ChSolverBB>();
                         solver_stab = std::make_shared<ChSolverBB>();
                         break;
                     case ChSolver::Type::PCG:
                         solver_speed = std::make_shared<ChSolverPCG>();
                         solver_stab = std::make_shared<ChSolverPCG>();
                         break;
                     case ChSolver::Type::APGD:
                         solver_speed = std::make_shared<ChSolverAPGD>();
                         solver_stab = std::make_shared<ChSolverAPGD>();
                         break;
                     case ChSolver::Type::MINRES:
                         solver_speed = std::make_shared<ChSolverMINRES>();
                         solver_stab = std::make_shared<ChSolverMINRES>();
                         break;
                     default:
                         solver_speed = std::make_shared<ChSolverSymmSOR>();
                         solver_stab = std::make_shared<ChSolverSymmSOR>();
                         break;*/
            }
        }

        /// Gets the current solver type.
        public ChSolver.Type GetSolverType() { return solver_speed.GetType(); }

        /// When using an iterative solver (es. SOR) set the maximum number of iterations.
        /// The higher the iteration number, the more precise the simulation (but more CPU time).
        public void SetMaxItersSolverSpeed(int mval) { max_iter_solver_speed = mval; }
        /// Current maximum number of iterations, if using an iterative solver.
        public int GetMaxItersSolverSpeed() { return max_iter_solver_speed; }

        /// When using an iterative solver (es. SOR) and a timestepping method
        /// requiring post-stabilization (e.g., EULER_IMPLICIT_PROJECTED), set the
        /// the maximum number of stabilization iterations. The higher the iteration
        /// number, the more precise the simulation (but more CPU time).
        public void SetMaxItersSolverStab(int mval) { max_iter_solver_stab = mval; }
        /// Current maxi. number of iterations, if using an iterative solver for stabilization.
        public int GetMaxItersSolverStab() { return max_iter_solver_stab; }

        /// If you want to easily turn ON/OFF the warm starting feature of both iterative solvers
        /// (the one for speed and the other for pos.stabilization) you can simply use the
        /// following instead of accessing them directly with GetSolver() and GetStabSolver()
        public void SetSolverWarmStarting(bool usewarm = true) {
            ChIterativeSolver iter_solver_speed = new ChIterativeSolver();
            if (iter_solver_speed == (ChIterativeSolver)(solver_speed)) {
                iter_solver_speed.SetWarmStart(usewarm);
            }
            ChIterativeSolver iter_solver_stab = new ChIterativeSolver();
            if (iter_solver_stab == (ChIterativeSolver)(solver_stab)) {
                iter_solver_stab.SetWarmStart(usewarm);
            }
        }
        /// Tell if the warm starting is enabled for the speed solver, (if iterative type).
        public bool GetSolverWarmStarting() {
            ChIterativeSolver iter_solver_speed = new ChIterativeSolver();
            if (iter_solver_speed == (ChIterativeSolver)(solver_speed))
            {
                return iter_solver_speed.GetWarmStart();
            }

            return false;
        }

        /// If you want to easily adjust the omega overrelaxation parameter of both iterative solvers
        /// (the one for speed and the other for position stabilization) you can simply use the
        /// following instead of accessing them directly with GetSolver() and GetStabSolver().
        /// Note, usually a good omega for Jacobi or GPU solver is 0.2; for other iter.solvers can be up to 1.0
        public void SetSolverOverrelaxationParam(double momega = 1.0) {
            ChIterativeSolver iter_solver_speed = new ChIterativeSolver();
            if (iter_solver_speed == (ChIterativeSolver)(solver_speed)) {
                iter_solver_speed.SetOmega(momega);
            }
            ChIterativeSolver iter_solver_stab = new ChIterativeSolver();
            if (iter_solver_stab == (ChIterativeSolver)(solver_stab)) {
                iter_solver_stab.SetOmega(momega);
            }
        }
        /// Tell the omega overrelaxation factor for the speed solver, (if iterative type).
        public double GetSolverOverrelaxationParam() {
            ChIterativeSolver iter_solver_speed = new ChIterativeSolver();
            if (iter_solver_speed == (ChIterativeSolver)(solver_speed)) {
                return iter_solver_speed.GetOmega();
            }
            return 1.0;
        }

        /// If you want to easily adjust the 'sharpness lambda' parameter of both iterative solvers
        /// (the one for speed and the other for pos.stabilization) you can simply use the
        /// following instead of accessing them directly with GetSolver() and GetStabSolver().
        /// Note, usually a good sharpness value is in 1..0.8 range (the lower, the more it helps exact
        /// convergence, but overall convergence gets also much slower so maybe better to tolerate some error)
        public void SetSolverSharpnessParam(double momega = 1.0) {
            ChIterativeSolver iter_solver_speed = new ChIterativeSolver();
            if (iter_solver_speed == (ChIterativeSolver)(solver_speed)) {
                iter_solver_speed.SetSharpnessLambda(momega);
            }
            ChIterativeSolver iter_solver_stab = new ChIterativeSolver();
            if (iter_solver_stab == (ChIterativeSolver)(solver_stab)) {
                iter_solver_stab.SetSharpnessLambda(momega);
            }
        }
        /// Tell the 'sharpness lambda' factor for the speed solver, (if iterative type).
        public double GetSolverSharpnessParam() {
            ChIterativeSolver iter_solver_speed = new ChIterativeSolver();
            if (iter_solver_speed == (ChIterativeSolver)(solver_speed)) {
                return iter_solver_speed.GetSharpnessLambda();
            }
            return 1.0;
        }

        /// Instead of using SetSolverType(), you can create your own custom solver (inherited from ChSolver)
        /// and plug it into the system using this function. 
        public virtual void SetStabSolver(ChSolver newsolver) {
            solver_stab = newsolver;
        }

        /// Access directly the stabilization solver, configured to be used for the stabilization
        /// of constraints (solve delta positions).
        public virtual ChSolver GetStabSolver() {
            // In case the solver is iterative, pre-configure it with the max. number of
            // iterations and with the convergence tolerance (convert the user-specified
            // tolerance for forces into a tolerance for impulses).
            ChIterativeSolver iter_solver = new ChIterativeSolver();
            if (iter_solver == (ChIterativeSolver)(solver_stab)) {
                iter_solver.SetMaxIterations(GetMaxItersSolverSpeed());
                iter_solver.SetTolerance(tol_force * step);
            }

            return solver_stab;
        }

        /// Instead of using SetSolverType(), you can create your own custom solver (suffice it is inherited
        /// from ChSolver) and plug it into the system using this function.
        public virtual void SetSolver(ChSolver newsolver) {
            solver_speed = newsolver;
        }

        /// Access directly the solver, configured to be used for the main differential
        /// inclusion problem (on speed-impulses).
        public virtual ChSolver GetSolver() {
            // In case the solver is iterative, pre-configure it with the max. number of
            // iterations and with the convergence tolerance (convert the user-specified
            // tolerance for forces into a tolerance for impulses).
            ChIterativeSolver iter_solver = new ChIterativeSolver();
            if (iter_solver == (ChIterativeSolver)(solver_speed)) {
                iter_solver.SetMaxIterations(GetMaxItersSolverSpeed());
                iter_solver.SetTolerance(tol_force * step);
            }

            return solver_speed;
        }

        /// Instead of using the default 'system descriptor', you can create your own custom descriptor
        /// (inherited from ChSystemDescriptor) and plug it into the system using this function.
        public void SetSystemDescriptor(ChSystemDescriptor newdescriptor) {
            //Debug.Assert(newdescriptor);
            descriptor = newdescriptor;
        }

        /// Access directly the 'system descriptor'.
        public ChSystemDescriptor GetSystemDescriptor() { return descriptor; }

        /// Changes the number of parallel threads (by default is n.of cores).
        /// Note that not all solvers use parallel computation.
        /// If you have a N-core processor, this should be set at least =N for maximum performance.
        public void SetParallelThreadNumber(int mthreads = 2) {
            if (mthreads < 1)
                mthreads = 1;

            parallel_thread_number = mthreads;

            descriptor.SetNumThreads(mthreads);

            /* if (solver_speed.GetType() == ChSolver.Type.SOR_MULTITHREAD)
             {
                 std::static_pointer_cast<ChSolverSORmultithread>(solver_speed).ChangeNumberOfThreads(mthreads);
                 std::static_pointer_cast<ChSolverSORmultithread>(solver_stab).ChangeNumberOfThreads(mthreads);
             }*/
        }
        /// Get the number of parallel threads.
        /// Note that not all solvers use parallel computation.
        public int GetParallelThreadNumber() { return parallel_thread_number; }

        /// Sets the G (gravity) acceleration vector, affecting all the bodies in the system.
        public void Set_G_acc(ChVector m_acc) { G_acc = m_acc; }
        /// Gets the G (gravity) acceleration vector affecting all the bodies in the system.
        public ChVector Get_G_acc() { return G_acc; }

        /// Initial system setup before analysis.
        /// This function must be called once the system construction is completed.
        public override void SetupInitial() {
            for (int i = 0; i < bodylist.Count; i++)
            {
                bodylist[i].SetupInitial();
            }
            for (int ip = 0; ip < linklist.Count; ++ip)
            {
                linklist[ip].SetupInitial();
            }
            /* for (int ip = 0; ip < meshlist.Count; ++ip)
             {
                 meshlist[ip].SetupInitial();
             }*/
            for (int ip = 0; ip < otherphysicslist.Count; ++ip)
            {
                otherphysicslist[ip].SetupInitial();
            }
            this.update();
        }



        //
        // DATABASE HANDLING.
        //

        /// Removes all bodies/marker/forces/links/contacts,
        /// also resets timers and events.
        public new void Clear() {
            // first the parent class data...
            base.Clear();

            contact_container.RemoveAllContacts();

            RemoveAllProbes();
            RemoveAllControls();

            // ResetTimers();
        }

        /// Return the contact method supported by this system.
        /// Bodies added to this system must be compatible.
        public abstract ChMaterialSurface.ContactMethod GetContactMethod();

        /// Create and return the pointer to a new body.
        /// The returned body is created with a contact model consistent with the type
        /// of this ChSystem and with the collision system currently associated with this
        /// ChSystem.  Note that the body is *not* attached to this system.
        public abstract ChBody NewBody();

        /// Create and return the pointer to a new body with auxiliary reference frame.
        /// The returned body is created with a contact model consistent with the type
        /// of this ChSystem and with the collision system currently associated with this
        /// ChSystem.  Note that the body is *not* attached to this system.
        public abstract ChBodyAuxRef NewBodyAuxRef();

        /// Attach a probe to this system.
        public void AddProbe(ChProbe newprobe) {
            // newprobe.SetSystem (this);
            probelist.Add(newprobe);
        }

        /// Attach a control to this system.
        public void AddControls(ChControls newcontrols) {
            // newcontrols.SetSystem (this);
            controlslist.Add(newcontrols);
        }

        /// Remove all probes from this system.
        public void RemoveAllProbes() {
            probelist.Clear();
        }

        /// Remove all controls from this system.
        public void RemoveAllControls() {
            controlslist.Clear();
        }

        /// Replace the contact container.
        public virtual void SetContactContainer(ChContactContainer container) {
            contact_container = container;
            contact_container.SetSystem(this);
        }

        /// Get the contact container
        public ChContactContainer GetContactContainer() { return contact_container; }

        /// Given inserted markers and links, restores the
        /// pointers of links to markers given the information
        /// about the marker IDs. Will be made obsolete in future with new serialization systems.
        public void Reference_LM_byID() {
            // TO DO
        }


        //
        // STATISTICS
        //

        /// Gets the number of contacts.
        public int GetNcontacts() {
            return contact_container.GetNcontacts();
        }

        /// Return the time (in seconds) spent for computing the time step.
        public virtual double GetTimerStep() { return timer_step.GetValue();
        }
        /// Return the time (in seconds) for time integration, within the time step.
        public virtual double GetTimerAdvance() { return timer_advance.GetValue(); }

        /// Return the time (in seconds) for the solver, within the time step.
        /// Note that this time excludes any calls to the solver's Setup function.
        public virtual double GetTimerSolver() { return timer_solver.GetValue(); }
        /// Return the time (in seconds) for the solver Setup phase, within the time step.
        public virtual double GetTimerSetup() { return timer_setup.GetValue(); }
        /// Return the time (in seconds) for calculating/loading Jacobian information, within the time step.
        public virtual double GetTimerJacobian() { return timer_jacobian.GetValue(); }
        /// Return the time (in seconds) for runnning the collision detection step, within the time step.
        public virtual double GetTimerCollision() { return timer_collision.GetValue(); }
        /// Return the time (in seconds) for updating auxiliary data, within the time step.
        public virtual double GetTimerUpdate() { return timer_update.GetValue(); }

        /// Return the time (in seconds) for broadphase collision detection, within the time step.
      //  public double GetTimerCollisionBroad() { return collision_system.GetTimerCollisionBroad(); }

        /// Return the time (in seconds) for narrowphase collision detection, within the time step.
       // public double GetTimerCollisionNarrow() { return collision_system.GetTimerCollisionNarrow(); }

        /// Resets the timers.
        public void ResetTimers()
        {
            timer_step.reset();
            timer_advance.reset();
            timer_solver.reset();
            timer_setup.reset();
            timer_jacobian.reset();
            timer_collision.reset();
            timer_update.reset();
            collision_system.ResetTimers();
        }

        /// Pushes all ChConstraints and ChVariables contained in links, bodies, etc.
        /// into the system descriptor.
        protected virtual void DescriptorPrepareInject(ref ChSystemDescriptor mdescriptor) {
            mdescriptor.BeginInsertion();  // This resets the vectors of constr. and var. pointers.

            InjectConstraints(ref mdescriptor);
            InjectVariables(ref mdescriptor);
            InjectKRMmatrices(ref mdescriptor);

            mdescriptor.EndInsertion();
        }

        //
        // PHYSICS ITEM INTERFACE
        //

        /// Counts the number of bodies and links.
        /// Computes the offsets of object states in the global state.
        /// Assumes that offset_x, offset_w, and offset_L are already set
        /// as starting point for offsetting all the contained sub objects.
        public override void Setup() {
            // inherit the parent class (compute offsets of bodies, links, etc.)
            base.Setup();

            // also compute offsets for contact container
            {
                contact_container.SetOffset_L(offset_L + ndoc_w);

                ndoc_w += contact_container.GetDOC();
                ndoc_w_C += contact_container.GetDOC_c();
                ndoc_w_D += contact_container.GetDOC_d();
            }

            ndoc = ndoc_w + nbodies;          // number of constraints including quaternion constraints.
            nsysvars = ncoords + ndoc;        // total number of variables (coordinates + lagrangian multipliers)
            nsysvars_w = ncoords_w + ndoc_w;  // total number of variables (with 6 dof per body)

            ndof = ncoords - ndoc;  // number of degrees of freedom (approximate - does not consider constr. redundancy, etc)
        }

        /// Updates all the auxiliary data and children of
        /// bodies, forces, links, given their current state.
        public override void update(bool update_assets = true) {

            timer_update.start();  // Timer for profiling

            // Executes the "forUpdate" in all controls of controlslist
            ExecuteControlsForUpdate();

            // Inherit parent class (recursively update sub objects bodies, links, etc)
            base.update(update_assets);

            // Update all contacts, if any
            contact_container.update(ChTime, update_assets);

            timer_update.stop();
        }

        // (Overload interfaces for global state vectors, see ChPhysicsItem for comments.)
        // (The following must be overload because there may be ChContactContainer objects in addition to base ChAssembly)
        public override void IntStateGather(int off_x,
                                        ref ChState x,
                                        int off_v,
                                        ref ChStateDelta v,
                                        ref double T)
        {
            int displ_x = off_x - offset_x;
            int displ_v = off_v - offset_w;

            // Inherit: operate parent method on sub objects (bodies, links, etc.)
            base.IntStateGather(off_x, ref x, off_v, ref v, ref T);
            // Use also on contact container:
            contact_container.IntStateGather(displ_x + contact_container.GetOffset_x(), ref x,
                                              displ_v + contact_container.GetOffset_w(), ref v, ref T);
        }

        public override void IntStateScatter(int off_x,
                                     ChState x,
                                     int off_v,
                                     ChStateDelta v,
                                     double T)
        {
            int displ_x = off_x - offset_x;
            int displ_v = off_v - offset_w;

            // Inherit: operate parent method on sub objects (bodies, links, etc.)
            base.IntStateScatter(off_x, x, off_v, v, T);
            // Use also on contact container:
            contact_container.IntStateScatter(displ_x + contact_container.GetOffset_x(), x,
                                               displ_v + contact_container.GetOffset_w(), v, T);
        }
        public override void IntStateGatherAcceleration(int off_a, ref ChStateDelta a) {
            int displ_a = off_a - offset_w;

            // Inherit: operate parent method on sub objects (bodies, links, etc.)
            base.IntStateGatherAcceleration(off_a, ref a);
            // Use also on contact container:
            contact_container.IntStateGatherAcceleration(displ_a + contact_container.GetOffset_w(), ref a);
        }

        /// From state derivative (acceleration) to system, sometimes might be needed
        public override void IntStateScatterAcceleration(int off_a, ChStateDelta a) {
            int displ_a = off_a - offset_w;

            // Inherit: operate parent method on sub objects (bodies, links, etc.)
            base.IntStateScatterAcceleration(off_a, a);
            // Use also on contact container:
            contact_container.IntStateScatterAcceleration(displ_a + contact_container.GetOffset_w(), a);
        }

        public override void IntStateGatherReactions(int off_L, ref ChVectorDynamic<double> L)
        {
            int displ_L = off_L - offset_L;

            // Inherit: operate parent method on sub objects (bodies, links, etc.)
            base.IntStateGatherReactions(off_L, ref L);
            // Use also on contact container:
            contact_container.IntStateGatherReactions(displ_L + contact_container.GetOffset_L(), ref L);
        }

        /// From reaction forces to system, ex. store last computed reactions in ChLink objects for plotting etc.
        public override void IntStateScatterReactions(int off_L, ChVectorDynamic<double> L) {
            int displ_L = off_L - offset_L;

            // Inherit: operate parent method on sub objects (bodies, links, etc.)
            base.IntStateScatterReactions(off_L, L);
            // Use also on contact container:
            contact_container.IntStateScatterReactions(displ_L + contact_container.GetOffset_L(), L);
        }

        public override void IntStateIncrement(int off_x,
                                               ref ChState x_new,
                                               ChState x,
                                               int off_v,
                                               ChStateDelta Dv)
        {
            int displ_x = (int)(off_x - offset_x);
            int displ_v = (int)(off_v - offset_w);

            // Inherit: operate parent method on sub objects (bodies, links, etc.)
            base.IntStateIncrement(off_x, ref x_new, x, (int)off_v, Dv);
            // Use also on contact container:
            contact_container.IntStateIncrement((int)(displ_x + contact_container.GetOffset_x()), ref x_new, x,
                                                 (int)(displ_v + contact_container.GetOffset_w()), Dv);
        }

        public override void IntLoadResidual_F(int off, ref ChVectorDynamic<double> R, double c) {
            int displ_v = off - offset_w;

            // Inherit: operate parent method on sub objects (bodies, links, etc.)
            base.IntLoadResidual_F(off, ref R, c);
            // Use also on contact container:
            contact_container.IntLoadResidual_F(displ_v + contact_container.GetOffset_w(), ref R, c);
        }

        public override void IntLoadResidual_Mv(int off,
                                                ref ChVectorDynamic<double> R,
                                                ChVectorDynamic<double> w,
                                                double c) {
            int displ_v = off - offset_w;

            // Inherit: operate parent method on sub objects (bodies, links, etc.)
            base.IntLoadResidual_Mv(off, ref R, w, c);
            // Use also on contact container:
            contact_container.IntLoadResidual_Mv(displ_v + contact_container.GetOffset_w(), ref R, w, c);
        }
        public override void IntLoadResidual_CqL(int off_L,
                                                 ref ChVectorDynamic<double> R,
                                                 ChVectorDynamic<double> L,
                                                 double c) {
            int displ_L = off_L - offset_L;

            // Inherit: operate parent method on sub objects (bodies, links, etc.)
            base.IntLoadResidual_CqL(off_L, ref R, L, c);
            // Use also on contact container:
            contact_container.IntLoadResidual_CqL(displ_L + contact_container.GetOffset_L(), ref R, L, c);
        }
        public override void IntLoadConstraint_C(int off_L,
                                                 ref ChVectorDynamic<double> Qc,
                                                 double c,
                                                 bool do_clamp,
                                                 double recovery_clamp) {
            int displ_L = off_L - offset_L;

            // Inherit: operate parent method on sub objects (bodies, links, etc.)
            base.IntLoadConstraint_C(off_L, ref Qc, c, do_clamp, recovery_clamp);
            // Use also on contact container:
            contact_container.IntLoadConstraint_C(displ_L + contact_container.GetOffset_L(), ref Qc, c, do_clamp, recovery_clamp);
        }
        public override void IntLoadConstraint_Ct(int off_L, ref ChVectorDynamic<double> Qc, double c) {
            int displ_L = off_L - offset_L;

            // Inherit: operate parent method on sub objects (bodies, links, etc.)
            base.IntLoadConstraint_Ct(off_L, ref Qc, c);
            // Use also on contact container:
            contact_container.IntLoadConstraint_Ct(displ_L + contact_container.GetOffset_L(), ref Qc, c);
        }
        public override void IntToDescriptor(int off_v,
                                             ChStateDelta v,
                                             ChVectorDynamic<double> R,
                                             int off_L,
                                             ChVectorDynamic<double> L,
                                             ChVectorDynamic<double> Qc) {
            int displ_L = off_L - offset_L;
            int displ_v = off_v - offset_w;

            // Inherit: operate parent method on sub objects (bodies, links, etc.)
            base.IntToDescriptor(off_v, v, R, off_L, L, Qc);
            // Use also on contact container:
            contact_container.IntToDescriptor(displ_v + contact_container.GetOffset_w(), v, R,
                                               displ_L + contact_container.GetOffset_L(), L, Qc);

        }
        public override void IntFromDescriptor(int off_v,
                                               ref ChStateDelta v,
                                               int off_L,
                                               ref ChVectorDynamic<double> L) {
            int displ_L = off_L - offset_L;
            int displ_v = off_v - offset_w;

            // Inherit: operate parent method on sub objects (bodies, links, etc.)
            base.IntFromDescriptor(off_v, ref v, off_L, ref L);

            // Use also on contact container:
            contact_container.IntFromDescriptor(displ_v + contact_container.GetOffset_w(), ref v,
                                                 displ_L + contact_container.GetOffset_L(), ref L);
        }

        public override void InjectVariables(ref ChSystemDescriptor mdescriptor) {
            // Inherit: operate parent method on sub objects (bodies, links, etc.)
            base.InjectVariables(ref mdescriptor);
            // Use also on contact container:
            contact_container.InjectVariables(ref mdescriptor);
        }

        public override void InjectConstraints(ref ChSystemDescriptor mdescriptor) {
            // Inherit: operate parent method on sub objects (bodies, links, etc.)
            base.InjectConstraints(ref mdescriptor);
            // Use also on contact container:
            contact_container.InjectConstraints(ref mdescriptor);
        }
        public override void ConstraintsLoadJacobians() {
            // Inherit: operate parent method on sub objects (bodies, links, etc.)
            base.ConstraintsLoadJacobians();
            // Use also on contact container:
            contact_container.ConstraintsLoadJacobians();
        }

        public override void InjectKRMmatrices(ref ChSystemDescriptor mdescriptor) {
            // Inherit: operate parent method on sub objects (bodies, links, etc.)
            base.InjectKRMmatrices(ref mdescriptor);
            // Use also on contact container:
            contact_container.InjectKRMmatrices(ref mdescriptor);
        }
        public override void KRMmatricesLoad(double Kfactor, double Rfactor, double Mfactor) {
            // Inherit: operate parent method on sub objects (bodies, links, etc.)
            base.KRMmatricesLoad(Kfactor, Rfactor, Mfactor);
            // Use also on contact container:
            contact_container.KRMmatricesLoad(Kfactor, Rfactor, Mfactor);
        }

        // Old bookkeeping system 
        public override void VariablesFbReset() {
            // Inherit: operate parent method on sub objects (bodies, links, etc.)
            base.VariablesFbReset();
            // Use also on contact container:
            contact_container.VariablesFbReset();
        }
        public override void VariablesFbLoadForces(double factor = 1) {
            // Inherit: operate parent method on sub objects (bodies, links, etc.)
            base.VariablesFbLoadForces();
            // Use also on contact container:
            contact_container.VariablesFbLoadForces();
        }

        public override void VariablesQbLoadSpeed() {
            // Inherit: operate parent method on sub objects (bodies, links, etc.)
            base.VariablesQbLoadSpeed();
            // Use also on contact container:
            contact_container.VariablesQbLoadSpeed();
        }

        public override void VariablesFbIncrementMq() {
            // Inherit: operate parent method on sub objects (bodies, links, etc.)
            base.VariablesQbLoadSpeed();
            // Use also on contact container:
            contact_container.VariablesQbLoadSpeed();
        }
        public override void VariablesQbSetSpeed(double step = 0) {
            // Inherit: operate parent method on sub objects (bodies, links, etc.)
            base.VariablesQbSetSpeed(step);
            // Use also on contact container:
            contact_container.VariablesQbSetSpeed(step);
        }
        public override void VariablesQbIncrementPosition(double dt_step) {
            // Inherit: operate parent method on sub objects (bodies, links, etc.)
            base.VariablesQbIncrementPosition(dt_step);
            // Use also on contact container:
            contact_container.VariablesQbIncrementPosition(dt_step);
        }
        public override void ConstraintsBiReset() {
            // Inherit: operate parent method on sub objects (bodies, links, etc.)
            base.ConstraintsBiReset();
            // Use also on contact container:
            contact_container.ConstraintsBiReset();
        }
        public override void ConstraintsBiLoad_C(double factor = 1, double recovery_clamp = 0.1, bool do_clamp = false) {
            // Inherit: operate parent method on sub objects (bodies, links, etc.)
            base.ConstraintsBiLoad_C(factor, recovery_clamp, do_clamp);
            // Use also on contact container:
            contact_container.ConstraintsBiLoad_C(factor, recovery_clamp, do_clamp);
        }
        public override void ConstraintsBiLoad_Ct(double factor = 1) {
            // Inherit: operate parent method on sub objects (bodies, links, etc.)
            base.ConstraintsBiLoad_Ct(factor);
            // Use also on contact container:
            contact_container.ConstraintsBiLoad_Ct(factor);
        }
        public override void ConstraintsBiLoad_Qc(double factor = 1) {
            // Inherit: operate parent method on sub objects (bodies, links, etc.)
            base.ConstraintsBiLoad_Qc(factor);
            // Use also on contact container:
            contact_container.ConstraintsBiLoad_Qc(factor);
        }
        public override void ConstraintsFbLoadForces(double factor = 1) {
            // Inherit: operate parent method on sub objects (bodies, links, etc.)
            base.ConstraintsFbLoadForces(factor);
            // Use also on contact container:
            contact_container.ConstraintsFbLoadForces(factor);
        }
        public override void ConstraintsFetch_react(double factor = 1) {
            // Inherit: operate parent method on sub objects (bodies, links, etc.)
            base.ConstraintsFetch_react(factor);
            // Use also on contact container:
            contact_container.ConstraintsFetch_react(factor);
        }

        //
        // TIMESTEPPER INTERFACE
        //

        /// Tells the number of position coordinates x in y = {x, v}
        public int GetNcoords_x() { return GetNcoords(); }

        /// Tells the number of speed coordinates of v in y = {x, v} and  dy/dt={v, a}
        public int GetNcoords_v() { return GetNcoords_w(); }

        /// Tells the number of lagrangian multipliers (constraints)
        public int GetNconstr() { return GetNdoc_w(); }

        public bool StateSolve(ref ChStateDelta dydt,              //< result: computed dydt
                            ref ChVectorDynamic<double> L,            //< result: computed lagrangian multipliers, if any
                            ChState y,                //< current state y
                            double T,                  //< current time T
                            double dt,                 //< timestep (if needed, ex. in DVI)
                            bool force_state_scatter = true  //< if false, y and T are not scattered to the system
                            )
        {
            ChState mx = new ChState(GetNcoords_x(), y.GetIntegrable());
            ChStateDelta mv = new ChStateDelta(GetNcoords_v(), y.GetIntegrable());
            mx.PasteClippedMatrix(y, 0, 0, GetNcoords_x(), 1, 0, 0);
            mv.PasteClippedMatrix(y, GetNcoords_x(), 0, GetNcoords_v(), 1, 0, 0);
            ChStateDelta ma = new ChStateDelta(GetNcoords_v(), y.GetIntegrable());

            // Solve with custom II order solver
            if (!StateSolveA(ref ma, ref L, mx, mv, T, dt, force_state_scatter))
            {
                return false;
            }

            dydt.PasteMatrix(mv, 0, 0);
            dydt.PasteMatrix(ma, GetNcoords_v(), 0);

            return true;

        }

        public void StateGather(ref ChState x, ref ChStateDelta v, ref double T) {
            IntStateGather(0, ref x, 0, ref v, ref T);
        }

        /// From state Y={x,v} to system.
        public void StateScatter(ChState x, ChStateDelta v, double T) {
            // m_intergral.StateScatter(x, T);
            IntStateScatter(0, x, 0, v, T);

            //update();  //***TODO*** optimize because maybe IntStateScatter above might have already called Update?
        }

        /// From system to state derivative (acceleration), some timesteppers might need last computed accel.
        public void StateGatherAcceleration(ref ChStateDelta a) {
            IntStateGatherAcceleration(0, ref a);
        }

        /// From state derivative (acceleration) to system, sometimes might be needed
        public void StateScatterAcceleration(ChStateDelta a) {
            IntStateScatterAcceleration(0, a);
        }

        /// From system to reaction forces (last computed) - some timestepper might need this
        public void StateGatherReactions(ref ChVectorDynamic<double> L) {
            IntStateGatherReactions(0, ref L);
        }


        /// Perform x_new = x + dx    for x in    Y = {x, dx/dt}
        /// It takes care of the fact that x has quaternions, dx has angular vel etc.
        /// NOTE: the system is not updated automatically after the state increment, so one might
        /// need to call StateScatter() if needed.
        public void StateIncrementX(ref ChState x_new,         //< resulting x_new = x + Dx
                                             ChState x,       //< initial state x
                                             ChStateDelta Dx  //< state increment Dx
                                 ) {
            IntStateIncrement(0, ref x_new, x, 0, Dx);
        }

        public bool StateSolveA(ref ChStateDelta Dvdt,              //< result: computed a for a=dv/dt
                             ref ChVectorDynamic<double> L,            //< result: computed lagrangian multipliers, if any
                             ChState x,                //< current state, x
                             ChStateDelta v,           //< current state, v
                             double T,                  //< current time T
                             double dt,                 //< timestep (if needed)
                             bool force_state_scatter = true  //< if false, x,v and T are not scattered to the system
                             )
        {
            if (force_state_scatter)
                StateScatter(x, v, T);

            ChVectorDynamic<double> R = new ChVectorDynamic<double>(GetNcoords_v());
            ChVectorDynamic<double> Qc = new ChVectorDynamic<double>(GetNconstr());
            const double Delta = 1e-6;

            LoadResidual_F(ref R, 1.0);

            LoadConstraint_C(ref Qc, -2.0 / (Delta * Delta));

            // numerical differentiation to get the Qc term in constraints
            ChStateDelta dx = new ChStateDelta(v);
            dx *= Delta;
            ChState xdx = new ChState(x.GetRows(), this);

            StateIncrement(ref xdx, x, dx);
            StateScatter(xdx, v, T + Delta);
            LoadConstraint_C(ref Qc, 1.0 / (Delta * Delta));

            StateIncrement(ref xdx, x, -dx);
            StateScatter(xdx, v, T - Delta);
            LoadConstraint_C(ref Qc, 1.0 / (Delta * Delta));

            StateScatter(x, v, T);  // back to original state

            bool success = StateSolveCorrection(ref Dvdt, ref L, R, Qc, 1.0, 0, 0, x, v, T, false, true);

            return success;
        }

        public bool StateSolveCorrection(
                                     ref ChStateDelta Dy,
                                     ref ChVectorDynamic<double> L,
                                     ChVectorDynamic<double> R,
                                     ChVectorDynamic<double> Qc,
                                     double a,
                                     double b,
                                     ChState y,
                                     double T,
                                     double dt,
                                     bool force_state_scatter = true,
                                     bool force_setup = true)
        { return true; }


        /// Assuming a DAE of the form
        ///       M*a = F(x,v,t) + Cq'*L
        ///       C(x,t) = 0
        /// this function computes the solution of the change Du (in a or v or x) for a Newton
        /// iteration within an implicit integration scheme.
        ///  |Du| = [ G   Cq' ]^-1 * | R |
        ///  |DL|   [ Cq  0   ]      | Qc|
        /// for residual R and  G = [ c_a*M + c_v*dF/dv + c_x*dF/dx ]
        /// This function returns true if successful and false otherwise.
        public bool StateSolveCorrection(
            ref ChStateDelta Dv,                    //< result: computed Dv
            ref ChVectorDynamic<double> L,          //< result: computed lagrangian multipliers, if any
            ChVectorDynamic<double> R,              //< the R residual
            ChVectorDynamic<double> Qc,             //< the Qc residual
            double c_a,                             //< the factor in c_a*M
            double c_v,                             //< the factor in c_v*dF/dv
            double c_x,                             //< the factor in c_x*dF/dv
            ChState x,                              //< current state, x part
            ChStateDelta v,                         //< current state, v part
            double T,                               //< current time T
            bool force_state_scatter,        //< if false, x,v and T are not scattered to the system
            bool force_setup                 //< if true, call the solver's Setup() function
            )
        {
            // m_intergral.StateSolveCorrection(ref Dv, ref L, R, Qc, c_a, c_v, c_x, x, v, T, force_state_scatter, force_setup);

            if (force_state_scatter)
                StateScatter(x, v, T);

            // R and Qc vectors  -. solver sparse solver structures  (also sets L and Dv to warmstart)
            IntToDescriptor(0, Dv, R, 0, L, Qc);

            // If the solver's Setup() must be called or if the solver's Solve() requires it,
            // fill the sparse system structures with information in G and Cq.
            if (force_setup || GetSolver().SolveRequiresMatrix())
            {
                timer_jacobian.start();

                // Cq  matrix
                ConstraintsLoadJacobians();

                // G matrix: M, K, R components
                if (c_a != 0 || c_v != 0 || c_x != 0)
                    KRMmatricesLoad(-c_x, -c_v, c_a);

                // For ChVariable objects without a ChKblock, just use the 'a' coefficient
                descriptor.SetMassFactor(c_a);

                timer_jacobian.stop();
            }

            // If indicated, first perform a solver setup.
            // Return 'false' if the setup phase fails.
            if (force_setup)
            {
                timer_setup.start();
                bool success = GetSolver().Setup(ref descriptor);
                timer_setup.stop();
                setupcount++;
                if (!success)
                    return false;
            }

            // Solve the problem
            // The solution is scattered in the provided system descriptor
            timer_solver.start();           
            GetSolver().Solve(ref descriptor);
            timer_solver.stop();


            // Dv and L vectors  <-- sparse solver structures
            IntFromDescriptor(0, ref Dv, 0, ref L);

            solvecount++;

            return true;
        }

        /// Increment a vector R with the term c*F:
        ///    R += c*F
        public void LoadResidual_F(ref ChVectorDynamic<double> R,  //< result: the R residual, R += c*F
                                            double c                        //< a scaling factor
                                )
        {
            // m_intergral.LoadResidual_F(ref R, c);
            IntLoadResidual_F(0, ref R, c);
        }

        /// Increment a vector R with a term that has M multiplied a given vector w:
        ///    R += c*M*w
        public void LoadResidual_Mv(ref ChVectorDynamic<double> R,        //< result: the R residual, R += c*M*v
                                             ChVectorDynamic<double> w,            //< the w vector
                                             double c                              //< a scaling factor
                                     ) {
            IntLoadResidual_Mv(0, ref R, w, c);
        }

        /// Increment a vectorR with the term Cq'*L:
        ///    R += c*Cq'*L
        public void LoadResidual_CqL(ref ChVectorDynamic<double> R,        //< result: the R residual, R += c*Cq'*L
                                              ChVectorDynamic<double> L,            //< the L vector
                                              double c                              //< a scaling factor
                                      ) {
            IntLoadResidual_CqL(0, ref R, L, c);
        }

        /// Increment a vector Qc with the term C:
        ///    Qc += c*C
        public void LoadConstraint_C(ref ChVectorDynamic<double> Qc,           //< result: the Qc residual, Qc += c*C
                                              double c,                                 //< a scaling factor
                                              bool do_clamp = false,                    //< enable optional clamping of Qc
                                              double mclam = 1e30                      //< clamping value
                                      )
        {
            IntLoadConstraint_C(0, ref Qc, c, do_clamp, mclam);
        }

        /// Increment a vector Qc with the term Ct = partial derivative dC/dt:
        ///    Qc += c*Ct
        public void LoadConstraint_Ct(ref ChVectorDynamic<double> Qc,          //< result: the Qc residual, Qc += c*Ct
                                               double c                                 //< a scaling factor
                                   )
        {
            IntLoadConstraint_Ct(0, ref Qc, c);
        }

        //
        // UTILITY FUNCTIONS
        //

        /// If ChProbe() objects are added to this system, using this command you force
        /// the ChProbe::Record() on all them, at once.
        public int RecordAllProbes() {
            int pcount = 0;

            for (int ip = 0; ip < probelist.Count; ++ip)
            {
                probelist[ip].Record(GetChTime());
            }

            return pcount;
        }

        /// If ChProbe() objects are added to this system, using this command you force
        /// the ChProbe::Reset() on all them, at once.
        public int ResetAllProbes() {
            int pcount = 0;

            for (int ip = 0; ip < probelist.Count; ++ip)
            {
                probelist[ip].Reset();
            }

            return pcount;
        }

        /// Executes custom processing at the end of step. By default it does nothing,
        /// but if you inherit a special ChSystem you can implement this.
        public virtual void CustomEndOfStep() { }

        /// If ChControl() objects are added to this system, using the following commands
        /// you call the execution of their scripts. You seldom call these functions directly,
        /// since the ChSystem() methods already call them automatically, at each step, update, etc.
        public bool ExecuteControlsForUpdate() {
            for (int ip = 0; ip < controlslist.Count; ++ip)
            {
                if (!controlslist[ip].ExecuteForUpdate())
                    return false;
            }
            return true;
        }
        public bool ExecuteControlsForStep() {
            for (int ip = 0; ip < controlslist.Count; ++ip)
            {
                if (!controlslist[ip].ExecuteForStep())
                    return false;
            }
            return true;
        }

        /// All bodies with collision detection data are requested to
        /// store the current position as "last position collision-checked"
        public void SynchronizeLastCollPositions() {
            for (int i = 0; i < bodylist.Count; i++)
            {
                if (bodylist[i].GetCollide())
                    bodylist[i].SynchronizeLastCollPos();
            }
        }

        /// Perform the collision detection.
        /// New contacts are inserted in the ChContactContainer object(s), and old ones are removed.
        /// This is mostly called automatically by time integration.
        public double ComputeCollisions()
        {
            double mretC = 0.0;

            timer_collision.start();

            // Update all positions of collision models: delegate this to the ChAssembly
            SyncCollisionModels();

            // Perform the collision detection ( broadphase and narrowphase )
            collision_system.Run();

            // Report and store contacts and/or proximities, if there are some
            // containers in the physic system. The default contact container
            // for ChBody and ChParticles is used always.

            collision_system.ReportContacts(contact_container);

            for (int ip = 0; ip < otherphysicslist.Count; ++ip)
            {
                ChContactContainer mcontactcontainer;
                if (mcontactcontainer = (ChContactContainer)otherphysicslist[ip])
                {
                    //collision_system.ReportContacts(mcontactcontainer.get()); ***TEST*** if one wants to populate a ChContactContainer this would clear it anyway...
                }


                ChProximityContainer mproximitycontainer;
                if (mproximitycontainer = (ChProximityContainer)otherphysicslist[ip])
                {
                    collision_system.ReportProximities(mproximitycontainer);
                }
            }


            // Invoke the custom collision callbacks (if any). These can potentially add
            // additional contacts to the contact container.
            for (int ic = 0; ic < collision_callbacks.Count; ic++)
                collision_callbacks[ic].OnCustomCollision(this);

            // Count the contacts of body-body type.
            ncontacts = contact_container.GetNcontacts();

            timer_collision.stop();

            return mretC;

        }

        /// Class to be used as a callback interface for user defined actions performed 
        /// at each collision detection step.  For example, additional contact points can
        /// be added to the underlying contact container.
        public class CustomCollisionCallback
        {
            public virtual void OnCustomCollision(ChSystem msys) { }
        };

        /// Specify a callback object to be invoked at each collision detection step.
        /// Multiple such callback objects can be registered with a system. If present,
        /// their OnCustomCollision() method is invoked 
        /// Use this if you want that some specific callback function is executed at each
        /// collision detection step (ex. all the times that ComputeCollisions() is automatically
        /// called by the integration method). For example some other collision engine could
        /// add further contacts using this callback.
        public void RegisterCustomCollisionCallback(CustomCollisionCallback mcallb)
        {
            collision_callbacks.Add(mcallb);
        }

        /// For higher performance (ex. when GPU coprocessors are available) you can create your own custom
        /// collision engine (inherited from ChCollisionSystem) and plug it into the system using this function. 
        /// Note: use only _before_ you start adding colliding bodies to the system!
        public void SetCollisionSystem(collision.ChCollisionSystem newcollsystem) {
            collision_system = newcollsystem;
        }

        /// Access the collision system, the engine which
        /// computes the contact points (usually you don't need to
        /// access it, since it is automatically handled by the
        /// client ChSystem object).
        public collision.ChCollisionSystem GetCollisionSystem() { return collision_system; }

        /// Turn on this feature to let the system put to sleep the bodies whose
        /// motion has almost come to a rest. This feature will allow faster simulation
        /// of large scenarios for real-time purposes, but it will affect the precision!
        /// This functionality can be turned off selectively for specific ChBodies.
        public void SetUseSleeping(bool ms) { use_sleeping = ms; }

        /// Tell if the system will put to sleep the bodies whose motion has almost come to a rest.
        public bool GetUseSleeping() { return use_sleeping; }

        private class _wakeup_reporter_class : ChContactContainer.ReportContactCallback
        {
            // Data
            public bool someone_sleeps;
            public bool need_Setup_A;

            // Callback, used to report contact points already added to the container.
            // If returns false, the contact scanning will be stopped.
            public override bool OnReportContact(
                ChVector pA,             // get contact pA
                ChVector pB,             // get contact pB
                ChMatrix33<double> plane_coord,  // get contact plane coordsystem (A column 'X' is contact normal)
                double distance,           // get contact distance
                double eff_radius,         // effective radius of curvature at contact
                ChVector react_forces,   // get react.forces (if already computed). In coordsystem 'plane_coord'
                ChVector react_torques,  // get react.torques, if rolling friction (if already computed).
                ChContactable contactobjA,  // get model A (note: some containers may not support it and could be zero!)
                ChContactable contactobjB   // get model B (note: some containers may not support it and could be zero!)
            )
            {
                if (!(contactobjA == null && contactobjB == null))
                    return true;
                ChBody b1 = new ChBody((ChBody)contactobjA);
                ChBody b2 = new ChBody((ChBody)contactobjB);
                if (!(b1 && b2))
                    return true;
                bool sleep1 = b1.GetSleeping();
                bool sleep2 = b2.GetSleeping();
                bool could_sleep1 = b1.BFlagGet(ChBody.BodyFlag.COULDSLEEP);
                bool could_sleep2 = b2.BFlagGet(ChBody.BodyFlag.COULDSLEEP);
                bool ground1 = b1.GetBodyFixed();
                bool ground2 = b2.GetBodyFixed();
                if (sleep1 && !(sleep2 || could_sleep2) && !ground2)
                {
                    b1.SetSleeping(false);
                    need_Setup_A = true;
                }
                if (sleep2 && !(sleep1 || could_sleep1) && !ground1)
                {
                    b2.SetSleeping(false);
                    need_Setup_A = true;
                }
                if (could_sleep1 && !(sleep2 || could_sleep2) && !ground2)
                {
                    b1.BFlagSet(ChBody.BodyFlag.COULDSLEEP, false);
                }
                if (could_sleep2 && !(sleep1 || could_sleep1) && !ground1)
                {
                    b2.BFlagSet(ChBody.BodyFlag.COULDSLEEP, false);
                }
                someone_sleeps = sleep1 | sleep2 | someone_sleeps;

                return true;  // to continue scanning contacts
            }
        }

        /// Put bodies to sleep if possible. Also awakens sleeping bodies, if needed.
        /// Returns true if some body changed from sleep to no sleep or viceversa,
        /// returns false if nothing changed. In the former case, also performs Setup()
        /// because the sleeping policy changed the totalDOFs and offsets.
        private bool ManageSleepingBodies()
        {

            if (!GetUseSleeping())
                return false;

            // STEP 1:
            // See if some body could change from no sleep. sleep

            for (int i = 0; i < bodylist.Count; i++)
            {
                // mark as 'could sleep' candidate
                bodylist[i].TrySleeping();
            }

            // STEP 2:
            // See if some sleeping or potential sleeping body is touching a non sleeping one,
            // if so, set to no sleep.
            // Make this class for iterating through contacts


            _wakeup_reporter_class my_waker = new _wakeup_reporter_class();
            my_waker.need_Setup_A = false;

            bool need_Setup_L = false;

            for (int i = 0; i < 1; i++)  //***TO DO*** reconfigurable number of wakeup cycles
            {
                my_waker.someone_sleeps = false;

                // scan all links and wake connected bodies
                for (int ip = 0; ip < linklist.Count; ++ip)  // ITERATE on links
                {
                    ChLink Lpointer = linklist[ip];

                    if (Lpointer.IsRequiringWaking()) {
                        // ChBody* b1 = dynamic_cast<ChBody*>(Lpointer->GetBody1());
                        // ChBody* b2 = dynamic_cast<ChBody*>(Lpointer->GetBody2());

                        // TODO  Needs to be resolved to have sleeping bodies.
                       /* ChBody b1 = (Lpointer).GetBody1();
                        ChBody b2 = (Lpointer).GetBody1();
                        //b1.BodyFrame = Lpointer.GetBody1();
                        //b2.BodyFrame = Lpointer.GetBody2();
                        if (b1 != null && b2 != null)
                        {
                            bool sleep1 = b1.GetSleeping();
                            bool sleep2 = b2.GetSleeping();

                            bool could_sleep1 = b1.BFlagGet(ChBody.BodyFlag.COULDSLEEP);
                            bool could_sleep2 = b2.BFlagGet(ChBody.BodyFlag.COULDSLEEP);
                            if (sleep1 && !(sleep2 || could_sleep2))
                            {
                                b1.SetSleeping(false);
                                need_Setup_L = true;
                            }
                            if (sleep2 && !(sleep1 || could_sleep1))
                            {
                                b2.SetSleeping(false);
                                need_Setup_L = true;
                            }
                            if (could_sleep1 && !(sleep2 || could_sleep2))
                            {
                                b1.BFlagSet(ChBody.BodyFlag.COULDSLEEP, false);
                            }
                            if (could_sleep2 && !(sleep1 || could_sleep1))
                            {
                                b2.BFlagSet(ChBody.BodyFlag.COULDSLEEP, false);
                            }
                        }*/
                    }
                }

                // scan all contacts and wake neighboring bodies
                contact_container.ReportAllContacts(my_waker);

                // bailout wakeup cycle prematurely, if all bodies are not sleeping
                if (!my_waker.someone_sleeps)
                    break;
            }
           
            return true;

        }
    

        /// Performs a single dynamical simulation step, according to
        /// current values of:  Y, time, step  (and other minor settings)
        /// Depending on the integration type, it switches to one of the following:
        public virtual bool Integrate_Y() {

            ResetTimers();

            timer_step.start();

            // Executes "forStep" in all controls of controlslist
            ExecuteControlsForStep();

            stepcount++;
            solvecount = 0;
            setupcount = 0;

            // Compute contacts and create contact constraints
            ComputeCollisions();

            // Counts dofs, statistics, etc. (not needed because already in Advance()...? )
            Setup();

            // Update everything - and put to sleep bodies that need it (not needed because already in Advance()...? )
            // No need to update visualization assets here.
            update(false); // PROBLEM slow approx 8ms

            // Re-wake the bodies that cannot sleep because they are in contact with
            // some body that is not in sleep state.
            ManageSleepingBodies();     // TODO Fix to have sleeping.

            // Prepare lists of variables and constraints.
            DescriptorPrepareInject(ref descriptor);
            descriptor.UpdateCountsAndOffsets();

            // Set some settings in timestepper object
            timestepper.SetQcDoClamp(true);
            timestepper.SetQcClamping(max_penetration_recovery_speed);


            // PERFORM TIME STEP HERE!
            {
                //CH_PROFILE("Advance");
                timer_advance.start();
                timestepper.Advance(step);
                timer_advance.stop();
            }

            // Executes custom processing at the end of step
            CustomEndOfStep();

            // If there are some probe objects in the probe list,
            // tell them to record their variables (usually x-y couples)
            RecordAllProbes();

            // Call method to gather contact forces/torques in rigid bodies
            contact_container.ComputeContactForces();

            // Time elapsed for step..
            timer_step.stop();

            return true;
        }


        // ---- DYNAMICS

        /// Advances the dynamical simulation for a single step, of
        /// length m_step. You can call this function many
        /// times in order to simulate up to a desired end time.
        /// This is the most important function for analysis, you
        /// can use it, for example, once per screen refresh in VR
        /// and interactive realtime applications, etc.
        public bool DoStepDynamics(double m_step) {
            step = m_step;
            return Integrate_Y();
        }

        /// Performs integration until the m_endtime is exactly
        /// reached, but current time step may be automatically "retouched" to
        /// meet exactly the m_endtime after n steps.
        /// Useful when you want to advance the simulation in a
        /// simulations (3d modeling software etc.) which needs updates
        /// of the screen at a fixed rate (ex.30th of second)  while
        /// the integration must use more steps.
        public bool DoFrameDynamics(double m_endtime) {
            double frame_step;
            double old_step = 0;
            double left_time;
            bool restore_oldstep = false;
            int counter = 0;
            double fixed_step_undo;

            frame_step = (m_endtime - ChTime);
            fixed_step_undo = step;

            while (ChTime < m_endtime)
            {
                restore_oldstep = false;
                counter++;

                left_time = m_endtime - ChTime;

                if (left_time < 1e-12)
                    break;  // - no integration if backward or null frame step.

                if (left_time < (1.3 * step))  // - step changed if too little frame step
                {
                    old_step = step;
                    step = left_time;
                    restore_oldstep = true;
                }

                if (!Integrate_Y())
                    break;  // ***  Single integration step,
                            // ***  updating Y, from t to t+dt.
                            // ***  This also changes local ChTime, and may change step

                if (last_err)
                    break;
            }

            if (restore_oldstep)
                step = old_step;  // if timestep was changed to meet the end of frametime, restore pre-last (even for
                                  // time-varying schemes)

            if (last_err)
                return false;
            return true;
        }

        /// Given the current state, the sw simulates the
        /// dynamical behavior of the system, until the end
        /// time is reached, repeating many steps (maybe the step size
        /// will be automatically changed if the integrator method supports
        /// step size adaption).
        public bool DoEntireDynamics() {
            Setup();

            // the system may have wrong layout, or too large
            // clearances in constraints, so it is better to
            // check for constraint violation each time the integration starts
            DoAssembly(Convert.ToInt32(AssemblyLevel.Enum.POSITION) | Convert.ToInt32(AssemblyLevel.Enum.VELOCITY) | Convert.ToInt32(AssemblyLevel.Enum.ACCELERATION));

            // Perform the integration steps until the end
            // time is reached.
            // All the updating (of Y, Y_dt and time) is done
            // automatically by Integrate()

            while (ChTime < end_time)
            {
                if (!Integrate_Y())
                    break;  // >>> 1- single integration step,
                            //        updating Y, from t to t+dt.
                if (last_err)
                    return false;
            }

            if (last_err)
                return false;
            return true;
        }

        /// Like "DoEntireDynamics", but results are provided at uniform
        /// steps "frame_step", using the DoFrameDynamics() many times.
        public bool DoEntireUniformDynamics(double frame_step) {
            // the initial system may have wrong layout, or too large
            // clearances in constraints.
            Setup();
            DoAssembly(Convert.ToInt32(AssemblyLevel.Enum.POSITION) | Convert.ToInt32(AssemblyLevel.Enum.VELOCITY) | Convert.ToInt32(AssemblyLevel.Enum.ACCELERATION));

            while (ChTime < end_time)
            {
                double goto_time = (ChTime + frame_step);
                if (!DoFrameDynamics(goto_time))
                    return false;
            }

            return true;
        }

        /// Return the total number of time steps taken so far.
        public int GetStepcount() { return stepcount; }

        /// Reset to 0 the total number of time steps.
        public void ResetStepcount() { stepcount = 0; }

        /// Return the number of calls to the solver's Solve() function.
        /// This counter is reset at each timestep.
        public int GetSolverCallsCount() { return solvecount; }

        /// Return the number of calls to the solver's Setup() function.
        /// This counter is reset at each timestep.
        public int GetSolverSetupCount() { return setupcount; }

        /// Set this to "true" to enable automatic saving of solver matrices at each time
        /// step, for debugging purposes. Note that matrices will be saved in the
        /// working directory of the exe, with format 0001_01_H.dat 0002_01_H.dat
        /// (if the timestepper requires multiple solves, also 0001_01. 0001_02.. etc.)
        /// The matrices being saved are:
        ///    dump_Z.dat   has the assembled optimization matrix (Matlab sparse format)
        ///    dump_rhs.dat has the assembled RHS
        ///    dump_H.dat   has usually H=M (mass), but could be also H=a*M+b*K+c*R or such. (Matlab sparse format)
        ///    dump_Cq.dat  has the jacobians (Matlab sparse format)
        ///    dump_E.dat   has the constr.compliance (Matlab sparse format)
        ///    dump_f.dat   has the applied loads
        ///    dump_b.dat   has the constraint rhs
        /// as passed to the solver in the problem
        ///  | H -Cq'|*|q|- | f|= |0| , l \f$\in Y, c \in Ny\f$, normal cone to Y
        ///  | Cq -E | |l|  |-b|  |c|

        public void SetDumpSolverMatrices(bool md) { dump_matrices = md; }
        public bool GetDumpSolverMatrices() { return dump_matrices; }

        /// Dump the current M mass matrix, K damping matrix, R damping matrix, Cq constraint jacobian
        /// matrix (at the current configuration). 
        /// These can be later used for linearized motion, modal analysis, buckling analysis, etc.
        /// The name of the files will be [path]_M.dat [path]_K.dat [path]_R.dat [path]_Cq.dat 
        /// Might throw ChException if file can't be saved.
        public void DumpSystemMatrices(bool save_M, bool save_K, bool save_R, bool save_Cq, char path) { }

        /// Compute the system-level mass matrix. 
        /// This function has a small overhead, because it must assembly the
        /// sparse matrix -which is used only for the purpose of this function.
        public void GetMassMatrix(ChSparseMatrix M)
        { //< fill this system mass matrix

          //IntToDescriptor(0, Dv, R, 0, L, Qc);
          //ConstraintsLoadJacobians();

            // Load all KRM matrices with the M part only
            KRMmatricesLoad(0, 0, 1.0);
            // For ChVariable objects without a ChKblock, but still with a mass:
            descriptor.SetMassFactor(1.0);

            // Fill system-level M matrix
            this.GetSystemDescriptor().ConvertToMatrixForm(null, M, null, null, null, null, false, false);

        }        

        /// Compute the system-level stiffness matrix, i.e. the jacobian -dF/dq where F are stiff loads.
        /// Note that not all loads provide a jacobian, as this is optional in their implementation.
        /// This function has a small overhead, because it must assembly the
        /// sparse matrix -which is used only for the purpose of this function.
        public void GetStiffnessMatrix(ChSparseMatrix K)
        { //< fill this system stiffness matrix

           // IntToDescriptor(0, ref Dv, ref R, 0, ref L, ref Qc);
           // ConstraintsLoadJacobians();

            // Load all KRM matrices with the K part only
            this.KRMmatricesLoad(1.0, 0, 0);
            // For ChVariable objects without a ChKblock, but still with a mass:
            descriptor.SetMassFactor(0.0);

            // Fill system-level K matrix
            this.GetSystemDescriptor().ConvertToMatrixForm(null, K, null, null, null, null, false, false);

        }         

        /// Compute the system-level damping matrix, i.e. the jacobian -dF/dv where F are stiff loads.
        /// Note that not all loads provide a jacobian, as this is optional in their implementation.
        /// This function has a small overhead, because it must assembly the
        /// sparse matrix -which is used only for the purpose of this function.
        public void GetDampingMatrix(ChSparseMatrix R) {
            //IntToDescriptor(0, Dv, R, 0, L, Qc);
            //ConstraintsLoadJacobians();

            // Load all KRM matrices with the R part only
            this.KRMmatricesLoad(0, 1.0, 0);
            // For ChVariable objects without a ChKblock, but still with a mass:
            descriptor.SetMassFactor(0.0);

            // Fill system-level R matrix
            this.GetSystemDescriptor().ConvertToMatrixForm(null, R, null, null, null, null, false, false);

        }   //< fill this system damping matrix

        /// Compute the system-level constraint jacobian matrix, i.e. the jacobian
        /// Cq=-dC/dq where C are constraints (the lower left part of the KKT matrix).
        /// This function has a small overhead, because it must assembly the
        /// sparse matrix -which is used only for the purpose of this function.
        public void GetConstraintJacobianMatrix(ChSparseMatrix Cq) {
            //IntToDescriptor(0, Dv, R, 0, L, Qc);

            // Load all jacobian matrices 
            this.ConstraintsLoadJacobians();

            // Fill system-level R matrix
            this.GetSystemDescriptor().ConvertToMatrixForm(Cq, null, null, null, null, null, false, false);
        }  //< fill this system damping matrix

        // ---- KINEMATICS

        /// Advances the kinematic simulation for a single step, of
        /// length m_step. You can call this function many
        /// times in order to simulate up to a desired end time.
        public bool DoStepKinematics(double m_step) {
            ChTime += m_step;

            update();

            // Newton Raphson kinematic equations solver
            DoAssembly(Convert.ToInt32(AssemblyLevel.Enum.POSITION) | Convert.ToInt32(AssemblyLevel.Enum.VELOCITY) | Convert.ToInt32(AssemblyLevel.Enum.ACCELERATION));

            if (last_err)
                return false;

            return true;
        }

        /// Performs kinematics until the m_endtime is exactly
        /// reached, but current time step may be automatically "retouched" to
        /// meet exactly the m_endtime after n steps.
        public bool DoFrameKinematics(double m_endtime) {
            double frame_step;
            double old_step = 0;
            double left_time;
            int restore_oldstep;
            int counter = 0;

            frame_step = (m_endtime - ChTime);

            double fixed_step_undo = step;

            while (ChTime < m_endtime)
            {
                restore_oldstep = 0;
                counter++;

                left_time = m_endtime - ChTime;

                if (left_time < 0.000000001)
                    break;  // - no kinematics for backward

                if (left_time < (1.3 * step))  // - step changed if too little frame step
                {
                    old_step = step;
                    step = left_time;
                    restore_oldstep = 0;
                }

                // Newton Raphson kinematic equations solver
                DoAssembly(Convert.ToInt32(AssemblyLevel.Enum.POSITION) | Convert.ToInt32(AssemblyLevel.Enum.VELOCITY) | Convert.ToInt32(AssemblyLevel.Enum.ACCELERATION));

                if (last_err)
                    return false;

                ChTime += step;

                if (restore_oldstep != 0)
                    step = old_step;  // if timestep was changed to meet the end of frametime
            }

            return true;
        }

        /// Given the current state, this kinematic simulation
        /// satisfies all the constraints with the "DoStepKinematics"
        /// procedure for each time step, from the current time
        /// to the end time.
        public bool DoEntireKinematics() {
            Setup();

            int action = Convert.ToInt32(AssemblyLevel.Enum.POSITION) | Convert.ToInt32(AssemblyLevel.Enum.VELOCITY) | Convert.ToInt32(AssemblyLevel.Enum.ACCELERATION);

            DoAssembly(action);
            // first check if there are redundant links (at least one NR cycle
            // even if the structure is already assembled)

            while (ChTime < end_time)
            {
                // Newton-Raphson iteration, closing constraints
                DoAssembly(action);

                if (last_err)
                    return false;

                // Update time and repeat.
                ChTime += step;
            }

            return true;
        }

        // ---- CONSTRAINT ASSEMBLATION

        /// Given the current time and state, attempt to satisfy all constraints, using
        /// a Newton-Raphson iteration loop. Used iteratively in inverse kinematics.
        /// Action can be one of AssemblyLevel::POSITION, AssemblyLevel::VELOCITY, or 
        /// AssemblyLevel::ACCELERATION (or a combination of these)
        /// Returns true if no errors and false if an error occurred (impossible assembly?)
        public bool DoAssembly(int action) {
            solvecount = 0;
            setupcount = 0;

            Setup();
            update();

            int old_maxsteps = GetMaxItersSolverSpeed();
            SetMaxItersSolverSpeed(300);

            double old_step = GetStep();
            double new_step = 1e-6;
            SetStep(new_step);

            double old_tol = GetTolForce();
            SetTolForce(1e-4);

            // Prepare lists of variables and constraints.
            DescriptorPrepareInject(ref descriptor);

            ChAssemblyAnalysis manalysis = new ChAssemblyAnalysis(this);
            manalysis.SetMaxAssemblyIters(GetMaxiter());

            // Perform analysis
            manalysis.AssemblyAnalysis(action, new_step);

            SetMaxItersSolverSpeed(old_maxsteps);
            SetStep(old_step);
            SetTolForce(old_tol);

            return true;
        }

        /// Shortcut for full position/velocity/acceleration assembly.
        public bool DoFullAssembly() {
            DoAssembly(Convert.ToInt32(AssemblyLevel.Enum.POSITION) | Convert.ToInt32(AssemblyLevel.Enum.VELOCITY) | Convert.ToInt32(AssemblyLevel.Enum.ACCELERATION));

            return last_err;
        }

        // ---- STATICS

        /// Solve the position of static equilibrium (and the
        /// reactions). This is a one-step only approach that solves
        /// the _linear_ equilibrium. To be used mostly for FEM
        /// problems with small deformations.
        public bool DoStaticLinear() {
            solvecount = 0;
            setupcount = 0;

            Setup();
            update();

            int old_maxsteps = GetMaxItersSolverSpeed();
            SetMaxItersSolverSpeed(300);

            // Prepare lists of variables and constraints.
            DescriptorPrepareInject(ref descriptor);

            ChStaticLinearAnalysis manalysis = new ChStaticLinearAnalysis(this);

            // Perform analysis
            manalysis.StaticAnalysis();

            SetMaxItersSolverSpeed(old_maxsteps);

            bool dump_data = false;

            if (dump_data)
            {
                GetSystemDescriptor().DumpLastMatrices();

                // optional check for correctness in result
                ChMatrix md = new ChMatrix();
                GetSystemDescriptor().BuildDiVector(ref md);  // d={f;-b}

                ChMatrix mx = new ChMatrix();
                GetSystemDescriptor().FromUnknownsToVector(ref mx);  // x ={q,-l}
               // chrono::ChStreamOutAsciiFile file_x("dump_x.dat");
                //mx.StreamOUTdenseMatlabFormat(file_x);

                ChMatrix mZx = new ChMatrix();
                GetSystemDescriptor().SystemProduct(ref mZx, mx);  // Zx = Z*x

               // GetLog() << "CHECK: norm of solver residual: ||Z*x-d|| -------------------\n";
               // GetLog() << (mZx - md).NormInf() << "\n";
            }

            return true;
        }

        /// Solve the position of static equilibrium (and the
        /// reactions). This tries to solve the equilibrium for the nonlinear
        /// problem (large displacements). The larger nsteps, the more the CPU time
        /// but the less likely the divergence.
        public bool DoStaticNonlinear(int nsteps = 10) {
            solvecount = 0;
            setupcount = 0;

            Setup();
            update();

            int old_maxsteps = GetMaxItersSolverSpeed();
            SetMaxItersSolverSpeed(300);

            // Prepare lists of variables and constraints.
            DescriptorPrepareInject(ref descriptor);

            ChStaticNonLinearAnalysis manalysis = new ChStaticNonLinearAnalysis(this);
            manalysis.SetMaxiters(nsteps);

            // Perform analysis
            manalysis.StaticAnalysis();

            SetMaxItersSolverSpeed(old_maxsteps);

            return true;
        }

        /// Finds the position of static equilibrium (and the
        /// reactions) starting from the current position.
        /// Since a truncated iterative method is used, you may need
        /// to call this method multiple times in case of large nonlinearities
        /// before coming to the precise static solution.
        public bool DoStaticRelaxing(int nsteps = 10)
        {
            solvecount = 0;
            setupcount = 0;

            int err = 0;
            bool reached_tolerance = false;

            if ((ncoords > 0) && (ndof >= 0))
            {
                for (int m_iter = 0; m_iter < nsteps; m_iter++)
                {
                    for (int i = 0; i < bodylist.Count; i++)
                    {
                        // Set no body speed and no body accel.
                        bodylist[i].SetNoSpeedNoAcceleration();
                    }
                    /* for (auto mesh : meshlist)
                     {
                         mesh.SetNoSpeedNoAcceleration();
                     }*/
                    for (int ip = 0; ip < otherphysicslist.Count; ++ip)
                    {
                        otherphysicslist[ip].SetNoSpeedNoAcceleration();
                    }

                    double m_undotime = GetChTime();
                    DoFrameDynamics(m_undotime + (nsteps * 1.8) * (((double)nsteps - (double)m_iter)) / (double)nsteps);
                    SetChTime(m_undotime);
                }

                for (int i = 0; i < bodylist.Count; i++)
                {
                    // Set no body speed and no body accel.
                    bodylist[i].SetNoSpeedNoAcceleration();
                }
                /*for (auto & mesh : meshlist)
                {
                    mesh.SetNoSpeedNoAcceleration();
                }*/
                for (int ip = 0; ip < otherphysicslist.Count; ++ip)
                {
                    otherphysicslist[ip].SetNoSpeedNoAcceleration();
                }
            }

            if (err != 0)
            {
                last_err = true;
                Debug.Log("WARNING: some constraints may be redundant, but couldn't be eliminated \n");
            }
            return last_err;
        }  

     
        

        /// Process a ".chr" binary file containing the full system object
        /// hierarchy as exported -for example- by the R3D modeler, with chrono plug-in version,
        /// or by using the FileWriteChR() function.
      //  public int FileProcessChR(ref ChStreamInBinary m_file) { }

        /// Write a ".chr" binary file containing the full system object
        /// hierarchy (bodies, forces, links, etc.) (deprecated function - obsolete)
       // public int FileWriteChR(ref ChStreamOutBinary m_file) { }


      /*  void OnGUI()
        {
            if (Application.isEditor)  // or check the app debug flag
            {
                //  GUI.Label(new Rect(200, 300, 100, 100), m_softstep.ToString());
                //  GUI.Label(new Rect(200, 350, 100, 100), Time.fixedDeltaTime.ToString());
            }
        }*/

      

        protected List<ChProbe> probelist = new List<ChProbe>();        //< list of 'probes' (variable-recording objects)
        protected List<ChControls> controlslist = new List<ChControls>();  //< list of 'controls' script objects

         protected ChContactContainer contact_container;  //< the container of contacts
        //protected GameObject contact_container;  //< the container of contacts

        public Vector3 gravity = new Vector3();
        protected ChVector G_acc = new ChVector(0, 0, 0);  //< gravitational acceleration

        protected double end_time;  //< end of simulation
        public double step;      //< time step
        protected double step_min;  //< min time step
        protected double step_max;  //< max time step

        protected double tol;        //< tolerance
        protected double tol_force;  //< tolerance for forces (used to obtain a tolerance for impulses)

        public int maxiter;  //< max iterations for nonlinear convergence in DoAssembly()

        public bool use_sleeping;  //< if true, put to sleep objects that come to rest

        protected ChSystemDescriptor descriptor;  //< the system descriptor
        protected ChSolver solver_speed;          //< the solver for speed problem
        protected ChSolver solver_stab;           //< the solver for position (stabilization) problem, if any

        public int max_iter_solver_speed;  //< maximum num iterations for the iterative solver
        public int max_iter_solver_stab;   //< maximum num iterations for the iterative solver for constraint stabilization
        protected int max_steps_simplex;      //< maximum number of steps for the simplex solver.

        protected double min_bounce_speed;                //< minimum speed for rebounce after impacts. Lower speeds are clamped to 0
        public double max_penetration_recovery_speed;  //< limit for the speed of penetration recovery (positive, speed of exiting)

        protected int parallel_thread_number;  //< used for multithreaded solver

        public int stepcount;  //< internal counter for steps

        protected int setupcount;  //< number of calls to the solver's Setup()
        protected int solvecount;  //< number of StateSolveCorrection (reset to 0 at each timestep of static analysis)

        protected bool dump_matrices;  //< for debugging

        public int ncontacts;  //< total number of contacts

        protected collision.ChCollisionSystem collision_system;  //< collision engine

        protected List<CustomCollisionCallback> collision_callbacks = new List<CustomCollisionCallback>();

        public ChMaterialCompositionStrategy<float> composition_strategy = new ChMaterialCompositionStrategy<float>(); /// material composition strategy

        // timers for profiling execution speed
        protected ChTimer<double> timer_step = new ChTimer<double>();       //< timer for integration step
        protected ChTimer<double> timer_advance = new ChTimer<double>();    //< timer for time integration
        protected ChTimer<double> timer_solver = new ChTimer<double>();     //< timer for solver (excluding setup phase)
        protected ChTimer<double> timer_setup = new ChTimer<double>();      //< timer for solver setup
        protected ChTimer<double> timer_jacobian = new ChTimer<double>();   //< timer for computing/loading Jacobian information
        protected ChTimer<double> timer_collision = new ChTimer<double>();  //< timer for collision detection
        protected ChTimer<double> timer_update = new ChTimer<double>();     //< timer for system update

        protected ChTimestepper timestepper;  //< time-stepper object

        protected bool last_err;  //< indicates error over the last kinematic/dynamics/statics

        public int debug;

        ChRealtimeStepTimer m_realtime_timer = new ChRealtimeStepTimer();
    }
}
