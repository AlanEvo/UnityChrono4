using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono
{

    /// @addtogroup chrono_timestepper
    /// @{

    /// Base class for timesteppers, i.e., time integrators that can advance a system state.
    /// It operates on systems inherited from ChIntegrable.
    public abstract class ChTimestepper
    {
        /// Available methods for time integration (time steppers).
        public enum Type
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

        /// Constructor
        public ChTimestepper(ChIntegrable mintegrable = null)
        {
            integrable = mintegrable;
            T = 0;
            L.Reset(0);
            verbose = false;
            Qc_do_clamp = false;
            Qc_clamping = 1e30;
        }

        /// Return type of the integration method.
        /// Default is CUSTOM. Derived classes should override this function.
        public virtual Type GetType() { return Type.CUSTOM; }

        /// Performs an integration timestep
        public abstract void Advance(double dt  //< timestep to advance
                         );

        /// Access the lagrangian multipliers, if any.
        public virtual ChVectorDynamic<double> get_L() { return L; }

        /// Set the integrable object.
        public virtual void SetIntegrable(ChIntegrable mintegrable) { integrable = mintegrable; }

        /// Get the integrable object.
        public ChIntegrable GetIntegrable() { return integrable; }

        /// Get the current time.
        public virtual double GetTime() { return T; }

        /// Set the current time.
        public virtual void SetTime(double mt) { T = mt; }

        /// Turn on/off logging of messages.
        public void SetVerbose(bool mverbose) { verbose = mverbose; }

        /// Turn on/off clamping on the Qcterm.
        public void SetQcDoClamp(bool mdc) { Qc_do_clamp = mdc; }

        /// Turn on/off clamping on the Qcterm.
        public void SetQcClamping(double mcl) { Qc_clamping = mcl; }



        protected ChIntegrable integrable;
        protected double T;

        protected ChVectorDynamic<double> L = new ChVectorDynamic<double>();

        protected bool verbose;

        protected bool Qc_do_clamp;
        protected double Qc_clamping;

    }

    /// Base class for implicit solvers (double inheritance)
    public interface IChImplicitTimestepper {

    };

    /// Base class for 2nd order timesteppers, i.e., a time integrator for a ChIntegrableIIorder.
    /// A ChIntegrableIIorder is a special subclass of integrable objects that have a state comprised
    /// of position and velocity y={x,v}, and state derivative dy/dt={v,a}, where a=acceleration.
    public class ChTimestepperIIorder : ChTimestepper {

        protected ChState X = new ChState();
        protected ChStateDelta V = new ChStateDelta();
        protected ChStateDelta A = new ChStateDelta();

        /// Constructor
        public ChTimestepperIIorder(ChIntegrableIIorder mintegrable = null) : base(mintegrable)
        {
            SetIntegrable(mintegrable);
        }

        /// Performs an integration timestep
        public override void Advance(double dt  //< timestep to advance
                         )
        { }

        /// Access the state, position part, at current time
        public virtual ChState get_X() { return X; }

        /// Access the state, speed part, at current time
        public virtual ChStateDelta get_V() { return V; }

        /// Access the acceleration, at current time
        public virtual ChStateDelta get_A() { return A; }

        /// Set the integrable object
        public virtual void SetIntegrable(ChIntegrableIIorder mintegrable)
        {
            base.SetIntegrable(mintegrable);
            X.Reset(1, mintegrable);
            V.Reset(1, mintegrable);
            A.Reset(1, mintegrable);
        }
    };

    /// Performs a step of Euler implicit for II order systems using the Anitescu/Stewart/Trinkle
    /// single-iteration method, that is a bit like an implicit Euler where one performs only the
    /// first Newton corrector iteration.
    /// If using an underlying CCP complementarity solver, this is the typical Anitescu stabilized
    /// timestepper for DVIs.
    public class ChTimestepperEulerImplicitLinearized : ChTimestepperIIorder, IChImplicitTimestepper {


        protected ChStateDelta Vold = new ChStateDelta();
        protected ChVectorDynamic<double> Dl = new ChVectorDynamic<double>();
        protected ChVectorDynamic<double> R = new ChVectorDynamic<double>();
        protected ChVectorDynamic<double> Qc = new ChVectorDynamic<double>();

        /// Constructors (default empty)
        public ChTimestepperEulerImplicitLinearized(ChIntegrableIIorder mintegrable = null)
        : base(mintegrable) { }

        public override Type GetType() { return Type.EULER_IMPLICIT_LINEARIZED; }

        /// Performs an integration timestep
        public override void Advance(double dt  //< timestep to advance
                         )
        {
            // downcast
            ChIntegrableIIorder mintegrable = (ChIntegrableIIorder)this.integrable;

            // setup main vectors
            mintegrable.StateSetup(ref X, ref V, ref A);

            // setup auxiliary vectors
            Dl.Reset(mintegrable.GetNconstr());
            R.Reset(mintegrable.GetNcoords_v());
            Qc.Reset(mintegrable.GetNconstr());
            L.Reset(mintegrable.GetNconstr());
            //Debug.Log("constr " + mintegrable.GetNconstr());
            

            mintegrable.StateGather(ref X, ref V, ref T);  // state <- system
            mintegrable.StateGatherReactions(ref L); // state <- system (may be needed for warm starting StateSolveCorrection) 

            L *= dt; // because reactions = forces, here L = impulses

            Vold = V;

            // solve only 1st NR step, using v_new = 0, so  Dv = v_new , therefore
            //
            // [ M - dt*dF/dv - dt^2*dF/dx    Cq' ] [ Dv     ] = [ M*(v_old - v_new) + dt*f]
            // [ Cq                           0   ] [ -dt*Dl ] = [ -C/dt - Ct ]
            //
            // becomes the Anitescu/Trinkle timestepper:
            //
            // [ M - dt*dF/dv - dt^2*dF/dx    Cq' ] [ v_new  ] = [ M*(v_old) + dt*f]
            // [ Cq                           0   ] [ -dt*l  ] = [ -C/dt - Ct ]

            mintegrable.LoadResidual_F(ref R, dt);
            mintegrable.LoadResidual_Mv(ref R, V, 1.0);
            mintegrable.LoadConstraint_C(ref Qc, 1.0 / dt, Qc_do_clamp, Qc_clamping);
            mintegrable.LoadConstraint_Ct(ref Qc, 1.0);
            
            // Can't shave anymore off this, Solver() is the only cpu drain.
            mintegrable.StateSolveCorrection(
                ref V, ref L, R, Qc,
                1.0,           // factor for  M
                -dt,           // factor for  dF/dv
                -dt * dt,      // factor for  dF/dx
                X, V, T + dt,  // not needed
                false,         // do not StateScatter update to Xnew Vnew T+dt before computing correction
                true           // force a call to the solver's Setup() function
                );


            L *= (1.0 / dt);  // Note it is not -(1.0/dt) because we assume StateSolveCorrection already flips sign of Dl

            mintegrable.StateScatterAcceleration(
                (V - Vold) * (1 / dt));  // -> system auxiliary data (i.e acceleration as measure, fits DVI/MDI)


            X += V * dt;

            T += dt;

            mintegrable.StateScatter(X, V, T);     // state -> system  // Big cpu drain
            mintegrable.StateScatterReactions(L);  // -> system auxiliary data*/

        }
    };

    /// Performs a step of Newmark constrained implicit for II order DAE systems.
    /// See Negrut et al. 2007.
    public class ChTimestepperNewmark : ChTimestepperIIorder
    {

    }
}
