using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono
{
    /// Enumerations for assembly level.
    namespace AssemblyLevel
    {
        public enum Enum { POSITION = 1 << 0, VELOCITY = 1 << 1, ACCELERATION = 1 << 2 };
    }

    /// Class for assembly analysis.
    /// Assembly is performed by satisfying constraints at a position, velocity, and acceleration levels.
    /// Assembly at position level involves solving a non-linear problem. Assembly at velocity level is
    /// performed by taking a small integration step. Consistent accelerations are obtained through
    /// finite differencing.
    public class ChAssemblyAnalysis
    {
        protected ChIntegrableIIorder integrable;

        protected ChState X;
        protected ChStateDelta V;
        protected ChStateDelta A;
        protected ChVectorDynamic<double> L;
        protected int max_assembly_iters;

        public ChAssemblyAnalysis(ChIntegrableIIorder mintegrable)
        {
            integrable = mintegrable;
            L.Reset(0);
            X.Reset(1, mintegrable);
            V.Reset(1, mintegrable);
            A.Reset(1, mintegrable);
            max_assembly_iters = 4;
        }

        /// Perform the assembly analysis.
        /// Assembly is performed by satisfying constraints at a position, velocity, and acceleration levels.
        /// Assembly at position level involves solving a non-linear problem. Assembly at velocity level is
        /// performed by taking a small integration step. Consistent accelerations are obtained through
        /// finite differencing.
        public void AssemblyAnalysis(int action, double dt = 1e-7)
        {
            ChVectorDynamic<double> R = new ChVectorDynamic<double>();
            ChVectorDynamic<double> Qc = new ChVectorDynamic<double>();
            double T = 0;

            // Set up main vectors
            integrable.StateSetup(ref X, ref V, ref A);

            if (action != 0 && AssemblyLevel.Enum.POSITION != 0)
            {
                ChStateDelta Dx = new ChStateDelta();

                for (int m_iter = 0; m_iter < max_assembly_iters; m_iter++)
                {
                    // Set up auxiliary vectors
                    Dx.Reset(integrable.GetNcoords_v(), GetIntegrable());
                    R.Reset(integrable.GetNcoords_v());
                    Qc.Reset(integrable.GetNconstr());
                    L.Reset(integrable.GetNconstr());

                    integrable.StateGather(ref X, ref V, ref T);  // state <- system

                    // Solve:
                    //
                    // [M          Cq' ] [ dx  ] = [ 0]
                    // [ Cq        0   ] [  l  ] = [ C]

                    integrable.LoadConstraint_C(ref Qc, 1.0);

                    integrable.StateSolveCorrection(
                        ref Dx, ref L, R, Qc,
                        1.0,      // factor for  M
                        0,        // factor for  dF/dv
                        0,        // factor for  dF/dx (the stiffness matrix)
                        X, V, T,  // not needed
                        false,    // do not StateScatter update to Xnew Vnew T+dt before computing correction
                        true      // force a call to the solver's Setup function
                        );

                    X += Dx;

                    integrable.StateScatter(X, V, T);  // state -> system
                }
            }

            if ((action != 0 & AssemblyLevel.Enum.VELOCITY != 0) || (action != 0 & AssemblyLevel.Enum.ACCELERATION != 0))
            {
                ChStateDelta Vold = new ChStateDelta();

                // setup auxiliary vectors
                Vold.Reset(integrable.GetNcoords_v(), GetIntegrable());
                R.Reset(integrable.GetNcoords_v());
                Qc.Reset(integrable.GetNconstr());
                L.Reset(integrable.GetNconstr());

                integrable.StateGather(ref X, ref V, ref T);  // state <- system

                Vold = V;

                // Perform a linearized semi-implicit Euler integration step
                //
                // [ M - dt*dF/dv - dt^2*dF/dx    Cq' ] [ v_new  ] = [ M*(v_old) + dt*f]
                // [ Cq                           0   ] [ -dt*l  ] = [ C/dt + Ct ]

                integrable.LoadResidual_F(ref R, dt);
                integrable.LoadResidual_Mv(ref R, V, 1.0);
                integrable.LoadConstraint_C(ref Qc, 1.0 / dt, false);
                integrable.LoadConstraint_Ct(ref Qc, 1.0);

                integrable.StateSolveCorrection(
                    ref V, ref L, R, Qc,
                    1.0,           // factor for  M
                    -dt,           // factor for  dF/dv
                    -dt * dt,      // factor for  dF/dx
                    X, V, T + dt,  // not needed
                    false,         // do not StateScatter update to Xnew Vnew T+dt before computing correction
                    true           // force a call to the solver's Setup() function
                    );

                integrable.StateScatter(X, V, T);  // state -> system

                L *= (1.0 / dt);  // Note it is not -(1.0/dt) because we assume StateSolveCorrection already flips sign of L

                if (action != 0 & AssemblyLevel.Enum.ACCELERATION != 0)
                {
                    integrable.StateScatterAcceleration(
                        (V - Vold) * (1 / dt));  // -> system auxiliary data (i.e acceleration as measure, fits DVI/MDI)

                    integrable.StateScatterReactions(L);  // -> system auxiliary data
                }
            }
        }

        /// Set the max number of Newton-Raphson iterations for the position assembly procedure.
        public void SetMaxAssemblyIters(int mi) { max_assembly_iters = mi; }
        /// Get the max number of Newton-Raphson iterations for the position assembly procedure.
        public int GetMaxAssemblyIters() { return max_assembly_iters; }

        /// Get the integrable object.
        public ChIntegrable GetIntegrable() { return integrable; }

        /// Access the Lagrange multipliers.
        public ChVectorDynamic<double> get_L() { return L; }

        /// Access the current position state vector.
        public ChState get_X() { return X; }

        /// Access the current velocity state vector.
        public ChStateDelta get_V() { return V; }

        /// Access the current acceleration state vector.
        public ChStateDelta get_A() { return A; }

    }
}
