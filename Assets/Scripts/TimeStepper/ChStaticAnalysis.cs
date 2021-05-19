using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono
{
    /// Base class for static analysis
    public abstract class ChStaticAnalysis
    {
        protected ChIntegrableIIorder integrable;

        protected ChState X;
        protected ChStateDelta V;
        protected ChStateDelta A;
        protected ChVectorDynamic<double> L;

        /// Constructor
        public ChStaticAnalysis(ChIntegrableIIorder mintegrable)
        {
            integrable = mintegrable;
            L.Reset(0);
            X.Reset(1, mintegrable);
            V.Reset(1, mintegrable);
            A.Reset(1, mintegrable);
        }

        /// Performs the static analysis
        public abstract void StaticAnalysis();

        /// Access the lagrangian multipliers, if any
        public virtual ChVectorDynamic<double> get_L() { return L; }

        /// Get the integrable object
        public ChIntegrable GetIntegrable() { return integrable; }

        /// Access the state, position part, at current analysis
        public virtual ChState get_X() { return X; }

        /// Access the state, speed part, at current analysis
        public virtual ChStateDelta get_V() { return V; }

        /// Access the acceleration, at current analysis
        public virtual ChStateDelta get_A() { return A; }
    }

    /// Linear static analysis

    public class ChStaticLinearAnalysis : ChStaticAnalysis
    {

        /// Constructor
        public ChStaticLinearAnalysis(ChIntegrableIIorder mintegrable) : base(mintegrable) { }


        /// Performs the static analysis,
        /// doing a linear solve.

        public override void StaticAnalysis()
        {
            ChIntegrableIIorder mintegrable = (ChIntegrableIIorder)this.integrable;

            // setup main vectors
            mintegrable.StateSetup(ref X, ref V, ref A);

            ChStateDelta Dx = new ChStateDelta();
            ChVectorDynamic<double> R = new ChVectorDynamic<double>();
            ChVectorDynamic<double> Qc = new ChVectorDynamic<double>();
            double T = 0;

            // setup auxiliary vectors
            Dx.Reset(mintegrable.GetNcoords_v(), GetIntegrable());
            R.Reset(mintegrable.GetNcoords_v());
            Qc.Reset(mintegrable.GetNconstr());
            L.Reset(mintegrable.GetNconstr());

            mintegrable.StateGather(ref X, ref V, ref T);  // state <- system

            // Set V speed to zero
            V.matrix.FillElem(0);
            mintegrable.StateScatter(X, V, T);  // state -> system

            // Solve:
            //
            // [-dF/dx     Cq' ] [ dx  ] = [ f]
            // [ Cq        0   ] [  l  ] = [ C]

            mintegrable.LoadResidual_F(ref R, 1.0);
            mintegrable.LoadConstraint_C(ref Qc, 1.0);

            mintegrable.StateSolveCorrection(
                ref Dx, ref L, R, Qc,
                0,        // factor for  M
                0,        // factor for  dF/dv
                -1.0,     // factor for  dF/dx (the stiffness matrix)
                X, V, T,  // not needed here
                false,    // do not StateScatter update to Xnew Vnew T+dt before computing correction
                true      // force a call to the solver's Setup() function
                );

            X += Dx;

            mintegrable.StateScatter(X, V, T);  // state -> system
            mintegrable.StateScatterReactions(L);  // -> system auxiliary data 
        }
    };

    /// Non-Linear static analysis

    public class ChStaticNonLinearAnalysis : ChStaticAnalysis
    {

        protected int maxiters;
        protected double tolerance;
        protected int incremental_steps;


        /// Constructor
        public ChStaticNonLinearAnalysis(ChIntegrableIIorder mintegrable)
        : base(mintegrable)
        {
            maxiters = 20;
            incremental_steps = 6;
            tolerance = 1e-10;
        }

        /// Performs the static analysis,
        /// doing a linear solve.

        public override void StaticAnalysis()
        {
            ChIntegrableIIorder mintegrable = (ChIntegrableIIorder)this.integrable;

            // setup main vectors
            mintegrable.StateSetup(ref X, ref V, ref A);

            ChState Xnew = new ChState();
            ChStateDelta Dx = new ChStateDelta();
            ChVectorDynamic<double> R = new ChVectorDynamic<double>();
            ChVectorDynamic<double> Qc = new ChVectorDynamic<double>();
            double T = 0;

            // setup auxiliary vectors
            Dx.Reset(mintegrable.GetNcoords_v(), GetIntegrable());
            Xnew.Reset(mintegrable.GetNcoords_x(), mintegrable);
            R.Reset(mintegrable.GetNcoords_v());
            Qc.Reset(mintegrable.GetNconstr());
            L.Reset(mintegrable.GetNconstr());

            mintegrable.StateGather(ref X, ref V, ref T);  // state <- system

            // Set speed to zero
            V.matrix.FillElem(0);

            // Extrapolate a prediction as warm start
            Xnew = X;

            // use Newton Raphson iteration to solve implicit Euler for v_new
            //
            // [ - dF/dx    Cq' ] [ Dx  ] = [ f ]
            // [ Cq         0   ] [ L   ] = [ C ]

            for (int i = 0; i < this.GetMaxiters(); ++i)
            {
                mintegrable.StateScatter(Xnew, V, T);  // state -> system
                R.Reset();
                Qc.Reset();
                mintegrable.LoadResidual_F(ref R, 1.0);
                mintegrable.LoadConstraint_C(ref Qc, 1.0);

                double cfactor = ChMaths.ChMin(1.0, ((double)(i + 2) / (double)(incremental_steps + 1)));
                R *= cfactor;
                Qc *= cfactor;

                //	GetLog()<< "Non-linear statics iteration=" << i << "  |R|=" << R.NormInf() << "  |Qc|=" << Qc.NormInf()
                //<< "\n";
                if ((R.matrix.NormInf() < this.GetTolerance()) && (Qc.matrix.NormInf() < this.GetTolerance()))
                    break;

                mintegrable.StateSolveCorrection(
                    ref Dx, ref L, R, Qc,
                    0,           // factor for  M
                    0,           // factor for  dF/dv
                    -1.0,        // factor for  dF/dx (the stiffness matrix)
                    Xnew, V, T,  // not needed here
                    false,       // do not StateScatter update to Xnew Vnew T+dt before computing correction
                    true         // force a call to the solver's Setup() function
                    );

                Xnew += Dx;
            }

            X = Xnew;

            mintegrable.StateScatter(X, V, T);  // state -> system
            mintegrable.StateScatterReactions(L);  // -> system auxiliary data 
        }

        /// Set the max number of iterations using the Newton Raphson procedure
        public void SetMaxiters(int miters)
        {
            maxiters = miters;
            if (incremental_steps > miters)
                incremental_steps = miters;
        }
        /// Get the max number of iterations using the Newton Raphson procedure
        public double GetMaxiters() { return maxiters; }

        /// Set the number of steps that, for the first iterations, make the residual
        /// growing linearly. If =0, no incremental application of residual, so it is
        /// a classic Newton Raphson iteration, otherwise acts as  continuation strategy.
        /// For values > 0 , it might help convergence. Must be less than maxiters.
        public void SetIncrementalSteps(int mist)
        {
            incremental_steps = mist;
            if (maxiters < incremental_steps)
                maxiters = incremental_steps;
        }
        /// Set the number of steps that, for the first iterations, make the residual
        /// growing linearly.
        public double GetIncrementalSteps() { return incremental_steps; }

        /// Set the tolerance for terminating the Newton Raphson procedure
        public void SetTolerance(double mtol) { tolerance = mtol; }
        /// Get the tolerance for terminating the Newton Raphson procedure
        public double GetTolerance() { return tolerance; }
    }

}
