using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

namespace chrono
{
    /// Class for state of time-integrable objects.
    /// This is a vector (one-column matrix), hence inherited from ChMatrixDynamic.
    /// It is used in ChIntegrable, ChTimestepper, etc.
    public class ChState : ChVectorDynamic<double>
    {
        /// Constructors
        public ChState(ChIntegrable mint = null) : base(1) { integrable = mint; }

        public ChState(int nrows, ChIntegrable mint) : base(nrows) { integrable = mint; }

        public ChState(ChMatrixDynamic<double> matr, ChIntegrable mint) : base(matr.matrix) {
            integrable = mint;
        }

        /// Copy constructor
        public ChState(ChState msource) : base(msource) { integrable = msource.integrable; }

        /// Increments this matrix by another matrix, in place
        /*  public static ChState operator +(ChState state, ChMatrix<double> matbis) {
              state.MatrInc(matbis);
              return state;
          }*/

        public static ChState operator +(ChState y, ChStateDelta Dy)
        {
            ChState tmp_y = new ChState(y);
            y.GetIntegrable().StateIncrement(ref y, tmp_y, Dy);
            return y;
        }


    /// Subtracts this matrix and another matrix.
    /// Performance warning: a new object is created.
    public static ChState operator -(ChState a, ChMatrix matbis)
        {
            ChState result = new ChState(matbis.rows, a.integrable);
            result.matrix.MatrSub(result.matrix, matbis);
            return result;
        }

        /// Reset to zeroes and (if needed) changes the size.
        public void Reset(int nrows, ChIntegrable mint)
        {
            Reset(nrows);
            integrable = mint;
        }


        public ChIntegrable GetIntegrable() { return integrable; }

        private ChIntegrable integrable;
    }

    /// Class for incremental form of state of time-integrable objects.
    /// Note that for many cases, this would be superfluous, because one could
    /// do y_new = y_old + dydt*td, where dydt is a ChState just like y and y_new, but there
    /// are cases where such simple "+" operations between vectors is not practical, for instance
    /// when integrating rotations in 3D space, where it is better to work with quaternions in y
    /// and y_new, but with spinors/angular velocities/etc. in dydt; so dim(y) is not dim(dydt);
    /// hence the need of this specific class for increments in states.
    /// This is a vector (one-column matrix), hence inherited from ChMatrixDynamic.
    /// It is used in ChIntegrable, ChTimestepper, etc.

    public class ChStateDelta : ChVectorDynamic<double>
    {
        /// Constructors
        public ChStateDelta(ChIntegrable mint = null) : base(1) { integrable = mint; }

        public ChStateDelta(int nrows, ChIntegrable mint) : base(nrows) { integrable = mint; }

        public ChStateDelta(ChMatrixDynamic<double> matr, ChIntegrable mint) : base(matr.matrix) {
            integrable = mint;
        }

        /// Copy constructor
        public ChStateDelta(ChStateDelta msource) : base(msource) { integrable = msource.integrable; }

        /// Increments this matrix by another matrix, in place
        public static ChStateDelta operator +(ChStateDelta state, ChMatrix matbis)
        {
            state.matrix.MatrInc(matbis);
            return state;
        }

        /// Subtracts this matrix and another matrix.
        /// Performance warning: a new object is created.
        public static ChStateDelta operator -(ChStateDelta a, ChMatrix matbis)
        {
            ChStateDelta result = new ChStateDelta(matbis.rows, a.integrable);
            result.matrix.MatrSub(a.matrix, matbis);
            return result;
        }

        public static ChStateDelta operator -(ChStateDelta a, ChStateDelta matbis)
        {
            ChStateDelta result = new ChStateDelta(matbis.matrix.rows, a.integrable);
            result.matrix.MatrSub(a.matrix, matbis.matrix);
            return result;
        }

        public static ChStateDelta operator *(ChStateDelta a, double factor)
        {
            ChStateDelta result = new ChStateDelta(a);
            result.matrix.MatrScale(factor);
            return result;
        }

        // Negates sign of the matrix.
        // Performance warning: a new object is created.
        public static ChStateDelta operator -(ChStateDelta a)
        {
            a.matrix.MatrNeg();
            return a;
        }

        /// Reset to zeroes and (if needed) changes the size.
        public void Reset(int nrows, ChIntegrable mint)
        {
            Reset(nrows);
            integrable = mint;
        }


        public ChIntegrable GetIntegrable() { return integrable; }

        private ChIntegrable integrable;
    }

}
