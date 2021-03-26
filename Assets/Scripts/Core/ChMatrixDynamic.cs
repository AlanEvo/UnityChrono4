using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Runtime.InteropServices;
using System;

namespace chrono
{

    ///
    /// ChMatrixDynamic
    ///
    ///  Specialized 'resizeable' matrix class where the elements are allocated on heap.
    /// The size of the matrix can be known even at compile-time, and the matrix can
    /// be freely resized also after creation. The size is unlimited (until you have memory).
    ///  Although this is the most generic type of matrix, please do not use it
    /// where you know in advance its size because there are more efficient
    /// types for those matrices with 'static' size (for example, 3x3 rotation
    /// matrices are faster if created as ChMatrix33).
    public unsafe class ChMatrixDynamic<Real> : ChMatrix
    {

        //
        // DATA
        //

        /// [simply use the  "Real* address" pointer of the base class

        //
        // CONSTRUCTORS
        //

        /// The default constructor builds a 3x3 matrix.
        public ChMatrixDynamic()
        {
            this.rows = 3;
            this.columns = 3;
            this.address = new double[9];
           /* this.address = (double*)Marshal.AllocHGlobal(9 * sizeof(double));
            double[] array = new double[9];
            for (int i = 0; i < array.Length; i++)
            {
                this.address[i] = array[i];
            }*/
            for (int i = 0; i < 9; ++i)
                this.address[i] = 0;
        }

        /// Copy constructor
        public ChMatrixDynamic(ChMatrixDynamic<Real> msource)
        {
            this.rows = msource.GetRows();
            this.columns = msource.GetColumns();
            // this.address = new double[this.rows * this.columns];
            /*this.address = (double*)Marshal.AllocHGlobal(this.rows * this.columns * sizeof(double));
            double[] array = new double[this.rows * this.columns];
            for (int i = 0; i < array.Length; i++)
            {
                this.address[i] = array[i];
            }*/
            for (int i = 0; i < this.rows * this.columns; ++i)
                this.address[i] = msource.GetAddress()[i];
        }

        /// Copy constructor from all types of base matrices
        public ChMatrixDynamic(ChMatrix msource)
        {
            this.rows = msource.GetRows();
            this.columns = msource.GetColumns();
            this.address = new double[this.rows * this.columns];
           /* this.address = (double*)Marshal.AllocHGlobal(this.rows * this.columns * sizeof(double));
            double[] array = new double[this.rows * this.columns];
            for (int i = 0; i < array.Length; i++)
            {
                this.address[i] = array[i];
            }*/
            for (int i = 0; i < this.rows * this.columns; ++i)
                this.address[i] = msource.GetAddress()[i];
        }

        /// The constructor for a generic nxm matrix.
        /// Rows and columns cannot be zero or negative.
        public ChMatrixDynamic(int row, int col) {
           // Debug.Assert(row >= 0 && col >= 0);
            this.rows = row;
            this.columns = col;
            this.address = new double[row * col + 3];
           /* this.address = (double*)Marshal.AllocHGlobal(row * col + 3 * sizeof(double));
            double[] array = new double[row * col + 3];
            for (int i = 0; i < array.Length; i++)
            {
                this.address[i] = array[i];
            }*/
            // SetZero(row*col);
            for (int i = 0; i < this.rows * this.columns; ++i)
                this.address[i] = 0;
        }

       
        //
        // FUNCTIONS
        //

        /// Reallocate memory for a new size.
        public override void Resize(int nrows, int ncols)
        {
           // Debug.Assert(nrows >= 0 && ncols >= 0);
            if ((nrows != this.rows) || (ncols != this.columns))
            {
                this.rows = nrows;
                this.columns = ncols;
                this.address = null;
                this.address = new double[this.rows * this.columns];
                /*this.address = (double*)Marshal.AllocHGlobal(this.rows * this.columns * sizeof(double));
                 double[] array = new double[this.rows * this.columns];
                 for (int i = 0; i < array.Length; i++)
                 {
                     this.address[i] = array[i];
                 }*/
                // SetZero(this->rows*this->columns);
                for (int i = 0; i < this.rows * this.columns; ++i)
                    this.address[i] = 0;
            }
        }

        public static ChMatrixDynamic<Real> operator *(ChMatrixDynamic<Real> mat, ChMatrix matbis)
        {
            ChMatrixDynamic<Real> result = new ChMatrixDynamic<Real>(mat.rows, matbis.GetColumns());
            result.MatrMultiply(mat, matbis);
            return result;
        }
    }
}
