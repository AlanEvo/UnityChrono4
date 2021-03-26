using System.Collections;
using System.Collections.Generic;
using System;
using System.Runtime.InteropServices;
using UnityEngine;

namespace chrono
{
    /// ChVectorDynamic
    ///
    ///  Specialized 'resizeable' vector class where the elements are allocated on heap.
    /// This is a 'one column matrix', so one could use ChMatrixDynamic<>(nrows,1) too,
    /// but this makes syntax more clear.
    ///  The size of the vector can be known even at compile-time, and the vector can
    /// be freely resized also after creation. The size is unlimited (until you have memory).
    ///  Although this is a generic type of vector, please do not use it for 3D vectors
    /// because there is already the specific ChVector<> class that implements lot of features
    /// for 3D vectors.
    public unsafe class ChVectorDynamic<Real> : ChMatrix where Real : unmanaged, IConvertible 
    {
        /// [simply use the  "Real* address" pointer of the base class


        //
        // CONSTRUCTORS
        //

        /// The default constructor builds a 1 element vector.
        public ChVectorDynamic()
        {
            this.rows = 1;
            this.columns = 1;
            this.address = new double[1];
           /* this.address = (double*)Marshal.AllocHGlobal(1 * sizeof(double));
            double[] array = new double[1];
            for (int i = 0; i < array.Length; i++)
            {
                this.address[i] = array[i];
            }*/
            // SetZero(1);
            this.address[0] = 0;
        }

        /// The constructor for a generic n sized vector.
        /// Rows cannot be zero or negative.
        public ChVectorDynamic(int rows)
        {
            //Debug.Assert(rows >= 0);
            this.rows = rows;
            this.columns = 1;
            this.address = new double[rows];
           /* this.address = (double*)Marshal.AllocHGlobal(rows * sizeof(double));
             double[] array = new double[rows];
             for (int i = 0; i < array.Length; i++)
             {
                 this.address[i] = array[i];
             }*/
            // SetZero(rows);
            for (int i = 0; i < this.rows; ++i)
                this.address[i] = 0;
        }

        /// Copy constructor
        public ChVectorDynamic(ChVectorDynamic<Real> msource)
        {
            this.rows = msource.GetRows();
            this.columns = 1;
            this.address = new double[this.rows];
            /*this.address = (double*)Marshal.AllocHGlobal(rows * sizeof(double));
             double[] array = new double[this.rows];
             for (int i = 0; i < array.Length; i++)
             {
                 this.address[i] = array[i];
             }*/
            // ElementsCopy(this->address, msource.GetAddress(), this->rows);
            for (int i = 0; i < this.rows; ++i)
                this.address[i] = msource.GetAddress()[i];
        }

        /// Copy constructor from all types of base matrices        /// Note! Assumes that the source matrix has one column only! There is no run-time check for the one-column sizing.

        public ChVectorDynamic(ChMatrix msource)
        {
          // Debug.Assert(msource.GetColumns() == 1);
            this.rows = msource.GetRows();
            this.columns = 1;
            this.address = new double[this.rows];
            /*this.address = (double*)Marshal.AllocHGlobal(this.rows * sizeof(double));
             double[] array = new double[this.rows];
             for (int i = 0; i < array.Length; i++)
             {
                 this.address[i] = array[i];
             }*/
            // ElementsCopy(this->address, msource.GetAddress(), this->rows);
            for (int i = 0; i < this.rows; ++i)
                this.address[i] = msource.GetAddress()[i];
        }

        /// Return the length of the vector
        public int GetLength() { return this.rows; }

        //
        // OPERATORS
        //

        public static ChVectorDynamic<Real> operator -(ChVectorDynamic<Real> a, ChMatrix matbis)
        {
            a.MatrSub(a, matbis);
            return a;
        }


        public static ChVectorDynamic<Real> operator *(ChVectorDynamic<Real> a, double factor)
        {
            a.MatrScale(factor);
            return a;
        }

        //
        // FUNCTIONS
        //

        /// Reallocate memory for a new size.
        public virtual void Resize(int nrows)
        {
           // Debug.Assert(nrows >= 0);
            //Debug.Log("constr " + this.rows);
            if (nrows != this.rows)
            {
                //Debug.Log("yaya! " + this.rows);
                this.rows = nrows;
                this.columns = 1;
                this.address = null;
                this.address = new double[this.rows];
               /* this.address = (double*)Marshal.AllocHGlobal(this.rows * sizeof(double));
                double[] array = new double[this.rows];
                for (int i = 0; i < array.Length; i++)
                {
                    this.address[i] = array[i];
                }*/
                // SetZero(this->rows);
                for (int i = 0; i < this.rows; ++i)
                    this.address[i] = 0;
            }
        }

        public override void Resize(int nrows, int ncols)
        {
          //  Debug.Assert(ncols == 1);
            Resize(nrows);
        }
        /// Reset to zeroes and (if needed) changes the size to have row and col
        public void Reset(int nrows)
        {
            Resize(nrows);
            //SetZero(rows);
            for (int i = 0; i < this.rows; ++i)
                this.address[i] = 0;
        }

        /// Resets the matrix to zero  (warning: simply sets memory to 0 bytes!)
        public override void Reset()
        {
            // SetZero(rows*columns); //memset(address, 0, sizeof(Real) * rows * columns);
            for (int i = 0; i < this.rows; ++i)
                this.address[i] = 0;
        }

    }
}
