using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Reflection;
using System.Runtime.InteropServices;
using System.Linq;

namespace chrono
{


    ///
    /// ChMatrixNM
    ///
    /// Specialized matrix class having a pre-allocated NxM buffer of elements on stack.
    ///  This means that elements are put the stack when the matrix is created, and this
    /// can be more efficient than heap allocation (but, please, do not use too large N or M
    /// sizes, this is meant to work up to 10x10, for example; prefer ChMatrixDyamic for larger).
    ///  The NxM size must be known 'statically', at compile-time; however later at run-time
    /// this matrix can be resized anyway (but for larger size than NxM, it falls back to
    /// heap allocation). Note that if resizing is often required, it may be better
    /// to create a ChMatrixDyamic instead, from the beginning.
    public unsafe class ChMatrixNM<A, B> : ChMatrix       
       // where Real : unmanaged
        where A : IntInterface.IBase, new()
        where B : IntInterface.IBase, new()
    {
        // private dynamic preall_rows = typeof(A);
        // private dynamic preall_columns = typeof(B);
        protected double[] buffer;// = new double[0];

        public static ChMatrixNM<IntInterface.Three, IntInterface.Four> NMNULL3_4 = new ChMatrixNM<IntInterface.Three, IntInterface.Four>();
        public static ChMatrixNM<IntInterface.Four, IntInterface.Four> NMNULL4_4 = new ChMatrixNM<IntInterface.Four, IntInterface.Four>();

        //
        // CONSTRUCTORS
        //

        /// The default constructor builds a NxN matrix
        public ChMatrixNM()   
        {
         
            this.rows = new A().Masta;
            this.columns = new B().Masta;
            buffer = new double[this.rows * this.columns + 3];
            this.address = buffer;
            /*this.address = (double*)Marshal.AllocHGlobal(buffer.Length * sizeof(double));
            for (int i = 0; i < this.buffer.Length; i++)
            {
                this.address[i] = this.buffer[i];
            }*/
            //for (int i = 0; i < buffer.Length; ++i)
            //  this.address[i] = buffer[i];
            SetZero(this.rows * this.columns);
        }

        public ChMatrixNM(bool a)
        {

            this.rows = 3;//new A().Masta;
            this.columns = 3;// new B().Masta;
            buffer = new double[this.rows * this.columns + 3];
            this.address = buffer;
            /*this.address = (double*)Marshal.AllocHGlobal(buffer.Length * sizeof(double));
            for (int i = 0; i < this.buffer.Length; i++)
            {
                this.address[i] = this.buffer[i];
            }*/
            //for (int i = 0; i < buffer.Length; ++i)
            //  this.address[i] = buffer[i];
            SetZero(this.rows * this.columns);
        }


        /// Copy constructor
        public ChMatrixNM(ChMatrixNM<A, B> msource)
        {
            this.rows = new A().Masta;
            this.columns = new B().Masta;
            buffer = new double[this.rows * this.columns + 3];
            // for (int i = 0; i < buffer.Length; ++i)
            this.address = buffer;
            /*this.address = (double*)Marshal.AllocHGlobal(buffer.Length * sizeof(double));
             for (int i = 0; i < this.buffer.Length; i++)
             {
                 this.address[i] = this.buffer[i];
             }*/
            //this.address = buffer;
            ElementsCopy(this.address, msource.address, this.rows * this.columns);
        }
        public void SetZero(int els)
        {
            for (int i = 0; i < els; ++i)
            {
                this.address[i] = 0;
            }
        }

        public void ElementsCopy(double[] to, double[] from, int els)
        {
            for (int i = 0; i < els; ++i)
                to[i] = from[i];
        }

        /// Copy constructor from all types of base matrices (only with same size)
        public ChMatrixNM(ChMatrix msource)
        {
           // Debug.Assert(msource.GetColumns() == preall_columns && msource.GetRows() == preall_rows);
           // this.rows = preall_rows;
           // this.columns = preall_columns;
           // this.address = buffer;
           /* this.address = (double*)Marshal.AllocHGlobal(buffer.Length * sizeof(double));
            for (int i = 0; i < this.buffer.Length; i++)
            {
                this.address[i] = this.buffer[i];
            }*/
            /*   for (int i = 0; i < buffer.Length; ++i)
                   this.address[i] = buffer[i];*/
            //  ElementsCopy(this.address, msource.GetAddress(), preall_rows * preall_columns);
            ElementsCopy(this.address, msource.address, this.rows * this.columns);
        }


        //
        // FUNCTIONS
        //

        /// Resize for this matrix is NOT SUPPORTED ! DO NOTHING!
        public override void Resize(int nrows, int ncols) { /*Debug.Assert((nrows == this.rows) && (ncols == this.columns));*/ }

    }
}
