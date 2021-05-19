using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Reflection;
using System.Runtime.InteropServices;
//using System.Linq;

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
    public struct ChMatrixNM<A, B>// : ChMatrix   
        
       // where Real : unmanaged
        where A : IntInterface.IBase, new()
        where B : IntInterface.IBase, new()
    {
        public ChMatrix matrix;

        // private dynamic preall_rows = typeof(A);
        // private dynamic preall_columns = typeof(B);
        public double[] buffer;// = new double[0];

        public static ChMatrixNM<IntInterface.One, IntInterface.One> NMNULL1_1 = new ChMatrixNM<IntInterface.One, IntInterface.One>(0);
        public static ChMatrixNM<IntInterface.Six, IntInterface.One> NMNULL6_1 = new ChMatrixNM<IntInterface.Six, IntInterface.One>(0);
        public static ChMatrixNM<IntInterface.Three, IntInterface.Four> NMNULL3_4 = new ChMatrixNM<IntInterface.Three, IntInterface.Four>(0);
        public static ChMatrixNM<IntInterface.Four, IntInterface.Four> NMNULL4_4 = new ChMatrixNM<IntInterface.Four, IntInterface.Four>(0);

        //
        // CONSTRUCTORS
        //

        /// The default constructor builds a NxN matrix
        public ChMatrixNM(int a) //: this()
        {
            matrix = new ChMatrix();
            this.matrix.rows = new A().Masta;
            this.matrix.columns = new B().Masta;
            buffer = new double[this.matrix.rows * this.matrix.columns + 3];
            this.matrix.address = buffer;
            /*this.address = (double*)Marshal.AllocHGlobal(buffer.Length * sizeof(double));
            for (int i = 0; i < this.buffer.Length; i++)
            {
                this.address[i] = this.buffer[i];
            }*/
            //for (int i = 0; i < buffer.Length; ++i)
            //  this.address[i] = buffer[i];
            SetZero(this.matrix.rows * this.matrix.columns);
        }

        public ChMatrixNM(bool a) //: this()
        {

            this.matrix.rows = 3;//new A().Masta;
            this.matrix.columns = 3;// new B().Masta;
            buffer = new double[this.matrix.rows * this.matrix.columns + 3];
            this.matrix.address = buffer;
            /*this.address = (double*)Marshal.AllocHGlobal(buffer.Length * sizeof(double));
            for (int i = 0; i < this.buffer.Length; i++)
            {
                this.address[i] = this.buffer[i];
            }*/
            //for (int i = 0; i < buffer.Length; ++i)
            //  this.address[i] = buffer[i];
            SetZero(this.matrix.rows * this.matrix.columns);
        }


        /// Copy constructor
        public ChMatrixNM(ChMatrixNM<A, B> msource)//: this()
        {
            this.matrix.rows = new A().Masta;
            this.matrix.columns = new B().Masta;
            buffer = new double[this.matrix.rows * this.matrix.columns + 3];
            // for (int i = 0; i < buffer.Length; ++i)
            this.matrix.address = buffer;
            /*this.address = (double*)Marshal.AllocHGlobal(buffer.Length * sizeof(double));
             for (int i = 0; i < this.buffer.Length; i++)
             {
                 this.address[i] = this.buffer[i];
             }*/
            //this.address = buffer;
            ElementsCopy(this.matrix.address, msource.matrix.address, this.matrix.rows * this.matrix.columns);
        }
        public void SetZero(int els)
        {
            for (int i = 0; i < els; ++i)
            {
                this.matrix.address[i] = 0;
            }
        }

        public void ElementsCopy(double[] to, double[] from, int els)
        {
            for (int i = 0; i < els; ++i)
                to[i] = from[i];
        }

        /// Copy constructor from all types of base matrices (only with same size)
        public ChMatrixNM(ChMatrix msource): this()
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
            ElementsCopy(this.matrix.address, msource.address, this.matrix.rows * this.matrix.columns);
        }


        //
        // FUNCTIONS
        //

        /// Resize for this matrix is NOT SUPPORTED ! DO NOTHING!
        public void Resize(int nrows, int ncols) { /*Debug.Assert((nrows == this.rows) && (ncols == this.columns));*/ }

    }
}
