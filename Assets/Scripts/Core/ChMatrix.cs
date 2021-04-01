using System.Collections.Generic;
using System;
using UnityEngine;
using System.Runtime.InteropServices;
using UnityEngine.Assertions;

namespace chrono
{



    ///
    /// ChMatrix:
    ///
    ///  A base class for matrix objects (tables of NxM numbers).
    /// To access elements, the indexes start from zero, and
    /// you must indicate first row, then column, that is: m(2,4)
    /// means the element at 3rd row, 5th column.
    ///  This is an abstract class, so you cannot instantiate
    /// objects from it: you must rather create matrices using the
    /// specialized child classes like ChMatrixDynamic, ChMatrixNM,
    /// ChMatrix33 and so on; all of them have this same base class.
    ///  Warning: for optimization reasons, not all functions will
    /// check about boundaries of element indexes and matrix sizes (in
    /// some cases, if sizes are wrong, debug asserts are used).
    public class ChMatrix
    {
        //
        // DATA
        //
        public int rows = 1;
        public int columns = 1;

        public double[] address;

        //
        // CONSTRUCTORS (none - abstract class that must be implemented with child classes)
        //
 

        //
        // OPERATORS OVERLOADING
        //         

        public ref double this[int row, int col]
        {
            get
            {
                return ref address[col + (row * columns)];
               // return ref (*(address + col + (row * columns)));
            }
        }

        public ref double this[int el]
        {
            get
            {
                return ref address[el];
                //return ref (*(address + el));
            }
        }

        /*  public static implicit operator ChMatrix<double>(int el)
          {
              Debug.Assert(row >= 0 && col >= 0 && row < rows && col < columns);
              return address[col + row * columns] = val;
          }*/

        /// *Alternative* Parenthesis () operator, to access a single element of the matrix, by
        /// supplying the row and the column (indexes start from 0).
        /// For example: m(3,5) gets the element at the 4th row, 6th column.
        /// Value is returned by reference, so it can be modified, like in m(1,2)=10.
        /* public double Operator(int row, int col)
         {
             Debug.Assert(row >= 0 && col >= 0 && row < rows && col < columns);
             return ((address + (dynamic)col + (row * columns)));
         }
         public void Operator(int row, int col, double val)
         {
             Debug.Assert(row >= 0 && col >= 0 && row < rows && col < columns);
             address[col + row * columns] = val;
         }

         public virtual double Operator(int el)
         {
             Debug.Assert(el >= 0 && el < rows * columns);
             return address[el];
         }

         public virtual void Operator(int el, double val)
         {
             Debug.Assert(el >= 0 && el < rows * columns);
             address[el] = val;
         }*/

        /// Multiplies this matrix by a factor, in place
        public static ChMatrix operator *(ChMatrix a, double factor)
        {
            a.MatrScale(factor);
            return a;
        }

        public static ChMatrix operator +(ChMatrix a, ChMatrixDynamic<double> mat)
        {
            a.MatrScale(mat);
            return a;
        }

        public static ChMatrix operator -(ChMatrix a, double factor)
        {
            a.MatrScale(factor);
            return a;
        }

        // Macro conversions
        public void Set33Element(int a, int b, double val) { SetElementN(((a * 3) + (b)), val); }
        public double Get33Element(int a, int b) { return GetElementN((a * 3) + (b)); }

        public void Set44Element(int a, int b, double val) { SetElementN(((a * 4) + (b)), val); }
        public double Get34Element(int a, int b) { return GetElementN((a * 4) + (b)); }



        //
        // FUNCTIONS
        //

        /// Sets the element at row,col position. Indexes start with zero.
        public void SetElement(int row, int col, double elem)
        {
            //Debug.Assert(row >= 0 && col >= 0 && row < rows && col < columns);  // boundary checks
            address[col + (row * columns)] = elem;
            //*(address + col + (row * columns)) = elem;
        }

        /// Gets the element at row,col position. Indexes start with zero.
        /// The return value is a copy of original value. Use Element() instead if you
        /// want to access directly by reference the original element.
        public double GetElement(int row, int col)
        {
            //Debug.Assert(row >= 0 && col >= 0 && row < rows && col < columns);  // boundary checks
            return address[col + (row * columns)];
           // return (*(address + col + (row * columns)));
        }

        /// Sets the Nth element, counting row after row.
        void SetElementN(int index, double elem)
        {
            // Debug.Assert(index >= 0 && index < (rows * columns));  // boundary checks
            address[index] = elem;
            //*(address + index) = elem;
        }

        /// Gets the Nth element, counting row after row.
        public double GetElementN(int index)
        {
            // Debug.Assert(index >= 0 && index < (rows * columns));
            return address[index];
           // return (*(address + index));
        }

        /// Access a single element of the matrix, by
        /// supplying the row and the column (indexes start from 0).
        /// Value is returned by reference, so it can be modified, like in m.Element(1,2)=10.
        public ref double Element(int row, int col)
        {
            // Debug.Assert(row >= 0 && col >= 0 && row < rows && col < columns);
             return ref address[col + row * columns];
           // return ref (*(address + col + (row * columns)));
        }

        /// Access a single element of the matrix, the Nth element, counting row after row.
        /// Value is returned by reference, so it can be modified, like in m.Element(5)=10.
        public ref double ElementN(int index)
        {
            // Debug.Assert(index >= 0 && index < (rows * columns));
            return ref address[index];
           // return ref (*(address + index));
        }

        /// Access directly the "double* address" buffer. Warning! this is a low level
        /// function, it should be used in rare cases, if really needed!
        public double[] GetAddress() { return address; }

        /// Gets the number of rows
        public int GetRows() { return rows; }

        /// Gets the number of columns
        public int GetColumns() { return columns; }

        /// Returns true if vector is identical to other matrix
        public bool Equals(ChMatrix other)
        {
            return Equals(other, 0.0);
        }

        /// Returns true if vector equals another vector, within a tolerance 'tol'
        public bool Equals(ChMatrix other, double tol)
        {
            if ((other.GetColumns() != this.columns) || (other.GetRows() != this.rows))
                return false;
            for (int nel = 0; nel < rows * columns; ++nel)
                if (Math.Abs(ElementN(nel) - other.ElementN(nel)) > tol)
                    return false;
            return true;
        }


        /// doublelocate memory for a new size. VIRTUAL! Must be implemented by child classes!
        public virtual void Resize(int nrows, int ncols) { }

        /// Swaps the columns a and b
        public void SwapColumns(int a, int b)
        {
            double temp;
            for (int i = 0; i < rows; i++)
            {
                temp = GetElement(i, a);
                SetElement(i, a, GetElement(i, b));
                SetElement(i, b, temp);
            }
        }

        /// Fill the diagonal elements, given a sample.
        /// Note that the matrix must already be square (no check for
        /// rectangular matrices!), and the extra-diagonal elements are
        /// not modified -this function does not set them to 0-
        public void FillDiag(double sample)
        {
            for (int i = 0; i < rows; ++i)
                SetElement(i, i, sample);
        }

        /// Fill the matrix with the same value in all elements
        public void FillElem(double sample)
        {
            for (int i = 0; i < rows * columns; ++i)
                SetElementN(i, sample);
        }

        /// Fill the matrix with random float numbers, falling within the
        /// "max"/"min" range.
        public void FillRandom(double max, double min)
        {
            for (int i = 0; i < rows * columns; ++i)
            {
                SetElementN(i, min + ChMaths.ChRandom() * (max - min));
            }
        }

        /// Resets the matrix to zero  (warning: simply sets memory to 0 bytes!)
        public virtual void Reset()
        {
            // SetZero(rows*columns); //memset(address, 0, sizeof(double) * rows * columns);
            for (int i = 0; i < rows * columns; ++i)
                this.address[i] = 0;
        }


        /// Reset to zeroes and (if needed) changes the size to have row and col
        public void Reset(int nrows, int ncols)
        {
            Resize(nrows, ncols);
            // SetZero(rows*columns); //memset(address, 0, sizeof(double) * rows * columns);
            for (int i = 0; i < rows * columns; ++i)
                this.address[i] = 0;
        }

        /// Reset to identity matrix (ones on diagonal, zero elsewhere)
        public void SetIdentity()
        {
            Reset();
            FillDiag(1.0);
        }

        /// Copy a matrix "matra" into this matrix. Note that
        /// the destination matrix will be resized if necessary.
        //template<class doubleB>
        public void CopyFromMatrix(ChMatrix matra)
        {
            Resize(matra.GetRows(), matra.GetColumns());
            for (int i = 0; i < rows * columns; ++i)
                address[i] = matra.GetAddress()[i];
        }

        /// Copy the transpose of matrix "matra" into this matrix. Note that
        /// the destination matrix will be resized if necessary.
        public void CopyFromMatrixT(ChMatrix matra)
        {
            Resize(matra.GetColumns(), matra.GetRows());
            for (int i = 0; i < matra.GetRows(); ++i)
                for (int j = 0; j < matra.GetColumns(); ++j)
                    SetElement(j, i, matra.Element(i, j));
        }

        /// Copy the transposed upper triangular part of "matra" in the lower triangular
        /// part of this matrix. (matra must be square)
        /// Note that the destination matrix will be resized if necessary.
        //                                                      _______                       //
        public void CopyTUpMatrix(ChMatrix matra)   //    \      |          |\          //
        {                                                 //      \  A'|   --->   |  \        //
            Resize(matra.GetRows(), matra.GetColumns());  //        \  |          |this\      //
            for (int i = 0; i < matra.GetRows(); i++)
            {                                             //          \|          |______\    //
                for (int j = 0; j < matra.GetRows(); j++)
                    SetElement(j, i, matra.GetElement(i, j));
            }
        }

        /// Copy the transposed lower triangulat part of "matra" in the upper triangular
        /// part of this matrix. (matra must be square)
        /// Note that the destination matrix will be resized if necessary.
        //                      _______     //
        public void CopyTLwMatrix(ChMatrix matra)  //    |\                \      |    //
        {                                                 //    |  \      --->      \this|    //
            Resize(matra.GetRows(), matra.GetColumns());  //    |A'  \                \  |    //
            for (int i = 0; i < matra.GetRows(); i++)
            {   //    |______\                \|    //
                for (int j = 0; j < matra.GetRows(); j++)
                    SetElement(i, j, matra.GetElement(j, i));
            }
        }


        //
        // MATH MEMBER FUNCTIONS.
        // For speed reasons, sometimes size checking of operands is left to the user!
        //

        /// Changes the sign of all the elements of this matrix, in place.
        public void MatrNeg()
        {
            for (int nel = 0; nel < rows * columns; ++nel)
                ElementN(nel) = -ElementN(nel);
        }

        /// Increments this matrix by \p val, as [this]+=val
        public void MatrInc(double val)
        {
            for (int nel = 0; nel < rows * columns; ++nel)
                ElementN(nel) += val;
        }

        public void MatrInc(ChMatrix matra)
        {
            //  Debug.Assert(matra.GetColumns() == columns && matra.GetRows() == rows);
            for (int nel = 0; nel < rows * columns; ++nel)
                ElementN(nel) += matra.ElementN(nel);
        }

        /// Returns the vector clipped from insrow, inscol.
        public ChVector ClipVector(int insrow, int inscol)
        {
            return new ChVector(Element(insrow, inscol), Element(insrow + 1, inscol), Element(insrow + 2, inscol));
        }

        /// Returns the quaternion clipped from insrow, inscol.
        public ChQuaternion ClipQuaternion(int insrow, int inscol)
        {
            return new ChQuaternion(Element(insrow, inscol), Element(insrow + 1, inscol), Element(insrow + 2, inscol),
                                      Element(insrow + 3, inscol));
        }

        /// Returns the coordsys clipped from insrow, inscol.
        public ChCoordsys<double> ClipCoordsys(int insrow, int inscol)
        {
            return new ChCoordsys<double>(ClipVector(insrow, inscol), ClipQuaternion(insrow + 3, inscol));
        }

        /// Paste a matrix "matra" into "this", inserting at location insrow-inscol.
        /// Normal copy for insrow=inscol=0
        public void PasteMatrix(ChMatrix matra, int insrow, int inscol)
        {
            for (int i = 0; i < matra.GetRows(); ++i)
                for (int j = 0; j < matra.GetColumns(); ++j)
                    Element(i + insrow, j + inscol) = matra.Element(i, j);
        }

        /// Paste a vector "va" into the matrix, summing it with preexisting values.
        public unsafe void PasteSumVector(ChVector va, int insrow, int inscol)
        {
            Element(insrow + 0, inscol) += va.x;
            Element(insrow + 1, inscol) += va.y;
            Element(insrow + 2, inscol) += va.z;
        }

        /// Paste a clipped portion of the matrix "matra" into "this",
        /// inserting the clip (of size nrows, ncolumns) at the location insrow-inscol.
        public void PasteClippedMatrix(ChMatrix matra,
                                int cliprow,
                                int clipcol,
                                int nrows,
                                int ncolumns,
                                int insrow,
                                int inscol)
        {
            for (int i = 0; i < nrows; ++i)
                for (int j = 0; j < ncolumns; ++j)
                    Element(i + insrow, j + inscol) = matra.Element(i + cliprow, j + clipcol);
        }

        /// Multiplies this 3x4 matrix by a quaternion, as v=[G]*q
        /// The matrix must be 3x4.
        ///  \return The result of the multiplication, i.e. a vector.
        public ChVector Matr34_x_Quat(ChQuaternion qua)
        {
            // Debug.Assert((rows == 3) && (columns == 4));
            return new ChVector(Get34Element(0, 0) * qua.e0 + Get34Element(0, 1) * qua.e1 +
                                      Get34Element(0, 2) * qua.e2 + Get34Element(0, 3) * qua.e3,
                                  Get34Element(1, 0) * qua.e0 + Get34Element(1, 1) * qua.e1 +
                                      Get34Element(1, 2) * qua.e2 + Get34Element(1, 3) * qua.e3,
                                  Get34Element(2, 0) * qua.e0 + Get34Element(2, 1) * qua.e1 +
                                      Get34Element(2, 2) * qua.e2 + Get34Element(2, 3) * qua.e3);
        }

        /// Multiplies two matrices (the second is considered transposed): [this]=[A]*[B]'
        /// Faster than doing B.MatrTranspose(); result.MatrMultiply(A,B);
        /// Note: no check on mistaken size of this!
        public void MatrMultiplyT(ChMatrix matra, ChMatrix matrb)
        {
            // Debug.Assert(matra.GetColumns() == matrb.GetColumns());
            // Debug.Assert(this.rows == matra.GetRows());
            // Debug.Assert(this.columns == matrb.GetRows());
            int col, row, colres;
            double sum;
            for (colres = 0; colres < matrb.GetRows(); ++colres)
            {
                for (row = 0; row < matra.GetRows(); ++row)
                {
                    sum = 0;
                    for (col = 0; col < matra.GetColumns(); ++col)
                    {
                        sum += matra.Element(row, col) * matrb.Element(colres, col);
                    }
                    SetElement(row, colres, sum);
                }
            }
        }

        /// Transpose this matrix in place
        public void MatrTranspose()
        {
            if (columns == rows)  // Square transp.is optimized
            {
                for (int row = 0; row < rows; ++row)
                    for (int col = row; col < columns; ++col)
                        if (row != col)
                        {
                            double temp = Element(row, col);
                            Element(row, col) = Element(col, row);
                            Element(col, row) = temp;
                        }
                int tmpr = rows;
                rows = columns;
                columns = tmpr;
            }
            else  // Naive implementation for rectangular case. Not in-place. Slower.
            {
                ChMatrixDynamic<double> matrcopy = (ChMatrixDynamic<double>)this;
                int tmpr = rows;
                rows = columns;
                columns = tmpr;  // dont' realloc buffer, anyway
                for (int row = 0; row < rows; ++row)
                    for (int col = 0; col < columns; ++col)
                        Element(row, col) = matrcopy.Element(col, row);
            }
        }

        //
        // BOOKKEEPING
        //

        /// Paste a matrix "matra" into "this", inserting at location insrow-inscol
        /// and performing a sum with the preexisting values.
        public void PasteSumMatrix(ChMatrix matra, int insrow, int inscol)
        {
            for (int i = 0; i < matra.GetRows(); ++i)
                for (int j = 0; j < matra.GetColumns(); ++j)
                    Element(i + insrow, j + inscol) += matra.Element(i, j);
        }

        //
        // MATH MEMBER FUNCTIONS.
        // For speed reasons, sometimes size checking of operands is left to the user!
        //

        /// Scales a matrix, multiplying all elements by a constant value: [this]*=f
        public void MatrScale(double factor)
        {
            for (int nel = 0; nel < rows * columns; ++nel)
                ElementN(nel) *= factor;
        }

        /// Transposes only the lower-right 3x3 submatrix of a hemisymmetric 4x4 matrix,
        /// used when the 4x4 matrix is a "star" matrix [q] coming from a quaternion q:
        /// the non commutative quat. product is:
        ///     q1 x q2  =  [q1]*q2  =  [q2st]*q1
        /// where [q2st] is the "semi-transpose of [q2].
        public void MatrXq_SemiTranspose()
        {
            SetElement(1, 2, -GetElement(1, 2));
            SetElement(1, 3, -GetElement(1, 3));
            SetElement(2, 1, -GetElement(2, 1));
            SetElement(2, 3, -GetElement(2, 3));
            SetElement(3, 1, -GetElement(3, 1));
            SetElement(3, 2, -GetElement(3, 2));
        }

        /// Change the sign of the 2nd, 3rd and 4th columns of a 4x4 matrix,
        /// The product between a quaternion q1 and the conjugate of q2 (q2'), is:
        ///    q1 x q2'  = [q1]*q2'   = [q1sn]*q2
        /// where [q1sn] is the semi-negation of the 4x4 matrix [q1].
        public void MatrXq_SemiNeg()
        {
            for (int i = 0; i < rows; ++i)
                for (int j = 1; j < columns; ++j)
                    SetElement(i, j, -GetElement(i, j));
        }

        /// Scales a matrix, multiplying all element by all other elements of
        /// matra (it is not the classical matrix multiplication!)
        public void MatrScale(ChMatrix matra)
        {
            // Debug.Assert(matra.GetColumns() == columns && matra.GetRows() == rows);
            for (int nel = 0; nel < rows * columns; ++nel)
                ElementN(nel) *= matra.ElementN(nel);
        }

        /// Multiplies two matrices, and stores the result in "this" matrix: [this]=[A]*[B].
        public void MatrMultiply(ChMatrix matra, ChMatrix matrb)
        {
            // Debug.Assert(matra.GetColumns() == matrb.GetRows());
            //Debug.Assert(this.rows == matra.GetRows());
            //Debug.Assert(this.columns == matrb.GetColumns());
            int col, row, colres;
            double sum;
            for (colres = 0; colres < matrb.GetColumns(); ++colres)
            {
                for (row = 0; row < matra.GetRows(); ++row)
                {
                    sum = 0;
                    for (col = 0; col < matra.GetColumns(); ++col)
                        sum += (matra.Element(row, col) * matrb.Element(col, colres));
                    SetElement(row, colres, sum);
                }
            }
        }

        /// Multiplies two matrices (the first is considered transposed): [this]=[A]'*[B]
        /// Faster than doing A.MatrTranspose(); result.MatrMultiply(A,B);
        public void MatrTMultiply(ChMatrix matra, ChMatrix matrb)
        {
            //Debug.Assert(matra.GetRows() == matrb.GetRows());
            //Debug.Assert(this.rows == matra.GetColumns());
            // Debug.Assert(this.columns == matrb.GetColumns());
            int col, row, colres;
            double sum;
            for (colres = 0; colres < matrb.GetColumns(); ++colres)
            {
                for (row = 0; row < matra.GetColumns(); ++row)
                {
                    sum = 0;
                    for (col = 0; col < (matra.GetRows()); ++col)
                        sum += (matra.Element(col, row) * matrb.Element(col, colres));
                    SetElement(row, colres, sum);
                }
            }
        }

        /// Gets the norm infinite of the matrix, i.e. the max.
        /// of its elements in absolute value.
        public double NormInf()
        {
            double norm = 0;
            for (int nel = 0; nel < rows * columns; ++nel)
                if ((Math.Abs(ElementN(nel))) > norm)
                    norm = Math.Abs(ElementN(nel));
            return norm;
        }

        public void PasteVector(ChVector va, int insrow, int inscol)
        {
            SetElement(insrow + 0, inscol, va.x);
            SetElement(insrow + 1, inscol, va.y);
            SetElement(insrow + 2, inscol, va.z);
        }

        /// Paste a quaternion into the matrix.
        public void PasteQuaternion(ChQuaternion qa, int insrow, int inscol)
        {
            SetElement(insrow + 0, inscol, qa.e0);
            SetElement(insrow + 1, inscol, qa.e1);
            SetElement(insrow + 2, inscol, qa.e2);
            SetElement(insrow + 3, inscol, qa.e3);
        }

        public void PasteCoordsys(ChCoordsys<double> cs, int insrow, int inscol)
        {
            PasteVector(cs.pos, insrow, inscol);
            PasteQuaternion(cs.rot, insrow + 3, inscol);
        }

        /// Paste a quaternion into the matrix, summing it with preexisting values.
        public void PasteSumQuaternion(ChQuaternion qa, int insrow, int inscol)
        {
            Element(insrow + 0, inscol) += qa.e0;
            Element(insrow + 1, inscol) += qa.e1;
            Element(insrow + 2, inscol) += qa.e2;
            Element(insrow + 3, inscol) += qa.e3;
        }

        //
        // MATH MEMBER FUNCTIONS.
        // For speed reasons, sometimes size checking of operands is left to the user!
        //

        /// Subtract two matrices, and stores the result in "this" matrix: [this]=[A]-[B].
        public void MatrSub(ChMatrix matra, ChMatrix matrb)
        {
            //  Debug.Assert(matra.GetColumns() == matrb.GetColumns() && matra.rows == matrb.GetRows());
            // Debug.Assert(this.columns == matrb.GetColumns() && this.rows == matrb.GetRows());
            for (int nel = 0; nel < rows * columns; ++nel)
                ElementN(nel) = (matra.ElementN(nel) - matrb.ElementN(nel));

        }

        //
        // MULTIBODY SPECIFIC MATH FUCTION
        //

        /// Fills a 4x4 matrix as the "star" matrix, representing quaternion cross product.
        /// That is, given two quaternions a and b, aXb= [Astar]*b
        public void Set_Xq_matrix(ChQuaternion q)
        {
            Set44Element(0, 0, q.e0);
            Set44Element(0, 1, -q.e1);
            Set44Element(0, 2, -q.e2);
            Set44Element(0, 3, -q.e3);
            Set44Element(1, 0, q.e1);
            Set44Element(1, 1, q.e0);
            Set44Element(1, 2, -q.e3);
            Set44Element(1, 3, q.e2);
            Set44Element(2, 0, q.e2);
            Set44Element(2, 1, q.e3);
            Set44Element(2, 2, q.e0);
            Set44Element(2, 3, -q.e1);
            Set44Element(3, 0, q.e3);
            Set44Element(3, 1, -q.e2);
            Set44Element(3, 2, q.e1);
            Set44Element(3, 3, q.e0);
        }
    };
}
