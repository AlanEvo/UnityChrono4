using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono
{

    /// Specialized class for representing a N-DOF item for a
    /// system, that is an item with mass matrix and
    /// associate variables.
    /// The main difference from the base class ChVariables
    /// is that the base class does not create any mass matrix,
    /// while this minimal specialization at least creates a
    /// NxN mass matrix. Of course a generic (uncompressed) NxN
    /// matrix is used. This means that, for example, this
    /// class could  be used for 3D rigid bodies if N=6, however
    /// it would be better to implement a more optimized class
    /// which does not create a full 6x6 matrix (since only few
    /// elements on the diagonal would be different from 0 in case
    /// of rigid bodies), so use the ChVariablesBody in this case..

    public class ChVariablesGeneric : ChVariables
    {


        private ChMatrixDynamic<double> Mmass = new ChMatrixDynamic<double>();
        private ChMatrixDynamic<double> inv_Mmass = new ChMatrixDynamic<double>();
        private int ndof = 1;


        public ChVariablesGeneric(int m_ndof = 1) : base(m_ndof)
        {
            ndof = m_ndof;
            Mmass = new ChMatrixDynamic<double>(ndof, ndof);
            Mmass.matrix.SetIdentity();
            inv_Mmass = new ChMatrixDynamic<double>(ndof, ndof);
            inv_Mmass.matrix.SetIdentity();
        }

        /// Assignment operator: copy from other object
        // ChVariablesGeneric& operator=(const ChVariablesGeneric& other);

        /// Access the inertia matrix
        public ChMatrix GetMass() { return Mmass.matrix; }

        /// Access the inverted inertia matrix
        public ChMatrix GetInvMass() { return inv_Mmass.matrix; }

        // IMPLEMENT PARENT CLASS METHODS

        /// The number of scalar variables in the vector qb
        /// (dof=degrees of freedom)
        public override int Get_ndof() { return this.ndof; }

        /// Computes the product of the inverse mass matrix by a
        /// vector, and add to result: result = [invMb]*vect
        public override void Compute_invMb_v(ChMatrix result, ChMatrix vect)
        {
           /* Debug.Assert(result.GetRows() == vect.GetRows());
            Debug.Assert(vect.GetRows() == Get_ndof());
            result = inv_Mmass.matrix * vect;*/
        }

        /// Computes the product of the inverse mass matrix by a
        /// vector, and increment result: result += [invMb]*vect
        public override void Compute_inc_invMb_v(ref ChMatrix result, ChMatrix vect)
        {
            //Debug.Assert(result.GetRows() == vect.GetRows());
            //Debug.Assert(vect.GetRows() == Get_ndof());
            result += inv_Mmass * vect;
        }

        /// Computes the product of the mass matrix by a
        /// vector, and set in result: result = [Mb]*vect
        public override void Compute_inc_Mb_v(ref ChMatrix result, ChMatrix vect)
        {
           // Debug.Assert(result.GetRows() == vect.GetRows());
          //  Debug.Assert(vect.GetRows() == Get_ndof());
            result += (Mmass) * vect;
        }

        /// Computes the product of the corresponding block in the
        /// system matrix (ie. the mass matrix) by 'vect', scale by c_a, and add to 'result'.
        /// NOTE: the 'vect' and 'result' vectors must already have
        /// the size of the total variables&constraints in the system; the procedure
        /// will use the ChVariable offsets (that must be already updated) to know the
        /// indexes in result and vect.
        public override void MultiplyAndAdd(ref ChMatrix result,
                                    ChMatrix vect,
                                    double c_a)
        {
            //Debug.Assert(result.GetColumns() == 1 && vect.GetColumns() == 1);

            for (int i = 0; i < Mmass.matrix.GetRows(); i++)
            {
                double tot = 0;
                for (int j = 0; j < Mmass.matrix.GetColumns(); j++)
                {
                    tot += Mmass.matrix[i, j] * vect[this.offset + i];
                }
                result[this.offset + i] += c_a * tot;
            }
        }

        /// Add the diagonal of the mass matrix scaled by c_a, to 'result'.
        /// NOTE: the 'result' vector must already have the size of system unknowns, ie
        /// the size of the total variables&constraints in the system; the procedure
        /// will use the ChVariable offset (that must be already updated) as index.
        public override void DiagonalAdd(ref ChMatrix result, double c_a)
        {
            //Debug.Assert(result.GetColumns() == 1);
            for (int i = 0; i < Mmass.matrix.GetRows(); i++)
            {
                result[this.offset + i] += c_a * Mmass.matrix[i, i];
            }
        }


        /// Build the mass matrix (for these variables) scaled by c_a, storing
        /// it in 'storage' sparse matrix, at given column/row offset.
        /// Note, most iterative solvers don't need to know mass matrix explicitly.
        public override void Build_M(ChSparseMatrix storage, int insrow, int inscol, double c_a)
        {
            for (int row = 0; row < Mmass.matrix.GetRows(); ++row)
                for (int col = 0; col < Mmass.matrix.GetColumns(); ++col)
                    storage.SetElement(insrow + row, inscol + col, c_a * Mmass.matrix.GetElement(row, col));
        }
    }
}

