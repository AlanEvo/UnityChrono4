using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono
{

    /// Base class for representing items which introduce block-sparse
    /// matrices, that is blocks that connect some 'variables'
    /// and build a matrix K in a sparse variational inequality VI(Z*x-d,K):
    ///
    /// See ChSystemDescriptor for more information about the overall
    /// problem and data representation.
    ///
    /// Note that K blocks often have a physical interpretation as stiffness,
    /// but not always, for example they can represent hessians.
    /// Note that all blocks in K, all masses and constraint
    /// jacobians Cq are not really assembled in large matrices, so to
    /// exploit sparsity.

    public class ChKblockGeneric : ChKblock
    {

        private ChMatrixDynamic<double> K = new ChMatrixDynamic<double>();
        List<ChVariables> variables = new List<ChVariables>();


        public ChKblockGeneric() {  }
        public ChKblockGeneric(List<ChVariables> mvariables)
        {
            SetVariables(mvariables);
        }
        public ChKblockGeneric(ChVariables mvariableA, ChVariables mvariableB)
        {
            List<ChVariables> mvars = new List<ChVariables>();
            mvars.Add(mvariableA);
            mvars.Add(mvariableB);
            SetVariables(mvars);
        }

        /// Set references to the constrained objects, each of ChVariables type,
        /// automatically creating/resizing K matrix if needed.
        public void SetVariables(List<ChVariables> mvariables)
        {
           // Debug.Assert(mvariables.Count > 0);

            variables = mvariables;

            // destroy the K matrix if needed
           // if (K != null)
            //    K = null;

            int msize = 0;
            for (int iv = 0; iv < variables.Count; iv++)
                msize += variables[iv].Get_ndof();

            // reallocate the K matrix
            K = new ChMatrixDynamic<double>(msize, msize);
        }

        /// Returns the number of referenced ChVariables items
        public override int GetNvars() { return variables.Count; }

        /// Access the m-th vector variable object
        public ChVariables GetVariableN(int m_var) { return variables[m_var]; }

        /// Access the K stiffness matrix as a single block,
        /// referring only to the referenced ChVariable objects
        public override ChMatrix Get_K() { return K.matrix; }

        /// Computes the product of the corresponding blocks in the
        /// system matrix (ie. the K matrix blocks) by 'vect', and add to 'result'.
        /// NOTE: the 'vect' and 'result' vectors must already have
        /// the size of the total variables&constraints in the system; the procedure
        /// will use the ChVariable offsets (that must be already updated) to know the
        /// indexes in result and vect.
        public override void MultiplyAndAdd(ref ChMatrix result, ChMatrix vect)
        {
           // Debug.Assert(K == null);

            int kio = 0;
            for (int iv = 0; iv < this.GetNvars(); iv++)
            {
                int io = this.GetVariableN(iv).GetOffset();
                int ina = this.GetVariableN(iv).Get_ndof();

                if (this.GetVariableN(iv).IsActive())
                {
                    int kjo = 0;
                    for (int jv = 0; jv < this.GetNvars(); jv++)
                    {
                        int jo = this.GetVariableN(jv).GetOffset();
                        int jn = this.GetVariableN(jv).Get_ndof();

                        if (this.GetVariableN(jv).IsActive())
                        {
                            // Multiply the iv,jv sub block of K
                            for (int r = 0; r < ina; r++) {
                                double tot = 0;
                                for (int c = 0; c < jn; c++)
                                {
                                    tot += this.K.matrix[kio + r, kjo + c] * vect[jo + c];
                                }
                                result[io + r] += tot;
                            }
                        }

                        kjo += jn;
                    }
                }

                kio += ina;
            }
        }

        /// Add the diagonal of the stiffness matrix block(s) as a column vector to 'result'.
        /// NOTE: the 'result' vector must already have the size of system unknowns, ie
        /// the size of the total variables&constraints in the system; the procedure
        /// will use the ChVariable offsets (that must be already updated) as index.
        public override void DiagonalAdd(ref ChMatrix result)
        {
           // Debug.Assert(result.GetColumns() == 1);

            int kio = 0;
            for (int iv = 0; iv < this.GetNvars(); iv++)
            {
                int io = this.GetVariableN(iv).GetOffset();
                int ina = this.GetVariableN(iv).Get_ndof();

                if (this.GetVariableN(iv).IsActive())
                {
                    for (int r = 0; r < ina; r++) {
                        // GetLog() << "Summing" << result(io+r) << " to " << (*this.K)(kio+r,kio+r) << "\n";
                        result[io + r] += this.K.matrix[kio + r, kio + r];
                    }
                }
                kio += ina;
            }
        }

        /// Writes the K matrix associated to these variables into
        /// a global 'storage' matrix, at the offsets of variables.
        /// Most solvers do not need this: the sparse 'storage' matrix is used for testing, for
        /// direct solvers, for dumping full matrix to Matlab for checks, etc.
        public override void Build_K(ref ChSparseMatrix storage, bool add)
        {
           // if (K == null)
            //    return;

            int kio = 0;
            for (int iv = 0; iv < this.GetNvars(); iv++)
            {
                int io = this.GetVariableN(iv).GetOffset();
                int ina = this.GetVariableN(iv).Get_ndof();

                if (this.GetVariableN(iv).IsActive())
                {
                    int kjo = 0;
                    for (int jv = 0; jv < this.GetNvars(); jv++)
                    {
                        int jo = this.GetVariableN(jv).GetOffset();
                        int jn = this.GetVariableN(jv).Get_ndof();

                        if (this.GetVariableN(jv).IsActive())
                        {
                            if (add)
                                storage.PasteSumClippedMatrix(K.matrix, kio, kjo, ina, jn, io, jo);
                            else
                                storage.PasteClippedMatrix(K.matrix, kio, kjo, ina, jn, io, jo);
                        }

                        kjo += jn;
                    }
                }

                kio += ina;
            }
        }
    }
}