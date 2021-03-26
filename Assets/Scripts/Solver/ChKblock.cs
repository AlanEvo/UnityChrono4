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
    public abstract class ChKblock
    {

        public ChKblock() { }

        /// Returns the number of referenced ChVariables items
        public abstract int GetNvars();

        /// Access the K stiffness matrix as a single block,
        /// referring only to the referenced ChVariable objects
        public abstract ChMatrix Get_K();

        /// Computes the product of the corresponding blocks in the
        /// system matrix (ie. the K matrix blocks) by 'vect', and add to 'result'.
        /// NOTE: the 'vect' and 'result' vectors must already have
        /// the size of the total variables&constraints in the system; the procedure
        /// will use the ChVariable offsets (that must be already updated) to know the
        /// indexes in result and vect.
        public abstract void MultiplyAndAdd(ref ChMatrix result, ChMatrix vect);

        /// Add the diagonal of the stiffness matrix block(s) as a column vector to 'result'.
        /// NOTE: the 'result' vector must already have the size of system unknowns, ie
        /// the size of the total variables&constraints in the system; the procedure
        /// will use the ChVariable offsets (that must be already updated) as index.
        public abstract void DiagonalAdd(ref ChMatrix result);

        /// Writes (and adds) the K matrix associated to these variables into
        /// a global 'storage' matrix, at the offsets of variables.
        /// Most solvers do not need this: the sparse 'storage' matrix is used for testing, for
        /// direct solvers, for dumping full matrix to Matlab for checks, etc.
        public abstract void Build_K(ref ChSparseMatrix storage, bool add = true);

    }
}
