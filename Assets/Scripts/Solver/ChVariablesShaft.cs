using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono
{


    /// Specialized class for representing a 1-DOF item for a system, that is
    /// a shaft, with inertia and associated variable (rotational speed)
    public class ChVariablesShaft : ChVariables
    {

        private ChShaft m_shaft;      //< associated shaft element
        private double m_inertia;      //< shaft inertia
        private double m_inv_inertia;  //< inverse of shaft inertia value


        public ChVariablesShaft() : base(1) {
            m_shaft = null;
            m_inertia = 1;
            m_inv_inertia = 1;  
        }

        /// The number of scalar variables in the vector qb
        /// (dof=degrees of freedom)
        public override int Get_ndof() { return 1; }

        /// Get the inertia associated with rotation of the shaft
        public double GetInertia() { return m_inertia; }

        /// Get the inverse of the inertia associated with rotation of the shaft
        public double GetInvInertia() { return m_inv_inertia; }

        /// Set the inertia associated with rotation of the shaft
        public void SetInertia(double inertia) {
            m_inertia = inertia;
            m_inv_inertia = 1 / inertia;
        }

        public ChShaft GetShaft() { return m_shaft; }
        public void SetShaft(ChShaft shaft) { m_shaft = shaft; }

        /// Computes the product of the inverse mass matrix by a
        /// vector, and set in result: result = [invMb]*vect
        public override void Compute_invMb_v(ChMatrix result, ChMatrix vect) {
            //Debug.Assert(vect.GetRows() == Get_ndof());
           // Debug.Assert(result.GetRows() == Get_ndof());
            result[0] = m_inv_inertia * vect[0];
        }

        /// Computes the product of the inverse mass matrix by a
        /// vector, and increment result: result += [invMb]*vect
        public override void Compute_inc_invMb_v(ref ChMatrix result, ChMatrix vect) {
           // Debug.Assert(vect.GetRows() == Get_ndof());
            //Debug.Assert(result.GetRows() == Get_ndof());
            result[0] += m_inv_inertia * vect[0];
        }

        /// Computes the product of the mass matrix by a
        /// vector, and set in result: result = [Mb]*vect
        public override void Compute_inc_Mb_v(ref ChMatrix result, ChMatrix vect) {
           // Debug.Assert(result.GetRows() == vect.GetRows());
           // Debug.Assert(vect.GetRows() == Get_ndof());
            result[0] += m_inertia * vect[0];
        }

        /// Computes the product of the corresponding block in the
        /// system matrix (ie. the mass matrix) by 'vect', scale by c_a, and add to 'result'.
        /// NOTE: the 'vect' and 'result' vectors must already have
        /// the size of the total variables&constraints in the system; the procedure
        /// will use the ChVariable offsets (that must be already updated) to know the
        /// indexes in result and vect.
        public override void MultiplyAndAdd(ref ChMatrix result,
                                ChMatrix vect,
                                double c_a) {
           // Debug.Assert(result.GetColumns() == 1 && vect.GetColumns() == 1);
            result[this.offset] += c_a * m_inertia * vect[this.offset];
        }

        /// Add the diagonal of the mass matrix scaled by c_a, to 'result'.
        /// NOTE: the 'result' vector must already have the size of system unknowns, ie
        /// the size of the total variables&constraints in the system; the procedure
        /// will use the ChVariable offset (that must be already updated) as index.
        public override void DiagonalAdd(ref ChMatrix result, double c_a) {
            //Debug.Assert(result.GetColumns() == 1);
            result[this.offset] += c_a * m_inertia;
        }
        /// Build the mass matrix (for these variables) scaled by c_a, storing
        /// it in 'storage' sparse matrix, at given column/row offset.
        /// Note, most iterative solvers don't need to know mass matrix explicitly.
        /// Optimized: doesn't fill unneeded elements except mass.
        public override void Build_M(ChSparseMatrix storage, int insrow, int inscol, double c_a) {
            storage.SetElement(insrow + 0, inscol + 0, c_a * m_inertia);
        }

    }
}
