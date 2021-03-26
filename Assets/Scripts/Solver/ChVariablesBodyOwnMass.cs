using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono
{
    /// Specialized class for representing a 6-DOF item for a
    /// system, that is a 3D rigid body, with mass matrix and
    /// associate variables (a 6 element vector, ex.speed)
    /// Differently from the 'naive' implementation ChVariablesGeneric,
    /// here a full 6x6 mass matrix is not built, since only the 3x3
    /// inertia matrix and the mass value are enough.
    public class ChVariablesBodyOwnMass : ChVariablesBody
    {

        private ChMatrix33<double> inertia = new ChMatrix33<double>();      //< 3x3 inertia matrix
        private double mass;                     //< mass value
        private ChMatrix33<double> inv_inertia = new ChMatrix33<double>();  //< inverse inertia matrix
        private double inv_mass;                 //< inverse of mass value


        public ChVariablesBodyOwnMass() {
            mass = 1;
            inv_mass = 1;
            inertia.Set33Identity();
            inv_inertia.Set33Identity();
        }

        /// Get the mass associated with translation of body
        public override double GetBodyMass() { return mass; }

        /// Access the 3x3 inertia matrix
        public override ChMatrix33<double> GetBodyInertia() { return inertia; }

        /// Access the 3x3 inertia matrix inverted
        public override ChMatrix33<double> GetBodyInvInertia() { return inv_inertia; }


        /// Set the inertia matrix
        public void SetBodyInertia(ChMatrix33<double> minertia) {
            inertia.CopyFromMatrix(minertia);
            inertia.FastInvert(inv_inertia);
        }

        /// Set the mass associated with translation of body
        public void SetBodyMass(double mmass) {
            mass = mmass;
            if (mass != 0)
                inv_mass = 1.0 / mass;
            else
                inv_mass = 1e32;
        }

        /// Computes the product of the inverse mass matrix by a
        /// vector, and set in result: result = [invMb]*vect
        //public override void Compute_invMb_v(ref ChMatrix<double> result, ChMatrix<double> vect) {
        public override void Compute_invMb_v(ChMatrix result, ChMatrix vect)
        {
           // ChMatrix<double> result2 = result as ChMatrix<double>;
           // Debug.Assert(vect.GetRows() == Get_ndof());
           // Debug.Assert(result.GetRows() == Get_ndof());
            // optimized unrolled operations
            result[0] = inv_mass * vect[0];
            result[1] = inv_mass * vect[1];
            result[2] = inv_mass * vect[2];
            result[3] = inv_inertia[0, 0] * vect[3] + inv_inertia[0, 1] * vect[4] + inv_inertia[0, 2] * vect[5];
            result[4] = inv_inertia[1, 0] * vect[3] + inv_inertia[1, 1] * vect[4] + inv_inertia[1, 2] * vect[5];
            result[5] = inv_inertia[2, 0] * vect[3] + inv_inertia[2, 1] * vect[4] + inv_inertia[2, 2] * vect[5];
        }

        /// Computes the product of the inverse mass matrix by a
        /// vector, and increment result: result += [invMb]*vect
        public override void Compute_inc_invMb_v(ref ChMatrix result, ChMatrix vect) {
           // Debug.Assert(vect.GetRows() == Get_ndof());
           // Debug.Assert(result.GetRows() == Get_ndof());
            // optimized unrolled operations
            result[0] += inv_mass * vect[0];
            result[1] += inv_mass * vect[1];
            result[2] += inv_mass * vect[2];
            result[3] += inv_inertia[0, 0] * vect[3] + inv_inertia[0, 1] * vect[4] + inv_inertia[0, 2] * vect[5];
            result[4] += inv_inertia[1, 0] * vect[3] + inv_inertia[1, 1] * vect[4] + inv_inertia[1, 2] * vect[5];
            result[5] += inv_inertia[2, 0] * vect[3] + inv_inertia[2, 1] * vect[4] + inv_inertia[2, 2] * vect[5];
        }

        /// Computes the product of the mass matrix by a
        /// vector, and set in result: result = [Mb]*vect
        public override void Compute_inc_Mb_v(ref ChMatrix result, ChMatrix vect) {
          //  Debug.Assert(vect.GetRows() == Get_ndof());
           // Debug.Assert(result.GetRows() == Get_ndof());
            // optimized unrolled operations
            result[0] += mass * vect[0];
            result[1] += mass * vect[1];
            result[2] += mass * vect[2];
            result[3] += (inertia[0, 0] * vect[3] + inertia[0, 1] * vect[4] + inertia[0, 2] * vect[5]);
            result[4] += (inertia[1, 0] * vect[3] + inertia[1, 1] * vect[4] + inertia[1, 2] * vect[5]);
            result[5] += (inertia[2, 0] * vect[3] + inertia[2, 1] * vect[4] + inertia[2, 2] * vect[5]);
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
          //  Debug.Assert(result.GetColumns() == 1 && vect.GetColumns() == 1);
            // optimized unrolled operations
            double q0 = vect[this.offset + 0];
            double q1 = vect[this.offset + 1];
            double q2 = vect[this.offset + 2];
            double q3 = vect[this.offset + 3];
            double q4 = vect[this.offset + 4];
            double q5 = vect[this.offset + 5];
            double scaledmass = c_a * mass;
            result[this.offset + 0] += scaledmass * q0;
            result[this.offset + 1] += scaledmass * q1;
            result[this.offset + 2] += scaledmass * q2;
            result[this.offset + 3] += c_a * (inertia[0, 0] * q3 + inertia[0, 1] * q4 + inertia[0, 2] * q5);
            result[this.offset + 4] += c_a * (inertia[1, 0] * q3 + inertia[1, 1] * q4 + inertia[1, 2] * q5);
            result[this.offset + 5] += c_a * (inertia[2, 0] * q3 + inertia[2, 1] * q4 + inertia[2, 2] * q5);
        }

        /// Add the diagonal of the mass matrix scaled by c_a, to 'result'.
        /// NOTE: the 'result' vector must already have the size of system unknowns, ie
        /// the size of the total variables&constraints in the system; the procedure
        /// will use the ChVariable offset (that must be already updated) as index.
        public override void DiagonalAdd(ref ChMatrix result, double c_a) {
          //  Debug.Assert(result.GetColumns() == 1);
            result[this.offset + 0] += c_a * mass;
            result[this.offset + 1] += c_a * mass;
            result[this.offset + 2] += c_a * mass;
            result[this.offset + 3] += c_a * inertia[0, 0];
            result[this.offset + 4] += c_a * inertia[1, 1];
            result[this.offset + 5] += c_a * inertia[2, 2];
        }

        /// Build the mass matrix (for these variables) scaled by c_a, storing
        /// it in 'storage' sparse matrix, at given column/row offset.
        /// Note, most iterative solvers don't need to know mass matrix explicitly.
        /// Optimized: doesn't fill unneeded elements except mass and 3x3 inertia.
        public override void Build_M(ChSparseMatrix storage, int insrow, int inscol, double c_a) {
            storage.SetElement(insrow + 0, inscol + 0, c_a * mass);
            storage.SetElement(insrow + 1, inscol + 1, c_a * mass);
            storage.SetElement(insrow + 2, inscol + 2, c_a * mass);
            ChMatrix33<double> scaledJ = (ChMatrix33<double>)(inertia * c_a);
            storage.PasteMatrix(scaledJ, insrow + 3, inscol + 3);
        }
    }
}
