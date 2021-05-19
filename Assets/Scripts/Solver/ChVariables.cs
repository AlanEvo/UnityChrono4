using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono
{
    /// Base class for representing objects that introduce
    /// 'variables' (also referred as 'v') and their associated mass submatrices
    /// for a sparse representation of the problem.
    ///
    /// See ChSystemDescriptor for more information about the overall
    /// problem and data representation.
    ///
    /// Each ChVariables object must be able to compute a mass submatrix,
    /// that will be assembled directly inside the global matrix Z,
    /// (in particular inside the block H or M).\n
    /// Because of this there is no need for ChVariables and derived classes
    /// to actually \e store their mass submatrix in memory:
    /// they just need to be able to compute it!
    ///
    /// Moreover, in some cases, the mass submatrix is not even needed.
    /// In fact, some Chrono solvers are matrix-free. \n
    /// This means that each ChVariables (and derived) object can be asked
    /// not to \e assemble its mass submatrix,
    /// but instead to provide operation related to it.\n
    /// E.g. M*x, M\\x, +=M*x, and so on...
    /// Each derived class must implement these methods!
    ///
    /// Because of this, the ChVariables class
    /// does \e not include any mass submatrix by default
    public abstract class ChVariables
    {

        private ChMatrixDynamic<double> qb;  //< variables (accelerations, speeds, etc. depending on the problem)
        private ChMatrixDynamic<double> fb;  //< known vector (forces, or impulses, etc. depending on the problem)
        private int ndof;        //< number of degrees of freedom (number of contained scalar variables)
        private bool disabled;   //< user activation/deactivation of variables


        protected int offset;  //< offset in global q state vector (needed by some solvers)

        public ChVariables()
        {
            disabled = false;
            ndof = 0;
            //qb = null;
           // fb = null;
            offset = 0;
        }
        public ChVariables(int m_ndof) {

            disabled = false;
            ndof = m_ndof;
            offset = 0;

           // int dof = Get_ndof();
            if (Get_ndof() > 0)
            {
                qb = new ChMatrixDynamic<double>(Get_ndof(), 1);
                fb = new ChMatrixDynamic<double>(Get_ndof(), 1);
            }
            else
            {
               // qb = fb = null;
            }
        }

        /// Deactivates/freezes the variable (these 'frozen',
        /// variables won't be modified by the system solver).
        public void SetDisabled(bool mdis) { disabled = mdis; }

        /// Tells if the variables have been deactivated (these 'frozen',
        /// variables won't be modified by the system solver).
        public bool IsDisabled()  { return disabled; }

        /// Tells if these variables are currently active, in general,
        /// that is tells if they must be included into the system solver or not.
        public bool IsActive()  { return !disabled; }

        /// The number of scalar variables in the vector qb
        /// (dof=degrees of freedom)
        /// *** This function MUST BE OVERRIDDEN by specialized
        /// inherited classes.
        public virtual int Get_ndof()  { return ndof; }

        /// Returns reference to qb, body-relative part of degrees
        /// of freedom q in system:
        ///    | M -Cq'|*|q|- | f|= |0| ,  c>0, l>0, l*r=0;
        ///    | Cq  0 | |l|  |-b|  |c|
        public ref ChMatrixDynamic<double> Get_qb() { return ref qb; }

        /// Compute fb, body-relative part of known
        /// vector f in system.
        /// *** This function MAY BE OVERRIDDEN by specialized
        /// inherited classes (example, for impulsive multibody simulation,
        /// this may be fb=dt*Forces+[M]*previous_v ).
        ///  Another option is to set values into fb vectors, accessing
        /// them by Get_fb() from an external procedure, for each body,
        /// before starting the solver.
        public virtual void Compute_fb() { }

        /// Returns reference to fb, body-relative part of known
        /// vector f in system.
        ///    | M -Cq'|*|q|- | f|= |0| ,  c>0, l>0, l*r=0;
        ///    | Cq  0 | |l|  |-b|  |c|
        /// This function can be used to set values of fb vector
        /// before starting the solver.
        public ref ChMatrixDynamic<double> Get_fb() { return ref fb; }

        /// Computes the product of the inverse mass matrix by a
        /// vector, and store in result: result = [invMb]*vect
        /// *** This function MUST BE OVERRIDDEN by specialized
        /// inherited classes
        //public abstract void Compute_invMb_v(ref ChMatrix<double> result,  ChMatrix<double> vect);
        public abstract void Compute_invMb_v(ChMatrix result, ChMatrix vect);

        /// Computes the product of the inverse mass matrix by a
        /// vector, and increment result: result += [invMb]*vect
        /// *** This function MUST BE OVERRIDDEN by specialized
        /// inherited classes
        public abstract void Compute_inc_invMb_v(ref ChMatrix result, ChMatrix vect);

        /// Computes the product of the mass matrix by a
        /// vector, and increment result: result = [Mb]*vect
        /// *** This function MUST BE OVERRIDDEN by specialized
        /// inherited classes
        public abstract void Compute_inc_Mb_v(ref ChMatrix result,  ChMatrix vect);

        /// Computes the product of the corresponding block in the
        /// system matrix (ie. the mass matrix) by 'vect', scale by c_a, and add to 'result'.
        /// NOTE: the 'vect' and 'result' vectors must already have
        /// the size of the total variables&raints in the system; the procedure
        /// will use the ChVariable offset (that must be already updated) to know the
        /// indexes in result and vect.
        public abstract void MultiplyAndAdd(ref ChMatrix result, ChMatrix vect,  double c_a);

        /// Add the diagonal of the mass matrix scaled by c_a, to 'result', as a vector.
        /// NOTE: the 'result' vector must already have the size of system unknowns, ie
        /// the size of the total variables&raints in the system; the procedure
        /// will use the ChVariable offset (that must be already updated) as index.
        public abstract void DiagonalAdd(ref ChMatrix result,  double c_a);

        /// Build the mass submatrix (for these variables) multiplied by c_a, storing
        /// it in 'storage' sparse matrix, at given column/row offset.
        /// Most iterative solvers don't need to know this matrix explicitly.
        /// *** This function MUST BE OVERRIDDEN by specialized
        /// inherited classes
        public abstract void Build_M(ChSparseMatrix storage, int insrow, int inscol,  double c_a);

        /// Set offset in global q vector (set automatically by ChSystemDescriptor)
        public void SetOffset(int moff) { offset = moff; }
        /// Get offset in global q vector
        public int GetOffset()  { return offset; }

        public virtual void ArchiveOUT(ref ChArchiveOut marchive) { }
        public virtual void ArchiveIN(ref ChArchiveIn marchive) { }
    }
}
