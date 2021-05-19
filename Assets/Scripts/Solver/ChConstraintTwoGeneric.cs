using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono
{

    /// This class is inherited from the base ChConstraintTwo(),
    /// which is a base for pairwise constraints. So here this class implements
    /// the functionality for a constraint between a COUPLE of TWO
    /// objects of type ChVariables(), with generic number of scalar
    /// variables each (ex.ChVariablesGeneric() or ChVariablesBody() )
    /// and defines two constraint matrices, whose column number automatically
    /// matches the number of elements in variables vectors.
    ///  Before starting the solver one must provide the proper
    /// values in constraints (and update them if necessary), i.e.
    /// must set at least the c_i and b_i values, and jacobians.

    public class ChConstraintTwoGeneric : ChConstraintTwo
    {
        protected ChMatrixDynamic<double> Cq_a;  ///< The [Cq_a] jacobian of the constraint
        protected ChMatrixDynamic<double> Cq_b;  ///< The [Cq_b] jacobian of the constraint

        // Auxiliary data: will be used by iterative constraint solvers:

        protected ChMatrixDynamic<double> Eq_a;  ///< The [Eq_a] product [Eq_a]=[invM_a]*[Cq_a]'
        protected ChMatrixDynamic<double> Eq_b;  ///< The [Eq_a] product [Eq_b]=[invM_b]*[Cq_b]'

        // TEST
        ChMatrixDynamic<double> mtemp1 = new ChMatrixDynamic<double>(0);
        ChMatrixDynamic<double> res = new ChMatrixDynamic<double>(0);

        /// Default constructor
        public ChConstraintTwoGeneric() {
           /* Cq_a = null; 
            Cq_b = null; 
            Eq_a = null; 
            Eq_b = null;*/
        }

        /// Construct and immediately set references to variables
        public ChConstraintTwoGeneric(ChVariables mvariables_a, ChVariables mvariables_b) {
           /* Cq_a = null;
            Cq_b = null;
            Eq_a = null;
            Eq_b = null;*/
            SetVariables(mvariables_a, mvariables_b);
        }

        /// Copy constructor
        public ChConstraintTwoGeneric(ChConstraintTwoGeneric other) {
           // Cq_a = Cq_b = Eq_a = Eq_b = null;
           // if (other.Cq_a != null)
                Cq_a = new ChMatrixDynamic<double>(other.Cq_a);
           // if (other.Cq_b != null)
                Cq_b = new ChMatrixDynamic<double>(other.Cq_b);
           // if (other.Eq_a != null)
                Eq_a = new ChMatrixDynamic<double>(other.Eq_a);
           // if (other.Eq_b != null)
                Eq_b = new ChMatrixDynamic<double>(other.Eq_b);
        }

        /// "Virtual" copy constructor (covariant return type).
        public override ChConstraint Clone() { return new ChConstraintTwoGeneric(this);
    }

        /// Access jacobian matrix
        public override ChMatrix Get_Cq_a() { return Cq_a.matrix; }
        /// Access jacobian matrix
        public override ChMatrix Get_Cq_b() { return Cq_b.matrix; }

        /// Access auxiliary matrix (ex: used by iterative solvers)
        public override ChMatrix Get_Eq_a() { return Eq_a.matrix; }
        /// Access auxiliary matrix (ex: used by iterative solvers)
        public override ChMatrix Get_Eq_b() { return Eq_b.matrix; }

        /// Set references to the constrained objects, each of ChVariables type,
        /// automatically creating/resizing jacobians if needed.
        public override void SetVariables(ChVariables mvariables_a, ChVariables mvariables_b) {
            if (mvariables_a == null || mvariables_b == null)
            {
                SetValid(false);
                return;
            }

            SetValid(true);
            variables_a = mvariables_a;
            variables_b = mvariables_b;

            if (variables_a.Get_ndof() != 0)
            {
               // if (Cq_a == null)
                 //   Cq_a = new ChMatrixDynamic<double>(1, variables_a.Get_ndof());
               // else
                    Cq_a.Resize(1, variables_a.Get_ndof());

               // if (Eq_a == null)
                //    Eq_a = new ChMatrixDynamic<double>(variables_a.Get_ndof(), 1);
              //  else
                    Eq_a.Resize(variables_a.Get_ndof(), 1);
            }
            else
            {
              /*  if (Cq_a != null)
                   Cq_a = null;
                Cq_a = null;
                if (Eq_a != null)
                    Eq_a = null;
                Eq_a = null;*/
            }

            if (variables_b.Get_ndof() != 0)
            {
               // if (Cq_b == null)
                //    Cq_b = new ChMatrixDynamic<double>(1, variables_b.Get_ndof());
               // else
                    Cq_b.Resize(1, variables_b.Get_ndof());

               // if (Eq_b == null)
                //    Eq_b = new ChMatrixDynamic<double>(variables_b.Get_ndof(), 1);
               // else
                    Eq_b.Resize(variables_b.Get_ndof(), 1);
            }
            else
            {
                /*if (Cq_b != null)
                    Cq_b = null;
                Cq_b = null;
                if (Eq_b != null)
                    Eq_b = null;
                Eq_b = null;*/
            }
        }

        /// This function updates the following auxiliary data:
        ///  - the Eq_a and Eq_b matrices
        ///  - the g_i product
        /// This is often called by solvers at the beginning
        /// of the solution process.
        /// Most often, inherited classes won't need to override this.
        public override void Update_auxiliary() {
            // 1- Assuming jacobians are already computed, now compute
            //   the matrices [Eq_a]=[invM_a]*[Cq_a]' and [Eq_b]
            if (variables_a.IsActive())
                if (variables_a.Get_ndof() != 0)
                {
                    //  ChMatrixDynamic<double> mtemp1 = new ChMatrixDynamic<double>(variables_a.Get_ndof(), 1);
                    mtemp1.New(variables_a.Get_ndof(), 1);
                    mtemp1.matrix.CopyFromMatrixT(Cq_a.matrix);
                    variables_a.Compute_invMb_v(Eq_a.matrix, mtemp1.matrix);
                }
            if (variables_b.IsActive())
                if (variables_b.Get_ndof() != 0)
                {
                    // ChMatrixDynamic<double> mtemp1 = new ChMatrixDynamic<double>(variables_b.Get_ndof(), 1);
                    mtemp1.New(variables_b.Get_ndof(), 1);
                    mtemp1.matrix.CopyFromMatrixT(Cq_b.matrix);
                    variables_b.Compute_invMb_v(Eq_b.matrix, mtemp1.matrix);
                }

            // 2- Compute g_i = [Cq_i]*[invM_i]*[Cq_i]' + cfm_i
            g_i = 0;
            //ChMatrixDynamic<double> res = new ChMatrixDynamic<double>(1, 1);
            res.New(1, 1);
            if (variables_a.IsActive())
                if (variables_a.Get_ndof() != 0)
                {
                    res.matrix.MatrMultiply(Cq_a.matrix, Eq_a.matrix);
                    g_i = res.matrix[0, 0];
                }
            if (variables_b.IsActive())
                if (variables_b.Get_ndof() != 0)
                {
                    res.matrix.MatrMultiply(Cq_b.matrix, Eq_b.matrix);
                    g_i += res.matrix[0, 0];
                }

            // 3- adds the constraint force mixing term (usually zero):
            if (cfm_i != 0)
                g_i += cfm_i;
        }

        ///  This function must computes the product between
        /// the row-jacobian of this constraint '[Cq_i]' and the
        /// vector of variables, 'v'. that is    CV=[Cq_i]*v
        ///  This is used for some iterative solvers.
        public override double Compute_Cq_q() {
            double ret = 0;

            if (variables_a.IsActive())
                for (int i = 0; i < Cq_a.matrix.GetColumns(); i++)
                    ret += Cq_a.matrix.ElementN(i) * variables_a.Get_qb().matrix.ElementN(i);

            if (variables_b.IsActive())
                for (int i = 0; i < Cq_b.matrix.GetColumns(); i++)
                    ret += Cq_b.matrix.ElementN(i) * variables_b.Get_qb().matrix.ElementN(i);

            return ret;
        }

        ///  This function must increment the vector of variables
        /// 'v' with the quantity [invM]*[Cq_i]'*deltal,that is
        ///   v+=[invM]*[Cq_i]'*deltal  or better: v+=[Eq_i]*deltal
        ///  This is used for some iterative solvers.
        public override void Increment_q(double deltal) {
            if (variables_a.IsActive())
                for (int i = 0; i < Eq_a.matrix.GetRows(); i++)
                    variables_a.Get_qb().matrix[i] += Eq_a.matrix.ElementN(i) * deltal;

            if (variables_b.IsActive())
                for (int i = 0; i < Eq_b.matrix.GetRows(); i++)
                    variables_b.Get_qb().matrix[i] += Eq_b.matrix.ElementN(i) * deltal;
        }

        /// Computes the product of the corresponding block in the
        /// system matrix by 'vect', and add to 'result'.
        /// NOTE: the 'vect' vector must already have
        /// the size of the total variables&constraints in the system; the procedure
        /// will use the ChVariable offsets (that must be already updated) to know the
        /// indexes in result and vect;
        public override void MultiplyAndAdd(ref double result, ChMatrix vect) {
            int off_a = variables_a.GetOffset();
            int off_b = variables_b.GetOffset();

            if (variables_a.IsActive())
                for (int i = 0; i < Cq_a.matrix.GetColumns(); i++)
                    result += vect[off_a + i] * Cq_a.matrix.ElementN(i);

            if (variables_b.IsActive())
                for (int i = 0; i < Cq_b.matrix.GetColumns(); i++)
                    result += vect[off_b + i] * Cq_b.matrix.ElementN(i);
        }

        /// Computes the product of the corresponding transposed blocks in the
        /// system matrix (ie. the TRANSPOSED jacobian matrix C_q') by 'l', and add to 'result'.
        /// NOTE: the 'result' vector must already have
        /// the size of the total variables&constraints in the system; the procedure
        /// will use the ChVariable offsets (that must be already updated) to know the
        /// indexes in result and vect;
        public override void MultiplyTandAdd(ChMatrix result, double l) {
            int off_a = variables_a.GetOffset();
            int off_b = variables_b.GetOffset();

            if (variables_a.IsActive())
                for (int i = 0; i < Cq_a.matrix.GetColumns(); i++)
                    result[off_a + i] += Cq_a.matrix.ElementN(i) * l;

            if (variables_b.IsActive())
                for (int i = 0; i < Cq_b.matrix.GetColumns(); i++)
                    result[off_b + i] += Cq_b.matrix.ElementN(i) * l;
        }

        /// Puts the two jacobian parts into the 'insrow' row of a sparse matrix,
        /// where both portions of the jacobian are shifted in order to match the
        /// offset of the corresponding ChVariable.
        public override void Build_Cq(ref ChSparseMatrix storage, int insrow) {
            if (variables_a.IsActive())
                storage.PasteMatrix(Cq_a.matrix, insrow, variables_a.GetOffset());
            if (variables_b.IsActive())
                storage.PasteMatrix(Cq_b.matrix, insrow, variables_b.GetOffset());
        }
        public override void Build_CqT(ref ChSparseMatrix storage, int inscol) {
            if (variables_a.IsActive())
                storage.PasteTranspMatrix(Cq_a.matrix, variables_a.GetOffset(), inscol);
            if (variables_b.IsActive())
                storage.PasteTranspMatrix(Cq_b.matrix, variables_b.GetOffset(), inscol);
        }

    }
}
