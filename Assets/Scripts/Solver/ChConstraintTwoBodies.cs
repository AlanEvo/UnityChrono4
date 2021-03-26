using System.Collections;
using System.Collections.Generic;
using UnityEngine;


namespace chrono
{

    /// This class inherits from the base ChConstraintTwo(),
    /// that implements the functionality for a constraint between
    /// a couple of two objects of type ChVariablesBody().
    public class ChConstraintTwoBodies : ChConstraintTwo
    {

        protected ChMatrixNM<IntInterface.One, IntInterface.Six> Cq_a = new ChMatrixNM<IntInterface.One, IntInterface.Six>();  ///< The [Cq_a] jacobian of the constraint
        protected ChMatrixNM<IntInterface.One, IntInterface.Six> Cq_b = new ChMatrixNM<IntInterface.One, IntInterface.Six>();  ///< The [Cq_b] jacobian of the constraint

        // Auxiliary data: will be used by iterative constraint solvers:

        protected ChMatrixNM<IntInterface.Six, IntInterface.One> Eq_a = new ChMatrixNM<IntInterface.Six, IntInterface.One>();  ///< The [Eq_a] product [Eq_a]=[invM_a]*[Cq_a]'
        protected ChMatrixNM<IntInterface.Six, IntInterface.One> Eq_b = new ChMatrixNM<IntInterface.Six, IntInterface.One>();  ///< The [Eq_a] product [Eq_b]=[invM_b]*[Cq_b]'


        /// Default constructor
        public ChConstraintTwoBodies() { }

        /// Construct and immediately set references to variables
        public ChConstraintTwoBodies(ChVariablesBody mvariables_a, ChVariablesBody mvariables_b) {
            SetVariables(mvariables_a, mvariables_b);
        }

        /// Copy constructor
        public ChConstraintTwoBodies(ChConstraintTwoBodies other) : base(other) {
            Cq_a = other.Cq_a;
            Cq_b = other.Cq_b;
            Eq_a = other.Eq_a;
            Eq_b = other.Eq_b;
        }


        /// "Virtual" copy constructor (covariant return type).
        public override ChConstraint Clone() { return new ChConstraintTwoBodies(this); }

        /// Assignment operator: copy from other object
        // ChConstraintTwoBodies& operator=(const ChConstraintTwoBodies& other);

        /// Access jacobian matrix
        public override ChMatrix Get_Cq_a() { return Cq_a; }
        /// Access jacobian matrix
        public override ChMatrix Get_Cq_b() { return Cq_b; }

        /// Access auxiliary matrix (ex: used by iterative solvers)
        public override ChMatrix Get_Eq_a() { return Eq_a; }
        /// Access auxiliary matrix (ex: used by iterative solvers)
        public override ChMatrix Get_Eq_b() { return Eq_b; }

        /// Set references to the constrained objects, each of ChVariablesBody type,
        /// automatically creating/resizing jacobians if needed.
        /// If variables aren't from ChVariablesBody class, an assert failure happens.
        public override void SetVariables(ChVariables mvariables_a, ChVariables mvariables_b) {
           // Debug.Assert(ChVariablesBody)mvariables_a);
           // assert(dynamic_cast<ChVariablesBody*>(mvariables_b));

            if (mvariables_a == null || mvariables_b == null)
            {
                SetValid(false);
                return;
            }

            SetValid(true);
            variables_a = mvariables_a;
            variables_b = mvariables_b;
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
            {
                ChMatrixNM<IntInterface.Six, IntInterface.One> mtemp1 = new ChMatrixNM<IntInterface.Six, IntInterface.One>();
                mtemp1.CopyFromMatrixT(Cq_a);
                variables_a.Compute_invMb_v(Eq_a, mtemp1);
            }
            if (variables_b.IsActive())
            {
                ChMatrixNM<IntInterface.Six, IntInterface.One> mtemp1 = new ChMatrixNM<IntInterface.Six, IntInterface.One>();
                mtemp1.CopyFromMatrixT(Cq_b);
                variables_b.Compute_invMb_v(Eq_b, mtemp1);
            }

            // 2- Compute g_i = [Cq_i]*[invM_i]*[Cq_i]' + cfm_i
            ChMatrixNM<IntInterface.One, IntInterface.One> res = new ChMatrixNM<IntInterface.One, IntInterface.One>();
            g_i = 0;
            if (variables_a.IsActive())
            {
                res.MatrMultiply(Cq_a, Eq_a);
                g_i = res[0, 0];
            }
            if (variables_b.IsActive())
            {
                res.MatrMultiply(Cq_b, Eq_b);
                g_i += res[0, 0];
            }

            // 3- adds the constraint force mixing term (usually zero):
            if (cfm_i != 0)
                g_i += cfm_i;

            //Debug.Log("Log " + g_i);
        }

        ///  This function must computes the product between
        /// the row-jacobian of this constraint '[Cq_i]' and the
        /// vector of variables, 'v'. that is    CV=[Cq_i]*v
        ///  This is used for some iterative solvers.
        public override double Compute_Cq_q() {
            double ret = 0;

            if (variables_a.IsActive())
                for (int i = 0; i < 6; i++)
                    ret += Cq_a.ElementN(i) * variables_a.Get_qb().ElementN(i);

            if (variables_b.IsActive())
                for (int i = 0; i < 6; i++)
                    ret += Cq_b.ElementN(i) * variables_b.Get_qb().ElementN(i);           

            return ret;
        }

        ///  This function must increment the vector of variables
        /// 'v' with the quantity [invM]*[Cq_i]'*deltal,that is
        ///   v+=[invM]*[Cq_i]'*deltal  or better: v+=[Eq_i]*deltal
        ///  This is used for some iterative solvers.
        public override void Increment_q(double deltal) {
            if (variables_a.IsActive())
                for (int i = 0; i < 6; i++)
                    variables_a.Get_qb()[i] += Eq_a.ElementN(i) * deltal;

            if (variables_b.IsActive())
                for (int i = 0; i < 6; i++)
                    variables_b.Get_qb()[i] += Eq_b.ElementN(i) * deltal;

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
                for (int i = 0; i < 6; i++)
                    result += vect[off_a + i] * Cq_a.ElementN(i);

            if (variables_b.IsActive())
                for (int i = 0; i < 6; i++)
                    result += vect[off_b + i] * Cq_b.ElementN(i);
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
                for (int i = 0; i < 6; i++)
                    result[off_a + i] += Cq_a.ElementN(i) * l;

            if (variables_b.IsActive())
                for (int i = 0; i < 6; i++)
                    result[off_b + i] += Cq_b.ElementN(i) * l;
        }

        /// Puts the two jacobian parts into the 'insrow' row of a sparse matrix,
        /// where both portions of the jacobian are shifted in order to match the
        /// offset of the corresponding ChVariable.The same is done
        /// on the 'insrow' column, so that the sparse matrix is kept symmetric.
        public override void Build_Cq(ref ChSparseMatrix storage, int insrow) {
            if (variables_a.IsActive())
                storage.PasteMatrix(Cq_a, insrow, variables_a.GetOffset());
            if (variables_b.IsActive())
                storage.PasteMatrix(Cq_b, insrow, variables_b.GetOffset());
        }
        public override void Build_CqT(ref ChSparseMatrix storage, int inscol) {
            if (variables_a.IsActive())
                storage.PasteTranspMatrix(Cq_a, variables_a.GetOffset(), inscol);
            if (variables_b.IsActive())
                storage.PasteTranspMatrix(Cq_b, variables_b.GetOffset(), inscol);
        }

    }
}
