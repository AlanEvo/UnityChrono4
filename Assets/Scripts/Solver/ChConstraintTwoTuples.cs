using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

namespace chrono
{
    /// This constraint is built on top of two ChConstraintTuple objects, each with a tuple of
    /// 1 or 2 or 3 differently-sized jacobian chunks. For instance, this might
    /// happen because you want a constraint between an edge (i.e. two xyz variables, each
    /// per end nodes) and a triangle face (i.e. three xyz variables, each per corner), so
    /// the jacobian row matrix is split in 2 + 3 chunks, here as two tuples.
    /// Templates Ta and Tb are of ChVariableTupleCarrier_Nvars classes
    public class ChConstraintTwoTuples<Ta, Tb> : ChConstraint
    {
       // public class type_constraint_tuple_a : ChVariableTupleCarrier_1vars<Ta>.type_constraint_tuple { }
       // public class type_constraint_tuple_b : ChContactable_1vars<Tb>.type_constraint_tuple { }


        //protected ChConstraintTuple_1vars<ChVariableTupleCarrier_1vars<Ta>> tuple_a = new ChConstraintTuple_1vars<ChVariableTupleCarrier_1vars<Ta>>();
        //protected ChConstraintTuple_1vars<ChVariableTupleCarrier_1vars<Tb>> tuple_b = new ChConstraintTuple_1vars<ChVariableTupleCarrier_1vars<Tb>>();
        protected IntInterface.ChVariableTupleCarrier_1vars.type_constraint_tuple tuple_a = new IntInterface.ChVariableTupleCarrier_1vars.type_constraint_tuple();// = new ChVariableTupleCarrier_1vars<Ta>();
        protected IntInterface.ChVariableTupleCarrier_1vars.type_constraint_tuple tuple_b = new IntInterface.ChVariableTupleCarrier_1vars.type_constraint_tuple();// = new ChVariableTupleCarrier_1vars<Tb>();


        /// Default constructor
        public ChConstraintTwoTuples() { }

        /// Copy constructor
        public ChConstraintTwoTuples(ChConstraintTwoTuples<Ta, Tb> other)
        {
            tuple_a = other.tuple_a;
            tuple_b = other.tuple_b;
        }

        /// "Virtual" copy constructor (covariant return type).
        public override ChConstraint Clone()
        {
            return new ChConstraintTwoTuples<Ta, Tb>(this);
        }

        /// Assignment operator: copy from other object
        /* public ChConstraintTwoTuples operator=(const ChConstraintTwoTuples& other)
         {
             tuple_a = other.tuple_a;
             tuple_b = other.tuple_b;
             return *this;
         }*/

        /// Access tuple a
        public ref IntInterface.ChVariableTupleCarrier_1vars.type_constraint_tuple Get_tuple_a() { return ref tuple_a; }
        /// Access tuple b
        public ref IntInterface.ChVariableTupleCarrier_1vars.type_constraint_tuple Get_tuple_b() { return ref tuple_b; }

        public override void Update_auxiliary()
        {
            g_i = 0;
            tuple_a.Update_auxiliary(ref g_i);
            tuple_b.Update_auxiliary(ref g_i);
            //  adds the constraint force mixing term (usually zero):
            if (cfm_i != 0)
                g_i += cfm_i;
        }

        /// This function must computes the product between
        /// the row-jacobian of this constraint '[Cq_i]' and the
        /// vector of variables, 'v'. that is    CV=[Cq_i]*v
        /// This is used for some iterative solvers.
        public override double Compute_Cq_q()
        {
            double ret = 0;
            ret += tuple_a.Compute_Cq_q();
            ret += tuple_b.Compute_Cq_q();
            return ret;
        }

        ///  This function must increment the vector of variables
        /// 'v' with the quantity [invM]*[Cq_i]'*deltal,that is
        ///  v+=[invM]*[Cq_i]'*deltal  or better: v+=[Eq_i]*deltal
        ///  This is used for some iterative solvers.
        public override void Increment_q(double deltal)
        {
            tuple_a.Increment_q(deltal);
            tuple_b.Increment_q(deltal);
        }

        /// Computes the product of the corresponding block in the
        /// system matrix by 'vect', and add to 'result'.
        /// NOTE: the 'vect' vector must already have
        /// the size of the total variables&constraints in the system; the procedure
        /// will use the ChVariable offsets (that must be already updated) to know the
        /// indexes in result and vect;
        public override void MultiplyAndAdd(ref double result, ChMatrix vect)
        {
            tuple_a.MultiplyAndAdd(ref result, vect);
            tuple_b.MultiplyAndAdd(ref result, vect);
        }

        /// Computes the product of the corresponding transposed blocks in the
        /// system matrix (ie. the TRANSPOSED jacobian matrix C_q') by 'l', and add to 'result'.
        /// NOTE: the 'result' vector must already have
        /// the size of the total variables&constraints in the system; the procedure
        /// will use the ChVariable offsets (that must be already updated) to know the
        /// indexes in result and vect;
        public override void MultiplyTandAdd(ChMatrix result, double l)
        {
            tuple_a.MultiplyTandAdd(result, l);
            tuple_b.MultiplyTandAdd(result, l);
        }

        /// Puts the two jacobian parts into the 'insrow' row of a sparse matrix,
        /// where both portions of the jacobian are shifted in order to match the
        /// offset of the corresponding ChVariable.The same is done
        /// on the 'insrow' column, so that the sparse matrix is kept symmetric.
        public override void Build_Cq(ref ChSparseMatrix storage, int insrow)
        {
            tuple_a.Build_Cq(ref storage, insrow);
            tuple_b.Build_Cq(ref storage, insrow);
        }
        public override void Build_CqT(ref ChSparseMatrix storage, int inscol)
        {
            tuple_a.Build_CqT(ref storage, inscol);
            tuple_b.Build_CqT(ref storage, inscol);
        }
    }
}
