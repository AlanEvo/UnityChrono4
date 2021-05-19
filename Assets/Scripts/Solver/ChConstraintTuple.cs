using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

namespace chrono
{



	/// This is a container for 'half' of a constraint, and contains a tuple of
	/// 1 or 2 or 3 differently-sized jacobian chunks. For instance, this might
	/// happen because you want a constraint between an edge (i.e. two xyz variables, each
	/// per end nodes) and a triangle face (i.e. three xyz variables, each per corner), so
	/// the jacobian row matrix is split in 2 + 3 chunks, here as two tuples.
	/// The complete constraint, ChConstraintTwoTuples, will use two of these classes.
	/// Template T is a class of ChVariableTupleCarrier_Nvars type
	public class ChConstraintTuple_1vars<T>
	{

		protected ChVariables variables;                                                            //< The constrained object
	    //protected ChMatrixNM<double, IntInterface.One, IntInterface.ChVariableTupleCarrier_1vars<T>.nvars1> Cq = new ChMatrixNM<double, IntInterface.One, IntInterface.ChVariableTupleCarrier_1vars<T>.nvars1>();  //< The [Cq] jacobian of the constraint
		//protected ChMatrixNM<double, IntInterface.ChVariableTupleCarrier_1vars<T>.nvars1, IntInterface.One> Eq = new ChMatrixNM<double, IntInterface.ChVariableTupleCarrier_1vars<T>.nvars1, IntInterface.One>();  //< The [Eq] product [Eq]=[invM]*[Cq]'
		protected ChMatrixNM<IntInterface.One, IntInterface.Six> Cq = new ChMatrixNM<IntInterface.One, IntInterface.Six>(0);  //< The [Cq] jacobian of the constraint
		protected ChMatrixNM<IntInterface.Six, IntInterface.One> Eq = new ChMatrixNM<IntInterface.Six, IntInterface.One>(0);  //< The [Eq] product [Eq]=[invM]*[Cq]'

		//TEST
		ChMatrixNM<IntInterface.Six, IntInterface.One> mtemp1 = new ChMatrixNM<IntInterface.Six, IntInterface.One>(0);
		ChMatrixNM<IntInterface.One, IntInterface.One> res = new ChMatrixNM<IntInterface.One, IntInterface.One>(0);

		/// Default constructor
		public ChConstraintTuple_1vars()  
		{
			variables = null;
		}

		/// Copy constructor
		public ChConstraintTuple_1vars(ChConstraintTuple_1vars<T> other)
		{
			variables = other.variables;
			Cq = other.Cq;
			Eq = other.Eq;
		}

		/// Assignment operator: copy from other object
		/*public ChConstraintTuple_1vars operator=(ChConstraintTuple_1vars other)
		{
			variables = other.variables;
			Cq = other.Cq;
			Eq = other.Eq;
			return this;
		}*/

		public ChMatrix Get_Cq() { return Cq.matrix; }

		public ChMatrix Get_Eq() { return Eq.matrix; }

		public ChVariables GetVariables() { return variables; }

		public void SetVariables<Ta>(ref Ta m_tuple_carrier)// where Ta : notnull
		{
			IntInterface.ChVariableTupleCarrier_1vars tuple_carrier = (ChContactable_1vars<IntInterface.Six>)m_tuple_carrier; // Limitations of Generics, so could be slow.
			if (tuple_carrier.GetVariables1() == null )
			{
				throw new ChException("ERROR. SetVariables() getting null pointer. \n");
			}
			variables = tuple_carrier.GetVariables1();
		}

		public void Update_auxiliary(ref double g_i)
		{
			// 1- Assuming jacobians are already computed, now compute
			//   the matrices [Eq]=[invM]*[Cq]' and [Eq]
			if (variables.IsActive())
			{
				//ChMatrixNM<IntInterface.Six, IntInterface.One> mtemp1 = new ChMatrixNM<IntInterface.Six, IntInterface.One>(0);
				mtemp1.matrix.CopyFromMatrixT(Cq.matrix);
				variables.Compute_invMb_v(Eq.matrix, mtemp1.matrix);
			}

			// 2- Compute g_i = [Cq_i]*[invM_i]*[Cq_i]' + cfm_i
			//ChMatrixNM<IntInterface.One, IntInterface.One> res = new ChMatrixNM<IntInterface.One, IntInterface.One>(0);
			if (variables.IsActive())
			{
				res.matrix.MatrMultiply(Cq.matrix, Eq.matrix);
				g_i += res.matrix[0, 0];
			}
		}

		public double Compute_Cq_q()
		{
			double ret = 0;

			if (variables.IsActive())
				for (int i = 0; i < 6; i++)
					ret += Cq.matrix.ElementN(i) * variables.Get_qb().matrix.ElementN(i);

			return ret;
		}

		public void Increment_q(double deltal) {
			if (variables.IsActive())
				for (int i = 0; i < 6; i++)
					variables.Get_qb().matrix[i] += Eq.matrix.ElementN(i) * deltal;
				
		}

		public void MultiplyAndAdd(ref double result, ChMatrix vect) {
			int off = variables.GetOffset();

			if (variables.IsActive())
				for (int i = 0; i < 6; i++)
					result += vect[off + i] * Cq.matrix.ElementN(i);
		}

		public void MultiplyTandAdd(ChMatrix result, double l)
		{
			int off = variables.GetOffset();

			if (variables.IsActive())
				for (int i = 0; i < 6; i++)
					result[off + i] += Cq.matrix.ElementN(i) * l;
			
		}

		public void Build_Cq(ref ChSparseMatrix storage, int insrow)
		{
			if (variables.IsActive())
				storage.PasteMatrix(Cq.matrix, insrow, variables.GetOffset());
		}

		public void Build_CqT(ref ChSparseMatrix storage, int inscol)
		{
			if (variables.IsActive())
				storage.PasteTranspMatrix(Cq.matrix, variables.GetOffset(), inscol);
		}



	}

	/// Case of tuple with reference to 3 ChVariable objects:
	public class ChConstraintTuple_3vars<T1> where T1 : notnull
	{

		protected ChVariables variables_1;                                                            //< The constrained object
		protected ChVariables variables_2;
		protected ChVariables variables_3;

		/// The [Cq] jacobian of the constraint, split in horizontal chunks
		protected ChMatrixNM<IntInterface.One, IntInterface.ChVariableTupleCarrier_3vars<T1, T1, T1>.nvars1> Cq_1;
		protected ChMatrixNM<IntInterface.One, IntInterface.ChVariableTupleCarrier_3vars<T1, T1, T1>.nvars2> Cq_2;
		protected ChMatrixNM<IntInterface.One, IntInterface.ChVariableTupleCarrier_3vars<T1, T1, T1>.nvars3> Cq_3;

		/// The [Eq] product [Eq]=[invM]*[Cq]' , split in horizontal chunks
		protected ChMatrixNM<IntInterface.ChVariableTupleCarrier_3vars<T1, T1, T1>.nvars1, IntInterface.One> Eq_1;
		protected ChMatrixNM<IntInterface.ChVariableTupleCarrier_3vars<T1, T1, T1>.nvars2, IntInterface.One> Eq_2;
		protected ChMatrixNM<IntInterface.ChVariableTupleCarrier_3vars<T1, T1, T1>.nvars3, IntInterface.One> Eq_3;

		/// Default constructor
		public ChConstraintTuple_3vars()
		{
			variables_1 = null;
			variables_2 = null;
			variables_3 = null;
		}

		/// Copy constructor
		public ChConstraintTuple_3vars(ChConstraintTuple_3vars<T1> other)
		{
			variables_1 = other.variables_1;
			variables_2 = other.variables_2;
			variables_3 = other.variables_3;
			Cq_1 = other.Cq_1;
			Cq_2 = other.Cq_2;
			Cq_3 = other.Cq_3;
			Eq_1 = other.Eq_1;
			Eq_2 = other.Eq_2;
			Eq_3 = other.Eq_3;
		}
	}


/*	public interface ChVariableTupleCarrier_1vars<N1>
	{
		public class type_constraint_tuple : ChConstraintTuple_1vars<ChVariableTupleCarrier_1vars<N1>> { }
		public class nvars1 {
			public static dynamic Value = typeof(N1);
		}
		public abstract ChVariables GetVariables1();
	}*/

/*	public abstract class ChVariableTupleCarrier_3vars<N1, N2, N3> where N1 : notnull where N2 : notnull where N3 : notnull
	{
		public class type_constraint_tuple : ChConstraintTuple_3vars<ChVariableTupleCarrier_3vars<N1, N2, N3>> { }
		public struct nvars1 { public static dynamic Value = typeof(N1); }
		public struct nvars2 { public static dynamic Value = typeof(N2); }
		public struct nvars3 { public static dynamic Value = typeof(N3); }
		public abstract ChVariables GetVariables1();
		public abstract ChVariables GetVariables2();
		public abstract ChVariables GetVariables3();
	}*/

}
