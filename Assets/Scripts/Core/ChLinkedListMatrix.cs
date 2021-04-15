using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Runtime.InteropServices;


namespace chrono
{
	/// Generic element of a sparse matrix ChLinkedListMatrix.
	public class ChMelement
	{
		public double val;
		public ChMelement next;
		public ChMelement prev;
		public int row;
		public int col;

		public ChMelement() { }

		public ChMelement(double mval, ChMelement mnext, ChMelement mprev, int mrow, int mcol)
		{
			this.val = mval;
			this.next = mnext;
			this.prev = mprev;
			this.row = mrow;
			this.col = mcol;
		}

		public void Initialize(double mval, ChMelement mnext, ChMelement mprev, int mrow, int mcol)
		{
			val = mval;
			next = mnext;
			prev = mprev;
			col = mcol;
			row = mrow;
		}
	}

	// -------------------------------
	// SPARSE LINKED LIST MATRIX CLASS
	// -------------------------------

	/// This class defines a sparse matrix, implemented using linked lists of
	/// non-zero elements in each row.
	public class ChLinkedListMatrix : ChSparseMatrix
	{
		private ChMelement[] elarray; // array of 1st column elements

		private ChList<ChMelement> mbufferlist = new ChList<ChMelement>(); // list of pointers to Melement* buffers
		private ChMelement[] mnextel; // address of next writable element in mbufferlist

		private int mtot_size; // total allocated space for elements (maybe in noncontiguous buffers list)
		private int mbuffer_size; // size of currently used buffer
		private int mbuffer_added; // mbuffer_added grows all time a new element is set non-zero,
								   // until it reaches mbuffer_size (if so, allocation of another buffer is needed). Handled internally.

		private List<int> m_pindices = new List<int>();
		private double m_determinant;

		public double PIVOT_ACCEPT_TRESHOLD = 0.8;
		public double ACCEPT_PIVOT = 1e-3;
		public double MIN_PIVOT = 1e-8;
		public double INFINITE_PIVOT = 1e+34;


		/// Default constructor.
		public ChLinkedListMatrix() : base(1, 1)
		{
			m_determinant = 0;
			this.Build(1.0);
		}

		/// Create a sparse matrix with given dimensions and fill-in.
		public ChLinkedListMatrix(int nrows, int ncols, double fill_in = ChSparseMatrix.SPM_DEF_FULLNESS) : base(nrows, ncols)
		{
			m_determinant = 0;
			this.Build(fill_in);
		}

		/// Create a sparse matrix from a given dense matrix.
		//public ChLinkedListMatrix(ChMatrix<double> mat) { 

		//}

		/// Copy constructor.
		public ChLinkedListMatrix(ChLinkedListMatrix other) : base(other)
		{

		}

		/// Destructor.
		~ChLinkedListMatrix() => Dispose(false);

		// Public implementation of Dispose pattern callable by consumers.
		public void Dispose()
		{
			Dispose(true);
			GC.SuppressFinalize(this);
		}

		protected virtual void Dispose(bool isDisposing)
		{
			// TODO If you need thread safety, use a lock around these 
			// operations, as well as in your methods that use the resource.
			try
			{
				if (!this.IsDisposed)
				{
					if (isDisposing)
					{
						// TODO Release all managed resources here
						mbuffer_size = 0;
						mbuffer_added = 0;

						mbufferlist.RemoveAll();  //  may be avoided, since it will automatically remove at list deletion
						mtot_size = 0;
					}

					// TODO Release all unmanaged resources here

					//free(elarray);
					//Marshal.FreeCoTaskMem((IntPtr)elarray);
					//elarray = null;

					/*for (ChNode<ChMelement> mnode = mbufferlist.GetHead(); mnode != null; mnode = mnode.next)
					{
						//free(mnode.data);  // deallocate memory buffers
						//Marshal.FreeCoTaskMem((IntPtr)mnode.data);
					}*/

					// TODO explicitly set root references to null to expressly tell the GarbageCollector
					// that the resources have been disposed of and its ok to release the memory allocated for them.


				}
			}
			finally
			{
				// explicitly call the base class Dispose implementation
				//base.Dispose(isDisposing);

				this.IsDisposed = true;
			}
		}

		public override void SetElement(int row, int col, double elem, bool overwrite = true) {
			//ChMelement pointer = new ChMelement();
			ChMelement guess;// = &pointer;
			guess = elarray[row];
			SetElement(row, col, elem, guess);
		}

		/// Returns the value of the element with index (\a row, \a col).
		/// Returns \c zero if an element is not stored.
		public override double GetElement(int row, int col) {
			double retv = 0;
			ChMelement guess;
			guess = elarray[row];
			(this).GetElement(row, col, ref retv, guess);
			return retv;
		}

		public override void Reset(int row, int col, int nonzeros = 0) { }
		public override bool Resize(int nrows, int ncols, int nonzeros = 0) { return false; }

		public void Build(double fill_in)
		{
			mtot_size = 0;
			mbuffer_added = 0;
			mnextel = null;

			// alloc buffer of elements
			mbuffer_size = (int)(m_num_rows * m_num_cols * fill_in);
			if (m_num_rows > 5000)
				mbuffer_size = (int)ChSparseMatrix.SPM_DEF_MAXELEMENTS;
			if (m_num_cols > 5000)
				mbuffer_size = (int)ChSparseMatrix.SPM_DEF_MAXELEMENTS;
			if (mbuffer_size > (int)ChSparseMatrix.SPM_DEF_MAXELEMENTS)
				mbuffer_size = (int)ChSparseMatrix.SPM_DEF_MAXELEMENTS;
			if (mbuffer_size < m_num_rows * 2)
				mbuffer_size = m_num_rows * 2;


			//var mbuffer = (ChMelement*)Marshal.AllocHGlobal(mbuffer_size *sizeof(ChMelement));
			ChMelement[] mbuffer = new ChMelement[mbuffer_size];
			for(int i = 0; i < mbuffer.Length; i++)
            {
				mbuffer[i] = new ChMelement();
			}

			mtot_size += mbuffer_size;
			mnextel = mbuffer;
			mbufferlist.AddHead(mbuffer);

			// alloc pointers to first column
			//elarray = (ChMelement[])calloc(m_num_rows, sizeof(ChMelement*));
			//IntPtr array = Marshal.AllocHGlobal(m_num_rows);
			//Marshal.FreeHGlobal(array);
			//ChMelement** array = stackalloc ChMelement*[m_num_rows];

			//elarray = (ChMelement**)Marshal.AllocHGlobal(m_num_rows * sizeof(ChMelement));
			elarray = new ChMelement[m_num_rows];
			for (int i = 0; i < elarray.Length; i++)
			{
				elarray[i] = new ChMelement();
			}

			for (int i = 0; i < m_num_rows; i++)
			{
				ChMelement pointer = mnextel[i];
				(elarray[i]) = pointer;                    // initialize vector of 1st col pointr pointers
				(mnextel[i]).Initialize(0, null, null, i, 0);  // initialize 1st col.elements
				mbuffer_added++;                             // increment the counter of "used" elements.
				//mnextel++;
				pointer = mnextel[i + 1];
			}
		}

		public void SwapRows(int a, int b)
		{

			ChMelement mtemp;
			mtemp = elarray[a];
			elarray[a] = elarray[b];  // row lists easy switched here...
			elarray[b] = mtemp;

			for (mtemp = elarray[a]; mtemp != null; mtemp = mtemp.next)
			{
				mtemp.row = a;
			}
			for (mtemp = elarray[b]; mtemp != null; mtemp = mtemp.next)
			{
				mtemp.row = b;
			}
		}

		/// Optimized SetElement,  returning the fetched Melement*
		public ChMelement SetElement(int row, int col, double val, ChMelement guess)
		{
			/*Debug.Assert(row >= 0);
			Debug.Assert(col >= 0);
			Debug.Assert(row < m_num_rows);
			Debug.Assert(col < m_num_cols);
			Debug.Assert(guess.row == row);*/

			ChMelement enext;
			ChMelement eprev;
			ChMelement newguess;

			while (guess.col != col)
			{
				// forward search
				if (guess.col < col)
				{
					enext = guess.next;
					if (enext != null)
					{
						if (enext.col <= col)
						{
							guess = enext; // .. and repeat
						}
						else // if (enext.col >  col)
						{
							newguess = NewElement(val, enext, guess, row, col);
							guess.next = newguess;
							enext.prev = newguess;
							return (newguess);
						}
					}
					else
					{
						newguess = NewElement(val, null, guess, row, col); // this one!  bast!
						guess.next = newguess;
						return (newguess);
					}
				}

				// backward search
				if (guess.col > col)
				{
					eprev = guess.prev;
					if (eprev != null)
					{
						if (eprev.col >= col)
						{
							guess = eprev; // .. and repeat
						}
						else // if (eprev.col <  col)
						{
							newguess = NewElement(val, guess, eprev, row, col);
							eprev.next = newguess;
							guess.prev = newguess;
							return (newguess);
						}
					}
					else
					{
						newguess = NewElement(val, guess, null, row, col);
						guess.prev = newguess;
						return (newguess);
					}
				}
			}

			guess.val = val;
			return (guess);
		}

		/// Get matrix determinant.
		/// Available after a factorization.
		public double GetDeterminant() { return m_determinant; }

		/// Optimized GetElement,  returning the fetched Melement*
		public ChMelement GetElement(int row, int col, ref double val, ChMelement guess)
		{
			/*Debug.Assert(row >= 0);
			Debug.Assert(col >= 0);
			Debug.Assert(row < m_num_rows);
			Debug.Assert(col < m_num_cols);
			Debug.Assert(guess.row == row);*/

			ChMelement enext;
			ChMelement eprev;

			while (guess.col != col)
			{
				// forward search
				if (guess.col < col)
				{
					enext = guess.next;
					if (enext != null)
					{
						if (enext.col <= col)
							guess = enext;  // .. and repeat
						else                // if (enext.col >  col)
						{
							val = 0;
							return (guess);
						}
					}
					else
					{
						val = 0;
						return (guess);
					}
				}

				// backward search
				if (guess.col > col)
				{
					eprev = guess.prev;
					if (eprev != null)
					{
						if (eprev.col >= col)
							guess = eprev;  // .. and repeat
						else                // if (eprev.col <  col)
						{
							val = 0;
							return (guess);
						}
					}
					else
					{
						val = 0;
						return (guess);
					}
				}

			}  // end while loop

			val = guess.val;
			return (guess);
		}

		/// Solve the general system A*x = b.
		/// Note that the matrix is modified in-place to contain the LU factors.
		public int SolveGeneral(ChMatrix b, ChMatrix x)
		{
			int err = Setup_LU();
			Solve_LU(b, ref x);
			return err;
		}

		/// Perform in-place LDL factorization with full pivoting.
		/// Note that this factorization can only be performed for symmetric matrices.
		/// During the factorization, only the upper triangular part of A is accessed.
		public int Setup_LDL()
		{
			//Debug.Assert(m_num_rows == m_num_cols);

			ChMelement rowel;
			ChMelement subrowel;
			int i, k, pivrow, eqpivoted;
			double r, leader, pivot, mval, subval = 0, newval;
			int err = 0;

			// initialize pivot index array
			//m_pindices.resize(m_num_rows);
			ListExtras.Resize(m_pindices, m_num_rows);
			for (int ind = 0; ind < m_num_rows; ind++)
			{
				m_pindices[ind] = ind;
			}

			m_determinant = 1;

			for (k = 1; k < m_num_rows; k++)
			{
				pivot = GetElement(k - 1, k - 1);

				if (Math.Abs(pivot) < ACCEPT_PIVOT)
				{
					// pivoting is needed, so ...
					pivrow = BestPivotDiag(k - 1);
					DiagPivotSymmetric(k - 1, pivrow);  // swap both column and row (only upper right!)
					m_determinant *= -1;

					// swap diagonal pivot indexes
					eqpivoted = m_pindices[pivrow];
					m_pindices[pivrow] = m_pindices[k - 1];
					m_pindices[k - 1] = eqpivoted;

					pivot = GetElement((k - 1), (k - 1));
				}

				// was unable to find better pivot: force solution to zero.and go ahead
				if (Math.Abs(pivot) <= MIN_PIVOT)
				{
					pivot = INFINITE_PIVOT;
					SetElement(k - 1, k - 1, pivot);
					if (err == 0)
					{
						m_determinant = 0;
						err = (1 + m_num_rows - k);
					}
				}
				else
				{
					m_determinant *= pivot;
				}

				for (i = k; i < m_num_rows; i++)
				{
					leader = GetElement(k - 1, i);

					if (leader != 0)
					{
						r = (leader / pivot);  // compute multiplier (mirror look A)

						subrowel = GetElarrayMel(i);   // initialize guessed sparse elements
						rowel = GetElarrayMel(k - 1);  // for speed optimization. They are in two rows.

						// advance top row element till it has (col >= k)

						// for (j=i; j < m_num_cols; j++)     only upper right part
						for (; rowel != null; rowel = rowel.next)
						{
							// only upper right part is filled!
							if (rowel.col >= i)
							{
								mval = rowel.val;
								subrowel = GetElement(i, rowel.col, ref subval, subrowel);
								newval = subval - r * mval;
								subrowel = SetElement(i, rowel.col, newval, subrowel);
							}
						}

						SetElement((k - 1), i, r);  // now can store the multiplier in L part (mirror look! is up)
					}
				}
			}

			pivot = GetElement(m_num_rows - 1, m_num_rows - 1);
			if (Math.Abs(pivot) <= MIN_PIVOT)
			{
				pivot = INFINITE_PIVOT;
				SetElement(m_num_rows - 1, m_num_rows - 1, pivot);
				if (err == 0)
				{
					m_determinant = 0;
					err = (1);
				}
			}
			else
			{
				m_determinant *= pivot;
			}

			return err;
		}

		// -----------------------------------------------------------------------------
		// Solution of general linear systems using LU factorization
		// -----------------------------------------------------------------------------

		// LU factorization
		public int Setup_LU()
		{
			//Debug.Assert(m_num_rows == m_num_cols);

			ChMelement rowel;
			ChMelement subrowel;
			int i, k, pivrow, eqpivoted;
			double r, pivot, mval, subval = 0, newval, leader;
			int err = 0;

			// initialize pivot index array
			//m_pindices.resize(m_num_rows);
			ListExtras.Resize(m_pindices, m_num_rows);
			for (int ind = 0; ind < m_num_rows; ind++)
			{
				m_pindices[ind] = ind;
			}

			m_determinant = 1;

			for (k = 1; k < m_num_rows; k++)
			{
				pivot = GetElement(k - 1, k - 1);

				if (Math.Abs(pivot) < ACCEPT_PIVOT)
				{
					// pivoting is needed, so swap equations
					pivrow = BestPivotRow(k - 1);
					SwapRows(k - 1, pivrow);
					m_determinant *= -1;

					eqpivoted = m_pindices[pivrow];  // swap eq.ids in pivot array
					m_pindices[pivrow] = m_pindices[k - 1];
					m_pindices[k - 1] = eqpivoted;

					pivot = GetElement(k - 1, k - 1);  // fetch new pivot
				}

				// was unable to find better pivot: force solution to zero.and go ahead
				if (Math.Abs(pivot) <= MIN_PIVOT)
				{
					pivot = INFINITE_PIVOT;
					SetElement(k - 1, k - 1, pivot);
					if (err == 0)
					{
						m_determinant = 0;
						err = (1 + m_num_rows - k);  // report deficiency
					}
				}
				else
				{
					m_determinant *= pivot;
				}

				for (i = k; i < m_num_rows; i++)
				{
					leader = GetElement(i, k - 1);

					if (leader != 0)
					{
						r = leader / pivot;       // compute multiplier
						SetElement(i, k - 1, r);  // store the multiplier in L part!!!

						subrowel = GetElarrayMel(i);   // initialize guessed sparse elements
						rowel = GetElarrayMel(k - 1);  // for speed optimization. They are in two rows.

						// for (j=k; j < m_num_cols; j++)  where j = rowel.col
						for (; rowel != null; rowel = rowel.next)
						{
							if (rowel.col >= k)
							{
								mval = rowel.val;
								subrowel = GetElement(i, rowel.col, ref subval, subrowel);
								newval = subval - r * mval;
								subrowel = SetElement(i, rowel.col, newval, subrowel);  // set the U part
							}
						}
					}
				}
			}

			pivot = GetElement((m_num_rows - 1), (m_num_rows - 1));
			if (Math.Abs(pivot) <= MIN_PIVOT)
			{
				pivot = INFINITE_PIVOT;
				SetElement(m_num_rows - 1, m_num_rows - 1, pivot);
				if (err == 0)
				{
					m_determinant = 0;
					err = 1;
				}
			}
			else
			{
				m_determinant *= pivot;
			}

			return err;
		}

		// Substitution using existing LU factorization
		public void Solve_LU(ChMatrix b, ref ChMatrix x)
		{
			//Debug.Assert(m_num_rows == b.GetRows());
			//Debug.Assert(m_num_cols == x.GetRows());

			// BACKWARD substitution - L
			double xlast = b.GetElement(m_pindices[0], 0);
			x.SetElement(0, 0, xlast);

			for (int k = 1; k < m_num_rows; k++)
			{
				double sum = 0;

				ChMelement rowel = GetElarrayMel(k);

				for (; (rowel != null && rowel.col < k); rowel = rowel.next)
				{
					sum += rowel.val * (x.GetElement(rowel.col, 0));
				}

				double val = (b.GetElement(m_pindices[k], 0) - sum);
				x.SetElement(k, 0, val);
			}

			// BACKWARD substitution - U
			xlast = x.GetElement(m_num_rows - 1, 0) / GetElement(m_num_rows - 1, m_num_rows - 1);
			x.SetElement(m_num_rows - 1, 0, xlast);

			for (int k = m_num_rows - 2; k >= 0; k--)
			{
				double sum = 0;

				ChMelement rowel = GetElarrayMel(k);

				for (; rowel != null; rowel = rowel.next)
				{
					if (rowel.col >= k + 1)
					{
						sum += rowel.val * (x.GetElement(rowel.col, 0));
					}
				}

				double val = (x.GetElement(k, 0) - sum) / GetElement(k, k);
				x.SetElement(k, 0, val);
			}
		}
		// Support function for LU factorization
		public int BestPivotRow(int current)
		{
			double temp = 0;
			int pivotrow = current;

			for (int i = current; i < m_num_rows; i++)
			{
				double val = Math.Abs(GetElement(i, current));
				if (val > temp)
				{
					temp = val;
					pivotrow = i;
				}
				if (temp >= PIVOT_ACCEPT_TRESHOLD)
					break;
			}
			return pivotrow;
		}

		// Support function for LDL factorization
		// Effect only on upper-left triangle!
		public void DiagPivotSymmetric(int rowA, int rowB)
		{
			int elcol, elrow;
			double temp;
			for (elrow = 0; elrow < rowA; elrow++)
			{
				temp = GetElement(elrow, rowA);
				SetElement(elrow, rowA, GetElement(elrow, rowB));
				SetElement(elrow, rowB, temp);
			}
			for (elcol = rowB + 1; elcol < m_num_cols; elcol++)
			{
				temp = GetElement(rowA, elcol);
				SetElement(rowA, elcol, GetElement(rowB, elcol));
				SetElement(rowB, elcol, temp);
			}
			for (elcol = rowA + 1; elcol < rowB; elcol++)
			{
				temp = GetElement(rowA, elcol);
				SetElement(rowA, elcol, GetElement(elcol, rowB));
				SetElement(elcol, rowB, temp);
			}
			temp = GetElement(rowA, rowA);
			SetElement(rowA, rowA, GetElement(rowB, rowB));
			SetElement(rowB, rowB, temp);
		}

		/// Add another buffer in buffer list, bigger of last
		/// (max total buffer size = col x rows) as in newbuffer = lastbuffer * inflate
		/// (use inflate >1, suggested 2)     [mostly used internally]
		public void MoreBuffer(double inflate) {
			mbuffer_size = (int)(inflate * (double)mbuffer_size);  // inflate buffer size

			if (m_num_rows < 5000)
				if ((mtot_size + mbuffer_size) > m_num_rows * m_num_cols)
					mbuffer_size = m_num_rows * m_num_cols - mtot_size;

			ChMelement[] mbuffer = new ChMelement[mbuffer_size];
			for(int i = 0; i < mbuffer.Length; i++)
            {
				mbuffer[i] = new ChMelement();
			}

			mtot_size += mbuffer_size;
			mnextel = mbuffer;
			mbufferlist.AddTail(mbuffer);  // add another buffer!

			mbuffer_added = 0;
		}

		// Support function for LDL factorization
		public int BestPivotDiag(int current)
		{
			double temp = 0;
			int i;
			int pivotrow = current;

			for (i = current; i < m_num_rows; i++)
			{
				if (Math.Abs(GetElement(i, i)) > temp)
				{
					temp = Math.Abs(GetElement(i, i));
					pivotrow = i;
				}
				if (temp >= PIVOT_ACCEPT_TRESHOLD)
					break;
			}
			return pivotrow;
		}

		private ChMelement GetElarrayMel(int num) { return elarray[num]; }
		private ChMelement NewElement(double mval, ChMelement mnext, ChMelement mprev, int mrow, int mcol)
		{
			if (mbuffer_added >= mbuffer_size)
			{
				MoreBuffer(2.0);
			}

			/*ChMelement[] newel = mnextel;
			mnextel.Initialize(mval, mnext, mprev, mrow, mcol);
			mbuffer_added++;
			mnextel++;*/

			ChMelement newel = mnextel[mbuffer_added];
			mnextel[mbuffer_added].Initialize(mval, mnext, mprev, mrow, mcol);
			mbuffer_added++;

			//ChMelement newel = new ChMelement();
			/*ChMelement newel = mnextel;
			for (int i = 1; i < mnextel.Length; i++) { 

				mnextel[i].Initialize(mval, mnext, mprev, mrow, mcol);
				mbuffer_added++;
				newel = mnextel[i + 1];
				//mnextel[i] = newel;
			}*/

			return (newel);
		}
	}
}
