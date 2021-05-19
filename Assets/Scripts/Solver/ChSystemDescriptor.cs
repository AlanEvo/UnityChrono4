using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono {
    /// Base class for collecting objects inherited from ChConstraint,
    /// ChVariables and optionally ChKblock. These objects
    /// can be used to define a sparse representation of the system.
    /// This collector is important because it contains all the required
    /// information that is sent to a solver (usually a VI/CCP solver, or
    /// as a subcase, a linear solver).\n
    /// The problem is described by a variational inequality VI(Z*x-d,K):\n
    /// The matrix \f$Z\f$ that represents the problem has this form:
    /// <pre>
    ///  | H -Cq'|*|q|- | f|= |0|
    ///  | Cq -E | |l|  |-b|  |c|
    /// </pre>
    /// with \f$Y \ni \mathbb{l}  \perp \mathbb{c} \in N_y\f$ \n
    /// where \f$N_y\f$ is the normal cone to \f$Y\f$ \n
    /// By flipping the sign of \f$l_i\f$, the matrix \f$Z\f$ can be symmetric (but in general non positive definite)
    /// <pre>
    /// | H  Cq'|*| q|-| f|=|0|
    /// | Cq  E | |-l| |-b| |c|
    /// </pre>
    ///
    /// * Linear Problem: \f$ \forall i \,Y_i = \mathbb{R}, N_{y_{i}} = 0\f$ (e.g. all bilateral)
    /// * Linear Complementarity Problem (LCP): \f$ 0\le c \perp l\ge0 \f$ (i.e. \f$Y_i = \mathbb{R}^+\f$)
    /// * Cone Complementarity Problem (CCP): \f$Y \ni \mathbb{l}  \perp \mathbb{c} \in N_y\f$ (\f$Y_i\f$ are friction
    /// cones)
    ///
    /// *Notes*
    /// 1. most often you call ConvertToMatrixForm() right after a dynamic simulation step,
    ///    in order to get the system matrices updated to the last timestep;
    /// 2. when using Anitescu default stepper, the 'f' vector contains forces*timestep = F*dt
    /// 3. when using Anitescu default stepper, 'q' represents the 'delta speed',
    /// 4. when using Anitescu default stepper, 'b' represents the dt/phi stabilization term.
    /// 5. usually, H = M, the mass matrix, but in some cases, ex. when using
    ///         implicit integrators, objects inherited from ChKblock can be added
    ///         too, hence H could be H=a*M+b*K+c*R (but not all solvers handle ChKblock!)
    ///
    /// All solvers require that the description of the system
    /// is passed by means of a ChSystemDescriptor object
    /// that holds a list of all the constraints, variables, masses, known terms
    ///	(ex.forces) under the form of ChVariables, ChConstraints and ChKblock.
    ///
    /// In this default implementation, the ChSystemDescriptor
    /// simply holds a vector of pointers to ChVariables
    /// or to ChConstraints, but more advanced implementations (ex. for
    /// supporting parallel GPU solvers) could store constraints
    /// and variables structures with other, more efficient data schemes.
    public class ChSystemDescriptor
    {
        //
        // DATA
        //
        protected List<ChConstraint> vconstraints = new List<ChConstraint>();  //< list of pointers to all the ChConstraint in the current Chrono system
        protected List<ChVariables> vvariables = new List<ChVariables>();     //< list of pointers to all the ChVariables in the current Chrono system
        protected List<ChKblock> vstiffness = new List<ChKblock>();        //< list of pointers to all the ChKblock in the current Chrono system

        protected int num_threads;

        protected ChSpinlock[] spinlocktable;

        protected double c_a;  // coefficient form M mass matrices in vvariables

        private int n_q;            //< number of active variables
        private int n_c;            //< number of active constraints
        private bool freeze_count;  //< for optimization: avoid to re-count the number of active variables and constraints

        public int CH_SPINLOCK_HASHSIZE = 203;

        /// Constructor
        public ChSystemDescriptor() {
            vconstraints.Clear();
            vvariables.Clear();
            vstiffness.Clear();

            c_a = 1.0;

            n_q = 0;
            n_c = 0;
            freeze_count = false;

            this.num_threads = CHOMPfunctions.GetNumProcs();

            spinlocktable = new ChSpinlock[CH_SPINLOCK_HASHSIZE];
        }

        //
        // DATA MANAGEMENT FUNCTIONS
        //

        /// Access the vector of constraints
        public ref List<ChConstraint> GetConstraintsList() { return ref vconstraints; }

        /// Access the vector of variables
        public ref List<ChVariables> GetVariablesList() { return ref vvariables; }

        /// Access the vector of stiffness matrix blocks
        public ref List<ChKblock> GetKblocksList() { return ref vstiffness; }

        /// Begin insertion of items
        public virtual void BeginInsertion()
        {
            vconstraints.Clear();
            vvariables.Clear();
            vstiffness.Clear();
        }

        /// Insert reference to a ChConstraint object
        public virtual void InsertConstraint(ChConstraint mc) { vconstraints.Add(mc); }

        /// Insert reference to a ChVariables object
        public virtual void InsertVariables(ChVariables mv) { vvariables.Add(mv); }

        /// Insert reference to a ChKblock object (a piece of matrix)
        public virtual void InsertKblock(ChKblock mk) { vstiffness.Add(mk); }

        /// End insertion of items
        public virtual void EndInsertion() { UpdateCountsAndOffsets(); }

        /// Count & returns the scalar variables in the system (excluding ChVariable objects
        /// that have  IsActive() as false). Note: the number of scalar variables is not necessarily
        /// the number of inserted ChVariable objects, some could be inactive.
        /// Note: this function also updates the offsets of all variables
        /// in 'q' global vector (see GetOffset() in ChVariables).
        public virtual int CountActiveVariables() {
            if (this.freeze_count)  // optimization, avoid list count all times
                return n_q;

            n_q = 0;
            for (int iv = 0; iv < vvariables.Count; iv++)
            {
                if (vvariables[iv].IsActive())
                {
                    vvariables[iv].SetOffset(n_q);  // also store offsets in state and MC matrix
                    n_q += vvariables[iv].Get_ndof();
                }
            }
            return n_q;
        }

        /// Count & returns the scalar constraints in the system (excluding ChConstraint objects
        /// that have  IsActive() as false).
        /// Note: this function also updates the offsets of all constraints
        /// in 'l' global vector (see GetOffset() in ChConstraint).
        public virtual int CountActiveConstraints() {
            if (this.freeze_count)  // optimization, avoid list count all times
                return n_c;

            n_c = 0;
            for (int ic = 0; ic < vconstraints.Count; ic++)
            {
                if (vconstraints[ic].IsActive())
                {
                    vconstraints[ic].SetOffset(n_c);  // also store offsets in state and MC matrix
                    n_c++;
                }
            }
            return n_c;
        }

        /// Updates counts of scalar variables and scalar constraints,
        /// if you added/removed some item or if you switched some active state,
        /// otherwise CountActiveVariables() and CountActiveConstraints() might fail.
        public virtual void UpdateCountsAndOffsets() {
            freeze_count = false;
            CountActiveVariables();
            CountActiveConstraints();
            freeze_count = true;
        }

        /// Sets the c_a coefficient (default=1) used for scaling the M masses of the vvariables
        /// when performing ShurComplementProduct(), SystemProduct(), ConvertToMatrixForm(),
        public virtual void SetMassFactor(double mc_a) { c_a = mc_a; }

        /// Gets the c_a coefficient (default=1) used for scaling the M masses of the vvariables
        /// when performing ShurComplementProduct(), SystemProduct(), ConvertToMatrixForm(),
        public virtual double GetMassFactor() { return c_a; }

        //
        // DATA <. MATH.VECTORS FUNCTIONS
        //

        /// Get a vector with all the 'fb' known terms ('forces'etc.) associated to all variables,
        /// ordered into a column vector. The column vector must be passed as a ChMatrix<>
        /// object, which will be automatically reset and resized to the proper length if necessary.
        public virtual int BuildFbVector(ref ChMatrix Fvector  //< matrix which will contain the entire vector of 'f'
    )
        {
            n_q = CountActiveVariables();
            Fvector.Reset(n_q, 1);  // fast! Reset() method does not realloc if size doesn't change

            // Fills the 'f' vector
            for (int iv = 0; iv < (int)vvariables.Count; iv++)
            {
                if (vvariables[iv].IsActive())
                {
                    Fvector.PasteMatrix(vvariables[iv].Get_fb().matrix, vvariables[iv].GetOffset(), 0);
                }
            }
            return this.n_q;
        }
        /// Get a vector with all the 'bi' known terms ('constraint residuals' etc.) associated to all constraints,
        /// ordered into a column vector. The column vector must be passed as a ChMatrix<>
        /// object, which will be automatically reset and resized to the proper length if necessary.
        public virtual int BuildBiVector(ChMatrix Bvector  //< matrix which will contain the entire vector of 'b'
    )
        {
            n_c = CountActiveConstraints();
            Bvector.Resize(n_c, 1);

            // Fill the 'b' vector
            for (int ic = 0; ic < (int)vconstraints.Count; ic++)
            {
                if (vconstraints[ic].IsActive())
                {
                    Bvector[vconstraints[ic].GetOffset()] = vconstraints[ic].Get_b_i();
                }
            }

            return n_c;
        }

        /// Get the d vector = {f; -b} with all the 'fb' and 'bi' known terms, as in  Z*y-d
        /// (it is the concatenation of BuildFbVector and BuildBiVector) The column vector must be passed as a ChMatrix<>
        /// object, which will be automatically reset and resized to the proper length if necessary.
        public virtual int BuildDiVector(ref ChMatrix Dvector  //< matrix which will contain the entire vector of {f;-b}
    )
        {
            n_q = CountActiveVariables();
            n_c = CountActiveConstraints();

            Dvector.Reset(n_q + n_c, 1);  // fast! Reset() method does not realloc if size doesn't change

            // Fills the 'f' vector part
            for (int iv = 0; iv < (int)vvariables.Count; iv++)
            {
                if (vvariables[iv].IsActive())
                {
                    Dvector.PasteMatrix(vvariables[iv].Get_fb().matrix, vvariables[iv].GetOffset(), 0);
                }
            }
            // Fill the '-b' vector (with flipped sign!)
            for (int ic = 0; ic < (int)vconstraints.Count; ic++)
            {
                if (vconstraints[ic].IsActive())
                {
                    Dvector[vconstraints[ic].GetOffset() + n_q] = -vconstraints[ic].Get_b_i();
                }
            }

            return n_q + n_c;
        }

        /// Get the D diagonal of the Z system matrix, as a single column vector (it includes all the diagonal
        /// masses of M, and all the diagonal E (-cfm) terms).
        /// The Diagonal_vect must already have the size of n. of unknowns, otherwise it will be resized if necessary).
        public virtual int BuildDiagonalVector(
            ref ChMatrix Diagonal_vect  //< matrix which will contain the entire vector of terms on M and E diagonal
    )
        {
            n_q = CountActiveVariables();
            n_c = CountActiveConstraints();

            Diagonal_vect.Reset(n_q + n_c, 1);  // fast! Reset() method does not realloc if size doesn't change

            // Fill the diagonal values given by ChKblock objects , if any
            // (This cannot be easily parallelized because of possible write concurrency).
            for (int i = 0; i < (int)vstiffness.Count; i++)
            {
                vstiffness[i].DiagonalAdd(ref Diagonal_vect);
            }

            // Get the 'M' diagonal terms given by ChVariables objects
            for (int iv = 0; iv < (int)vvariables.Count; iv++)
            {
                if (vvariables[iv].IsActive())
                {
                    vvariables[iv].DiagonalAdd(ref Diagonal_vect, this.c_a);
                }
            }

            // Get the 'E' diagonal terms (note the sign: E_i = -cfm_i )
            for (int ic = 0; ic < (int)vconstraints.Count; ic++)
            {
                if (vconstraints[ic].IsActive())
                {
                    Diagonal_vect[vconstraints[ic].GetOffset() + n_q] = -vconstraints[ic].Get_cfm_i();
                }
            }
            return n_q + n_c;
        }

        /// Using this function, one may get a vector with all the variables 'q'
        /// ordered into a column vector. The column vector must be passed as a ChMatrix<>
        /// object, which will be automatically reset and resized to the proper length if necessary
        /// (but if you are sure that the vector has already the proper size, you can optimize
        /// the performance a bit by setting resize_vector as false).
        /// \return  the number of scalar variables (i.e. the rows of the column vector).
        public virtual int FromVariablesToVector(
            ChMatrix mvector,       //< matrix which will contain the entire vector of 'q'
            bool resize_vector = true  //< if true the vector size will be checked & resized if necessary
    )
        {
            // Count active variables and resize vector if necessary
            if (resize_vector)
            {
                n_q = CountActiveVariables();
                mvector.Resize(n_q, 1);
            }

            // Fill the vector
            for (int iv = 0; iv < vvariables.Count; iv++)
            {
                if (vvariables[iv].IsActive())
                {
                    mvector.PasteMatrix(vvariables[iv].Get_qb().matrix, vvariables[iv].GetOffset(), 0);
                }
            }

            return n_q;
        }

        /// Using this function, one may go in the opposite direction of the FromVariablesToVector()
        /// function, i.e. one gives a vector with all the variables 'q' ordered into a column vector, and
        /// the variables objects are updated according to these values.
        /// NOTE!!! differently from  FromVariablesToVector(), which always works, this
        /// function will fail if mvector does not match the amount and ordering of
        /// the variable objects!!! (it is up to the user to check this!) btw: most often,
        /// this is called after FromVariablesToVector() to do a kind of 'undo', for example.
        /// \return  the number of scalar variables (i.e. the rows of the column vector).
        public virtual int FromVectorToVariables(ChMatrix mvector  //< matrix which contains the entire vector of 'q'
    )
        {
          //  Debug.Assert(CountActiveVariables() == mvector.GetRows());
          //  Debug.Assert(mvector.GetColumns() == 1);

            // fetch from the vector
            for (int iv = 0; iv < vvariables.Count; iv++)
            {
                if (vvariables[iv].IsActive())
                {
                    vvariables[iv].Get_qb().matrix.PasteClippedMatrix(mvector, vvariables[iv].GetOffset(), 0,
                                                                vvariables[iv].Get_ndof(), 1, 0, 0);
                }
            }

            return n_q;
        }

        /// Using this function, one may get a vector with all the constraint reactions 'l_i'
        /// ordered into a column vector. The column vector must be passed as a ChMatrix<>
        /// object, which will be automatically reset and resized to the proper length if necessary
        /// (but uf you are sure that the vector has already the proper size, you can optimize
        /// the performance a bit by setting resize_vector as false).
        /// Optionally, you can pass an 'enabled' vector of bools, that must have the same
        /// length of the l_i reactions vector; constraints with enabled=false are not handled.
        /// \return  the number of scalar constr.multipliers (i.e. the rows of the column vector).
        public virtual int FromConstraintsToVector(
            ChMatrix mvector,       //< matrix which will contain the entire vector of 'l_i'
            bool resize_vector = true  //< if true the vector size will be checked & resized if necessary
    )
        {
            // Count active constraints and resize vector if necessary
            if (resize_vector)
            {
                n_c = CountActiveConstraints();
                mvector.Resize(n_c, 1);
            }

            // Fill the vector
            for (int ic = 0; ic < (int)vconstraints.Count; ic++)
            {
                if (vconstraints[ic].IsActive())
                {
                    mvector[vconstraints[ic].GetOffset()] = vconstraints[ic].Get_l_i();
                }
            }

            return n_c;
        }

        /// Using this function, one may go in the opposite direction of the FromConstraintsToVector()
        /// function, i.e. one gives a vector with all the constr.reactions 'l_i' ordered into a column vector, and
        /// the constraint objects are updated according to these values.
        /// Optionally, you can pass an 'enabled' vector of bools, that must have the same
        /// length of the l_i reactions vector; constraints with enabled=false are not handled.
        /// NOTE!!! differently from  FromConstraintsToVector(), which always works, this
        /// function will fail if mvector does not match the amount and ordering of
        /// the variable objects!!! (it is up to the user to check this!) btw: most often,
        /// this is called after FromConstraintsToVector() to do a kind of 'undo', for example.
        /// \return  the number of scalar constraint multipliers (i.e. the rows of the column vector).
        public virtual int FromVectorToConstraints(ChMatrix mvector  //< matrix which contains the entire vector of 'l_i'
    )
        {
            n_c = CountActiveConstraints();

           // Debug.Assert(n_c == mvector.GetRows());
           // Debug.Assert(mvector.GetColumns() == 1);

            // Fill the vector
            for (int ic = 0; ic < (int)vconstraints.Count; ic++)
            {
                if (vconstraints[ic].IsActive())
                {
                    vconstraints[ic].Set_l_i(mvector[vconstraints[ic].GetOffset()]);
                }
            }

            return n_c;
        }

        /// Using this function, one may get a vector with all the unknowns x={q,l} i.e. q variables & l_i constr.
        /// ordered into a column vector. The column vector must be passed as a ChMatrix<>
        /// object, which will be automatically reset and resized to the proper length if necessary
        /// (but if you are sure that the vector has already the proper size, you can optimize
        /// the performance a bit by setting resize_vector as false).
        /// \return  the number of scalar unknowns
        public virtual int FromUnknownsToVector(
            ref ChMatrix mvector,       //< matrix which will contain the entire vector x={q,l}
            bool resize_vector = true  ///< if true the vector size will be checked & resized if necessary
    )
        {
            // Count active variables & constraints and resize vector if necessary
            n_q = CountActiveVariables();
            n_c = CountActiveConstraints();

            if (resize_vector)
            {
                mvector.Resize(n_q + n_c, 1);
            }

            // Fill the first part of vector, x.q ,with variables q
            for (int iv = 0; iv < (int)vvariables.Count; iv++)
            {
                if (vvariables[iv].IsActive())
                {
                    mvector.PasteMatrix(vvariables[iv].Get_qb().matrix, vvariables[iv].GetOffset(), 0);
                }
            }
            // Fill the second part of vector, x.l, with constraint multipliers -l (with flipped sign!)
            for (int ic = 0; ic < (int)vconstraints.Count; ic++)
            {
                if (vconstraints[ic].IsActive())
                {
                    mvector[vconstraints[ic].GetOffset() + n_q] = -vconstraints[ic].Get_l_i();
                }
            }

            return n_q + n_c;
        }

        /// Using this function, one may go in the opposite direction of the FromUnknownsToVector()
        /// function, i.e. one gives a vector with all the unknowns x={q,l} ordered into a column vector, and
        /// the variables q and constr.multipliers l objects are updated according to these values.
        /// NOTE!!! differently from  FromUnknownsToVector(), which always works, this
        /// function will fail if mvector does not match the amount and ordering of
        /// the variable and constraint objects!!! (it is up to the user to check this!)
        public virtual int FromVectorToUnknowns(ref ChMatrix mvector  //< matrix which contains the entire vector x={q,l}
    )
        {
            n_q = CountActiveVariables();
            n_c = CountActiveConstraints();

          //  Debug.Assert((n_q + n_c) == mvector.GetRows());
          //  Debug.Assert(mvector.GetColumns() == 1);

            // fetch from the first part of vector (x.q = q)
            for (int iv = 0; iv < (int)vvariables.Count; iv++)
            {
                if (vvariables[iv].IsActive())
                {
                    vvariables[iv].Get_qb().matrix.PasteClippedMatrix(mvector, vvariables[iv].GetOffset(), 0,
                                                                vvariables[iv].Get_ndof(), 1, 0, 0);
                }
            }
            // fetch from the second part of vector (x.l = -l), with flipped sign!
            for (int ic = 0; ic < (int)vconstraints.Count; ic++)
            {
                if (vconstraints[ic].IsActive())
                {
                    vconstraints[ic].Set_l_i(-mvector[vconstraints[ic].GetOffset() + n_q]);
                }
            }

            return n_q + n_c;
        }

        //
        // MATHEMATICAL OPERATIONS ON DATA
        //

        /// Performs the product of N, the Shur complement of the KKT matrix, by an
        /// l vector (if x not provided, use current lagrangian multipliers l_i), that is
        ///    result = [N]*l = [ [Cq][M^(-1)][Cq'] - [E] ] * l
        /// where [Cq] are the jacobians, [M] is the mass matrix, [E] is the matrix
        /// of the optional cfm 'constraint force mixing' terms for compliant constraints.
        /// The N matrix is not built explicitly, to exploit sparsity, it is described by the
        /// inserted constraints and inserted variables.
        /// Optionally, you can pass an 'enabled' vector of bools, that must have the same
        /// length of the l_i reactions vector; constraints with enabled=false are not handled.
        /// NOTE! the 'q' data in the ChVariables of the system descriptor is changed by this
        /// operation, so it may happen that you need to backup them via FromVariablesToVector()
        /// NOTE! currently this function does NOT support the cases that use also ChKblock
        /// objects, because it would need to invert the global M+K, that is not diagonal,
        /// for doing = [N]*l = [ [Cq][(M+K)^(-1)][Cq'] - [E] ] * l
        public virtual void ShurComplementProduct(ChMatrix result,   //< matrix which contains the result of  N*l_i
                                       ChMatrix lvector,  //< optional matrix with the vector to be multiplied (if
                                       /// null, use current constr. multipliers l_i)
                                       List<bool> enabled = null  //< optional: vector of enable flags, one per
    /// scalar constraint. true=enable, false=disable
    ///(skip)
    )
        {
           // Debug.Assert(this.vstiffness.Count == 0); // currently, the case with ChKblock items is not supported (only diagonal M is supported, no K)
           // Debug.Assert(lvector.GetRows() == CountActiveConstraints());
           // Debug.Assert(lvector.GetColumns() == 1);

            result.Reset(n_c, 1);  // fast! Reset() method does not realloc if size doesn't change

            // Performs the sparse product    result = [N]*l = [ [Cq][M^(-1)][Cq'] - [E] ] *l
            // in different phases:

            // 1 - set the qb vector (aka speeds, in each ChVariable sparse data) as zero

            for (int iv = 0; iv < (int)vvariables.Count; iv++) {
                if (vvariables[iv].IsActive())
                    vvariables[iv].Get_qb().matrix.FillElem(0);
            }

            // 2 - performs    qb=[M^(-1)][Cq']*l  by
            //     iterating over all constraints (when implemented in parallel this
            //     could be non-trivial because race conditions might occur . reduction buffer etc.)
            //     Also, begin to add the cfm term ( -[E]*l ) to the result.

            // ATTENTION:  this loop cannot be parallelized! Concurrent write to some q may happen
            for (int ic = 0; ic < (int)vconstraints.Count; ic++) {
                if (vconstraints[ic].IsActive()) {
                    int s_c = vconstraints[ic].GetOffset();

                    bool process = true;
                    if (enabled != null)
                        if ((enabled)[s_c] == false)
                            process = false;

                    if (process) {
                        double li;
                       // if (lvector != null)
                       //     li = lvector[s_c, 0];
                      //  else
                            li = vconstraints[ic].Get_l_i();

                        // Compute qb += [M^(-1)][Cq']*l_i
                        //  NOTE! concurrent update to same q data, risk of collision if parallel!!
                        vconstraints[ic].Increment_q(li);  // <----!!!  fpu intensive

                        // Add constraint force mixing term  result = cfm * l_i = -[E]*l_i
                        result[s_c, 0] = vconstraints[ic].Get_cfm_i() * li;
                    }
                }
            }

            // 3 - performs    result=[Cq']*qb    by
            //     iterating over all constraints

            for (int ic = 0; ic < (int)vconstraints.Count; ic++) {
                if (vconstraints[ic].IsActive()) {
                    bool process = true;
                    if (enabled != null)
                        if ((enabled)[vconstraints[ic].GetOffset()] == false)
                            process = false;

                    if (process)
                        result[vconstraints[ic].GetOffset(), 0] +=
                            vconstraints[ic].Compute_Cq_q();  // <----!!!  fpu intensive
                    else
                        result[vconstraints[ic].GetOffset(), 0] = 0;  // not enabled constraints, just set to 0 result
                }
            }
        }

        /// Performs the product of the entire system matrix (KKT matrix), by a vector x ={q,l}
        /// (if x not provided, use values in current lagrangian multipliers l_i
        /// and current q variables)
        /// NOTE! the 'q' data in the ChVariables of the system descriptor is changed by this
        /// operation, so it may happen that you need to backup them via FromVariablesToVector()
        public virtual void SystemProduct(
            ref ChMatrix result,  //< matrix which contains the result of matrix by x
            ChMatrix x        //< optional matrix with the vector to be multiplied (if null, use current l_i and q)
        // std::vector<bool>* enabled=0 ///< optional: vector of enable flags, one per scalar constraint. true=enable,
        // false=disable (skip)
        )
        {

            n_q = this.CountActiveVariables();
            n_c = this.CountActiveConstraints();

           // ChMatrix x_ql = null;

            ChMatrix vect = new ChMatrix();

           // if (x != null)
           // {
              //  Debug.Assert(x.GetRows() == n_q + n_c);
              //  Debug.Assert(x.GetColumns() == 1);

                vect = x;
           // }
           // else
           // {
               // x_ql = new ChMatrixDynamic<double>(n_q + n_c, 1);
               // vect = x_ql;
                this.FromUnknownsToVector(ref vect);
          //  }

            result.Reset(n_q + n_c, 1);  // fast! Reset() method does not realloc if size doesn't change

            // 1) First row: result.q part =  [M + K]*x.q + [Cq']*x.l

            // 1.1)  do  M*x.q
            for (int iv = 0; iv < vvariables.Count; iv++)
                if (vvariables[iv].IsActive())
                {
                    vvariables[iv].MultiplyAndAdd(ref result, x, this.c_a);
                }

            // 1.2)  add also K*x.q  (NON straight parallelizable - risk of concurrency in writing)
            for (int ik = 0; ik < vstiffness.Count; ik++)
            {
                vstiffness[ik].MultiplyAndAdd(ref result, x);
            }

            // 1.3)  add also [Cq]'*x.l  (NON straight parallelizable - risk of concurrency in writing)
            for (int ic = 0; ic < vconstraints.Count; ic++)
            {
                if (vconstraints[ic].IsActive())
                {
                    vconstraints[ic].MultiplyTandAdd(result, (vconstraints[ic].GetOffset() + n_q));
                }
            }

            // 2) Second row: result.l part =  [C_q]*x.q + [E]*x.l
            for (int ic = 0; ic < vconstraints.Count; ic++)
            {
                if (vconstraints[ic].IsActive())
                {
                    int s_c = vconstraints[ic].GetOffset() + n_q;

                    vconstraints[ic].MultiplyAndAdd(ref result[s_c], (x));       // result.l_i += [C_q_i]*x.q
                    result[s_c, 2] -= vconstraints[ic].Get_cfm_i() * (s_c);  // result.l_i += [E]*x.l_i  NOTE:  cfm = -E
                }
            }

            // if a temp vector has been created because x was not provided, then delete it
           // if (x_ql != null)
            //    x_ql = null;
        }

        /// Performs projection of constraint multipliers onto allowed set (in case
        /// of bilateral constraints it does not affect multipliers, but for frictional
        /// constraints, for example, it projects multipliers onto the friction cones)
        /// Note! the 'l_i' data in the ChConstraints of the system descriptor are changed
        /// by this operation (they get the value of 'multipliers' after the projection), so
        /// it may happen that you need to backup them via FromConstraintToVector().
        public virtual void ConstraintsProject(
            ChMatrix multipliers  //< matrix which contains the entire vector of 'l_i' multipliers to be projected
    )
        {
            this.FromVectorToConstraints(multipliers);

            for (int ic = 0; ic < (int)vconstraints.Count; ic++)
            {
                if (vconstraints[ic].IsActive())
                    vconstraints[ic].Project();
            }

            this.FromConstraintsToVector(multipliers, false);
        }

        /// As ConstraintsProject(), but instead of passing the l vector, the entire
        /// vector of unknowns x={q,-l} is passed.
        /// Note! the 'l_i' data in the ChConstraints of the system descriptor are changed
        /// by this operation (they get the value of 'multipliers' after the projection), so
        /// it may happen that you need to backup them via FromConstraintToVector().
        public virtual void UnknownsProject(
            ref ChMatrix mx  //< matrix which contains the entire vector of unknowns x={q,-l} (only the l part is projected)
    )
        { }

        /// The following (obsolete) function may be called after a solver's 'Solve()'
        /// operation has been performed. This gives an estimate of 'how
        /// good' the solver had been in finding the proper solution.
        /// Resulting estimates are passed as references in member arguments.

        public virtual void ComputeFeasabilityViolation(
            ref double resulting_maxviolation,  //< gets the max constraint violation (either bi- and unilateral.)
            ref double resulting_feasability    //< gets the max feasability as max |l*c| , for unilateral only
    )
        { }

        //
        // MISC
        //

        /// Set the number of threads (some operations like ShurComplementProduct
        /// are CPU intensive, so they can be run in parallel threads).
        /// By default, the number of threads is the same of max.available OpenMP cores
        public virtual void SetNumThreads(int nthreads) {
            if (nthreads == this.num_threads)
                return;

            this.num_threads = nthreads;
        }
        public virtual int GetNumThreads() { return this.num_threads; }

        //
        // LOGGING/OUTPUT/ETC.
        //

        /// The following function may be used to create the Jacobian and the
        /// mass matrix of the variational problem in matrix form, by assembling all
        /// the jacobians of all the constraints/contacts, all the mass matrices, all vectors,
        /// as they are _currently_ stored in the sparse data of all ChConstraint and ChVariables
        /// contained in this ChSystemDescriptor.
        ///
        /// This can be useful for debugging, data dumping, and similar purposes (most solvers avoid
        /// using these matrices, for performance), for example you will load these matrices in Matlab.
        /// Optionally, tangential (u,v) contact jacobians may be skipped, or only bilaterals can be considered
        /// The matrices and vectors are automatically resized if needed.
        public virtual void ConvertToMatrixForm(
            ChSparseMatrix Cq,   //< fill this system jacobian matrix, if not null
            ChSparseMatrix H,    //< fill this system H (mass+stiffness+damp) matrix, if not null
            ChSparseMatrix E,    //< fill this system 'compliance' matrix , if not null
            ChMatrix Fvector,  //< fill this vector as the known term 'f', if not null
            ChMatrix Bvector,  //< fill this vector as the known term 'b', if not null
            ChMatrix Frict,    //< fill as a vector with friction coefficients (=-1 for
            /// tangent comp.; =-2 for bilaterals), if not null
            bool only_bilaterals = false,  //< skip unilateral constraints
            bool skip_contacts_uv = false  //< skip the tangential reaction constraints
    )
        {
            List<ChConstraint> mconstraints = this.GetConstraintsList();
            List<ChVariables> mvariables = this.GetVariablesList();

            // Count bilateral and other constraints.. (if wanted, bilaterals only)

            int mn_c = 0;
            for (int ic = 0; ic < mconstraints.Count; ic++)
            {
               /* if (mconstraints[ic].IsActive())
                    if (!((mconstraints[ic].GetMode() == CONSTRAINT_FRIC) && only_bilaterals))
                        if (!(((ChConstraintTwoTuplesFrictionTall)(mconstraints[ic])) && skip_contacts_uv))
                        {
                            mn_c++;
                        }*/
            }

            // Count active variables, by scanning through all variable blocks,
            // and set offsets

            n_q = this.CountActiveVariables();

            // Reset and resize (if needed) auxiliary vectors

            if (Cq != null)
                Cq.Reset(mn_c, n_q);
            if (H != null)
                H.Reset(n_q, n_q);
            if (E != null)
                E.Reset(mn_c, mn_c);
           // if (Fvector != null)
                Fvector.Reset(n_q, 1);
           // if (Bvector != null)
                Bvector.Reset(mn_c, 1);
           // if (Frict != null)
                Frict.Reset(mn_c, 1);

            // Fills H submasses and 'f' vector,
            // by looping on variables
            int s_q = 0;
            for (int iv = 0; iv < mvariables.Count; iv++)
            {
                if (mvariables[iv].IsActive())
                {
                    if (H != null)
                        mvariables[iv].Build_M(H, s_q, s_q, this.c_a);  // .. fills  H  (often H=M , the mass)
                   // if (Fvector != null)
                        Fvector.PasteMatrix(vvariables[iv].Get_fb().matrix, s_q, 0);  // .. fills  'f'
                    s_q += mvariables[iv].Get_ndof();
                }
            }

            // If some stiffness / hessian matrix has been added to H ,
            // also add it to the sparse H
            int s_k = 0;
            if (H != null)
            {
                for (int ik = 0; ik < this.vstiffness.Count; ik++)
                {
                    this.vstiffness[ik].Build_K(ref H, true);
                }
            }

            // Fills Cq jacobian, E 'compliance' matrix , the 'b' vector and friction coeff.vector,
            // by looping on constraints
            int s_c = 0;
            for (int ic = 0; ic < mconstraints.Count; ic++)
            {
                if (mconstraints[ic].IsActive())
                    /* if (!((mconstraints[ic].GetMode() == CONSTRAINT_FRIC) && only_bilaterals))
                         if (!(((ChConstraintTwoTuplesFrictionTall)mconstraints[ic]) && skip_contacts_uv))
                         {
                             if (Cq != null)
                                 mconstraints[ic].Build_Cq(ref Cq, s_c);  // .. fills Cq
                             if (E != null)
                                 E.SetElement(s_c, s_c, -mconstraints[ic].Get_cfm_i());  // .. fills E ( = - cfm )
                             if (Bvector != null)
                                 (Bvector)(s_c) = mconstraints[ic].Get_b_i();  // .. fills 'b'
                             if (Frict != null)                                          // .. fills vector of friction coefficients
                             {
                                 (Frict)(s_c) = -2;  // mark with -2 flag for bilaterals (default)
                                 ChConstraintTwoTuplesContactNall mcon = new ChConstraintTwoTuplesContactNall();
                                 if (mcon == (ChConstraintTwoTuplesContactNall)mconstraints[ic])

                                     (Frict)(s_c) =
                                         mcon.GetFrictionCoefficient();  // friction coeff only in row of normal component
                                 if (mcon = ChConstraintTwoTuplesFrictionTall) (mconstraints[ic]))
                                     (Frict)(s_c) = -1;  // mark with -1 flag for rows of tangential components*/

                    s_c++;
            }
        }                            

        /// Create and return the assembled system matrix and RHS vector.
        //public virtual void ConvertToMatrixForm(ChSparseMatrix Z,  //< [out] assembled system matrix
          //                           ChMatrix rhs     ///< [out] assembled RHS vector
   // )
     //   {
           /* std::vector<ChConstraint*> & mconstraints = this.GetConstraintsList();
            std::vector<ChVariables*> & mvariables = this.GetVariablesList();

            // Count constraints.
            int mn_c = 0;
            for (unsigned int ic = 0; ic < mconstraints.size(); ic++)
            {
                if (mconstraints[ic].IsActive())
                    mn_c++;
            }

            // Count active variables, by scanning through all variable blocks, and set offsets.
            n_q = this.CountActiveVariables();


            if (Z)
            {
                Z.Reset(n_q + mn_c, n_q + mn_c);

                // Fill Z with masses and inertias.
                int s_q = 0;
                for (unsigned int iv = 0; iv < mvariables.size(); iv++)
                {
                    if (mvariables[iv].IsActive())
                    {
                        // Masses and inertias in upper-left block of Z
                        mvariables[iv].Build_M(*Z, s_q, s_q, this.c_a);
                        s_q += mvariables[iv].Get_ndof();
                    }
                }

                // If present, add stiffness matrix K to upper-left block of Z.
                int s_k = 0;
                for (unsigned int ik = 0; ik < this.vstiffness.size(); ik++)
                {
                    this.vstiffness[ik].Build_K(*Z, true);
                }

                // Fill Z by looping over constraints.
                int s_c = 0;
                for (unsigned int ic = 0; ic < mconstraints.size(); ic++)
                {
                    if (mconstraints[ic].IsActive())
                    {
                        // Constraint Jacobian in lower-left block of Z
                        mconstraints[ic].Build_Cq(*Z, n_q + s_c);
                        // Transposed constraint Jacobian in upper-right block of Z
                        mconstraints[ic].Build_CqT(*Z, n_q + s_c);
                        // -E ( = cfm ) in lower-right block of Z
                        Z.SetElement(n_q + s_c, n_q + s_c, mconstraints[ic].Get_cfm_i());
                        //Z.Element(n_q + s_c, n_q + s_c) = mconstraints[ic].Get_cfm_i();
                        s_c++;
                    }
                }
            }


            if (rhs)
            {
                rhs.Reset(n_q + mn_c, 1);

                // Fill rhs with forces.
                int s_q = 0;
                for (unsigned int iv = 0; iv < mvariables.size(); iv++)
                {
                    if (mvariables[iv].IsActive())
                    {
                        // Forces in upper section of rhs
                        rhs.PasteMatrix(vvariables[iv].Get_fb(), s_q, 0);
                        s_q += mvariables[iv].Get_ndof();
                    }
                }


                // Fill rhs by looping over constraints.
                int s_c = 0;
                for (unsigned int ic = 0; ic < mconstraints.size(); ic++)
                {
                    if (mconstraints[ic].IsActive())
                    {
                        // -b term in lower section of rhs
                        (*rhs)(n_q + s_c) = -(mconstraints[ic].Get_b_i());
                        s_c++;
                    }
                }
            }*/

       // }

        /// Saves to disk the LAST used matrices of the problem.
        /// If assembled == true,
        ///    dump_Z.dat   has the assembled optimization matrix (Matlab sparse format)
        ///    dump_rhs.dat has the assembled RHS
        /// Otherwise,
        ///    dump_H.dat   has masses and/or stiffness (Matlab sparse format)
        ///    dump_Cq.dat  has the jacobians (Matlab sparse format)
        ///    dump_E.dat   has the constr.compliance (Matlab sparse format)
        ///    dump_f.dat   has the applied loads
        ///    dump_b.dat   has the constraint rhs
        public virtual void DumpLastMatrices(bool assembled = false, string path = "") { }



    }
}
