using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono
{
	/// Base class for items that can contain objects of ChVariables or ChConstraints,
	/// such as rigid bodies, mechanical joints, etc.
	public class ChPhysicsItem : ChObj
	{
		public static ChSystem system;  //< parent system
       // public ChSystem GetSystem() { return system; }

		//protected List<ChAsset> assets;  //< set of assets

		protected int offset_x;  //< offset in vector of state (position part)
		protected int offset_w;  //< offset in vector of state (speed part)
		protected int offset_L;  //< offset in vector of lagrangian multipliers


        public virtual void SetupInitial() { }

        public ChPhysicsItem() {
			system = null;
			offset_x = 0;
			offset_w = 0;
			offset_L = 0; 
		}

		public ChPhysicsItem(ChPhysicsItem other) { 
		
		}

		/// "Virtual" copy constructor (covariant return type).
		public override ChObj Clone() { return new ChPhysicsItem(this); }


		/// Get the pointer to the parent ChSystem()
		public ChSystem GetSystem() { return system; }

		/// Set the pointer to the parent ChSystem() and
		/// also add to new collision system / remove from old coll.system
		public virtual void SetSystem(ChSystem m_system) {
            if (system == m_system)  // shortcut if no change
                return;
            if (system)
            {
                if (GetCollide()) { }
                    RemoveCollisionModelsFromSystem();
            }
            system = m_system;  // set here
            bool nurse = GetCollide();
            if (system)
            {
                if (GetCollide()) { }
                    AddCollisionModelsToSystem();
            }
        }

        public virtual void SyncCollisionModels() { }

        //                   --- INTERFACES ---
        // inherited classes might/should implement some of the following functions.

        // Collisions - override these in child classes if needed

        /// Tell if the object is subject to collision.
        /// Only for interface; child classes may override this, using internal flags.
        public virtual bool GetCollide() { return false; }

        /// Counts the number of bodies, links, and meshes.
        /// Computes the offsets of object states in the global state.
        /// Assumes that this->offset_x this->offset_w this->offset_L are already set
        /// as starting point for offsetting all the contained sub objects.
        public virtual void Setup() { }

        /// If this physical item contains one or more collision models,
        /// add them to the system's collision engine.
        public virtual void AddCollisionModelsToSystem() { }

        /// If this physical item contains one or more collision models,
        /// remove them from the system's collision engine.
        public virtual void RemoveCollisionModelsFromSystem() { }

        /// Updates all the auxiliary data and children of
        /// bodies, forces, links, given their current state.
        public virtual void update(double mytime, bool update_assets = true) {
            ChTime = mytime;

            // Don't need this because Unity draws it's own visual objects
            if (update_assets)
            {
              //  for (int ia = 0; ia < assets.Count; ++ia)
                //    assets[ia].Update(this, GetAssetsFrame().GetCoord());
            }
        }

        /// Updates all the auxiliary data and children of
        /// bodies, forces, links, given their current state.
        public virtual void update(bool update_assets = true) { update(ChTime, update_assets); }

        /// Set zero speed (and zero accelerations) in state, without changing the position.
        /// Child classes should implement this function if GetDOF() > 0.
        /// It is used by owner ChSystem for some static analysis.
        public virtual void SetNoSpeedNoAcceleration() { }

        // STATE FUNCTIONS
        //
        // These functions are used for bookkeeping in ChSystem, so that states (position, speeds)
        // of multiple physics items can be mapped in a single system state vector.
        // These will be used to interface to time integrators.
        // Note: these are not 'pure virtual' interfaces to avoid the burden of implementing all them
        // when just few are needed, so here is a default fallback that represent a 0 DOF, 0 DOC item, but
        // the children classes should override them.

        /// Get the number of scalar coordinates (variables), if any, in this item.
        /// Children classes must override this.
        public virtual int GetDOF() { return 0; }
        /// Get the number of scalar coordinates of variables derivatives (usually = DOF, but might be
        /// different than DOF, ex. DOF=4 for quaternions, but DOF_w = 3 for its Lie algebra, ex angular velocity)
        /// Children classes might override this.
        public virtual int GetDOF_w() { return GetDOF(); }
        /// Get the number of scalar constraints, if any, in this item
        public virtual int GetDOC() { return GetDOC_c() + GetDOC_d(); }
        /// Get the number of scalar constraints, if any, in this item (only bilateral constr.)
        /// Children classes might override this.
        public virtual int GetDOC_c() { return 0; }
        /// Get the number of scalar constraints, if any, in this item (only unilateral constr.)
        /// Children classes might override this.
        public virtual int GetDOC_d() { return 0; }

        /// Get offset in the state vector (position part)
        public int GetOffset_x() { return offset_x; }
        /// Get offset in the state vector (speed part)
        public int GetOffset_w() { return offset_w; }
        /// Get offset in the lagrangian multipliers
        public int GetOffset_L() { return offset_L; }

        /// Set offset in the state vector (position part)
        /// Note: only the ChSystem::Setup function should use this
        public void SetOffset_x(int moff) { offset_x = moff; }
        /// Set offset in the state vector (speed part)
        /// Note: only the ChSystem::Setup function should use this
        public void SetOffset_w(int moff) { offset_w = moff; }
        /// Set offset in the lagrangian multipliers
        /// Note: only the ChSystem::Setup function should use this
        public void SetOffset_L(int moff) { offset_L = moff; }

        /// From item's state to global state vectors y={x,v}
        /// pasting the states at the specified offsets.
        public virtual void IntStateGather(int off_x,  //< offset in x state vector
                                ref ChState x,                //< state vector, position part
                                int off_v,  ///< offset in v state vector
                                ref ChStateDelta v,           ///< state vector, speed part
                                ref double T                  ///< time
    )
    { }

        /// From global state vectors y={x,v} to  item's state (and update)
        /// fetching the states at the specified offsets.
        public virtual void IntStateScatter(int off_x,  //< offset in x state vector
                                 ChState x,          ///< state vector, position part
                                 int off_v,  //< offset in v state vector
                                 ChStateDelta v,     //< state vector, speed part
                                 double T             ///< time
    )
    {
        // Default behavior: even if no state is used, at least call Update()
        //update(T);
    }

        /// From item's state acceleration to global acceleration vector
        public virtual void IntStateGatherAcceleration(int off_a,  //< offset in a accel. vector
                                            ref ChStateDelta a            //< acceleration part of state vector derivative
    )
    { }

        /// From global acceleration vector to item's state acceleration
        public virtual void IntStateScatterAcceleration(int off_a,  //< offset in a accel. vector
                                             ChStateDelta a  //< acceleration part of state vector derivative
    )
    { }

        /// From item's reaction forces to global reaction vector
        public virtual void IntStateGatherReactions(int off_L,  //< offset in L vector
                                         ref ChVectorDynamic<double> L       //< L vector of reaction forces
    )
    { }

        /// From global reaction vector to item's reaction forces
        public virtual void IntStateScatterReactions(int off_L,   //< offset in L vector
                                          ChVectorDynamic<double> L  //< L vector of reaction forces
    )
    { }

        /// Computes x_new = x + Dt , using vectors at specified offsets.
        /// By default, when DOF = DOF_w, it does just the sum, but in some cases (ex when using quaternions
        /// for rotations) it could do more complex stuff, and children classes might overload it.
        public virtual void IntStateIncrement(int off_x,  //< offset in x state vector
                                   ref ChState x_new,            //< state vector, position part, incremented result
                                   ChState x,          //< state vector, initial position part
                                   int off_v,  //< offset in v state vector
                                   ChStateDelta Dv     //< state vector, increment
    )
    {
       /* for (int i = 0; i < GetDOF(); ++i){
                x_new[off_x + i] = x[off_x + i] + Dv[off_v + i];
        }*/
    }

        /// Takes the F force term, scale and adds to R at given offset:
        ///    R += c*F
        public virtual void IntLoadResidual_F(int off,  //< offset in R residual
                                   ref ChVectorDynamic<double> R,    //< result: the R residual, R += c*F
                                   double c           //< a scaling factor
    )
    {

        }

        /// Takes the M*v  term,  multiplying mass by a vector, scale and adds to R at given offset:
        ///    R += c*M*w
        public virtual void IntLoadResidual_Mv(int off,      //< offset in R residual
                                    ref ChVectorDynamic<double> R,        //< result: the R residual, R += c*M*v
                                    ChVectorDynamic<double> w,  //< the w vector
                                    double c               //< a scaling factor
    )
    { }

        /// Takes the term Cq'*L, scale and adds to R at given offset:
        ///    R += c*Cq'*L
        public virtual void IntLoadResidual_CqL(int off_L,    //< offset in L multipliers
                                     ref ChVectorDynamic<double> R,        //< result: the R residual, R += c*Cq'*L
                                     ChVectorDynamic<double> L,  //< the L vector
                                     double c               //< a scaling factor
    )
    { }

        /// Takes the term C, scale and adds to Qc at given offset:
        ///    Qc += c*C
        public virtual void IntLoadConstraint_C(int off,  //< offset in Qc residual
                                     ref ChVectorDynamic<double> Qc,   //< result: the Qc residual, Qc += c*C
                                     double c,          //< a scaling factor
                                     bool do_clamp,           //< apply clamping to c*C?
                                     double recovery_clamp    //< value for min/max clamping of c*C
    )
        { }

        /// Takes the term Ct, scale and adds to Qc at given offset:
        ///    Qc += c*Ct
        public virtual void IntLoadConstraint_Ct(int off,  //< offset in Qc residual
                                      ref ChVectorDynamic<double> Qc,   //< result: the Qc residual, Qc += c*Ct
                                      double c           //< a scaling factor
    )
        { }

        /// Prepare variables and constraints to accommodate a solution:
        public virtual void IntToDescriptor(
            int off_v,                     //< offset for \e v and \e R
            ChStateDelta v,                 //< vector that will be copied into the \e q 'unknowns' term of the variables (for warm starting)
            ChVectorDynamic<double> R,            //< vector that will be copied into the \e F 'force' term of the variables
            int off_L,                     //< offset for \e L and \e Qc
            ChVectorDynamic<double> L,      //< vector that will be copied into the \e L 'lagrangian ' term of the constraints (for warm starting)
            ChVectorDynamic<double> Qc      //< vector that will be copied into the \e Qb 'constraint' term of the constraints
    )
        { }

        /// After a solver solution, fetch values from variables and constraints into vectors:
        public virtual void IntFromDescriptor(
        int off_v,  //< offset for \e v
        ref ChStateDelta v,           //< vector to where the \e q 'unknowns' term of the variables will be copied
        int off_L,  //< offset for \e L
        ref ChVectorDynamic<double> L       //< vector to where \e L 'lagrangian ' term of the constraints will be copied
    )
    { }
        // SOLVER SYSTEM FUNCTIONS
        //
        // These are the functions that are used to manage ChConstraint and/or ChVariable
        // objects that are sent to the system solver.
        // The children classes, inherited from ChPhysicsItem, can implement them (by default,
        // the base ChPhysicsItem does not introduce any variable nor any constraint).

        /// Sets the 'fb' part (the known term) of the encapsulated ChVariables to zero.
        public virtual void VariablesFbReset() { }

        /// Adds the current forces (applied to item) into the
        /// encapsulated ChVariables, in the 'fb' part: qf+=forces*factor
        public virtual void VariablesFbLoadForces(double factor = 1) { }

        /// Initialize the 'qb' part of the ChVariables with the
        /// current value of speeds. Note: since 'qb' is the unknown, this
        /// function seems unnecessary, unless used before VariablesFbIncrementMq()
        public virtual void VariablesQbLoadSpeed() { }

        /// Adds M*q (masses multiplied current 'qb') to Fb, ex. if qb is initialized
        /// with v_old using VariablesQbLoadSpeed, this method can be used in
        /// timestepping schemes that do: M*v_new = M*v_old + forces*dt
        public virtual void VariablesFbIncrementMq() { }

        /// Fetches the item speed (ex. linear and angular vel.in rigid bodies) from the
        /// 'qb' part of the ChVariables and sets it as the current item speed.
        /// If 'step' is not 0, also should compute the approximate acceleration of
        /// the item using backward differences, that is  accel=(new_speed-old_speed)/step.
        /// Mostly used after the solver provided the solution in ChVariables.
        public virtual void VariablesQbSetSpeed(double step = 0) { }

        /// Increment item positions by the 'qb' part of the ChVariables,
        /// multiplied by a 'step' factor.
        ///     pos+=qb*step
        /// If qb is a speed, this behaves like a single step of 1-st order
        /// numerical integration (Eulero integration).
        public virtual void VariablesQbIncrementPosition(double step) { }

        /// Tell to a system descriptor that there are variables of type
        /// ChVariables in this object (for further passing it to a solver)
        /// Basically does nothing, but maybe that inherited classes may specialize this.
        public virtual void InjectVariables(ref ChSystemDescriptor mdescriptor) { }

        /// Tell to a system descriptor that there are constraints of type
        /// ChConstraint in this object (for further passing it to a solver)
        /// Basically does nothing, but maybe that inherited classes may specialize this.
        public virtual void InjectConstraints(ref ChSystemDescriptor mdescriptor) { }

        /// Sets to zero the known term (b_i) of encapsulated ChConstraints
        public virtual void ConstraintsBiReset() { }

        /// Adds the current C (constraint violation) to the known term (b_i) of
        /// encapsulated ChConstraints
        public virtual void ConstraintsBiLoad_C(double factor = 1, double recovery_clamp = 0.1, bool do_clamp = false) { }

        /// Adds the current Ct (partial t-derivative, as in C_dt=0-> [Cq]*q_dt=-Ct)
        /// to the known term (b_i) of encapsulated ChConstraints
        public virtual void ConstraintsBiLoad_Ct(double factor = 1) { }

        /// Adds the current Qc (the vector of C_dtdt=0 -> [Cq]*q_dtdt=Qc )
        /// to the known term (b_i) of encapsulated ChConstraints
        public virtual void ConstraintsBiLoad_Qc(double factor = 1) { }

        /// Adds the current link-forces, if any, (caused by springs, etc.) to the 'fb' vectors
        /// of the ChVariables referenced by encapsulated ChConstraints
        public virtual void ConstraintsFbLoadForces(double factor = 1) { }

        /// Adds the current jacobians in encapsulated ChConstraints
        public virtual void ConstraintsLoadJacobians() { }

        /// Fetches the reactions from the lagrangian multiplier (l_i)
        /// of encapsulated ChConstraints.
        /// Mostly used after the solver provided the solution in ChConstraints.
        /// Also, should convert the reactions obtained from dynamical simulation,
        /// from link space to intuitive react_force and react_torque.
        public virtual void ConstraintsFetch_react(double factor = 1) { }

        /// Tell to a system descriptor that there are items of type
        /// ChKblock in this object (for further passing it to a solver)
        /// Basically does nothing, but maybe that inherited classes may specialize this.
        public virtual void InjectKRMmatrices(ref ChSystemDescriptor mdescriptor) { }

        /// Adds the current stiffness K and damping R and mass M matrices in encapsulated
        /// ChKblock item(s), if any. The K, R, M matrices are added with scaling
        /// values Kfactor, Rfactor, Mfactor.
        /// NOTE: signs are flipped respect to the ChTimestepper dF/dx terms:  K = -dF/dq, R = -dF/dv
        public virtual void KRMmatricesLoad(double Kfactor, double Rfactor, double Mfactor) { }



	}
}
