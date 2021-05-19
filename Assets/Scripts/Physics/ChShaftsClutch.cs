using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;


namespace chrono
{

    /// Class for defining a clutch or a brake (1D model) between two one-degree-of-freedom
    /// parts; i.e., shafts that can be used to build 1D models of powertrains.

    public class ChShaftsClutch : ChShaftsCouple
    {


        public double maxT;                             //< clutch max transmissible torque (for forward direction
        public double minT;                             //< clutch min transmissible torque (for backward direction)
        public double modulation;                       //< 0...1  (default 1).
        public double torque_react;                     //< reaction torque
        public ChConstraintTwoGenericBoxed constraint = new ChConstraintTwoGenericBoxed();  //< used as an interface to the solver


        public ChShaftsClutch()
        {
            maxT = 1;
            minT = -1;
            modulation = 1;
            torque_react = 0;
        }

        public ChShaftsClutch(ChShaftsClutch other) : base(other)
        {
            maxT = other.maxT;
            minT = other.minT;
            modulation = other.modulation;
            torque_react = other.torque_react;
        }

        // Use this for initialization
        public void Start()
        {
            SetTorqueLimit(maxT);
            Initialize(shaft1, shaft2);
            SetModulation(0);
            ChSystem.system.Add(this);
        }

        /// "Virtual" copy constructor (covariant return type).
        public override ChObj Clone()
        {
            return new ChShaftsClutch(this);
        }

        /// Number of scalar constraints, for statistical reasons
        public override int GetDOC_c() { return 1; }

        // (override/implement interfaces for global state vectors, see ChPhysicsItem for comments.)
        public override void IntStateGatherReactions(int off_L, ref ChVectorDynamic<double> L)
        {
            L.matrix[off_L] = torque_react;
        }

        public override void IntStateScatterReactions(int off_L, ChVectorDynamic<double> L)
        {
            torque_react = L.matrix[off_L];
        }

        public override void IntLoadResidual_CqL(int off_L,
                                         ref ChVectorDynamic<double> R,
                                         ChVectorDynamic<double> L,
                                         double c)
        {
            constraint.MultiplyTandAdd(R.matrix, L.matrix[off_L] * c);
        }

        public override void IntLoadConstraint_C(int off_L,
                                         ref ChVectorDynamic<double> Qc,
                                         double c,
                                         bool do_clamp,
                                         double recovery_clamp)
        {
            double res = 0;  // no residual anyway! allow drifting...

            double cnstr_violation = c * res;

            if (do_clamp)
            {
                cnstr_violation = ChMaths.ChMin(ChMaths.ChMax(cnstr_violation, -recovery_clamp), recovery_clamp);
            }

            Qc.matrix[off_L] += cnstr_violation;
        }

        public override void IntLoadConstraint_Ct(int off, ref ChVectorDynamic<double> Qc, double c) { }

        public override void IntLoadResidual_F(int off, ref ChVectorDynamic<double> R, double c)
        {
            // Might not be the best place to put this, but it works.
            // Update the limits on lagrangian multipliers:
            double dt = c;  // note: not always c=dt, this is true for euler implicit linearized and similar DVI timesteppers,
                            // might be not the case in future
                            // double dt = system->GetStep(); // this could be another option.. but with variable-dt timesteppers it
                            // should go deeper..
            constraint.SetBoxedMinMax(dt * minT * modulation, dt * maxT * modulation);
        }
        public override void IntToDescriptor(int off_v,
                                     ChStateDelta v,
                                     ChVectorDynamic<double> R,
                                     int off_L,
                                     ChVectorDynamic<double> L,
                                     ChVectorDynamic<double> Qc)
        {
            constraint.Set_l_i(L.matrix[off_L]);

            constraint.Set_b_i(Qc.matrix[off_L]);
        }

        public override void IntFromDescriptor(int off_v,
                                       ref ChStateDelta v,
                                       int off_L,
                                       ref ChVectorDynamic<double> L)
        {
            L.matrix[off_L] = constraint.Get_l_i();
        }

        // Override/implement system functions of ChShaftsCouple
        // (to assemble/manage data for system solver)

        public override void InjectConstraints(ref ChSystemDescriptor mdescriptor)
        {
            // if (!IsActive())
            //	return;

            mdescriptor.InsertConstraint(constraint);
        }

        public override void ConstraintsBiReset()
        {
            constraint.Set_b_i(0.0);
        }
        public override void ConstraintsBiLoad_C(double factor = 1, double recovery_clamp = 0.1, bool do_clamp = false)
        {
            // if (!IsActive())
            //	return;

            double res = 0;  // no residual

            constraint.Set_b_i(constraint.Get_b_i() + factor * res);
        }
        public override void ConstraintsBiLoad_Ct(double factor = 1)
        {
            // if (!IsActive())
            //	return;

            // nothing
        }
        public override void ConstraintsFbLoadForces(double factor = 1)
        {
            // no forces

            // compute jacobians
            double m_dt = factor;

            constraint.SetBoxedMinMax(m_dt * minT * modulation, m_dt * maxT * modulation);
        }
        public override void ConstraintsLoadJacobians()
        {
            constraint.Get_Cq_a().SetElement(0, 0, 1.0);
            constraint.Get_Cq_b().SetElement(0, 0, -1.0);
        }
        public override void ConstraintsFetch_react(double factor = 1)
        {
            // From constraints to react vector:
            torque_react = constraint.Get_l_i() * factor;
        }

        // Other functions

        /// Use this function after gear creation, to initialize it, given
        /// two shafts to join.
        /// Each shaft must belong to the same ChSystem.
        public override bool Initialize(ChShaft mshaft1,  //< first  shaft to join
                        ChShaft mshaft2  //< second shaft to join
                        )
        {
            // parent class initialization
            if (!base.Initialize(mshaft1, mshaft2))
                return false;

            ChShaft mm1 = mshaft1;
            ChShaft mm2 = mshaft2;

            constraint.SetVariables(mm1.Variables(), mm2.Variables());

            SetSystem(shaft1.GetSystem());

            return true;
        }

        /// Set the transmissible torque limit (the maximum torque that
        /// the clutch can transmit between the two shafts).
        /// You can specify two values for backward/forward directions: usually
        /// these are equal (ex. -100,100) in most commercial clutches, but
        /// if you define (0,100), for instance, you can create a so called
        /// freewheel or overrunning clutch that works only in one direction.
        public void SetTorqueLimit(double ml, double mu)
        {
            minT = ml;
            maxT = mu;
        }
        /// Set the transmissible torque limit (the maximum torque that
        /// the clutch can transmit between the two shafts), for both
        /// forward and backward direction.
        public void SetTorqueLimit(double ml) { SetTorqueLimit(-Math.Abs(ml), Math.Abs(ml)); }

        /// Get the torque limit for forward rotation
        public double GetTorqueLimitF() { return maxT; }
        /// Get the torque limit for backward rotation
        public double GetTorqueLimitB() { return minT; }
        /// Get the torque limit (when this is a clutch with symmetric forw/backw limits)
        public double GetTorqueLimit() { return maxT; }

        /// Set the user modulation of the torque (or brake, if you use it between
        /// a fixed shaft and a free shaft). The modulation must range from
        /// 0 (switched off) to 1 (max torque). Default is 1, when clutch is created.
        /// You can update this during integration loop to simulate the pedal pushing by the driver.
        public void SetModulation(double mm) { modulation = ChMaths.ChMax(ChMaths.ChMin(mm, 1.0), 0.0); }
        /// Get the the user modulation.
        public double GetModulation() { return modulation; }

        /// Get the actual angle slippage of the clutch, in terms of phase of shaft 1 respect to 2.
        public double GetSlippage() { return GetRelativeRotation(); }
        /// Get the actual slippage speed of the clutch, in terms of speed of shaft 1 respect to 2.
        public double GetSlippage_dt() { return GetRelativeRotation_dt(); }
        /// Get the actual slippage acceleration of the clutch, in terms of accel. of shaft 1 respect to 2.
        public double GetSlippage_dtdt() { return GetRelativeRotation_dtdt(); }

        /// Get the reaction torque exchanged between the two shafts,
        /// considered as applied to the 1st axis.
        public override double GetTorqueReactionOn1() { return torque_react; }

        /// Get the reaction torque exchanged between the two shafts,
        /// considered as applied to the 2nd axis.
        public override double GetTorqueReactionOn2() { return -torque_react; }

        /// Update all auxiliary data of the gear transmission at given time
        public override void update(double mytime, bool update_assets = true)
        {
            // Inherit time changes of parent class
            base.update(mytime, update_assets);

            // update class data
            // ...
        }
    }
}
