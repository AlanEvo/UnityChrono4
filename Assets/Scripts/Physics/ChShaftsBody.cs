using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;


namespace chrono
{
    /// Class for creating a constraint between a 3D ChBody object and a 1D ChShaft object.
    /// A rotation axis must be specified (to tell along which direction the shaft inertia
    /// and rotation affects the body).
    /// This constraint is useful, for example, when you have modeled a 3D car using ChBody
    /// items and a 1D powertrain (gears, differential, etc.) using ChShaft objects: you
    /// can connect the former (at least, the wheels) to the latter using this constraint.
    /// 
    public class ChShaftsBody : ChPhysicsItem
    {

        private double torque_react;                //< reaction torque
        private ChConstraintTwoGeneric constraint = new ChConstraintTwoGeneric();  //< used as an interface to the solver
        private ChShaft shaft = new ChShaft();                     //< connected shaft
        private ChBodyFrame body = new ChBodyFrame();                  //< connected body
        private ChVector shaft_dir = new ChVector(0, 0, 0);               //< shaft direction


        public ChShaftsBody()
        {

        }
        public ChShaftsBody(ChShaftsBody other)
        {
            torque_react = other.torque_react;
            shaft_dir = other.shaft_dir;
            shaft = null;
            body = null;
        }

        /// "Virtual" copy constructor (covariant return type).
        public override ChObj Clone()
        {
            return new ChShaftsBody(this);
        }

        /// Get the number of scalar variables affected by constraints in this link
        public virtual int GetNumCoords() { return 6 + 1; }

        /// Number of scalar constraints
        public override int GetDOC_c() { return 1; }

        // Override/implement interfaces for global state vectors, see ChPhysicsItem for comments.

        public override void IntStateGatherReactions(int off_L, ref ChVectorDynamic<double> L)
        {
            L[off_L] = -torque_react;
        }
        public override void IntStateScatterReactions(int off_L, ChVectorDynamic<double> L)
        {
            torque_react = -L[off_L];
        }
        public override void IntLoadResidual_CqL(int off_L,
                                     ref ChVectorDynamic<double> R,
                                     ChVectorDynamic<double> L,
                                     double c)
        {
            constraint.MultiplyTandAdd(R, L[off_L] * c);
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

            Qc[off_L] += cnstr_violation;
        }
        public override void IntLoadConstraint_Ct(int off, ref ChVectorDynamic<double> Qc, double c)
        {

        }
        public override void IntToDescriptor(int off_v,
                                 ChStateDelta v,
                                 ChVectorDynamic<double> R,
                                 int off_L,
                                 ChVectorDynamic<double> L,
                                 ChVectorDynamic<double> Qc)
        {
            constraint.Set_l_i(L[off_L]);

            constraint.Set_b_i(Qc[off_L]);
        }
        public override void IntFromDescriptor(int off_v,
                                   ref ChStateDelta v,
                                   int off_L,
                                   ref ChVectorDynamic<double> L)
        {
            L[off_L] = constraint.Get_l_i();
        }

        // Override/implement system functions of ChPhysicsItem
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
        public override void ConstraintsLoadJacobians()
        {
            // compute jacobians
            // ChVector<> jacw = body.TransformDirectionParentToLocal(shaft_dir);
            ChVector jacw = shaft_dir;

            constraint.Get_Cq_a().ElementN(0) = -1;

            constraint.Get_Cq_b().ElementN(0) = 0;
            constraint.Get_Cq_b().ElementN(1) = 0;
            constraint.Get_Cq_b().ElementN(2) = 0;
            constraint.Get_Cq_b().ElementN(3) = jacw.x;
            constraint.Get_Cq_b().ElementN(4) = jacw.y;
            constraint.Get_Cq_b().ElementN(5) = jacw.z;
        }
        public override void ConstraintsFetch_react(double factor = 1)
        {
            // From constraints to react vector:
            torque_react = -constraint.Get_l_i() * factor;
        }

        /// Use this function after object creation, to initialize it, given
        /// the 1D shaft and 3D body to join.
        /// Each item must belong to the same ChSystem.
        /// Direction is expressed in the local coordinates of the body.
        public bool Initialize(ChShaft mshaft,  //< shaft to join
                    ChBodyFrame mbody,              //< body to join
                    ChVector mdir  //< the direction of the shaft on 3D body (applied on COG: pure torque)
                    )
        {
            ChShaft mm1 = mshaft;
            ChBodyFrame mm2 = mbody;
            //Debug.Assert(mm1 == null && mm2 == null);

            shaft = mm1;
            body = mm2;
            shaft_dir = ChVector.Vnorm(mdir);

            constraint.SetVariables(mm1.Variables(), mm2.Variables());

            SetSystem(shaft.GetSystem());
            return true;
        }

        /// Get the shaft
        public ChShaft GetShaft() { return shaft; }
        /// Get the body
        public ChBodyFrame GetBody() { return body; }

        /// Set the direction of the shaft respect to 3D body, as a
        /// normalized vector expressed in the coordinates of the body.
        /// The shaft applies only torque, about this axis.
        public void SetShaftDirection(ChVector md) { shaft_dir = ChVector.Vnorm(md); }

        /// Get the direction of the shaft respect to 3D body, as a
        /// normalized vector expressed in the coordinates of the body.
        public ChVector GetShaftDirection() { return shaft_dir; }

        /// Get the reaction torque considered as applied to ChShaft.
        public double GetTorqueReactionOnShaft() { return -(torque_react); }

        /// Get the reaction torque considered as applied to ChBody,
        /// expressed in the coordinates of the body.
        public ChVector GetTorqueReactionOnBody() { return (shaft_dir * torque_react); }

        /// Update all auxiliary data of the gear transmission at given time
        public override void update(double mytime, bool update_assets = true)
        {
            // Inherit time changes of parent class
            base.update(mytime, update_assets);

            // update class data
            // ...
        }

    };


}
