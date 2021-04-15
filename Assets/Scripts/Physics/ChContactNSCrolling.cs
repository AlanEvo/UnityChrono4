using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono
{
    /// Class for non-smooth contact between two generic ChContactable objects.
    /// It inherits ChContactNSC, that has three reaction forces (N,U,V), but also adds
    /// three rolling reaction torques.
    /// This means that it requires about twice the memory required by the ChContactNSC.
    /// Ta and Tb are of ChContactable sub classes.
    public class ChContactNSCrolling<Ta, Tb> : ChContactNSC<Ta, Tb>// where Ta : notnull where Tb : notnull
    {
       // public new class typecarr_a : ChContactTuple<Ta, Tb>.typecarr_a { }
      //  public new class typecarr_b : ChContactTuple<Ta, Tb>.typecarr_b { }

        // The three scalar constraints, to be feed into the
        // system solver. They contain jacobians data and special functions.
        protected ChConstraintTwoTuplesRollingN<Ta, Tb> Rx = new ChConstraintTwoTuplesRollingN<Ta, Tb>();
        protected ChConstraintTwoTuplesRollingT<Ta, Tb> Ru = new ChConstraintTwoTuplesRollingT<Ta, Tb>();
        protected ChConstraintTwoTuplesRollingT<Ta, Tb> Rv = new ChConstraintTwoTuplesRollingT<Ta, Tb>();

        protected ChVector react_torque;

        protected float complianceRoll;
        protected float complianceSpin;

        public ChContactNSCrolling()
        {
            Rx.SetRollingConstraintU(this.Ru);
            Rx.SetRollingConstraintV(this.Rv);
            Rx.SetNormalConstraint(this.Nx);
        }

        public ChContactNSCrolling(ChContactContainer mcontainer,          //< contact container
                        Ta mobjA,                               //< collidable object A
                        Tb mobjB,                               //< collidable object B
                        collision.ChCollisionInfo cinfo  //< data for the contact pair
                        )
        : base(mcontainer, mobjA, mobjB, cinfo)
        {
            Rx.SetRollingConstraintU(this.Ru);
            Rx.SetRollingConstraintV(this.Rv);
            Rx.SetNormalConstraint(this.Nx);

            Reset(mobjA, mobjB, cinfo);
        }

        /// Initialize again this constraint.
        public override void Reset(Ta mobjA,
                            Tb mobjB,
                            collision.ChCollisionInfo cinfo) {
            // Base method call:
            base.Reset(mobjA, mobjB, cinfo);

            ChBody oA = (this.objA as ChBody);
            ChBody oB = (this.objB as ChBody);

            Rx.Get_tuple_a().SetVariables(ref this.objA);
            Rx.Get_tuple_b().SetVariables(ref this.objB);
            Ru.Get_tuple_a().SetVariables(ref this.objA);
            Ru.Get_tuple_b().SetVariables(ref this.objB);
            Rv.Get_tuple_a().SetVariables(ref this.objA);
            Rv.Get_tuple_b().SetVariables(ref this.objB);

            // Calculate composite material properties
              ChMaterialCompositeNSC mat = new ChMaterialCompositeNSC(
                  this.container.GetSystem().composition_strategy,
                  (ChMaterialSurfaceNSC)oA.GetMaterialSurfaceBase(),
                  (ChMaterialSurfaceNSC)oB.GetMaterialSurfaceBase());

              Rx.SetRollingFrictionCoefficient(mat.rolling_friction);
              Rx.SetSpinningFrictionCoefficient(mat.spinning_friction);

              this.complianceRoll = mat.complianceRoll;
              this.complianceSpin = mat.complianceSpin;

              // COMPUTE JACOBIANS

              // delegate objA to compute its half of jacobian
              oA.ComputeJacobianForRollingContactPart(this.p1, ref this.contact_plane, ref Rx.Get_tuple_a(),
                                                               ref Ru.Get_tuple_a(), ref Rv.Get_tuple_a(), false);

              // delegate objB to compute its half of jacobian
              oB.ComputeJacobianForRollingContactPart(this.p2, ref this.contact_plane, ref Rx.Get_tuple_b(),
                                                               ref Ru.Get_tuple_b(), ref Rv.Get_tuple_b(), true);

            this.react_torque = new ChVector(0, 0, 0);
        }

        /// Get the contact force, if computed, in contact coordinate system
        public virtual ChVector GetContactTorque() { return react_torque; }

        /// Get the contact rolling friction coefficient
        public virtual float GetRollingFriction() { return Rx.GetRollingFrictionCoefficient(); }
        /// Set the contact rolling friction coefficient
        public virtual void SetRollingFriction(float mf) { Rx.SetRollingFrictionCoefficient(mf); }

        /// Get the contact spinning friction coefficient
        public virtual float GetSpinningFriction() { return Rx.GetSpinningFrictionCoefficient(); }
        /// Set the contact spinning friction coefficient
        public virtual void SetSpinningFriction(float mf) { Rx.SetSpinningFrictionCoefficient(mf); }

        // UPDATING FUNCTIONS

        public override void ContIntStateGatherReactions(int off_L, ref ChVectorDynamic<double> L) {
            // base behaviour too
            base.ContIntStateGatherReactions(off_L, ref L);

            L[off_L + 3] = react_torque.x;
            L[off_L + 4] = react_torque.y;
            L[off_L + 5] = react_torque.z;
        }

        public override void ContIntStateScatterReactions(int off_L, ChVectorDynamic<double> L)
        {
            // base behaviour too
            base.ContIntStateScatterReactions(off_L, L);

            react_torque.x = L[off_L + 3];
            react_torque.y = L[off_L + 4];
            react_torque.z = L[off_L + 5];
        }

        public override void ContIntLoadResidual_CqL(int off_L,
                                        ref ChVectorDynamic<double> R,
                                            ChVectorDynamic<double> L,
                                            double c
                                         ) {
            // base behaviour too
            base.ContIntLoadResidual_CqL(off_L, ref R, L, c);

            this.Rx.MultiplyTandAdd(R, L[off_L + 3] * c);
            this.Ru.MultiplyTandAdd(R, L[off_L + 4] * c);
            this.Rv.MultiplyTandAdd(R, L[off_L + 5] * c);
        }

        public override void ContIntLoadConstraint_C(int off_L,
                                             ref ChVectorDynamic<double> Qc,
                                             double c,
                                             bool do_clamp,
                                             double recovery_clamp
                                             )
        {
            // base behaviour too
            base.ContIntLoadConstraint_C(off_L, ref Qc, c, do_clamp, recovery_clamp);

            // If rolling and spinning compliance, set the cfm terms
            double h = this.container.GetSystem().GetStep();

            //***TODO*** move to KRMmatricesLoad() the following, and only for !bounced case
            double alpha = this.dampingf;              // [R]=alpha*[K]
            double inv_hpa = 1.0 / (h + alpha);         // 1/(h+a)
            double inv_hhpa = 1.0 / (h * (h + alpha));  // 1/(h*(h+a))

            this.Ru.Set_cfm_i((inv_hhpa) * this.complianceRoll);
            this.Rv.Set_cfm_i((inv_hhpa) * this.complianceRoll);
            this.Rx.Set_cfm_i((inv_hhpa) * this.complianceSpin);
        }

        // virtual void ContIntLoadResidual_F(ChVectorDynamic<>& R, const double c)  {
        // no force to add - this is NSC, not SMC
        //};

        public override void ContIntToDescriptor(int off_L,
                                      ChVectorDynamic<double> L,
                                      ChVectorDynamic<double> Qc) {
            // base behaviour too
            base.ContIntToDescriptor(off_L, L, Qc);

            Rx.Set_l_i(L[off_L + 3]);
            Ru.Set_l_i(L[off_L + 4]);
            Rv.Set_l_i(L[off_L + 5]);

            Rx.Set_b_i(Qc[off_L + 3]);
            Ru.Set_b_i(Qc[off_L + 4]);
            Rv.Set_b_i(Qc[off_L + 5]);
        }

        public override void ContIntFromDescriptor(int off_L,
                                           ref ChVectorDynamic<double> L)
        {
            // base behaviour too
            base.ContIntFromDescriptor(off_L, ref L);

            L[off_L + 3] = Rx.Get_l_i();
            L[off_L + 4] = Ru.Get_l_i();
            L[off_L + 5] = Rv.Get_l_i();
        }

        public override void InjectConstraints(ref ChSystemDescriptor mdescriptor)
        {
            // base behaviour too
            base.InjectConstraints(ref mdescriptor);

            mdescriptor.InsertConstraint(Rx);
            mdescriptor.InsertConstraint(Ru);
            mdescriptor.InsertConstraint(Rv);
        }

        public override void ConstraintsBiReset()
        {
            // base behaviour too
            base.ConstraintsBiReset();

            Rx.Set_b_i(0.0);
            Ru.Set_b_i(0.0);
            Rv.Set_b_i(0.0);
        }

        public override void ConstraintsBiLoad_C(double factor = 1.0, double recovery_clamp = 0.1, bool do_clamp = false)
        {
            // base behaviour too
            base.ConstraintsBiLoad_C(factor, recovery_clamp, do_clamp);

            // If rolling and spinning compliance, set the cfm terms
            double h = this.container.GetSystem().GetStep();

            //***TODO*** move to KRMmatricesLoad() the following, and only for !bounced case
            double alpha = this.dampingf;              // [R]=alpha*[K]
            double inv_hpa = 1.0 / (h + alpha);         // 1/(h+a)
            double inv_hhpa = 1.0 / (h * (h + alpha));  // 1/(h*(h+a))

            this.Ru.Set_cfm_i((inv_hhpa) * this.complianceRoll);
            this.Rv.Set_cfm_i((inv_hhpa) * this.complianceRoll);
            this.Rx.Set_cfm_i((inv_hhpa) * this.complianceSpin);

            // Assume no residual ever, do not load in C
        }

        public override void ConstraintsFetch_react(double factor)
        {
            // base behaviour too
            base.ConstraintsFetch_react(factor);

            // From constraints to react torque:
            react_torque.x = Rx.Get_l_i() * factor;
            react_torque.y = Ru.Get_l_i() * factor;
            react_torque.z = Rv.Get_l_i() * factor;
        }

    }

}
