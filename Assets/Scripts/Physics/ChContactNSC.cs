using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;


namespace chrono
{

    /// Class for non-smooth contact between two generic ChContactable objects.
    /// Ta and Tb are of ChContactable sub classes.
    public class ChContactNSC<Ta, Tb> : ChContactTuple<Ta, Tb>
    {
      //  public class typecarr_a : ChContactTuple<Ta, Tb>.typecarr_a { }
      //  public class typecarr_b : ChContactTuple<Ta, Tb>.typecarr_b { }


        protected float[] reactions_cache;  ///< N,U,V reactions which might be stored in a persistent contact manifold

                                          /// The three scalar constraints, to be fed into the system solver.
                                          /// They contain jacobians data and special functions.
        protected ChConstraintTwoTuplesContactN<Ta, Tb> Nx = new ChConstraintTwoTuplesContactN<Ta, Tb>();
        protected ChConstraintTwoTuplesFrictionT<Ta, Tb> Tu = new ChConstraintTwoTuplesFrictionT<Ta, Tb>();
        protected ChConstraintTwoTuplesFrictionT<Ta, Tb> Tv = new ChConstraintTwoTuplesFrictionT<Ta, Tb>();

        protected ChVector react_force = new ChVector(0, 0, 0);

        protected double compliance;
        protected double complianceT;
        protected double restitution;
        protected double dampingf;


        public ChContactNSC()
        {
            Nx.SetTangentialConstraintU(Tu);
            Nx.SetTangentialConstraintV(Tv);
        }

        public ChContactNSC(ChContactContainer mcontainer,          //< contact container
                 Ta mobjA,                               //< collidable object A
                 Tb mobjB,                               //< collidable object B
                 collision.ChCollisionInfo cinfo  //< data for the contact pair
                 )
        : base(mcontainer, mobjA, mobjB, cinfo)
        {
            Nx.SetTangentialConstraintU(Tu);
            Nx.SetTangentialConstraintV(Tv);

            Reset(mobjA, mobjB, cinfo);
        }

        /// Initialize again this constraint.
        public override void Reset(Ta mobjA,                               //< collidable object A
                       Tb mobjB,                               //< collidable object B
                       collision.ChCollisionInfo cinfo  //< data for the contact pair
                       ) //where T1: IntInterface.IBase
        {
            // inherit base class:
            base.Reset(mobjA, mobjB, cinfo);

            ChBody oA = (this.objA as ChBody);
            ChBody oB = (this.objB as ChBody);

            Nx.Get_tuple_a().SetVariables(ref this.objA);
            Nx.Get_tuple_b().SetVariables(ref this.objB);
            Tu.Get_tuple_a().SetVariables(ref this.objA);
            Tu.Get_tuple_b().SetVariables(ref this.objB);
            Tv.Get_tuple_a().SetVariables(ref this.objA);
            Tv.Get_tuple_b().SetVariables(ref this.objB);

            // Calculate composite material properties
             ChMaterialCompositeNSC mat = new ChMaterialCompositeNSC(
                 this.container.GetSystem().composition_strategy,
                 (ChMaterialSurfaceNSC)(oA.GetMaterialSurfaceBase()),
                 (ChMaterialSurfaceNSC)(oB.GetMaterialSurfaceBase()));

            // Check for a user-provided callback to modify the material
            if (this.container.GetAddContactCallback() != null)
            {
                this.container.GetAddContactCallback().OnAddContact(cinfo, mat);
            }

            Nx.Constraint2TuplesNall.SetFrictionCoefficient(mat.static_friction);
            Nx.Constraint2TuplesNall.SetCohesion(mat.cohesion);

            this.restitution = mat.restitution;
            this.dampingf = mat.dampingf;
            this.compliance = mat.compliance;
            this.complianceT = mat.complianceT;

            this.reactions_cache = cinfo.reaction_cache;

            // COMPUTE JACOBIANS

            
            // delegate objA to compute its half of jacobian
            oA.ComputeJacobianForContactPart(this.p1, ref this.contact_plane, ref Nx.Get_tuple_a(), ref Tu.Get_tuple_a(),
                                                   ref Tv.Get_tuple_a(), false);

             // delegate objB to compute its half of jacobian
            oB.ComputeJacobianForContactPart(this.p2, ref this.contact_plane, ref Nx.Get_tuple_b(), ref Tu.Get_tuple_b(),
                                                   ref Tv.Get_tuple_b(), true);

            if (reactions_cache != null)
            {
                react_force.x = reactions_cache[0];
                react_force.y = reactions_cache[1];
                react_force.z = reactions_cache[2];
                //GetLog() << "Reset Fn=" << (double)reactions_cache[0] << "  at cache address:" << (int)this->reactions_cache << "\n";
            }
            else
            {
                react_force = new ChVector(0, 0, 0);
            }
        }

        /// Get the contact force, if computed, in contact coordinate system
        public override ChVector GetContactForce() { return react_force; }

        /// Get the contact friction coefficient
        public virtual double GetFriction() { return Nx.Constraint2TuplesNall.GetFrictionCoefficient(); }

        /// Set the contact friction coefficient
        public virtual void SetFriction(double mf) { Nx.Constraint2TuplesNall.SetFrictionCoefficient(mf); }

        // UPDATING FUNCTIONS

        public override void ContIntStateGatherReactions(int off_L, ref ChVectorDynamic<double> L)
        {

            L.matrix[off_L] = react_force.x;
            L.matrix[off_L + 1] = react_force.y;
            L.matrix[off_L + 2] = react_force.z;
        }

        public override void ContIntStateScatterReactions(int off_L, ChVectorDynamic<double> L)
        {
            react_force.x = L.matrix[off_L];
            react_force.y = L.matrix[off_L + 1];
            react_force.z = L.matrix[off_L + 2];

            if (reactions_cache != null)
            {
                reactions_cache[0] = (float)L.matrix[off_L];      // react_force.x();
                reactions_cache[1] = (float)L.matrix[off_L + 1];  // react_force.y();
                reactions_cache[2] = (float)L.matrix[off_L + 2];  // react_force.z();
            }
        }

        public override void ContIntLoadResidual_CqL(int off_L,
                                         ref ChVectorDynamic<double> R,
                                         ChVectorDynamic<double> L,
                                         double c
                                         )
        {
            this.Nx.MultiplyTandAdd(R.matrix, L.matrix[off_L] * c);
            this.Tu.MultiplyTandAdd(R.matrix, L.matrix[off_L + 1] * c);
            this.Tv.MultiplyTandAdd(R.matrix, L.matrix[off_L + 2] * c);
        }

        public override void ContIntLoadConstraint_C(int off_L,
                                             ref ChVectorDynamic<double> Qc,
                                             double c,
                                             bool do_clamp,
                                             double recovery_clamp
                                             )
        {
            bool bounced = false;

            // Elastic Restitution model (use simple Newton model with coefficient e=v(+)/v(-))
            // Note that this works only if the two connected items are two ChBody.

            if (this.objA != null && this.objB != null)
            {
                var oA = (ChBody)(object)this.objA;
                var oB = (ChBody)(object)this.objB;
                if (this.restitution != 0)
                {
                    // compute normal rebounce speed
                    ChVector V1_w = oA.GetContactPointSpeed(this.p1);
                    ChVector V2_w = oB.GetContactPointSpeed(this.p2);
                    ChVector Vrel_w = V2_w - V1_w;
                    ChVector Vrel_cplane = this.contact_plane.MatrT_x_Vect(Vrel_w);

                    double h = this.container.GetSystem().GetStep();  // = 1.0 / c;  // not all steppers have c = 1/h

                    double neg_rebounce_speed = Vrel_cplane.x * this.restitution;
                    if (neg_rebounce_speed < -this.container.GetSystem().GetMinBounceSpeed())
                        if (this.norm_dist + neg_rebounce_speed * h < 0)
                        {
                            // CASE: BOUNCE
                            bounced = true;
                            Qc.matrix[off_L] += neg_rebounce_speed;
                        }
                }
            }

            if (!bounced)
            {
                // CASE: SETTLE (most often, and also default if two colliding items are not two ChBody)

                if (this.compliance != 0)
                {
                    double h = 1.0 / c;  // was: this->container->GetSystem()->GetStep(); note not all steppers have c = 1/h

                    double alpha = this.dampingf;              // [R]=alpha*[K]
                    double inv_hpa = 1.0 / (h + alpha);         // 1/(h+a)
                    double inv_hhpa = 1.0 / (h * (h + alpha));  // 1/(h*(h+a))

                    //***TODO*** move to KRMmatricesLoad() the following, and only for !bounced case
                    Nx.Set_cfm_i((inv_hhpa) * this.compliance);
                    Tu.Set_cfm_i((inv_hhpa) * this.complianceT);
                    Tv.Set_cfm_i((inv_hhpa) * this.complianceT);

                    double qc = inv_hpa * this.norm_dist;  //***TODO*** see how to move this in KRMmatricesLoad()

                    // Note: clamping of Qc in case of compliance is questionable: it does not limit only the outbound
                    // speed, but also the reaction, so it might allow longer 'sinking' not related to the real compliance.
                    // I.e. If clamping kicks in (when using large timesteps and low compliance), it acts as a numerical damping.
                    if (do_clamp)
                    {
                        qc = ChMaths.ChMax(qc, -recovery_clamp);
                    }

                    Qc.matrix[off_L] += qc;

                } else{
                    if (do_clamp)
                        if (this.Nx.Constraint2TuplesNall.GetCohesion() != 0)
                        {
                            Qc.matrix[off_L] += ChMaths.ChMin(0.0, ChMaths.ChMax(c * this.norm_dist, -recovery_clamp));
                        }else {
                            Qc.matrix[off_L] += ChMaths.ChMax(c * this.norm_dist, -recovery_clamp);
                        }
                    else
                    {
                        Qc.matrix[off_L] += c * this.norm_dist;
                    }
                }
            }
        }

        public override void ContIntToDescriptor(int off_L,
                                     ChVectorDynamic<double> L,
                                     ChVectorDynamic<double> Qc
                                     )
        {
            // only for solver warm start
            Nx.Set_l_i(L.matrix[off_L]);
            Tu.Set_l_i(L.matrix[off_L + 1]);
            Tv.Set_l_i(L.matrix[off_L + 2]);

            // solver known terms
            Nx.Set_b_i(Qc.matrix[off_L]);
            Tu.Set_b_i(Qc.matrix[off_L + 1]);
            Tv.Set_b_i(Qc.matrix[off_L + 2]);
        }

        public override void ContIntFromDescriptor(int off_L,
                                           ref ChVectorDynamic<double> L
                                           )
        {
            L.matrix[off_L] = Nx.Get_l_i();
            L.matrix[off_L + 1] = Tu.Get_l_i();
            L.matrix[off_L + 2] = Tv.Get_l_i();
            
        }

        public override void InjectConstraints(ref ChSystemDescriptor mdescriptor)
        {
            mdescriptor.InsertConstraint(Nx);
            mdescriptor.InsertConstraint(Tu);
            mdescriptor.InsertConstraint(Tv);
        }

        public override void ConstraintsBiReset()
        {
            Nx.Set_b_i(0.0);
            Tu.Set_b_i(0.0);
            Tv.Set_b_i(0.0);
        }


        public override void ConstraintsBiLoad_C(double factor = 1.0, double recovery_clamp = 0.1, bool do_clamp = false)
        {
            bool bounced = false;

            // Elastic Restitution model (use simple Newton model with coefficient e=v(+)/v(-))
            // Note that this works only if the two connected items are two ChBody.

            if (this.objA != null && this.objB != null)
            {
                if (this.restitution != 0)
                {
                   /* var oA = (ChContactable_1vars<Ta>)(object)this.objA;
                    var oB = (ChContactable_1vars<Ta>)(object)this.objB;
                    // compute normal rebounce speed
                    ChVector V1_w = oA.GetContactPointSpeed(this.p1);
                    ChVector V2_w = oB.GetContactPointSpeed(this.p2);
                    ChVector Vrel_w = V2_w - V1_w;
                    ChVector Vrel_cplane = this.contact_plane.MatrT_x_Vect(Vrel_w);

                    double h = 1.0 / factor;  // inverse timestep is factor

                    double neg_rebounce_speed = Vrel_cplane.x * this.restitution;
                    if (neg_rebounce_speed < -this.container.GetSystem().GetMinBounceSpeed())
                        if (this.norm_dist + neg_rebounce_speed * h < 0)
                        {
                            // CASE: BOUNCE
                            bounced = true;
                            Nx.Set_b_i(Nx.Get_b_i() + neg_rebounce_speed);
                        }*/
                }
            }

            if (!bounced)
            {
                // CASE: SETTLE (most often, and also default if two colliding items are not two ChBody)

                if (this.compliance != 0)
                {
                    //  inverse timestep is factor
                   /* double h = 1.0 / factor;

                    double alpha = this.dampingf;              // [R]=alpha*[K]
                    double inv_hpa = 1.0 / (h + alpha);         // 1/(h+a)
                    double inv_hhpa = 1.0 / (h * (h + alpha));  // 1/(h*(h+a))

                    Nx.Set_cfm_i((inv_hhpa) * this.compliance);  // was (inv_hh)* ...   //***TEST DAMPING***/
                   /* Tu.Set_cfm_i((inv_hhpa) * this.complianceT);
                    Tv.Set_cfm_i((inv_hhpa) * this.complianceT);

                    double qc = inv_hpa * this.norm_dist;

                    // If clamping kicks in(when using large timesteps and low compliance), it acts as a numerical damping.
                    if (do_clamp)
                        qc = ChMaths.ChMax(qc, -recovery_clamp); */                   

                }
                else
                {
                   /* if (do_clamp)
                        if (this.Nx.Constraint2TuplesNall.GetCohesion() != 0)
                            Nx.Set_b_i(Nx.Get_b_i() + ChMaths.ChMin(0.0, ChMaths.ChMax(factor * this.norm_dist, -recovery_clamp)));
                        else
                            Nx.Set_b_i(Nx.Get_b_i() + ChMaths.ChMax(factor * this.norm_dist, -recovery_clamp));
                    else
                        Nx.Set_b_i(Nx.Get_b_i() + factor * this.norm_dist);*/
                }
            }
        }

        public override void ConstraintsFetch_react(double factor)
        {
            // From constraints to react vector:
            react_force.x = Nx.Get_l_i() * factor;
            react_force.y = Tu.Get_l_i() * factor;
            react_force.z = Tv.Get_l_i() * factor;
        }
    }
}
