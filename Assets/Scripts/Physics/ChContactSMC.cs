using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;


namespace chrono
{

    /// Class for smooth (penalty-based) contact between two generic contactable objects.
    /// Ta and Tb are of ChContactable sub classes.
    public class ChContactSMC<Ta, Tb> : ChContactTuple<Ta, Tb>
    {


        public class ChContactJacobian
        {
            public ChKblockGeneric m_KRM = new ChKblockGeneric();        //< sum of scaled K and R, with pointers to sparse variables
            public ChMatrixDynamic<double> m_K = new ChMatrixDynamic<double>();  //< K = dQ/dx
            public ChMatrixDynamic<double> m_R = new ChMatrixDynamic<double>();  //< R = dQ/dv
        };

        private ChVector m_force = new ChVector(0, 0, 0);        //< contact force on objB
        private ChContactJacobian m_Jac = new ChContactJacobian();                                  //< contact Jacobian data

        public ChContactSMC()
        {
           // m_Jac = null;
        }

        public ChContactSMC(ChContactContainer mcontainer,          //< contact container
                Ta mobjA,                               //< collidable object A
                Tb mobjB,                               //< collidable object B
                collision.ChCollisionInfo cinfo  //< data for the contact pair
                )
       : base(mcontainer, mobjA, mobjB, cinfo)
        {
           // m_Jac = null;
            Reset(mobjA, mobjB, cinfo);
        }

        /// Get the contact force, if computed, in contact coordinate system
        public override ChVector GetContactForce() { return this.contact_plane.MatrT_x_Vect(m_force); }

        /// Get the contact penetration (positive if there is overlap).
        public double GetContactPenetration() { return -this.norm_dist; }

        /// Get the contact force, expressed in the frame of the contact.
        public ChVector GetContactForceAbs() { return m_force; }

        /// Access the proxy to the Jacobian.
        public ChKblockGeneric GetJacobianKRM() { return m_Jac.m_KRM;/* ? (m_Jac.m_KRM) : null;*/ }
        public ChMatrixDynamic<double> GetJacobianK() { return m_Jac.m_K;/* ? (m_Jac.m_K) : null; */}
        public ChMatrixDynamic<double> GetJacobianR() { return m_Jac.m_R; /*? (m_Jac.m_R) : null; */}

        /// Initialize again this constraint.
        public override void Reset(Ta mobjA,                               //< collidable object A
                       Tb mobjB,                               //< collidable object B
                       collision.ChCollisionInfo cinfo  //< data for the contact pair
                       ) //where T1: IntInterface.IBase
        {
            // Inherit base class.
            base.Reset(mobjA, mobjB, cinfo);

            // Note: cinfo.distance is the same as this.norm_dist.
           // Debug.Assert(cinfo.distance < 0);

            ChBody oA = (this.objA as ChBody);
            ChBody oB = (this.objB as ChBody);

            // Calculate composite material properties
            ChMaterialCompositeSMC mat = new ChMaterialCompositeSMC(
                this.container.GetSystem().composition_strategy,
                (ChMaterialSurfaceSMC)(oA.GetMaterialSurfaceBase()),
            (ChMaterialSurfaceSMC)(oB.GetMaterialSurfaceBase()));

            // Check for a user-provided callback to modify the material
            if (this.container.GetAddContactCallback() != null)
            {
                this.container.GetAddContactCallback().OnAddContact(cinfo, mat);
            }

            // Calculate contact force.
            m_force = CalculateForce(-this.norm_dist,                            // overlap (here, always positive)
                                     this.normal,                                // normal contact direction
                                     oA.GetContactPointSpeed(this.p1),  // velocity of contact point on objA
                                     oB.GetContactPointSpeed(this.p2),  // velocity of contact point on objB
                                     mat                                          // composite material for contact pair
            );

            ChSystemSMC smc = (ChSystemSMC)this.container.GetSystem();

            // Set up and compute Jacobian matrices.
            if (smc.GetStiffContact() == true)
            {
                CreateJacobians();
                CalculateJacobians(mat);
            }
        }

        /// Calculate contact force, expressed in absolute coordinates.
        public ChVector CalculateForce(
            double delta,                      //< overlap in normal direction
        ChVector normal_dir,      //< normal contact direction (expressed in global frame)
        ChVector vel1,            //< velocity of contact point on objA (expressed in global frame)
        ChVector vel2,            //< velocity of contact point on objB (expressed in global frame)
        ChMaterialCompositeSMC mat  //< composite material for contact pair
    )
        {
            // Set contact force to zero if no penetration.
            if (delta <= 0)
            {
                return new ChVector(0, 0, 0);
            }

            // Extract parameters from containing system
            ChSystemSMC sys = (ChSystemSMC)(this.container.GetSystem());
            double dT = sys.GetStep();
            bool use_mat_props = sys.UsingMaterialProperties();
            ChSystemSMC.ContactForceModel contact_model = sys.GetContactForceModel();
            ChSystemSMC.AdhesionForceModel adhesion_model = sys.GetAdhesionForceModel();
            ChSystemSMC.TangentialDisplacementModel tdispl_model = sys.GetTangentialDisplacementModel();

            // Relative velocity at contact
            ChVector relvel = vel2 - vel1;
            double relvel_n_mag = relvel.Dot(normal_dir);
            ChVector relvel_n = relvel_n_mag * normal_dir;
            ChVector relvel_t = relvel - relvel_n;
            double relvel_t_mag = relvel_t.Length();

            ChBody oA = (this.objA as ChBody);
            ChBody oB = (this.objB as ChBody);

            // Calculate effective mass
            double eff_mass = oA.GetContactableMass() * oB.GetContactableMass() /
                              (oA.GetContactableMass() + oB.GetContactableMass());

            // Calculate stiffness and viscous damping coefficients.
            // All models use the following formulas for normal and tangential forces:
            //     Fn = kn * delta_n - gn * v_n
            //     Ft = kt * delta_t - gt * v_t
            double kn = 0;
            double kt = 0;
            double gn = 0;
            double gt = 0;

            double eps = double.Epsilon;

            switch (contact_model)
            {
                case ChSystemSMC.ContactForceModel.Hooke:
                    if (use_mat_props)
                    {
                        double tmp_k = (16.0 / 15) * Math.Sqrt(this.eff_radius) * mat.E_eff;
                        double v2 = sys.GetCharacteristicImpactVelocity() * sys.GetCharacteristicImpactVelocity();
                        double loge = (mat.cr_eff < eps) ? Math.Log(eps) : Math.Log(mat.cr_eff);
                        loge = (mat.cr_eff > 1 - eps) ? Math.Log(1 - eps) : loge;
                        double tmp_g = 1 + Math.Pow(ChMaths.CH_C_PI / loge, 2);
                        kn = tmp_k * Math.Pow(eff_mass * v2 / tmp_k, 1.0 / 5);
                        kt = kn;
                        gn = Math.Sqrt(4 * eff_mass * kn / tmp_g);
                        gt = gn;
                    }
                    else
                    {
                        kn = mat.kn;
                        kt = mat.kt;
                        gn = eff_mass * mat.gn;
                        gt = eff_mass * mat.gt;
                    }

                    break;

                case ChSystemSMC.ContactForceModel.Hertz:
                    if (use_mat_props)
                    {
                        double sqrt_Rd = Math.Sqrt(this.eff_radius * delta);
                        double Sn = 2 * mat.E_eff * sqrt_Rd;
                        double St = 8 * mat.G_eff * sqrt_Rd;
                        double loge = (mat.cr_eff < eps) ? Math.Log(eps) : Math.Log(mat.cr_eff);
                        double beta = loge / Math.Sqrt(loge * loge + ChMaths.CH_C_PI * ChMaths.CH_C_PI);
                        kn = (2.0 / 3) * Sn;
                        kt = St;
                        gn = -2 * Math.Sqrt(5.0 / 6) * beta * Math.Sqrt(Sn * eff_mass);
                        gt = -2 * Math.Sqrt(5.0 / 6) * beta * Math.Sqrt(St * eff_mass);
                    }
                    else
                    {
                        double tmp = this.eff_radius * Math.Sqrt(delta);
                        kn = tmp * mat.kn;
                        kt = tmp * mat.kt;
                        gn = tmp * eff_mass * mat.gn;
                        gt = tmp * eff_mass * mat.gt;
                    }

                    break;

                case ChSystemSMC.ContactForceModel.PlainCoulomb:
                    if (use_mat_props)
                    {
                        double sqrt_Rd = Math.Sqrt(delta);
                        double Sn = 2 * mat.E_eff * sqrt_Rd;
                        double St = 8 * mat.G_eff * sqrt_Rd;
                        double loge = (mat.cr_eff < eps) ? Math.Log(eps) : Math.Log(mat.cr_eff);
                        double beta = loge / Math.Sqrt(loge * loge + ChMaths.CH_C_PI * ChMaths.CH_C_PI);
                        kn = (2.0 / 3) * Sn;
                        gn = -2 * Math.Sqrt(5.0 / 6) * beta * Math.Sqrt(Sn * eff_mass);
                    }
                    else
                    {
                        double tmp = Math.Sqrt(delta);
                        kn = tmp * mat.kn;
                        gn = tmp * mat.gn;
                    }

                    kt = 0;
                    gt = 0;

                    {
                        double forceNb = kn * delta - gn * relvel_n_mag;
                        if (forceNb < 0)
                            forceNb = 0;
                        double forceTb = mat.mu_eff * Math.Tanh(5.0 * relvel_t_mag) * forceNb;
                        switch (adhesion_model)
                        {
                            case ChSystemSMC.AdhesionForceModel.Constant:
                                forceNb -= mat.adhesion_eff;
                                break;
                            case ChSystemSMC.AdhesionForceModel.DMT:
                                forceNb -= mat.adhesionMultDMT_eff * Math.Sqrt(this.eff_radius);
                                break;
                        }
                        ChVector forceb = forceNb * normal_dir;
                        if (relvel_t_mag >= sys.GetSlipVelocityThreshold())
                            forceb -= (forceTb / relvel_t_mag) * relvel_t;

                        return forceb;
                    }
            }

            // Tangential displacement (magnitude)
            double delta_t = 0;
            switch (tdispl_model)
            {
                case ChSystemSMC.TangentialDisplacementModel.OneStep:
                    delta_t = relvel_t_mag * dT;
                    break;
                case ChSystemSMC.TangentialDisplacementModel.MultiStep:
                    //// TODO: implement proper MultiStep mode
                    delta_t = relvel_t_mag * dT;
                    break;
                default:
                    break;
            }

            // Calculate the magnitudes of the normal and tangential contact forces
            double forceN = kn * delta - gn * relvel_n_mag;
            double forceT = kt * delta_t + gt * relvel_t_mag;

            // If the resulting normal contact force is negative, the two shapes are moving
            // away from each other so fast that no contact force is generated.
            if (forceN < 0)
            {
                forceN = 0;
                forceT = 0;
            }

            // Include adhesion force
            switch (adhesion_model)
            {
                case ChSystemSMC.AdhesionForceModel.Constant:
                    forceN -= mat.adhesion_eff;
                    break;
                case ChSystemSMC.AdhesionForceModel.DMT:
                    forceN -= mat.adhesionMultDMT_eff * Math.Sqrt(this.eff_radius);
                    break;
            }

            // Coulomb law
            forceT = Math.Min(forceT, mat.mu_eff * Math.Abs(forceN));

            // Accumulate normal and tangential forces
            ChVector force = forceN * normal_dir;
            if (relvel_t_mag >= sys.GetSlipVelocityThreshold())
                force -= (forceT / relvel_t_mag) * relvel_t;

            return force;
        }

        /// Compute all forces in a contiguous array.
        /// Used in finite-difference Jacobian approximation.
        public void CalculateQ(ChState stateA_x,            //< state positions for objA
                    ChStateDelta stateA_w,       //< state velocities for objA
                    ChState stateB_x,            //< state positions for objB
                    ChStateDelta stateB_w,       //< state velocities for objB
                    ChMaterialCompositeSMC mat,  //< composite material for contact pair
                    ref ChVectorDynamic<double> Q                //< output generalized forces
                    )
        {
            ChBody oA = (this.objA as ChBody);
            ChBody oB = (this.objB as ChBody);

            // Express contact points in local frames.
            // We assume that these points remain fixed to their respective contactable objects.
            ChVector p1_loc = oA.GetCsysForCollisionModel().TransformPointParentToLocal(this.p1);
            ChVector p2_loc = oB.GetCsysForCollisionModel().TransformPointParentToLocal(this.p2);

            // Express the local points in global frame
            ChVector p1_abs = oA.GetContactPoint(p1_loc, stateA_x);
            ChVector p2_abs = oB.GetContactPoint(p2_loc, stateB_x);

            /*
                Note: while this can be somewhat justified for a ChBody, it will not work
                      for a mesh vertex for instance...

            // Project the points onto the unperturbed normal line
            p1_abs = this.p1 + Vdot(p1_abs - this.p1, this.normal) * this.normal;
            p2_abs = this.p2 + Vdot(p2_abs - this.p2, this.normal) * this.normal;
            */

            // Calculate normal direction (expressed in global frame)
            ChVector normal_dir = (p1_abs - p2_abs).GetNormalized();

            // Calculate penetration depth
            double delta = (p1_abs - p2_abs).Length();

            // If the normal direction flipped sign, change sign of delta
            if (ChVector.Vdot(normal_dir, this.normal) < 0)
                delta = -delta;

            // Calculate velocity of contact points (expressed in global frame)
            ChVector vel1 = oA.GetContactPointSpeed(p1_loc, stateA_x, stateA_w);
            ChVector vel2 = oB.GetContactPointSpeed(p2_loc, stateB_x, stateB_w);

            // Compute the contact force.
            ChVector force = CalculateForce(delta, normal_dir, vel1, vel2, mat);

            // Compute and load the generalized contact forces.
            oA.ContactForceLoadQ(-force, p1_abs, stateA_x, ref Q, 0);
            oB.ContactForceLoadQ(force, p2_abs, stateB_x, ref Q, oA.ContactableGet_ndof_w());
        }

        /// Create the Jacobian matrices.
        /// These matrices are created/resized as needed.
        public void CreateJacobians()
        {
            // m_Jac = null;
            m_Jac = new ChContactJacobian();

            ChBody oA = (this.objA as ChBody);
            ChBody oB = (this.objB as ChBody);

            // Set variables and resize Jacobian matrices.
            // NOTE: currently, only contactable objects derived from ChContactable_1vars<6>,
            //       ChContactable_1vars<3>, and ChContactable_3vars<3,3,3> are supported.
            int ndof_w = 0;
            List<ChVariables> vars = new List<ChVariables>();

            // ChContactable_3vars<IntInterface.Three, IntInterface.Three, IntInterface.Three> contactableA;

              // ChContactTuple<IntInterface.Three, IntInterface.Three> objA_333 = (this.objA as (ChBody)(ChContactable_3vars<IntInterface.Three, IntInterface.Three, IntInterface.Three>));

               vars.Add(oA.GetVariables1());
               //ChContactable_3vars<IntInterface.Three, IntInterface.Three, IntInterface.Three>> objA_333;// = new ChBody();
               /*if (objA_333 == this.objA as ChContactable_3vars<IntInterface.Three, IntInterface.Three, IntInterface.Three>){ }
                   vars.Add(objA_333.GetVariables2());
                   vars.Add(objA_333.GetVariables3());
               }*/
               ndof_w += oA.ContactableGet_ndof_w();

               vars.Add(oB.GetVariables1());
              /* if (auto objB_333 = dynamic_cast < ChContactable_3vars < 3, 3, 3 > *> (this.objB)) {
                   vars.push_back(objB_333.GetVariables2());
                   vars.push_back(objB_333.GetVariables3());
               }*/
               ndof_w += oB.ContactableGet_ndof_w();

               m_Jac.m_KRM.SetVariables(vars);
               m_Jac.m_K.matrix.Reset(ndof_w, ndof_w);
               m_Jac.m_R.matrix.Reset(ndof_w, ndof_w);
               Debug.Assert(m_Jac.m_KRM.Get_K().GetColumns() == ndof_w);
        }

        /// Calculate Jacobian of generalized contact forces.
        public void CalculateJacobians(ChMaterialCompositeSMC mat)
        {
            // Compute a finite-difference approximations to the Jacobians of the contact forces and
            // load dQ/dx into m_Jac.m_K and dQ/dw into m_Jac.m_R.
            // Note that we only calculate these Jacobians whenever the contact force itself is calculated,
            // that is only once per step.  The Jacobian of generalized contact forces will therefore be
            // constant over the time step.

            ChBody oA = (this.objA as ChBody);
            ChBody oB = (this.objB as ChBody);

            // Get states for objA
            int ndofA_x = oA.ContactableGet_ndof_x();
            int ndofA_w = oA.ContactableGet_ndof_w();
            ChState stateA_x = new ChState(ndofA_x, null);
            ChStateDelta stateA_w = new ChStateDelta(ndofA_w, null);
            oA.ContactableGetStateBlock_x(ref stateA_x);
            oA.ContactableGetStateBlock_w(ref stateA_w);

            // Get states for objB
            int ndofB_x = oB.ContactableGet_ndof_x();
            int ndofB_w = oB.ContactableGet_ndof_w();
            ChState stateB_x = new ChState(ndofB_x, null);
            ChStateDelta stateB_w = new ChStateDelta(ndofB_w, null);
            oB.ContactableGetStateBlock_x(ref stateB_x);
            oB.ContactableGetStateBlock_w(ref stateB_w);

            // Compute Q at current state
            ChVectorDynamic<double> Q0 = new ChVectorDynamic<double>(ndofA_w + ndofB_w);
            CalculateQ(stateA_x, stateA_w, stateB_x, stateB_w, mat, ref Q0);

            // Finite-difference approximation perturbation.
            // Note that ChState and ChStateDelta are set to 0 on construction.
            // To accommodate objects with quaternion states, use the method ContactableIncrementState while
            // calculating Jacobian columns corresponding to position states.
            double perturbation = 1e-5;
            ChState stateA_x1 = new ChState(ndofA_x, null);
            ChState stateB_x1 = new ChState(ndofB_x, null);
            ChStateDelta prtrbA = new ChStateDelta(ndofA_w, null);
            ChStateDelta prtrbB = new ChStateDelta(ndofB_w, null);

            ChVectorDynamic<double> Q1 = new ChVectorDynamic<double>(ndofA_w + ndofB_w);
            ChVectorDynamic<double> Jcolumn = new ChVectorDynamic<double>(ndofA_w + ndofB_w);

            // Jacobian w.r.t. variables of objA
            for (int i = 0; i < ndofA_w; i++)
            {
                prtrbA.matrix[i] += perturbation;
                oA.ContactableIncrementState(stateA_x, prtrbA, ref stateA_x1);
                CalculateQ(stateA_x1, stateA_w, stateB_x, stateB_w, mat, ref Q1);
                prtrbA.matrix[i] -= perturbation;

                Jcolumn = (Q1 - Q0.matrix) * (-1 / perturbation);  // note sign change

                m_Jac.m_K.matrix.PasteMatrix(Jcolumn.matrix, 0, i);

                stateA_w.matrix[i] += perturbation;
                CalculateQ(stateA_x, stateA_w, stateB_x, stateB_w, mat, ref Q1);
                stateA_w.matrix[i] -= perturbation;

                Jcolumn = (Q1 - Q0.matrix) * (-1 / perturbation);  // note sign change
                m_Jac.m_R.matrix.PasteMatrix(Jcolumn.matrix, 0, i);
            }

            // Jacobian w.r.t. variables of objB
            for (int i = 0; i < ndofB_w; i++)
            {
                prtrbB.matrix[i] += perturbation;
                oB.ContactableIncrementState(stateB_x, prtrbB, ref stateB_x1);
                CalculateQ(stateA_x, stateA_w, stateB_x1, stateB_w, mat, ref Q1);
                prtrbB.matrix[i] -= perturbation;

                Jcolumn = (Q1 - Q0.matrix) * (-1 / perturbation);  // note sign change
                m_Jac.m_K.matrix.PasteMatrix(Jcolumn.matrix, 0, ndofA_w + i);

                stateB_w.matrix[i] += perturbation;
                CalculateQ(stateA_x, stateA_w, stateB_x, stateB_w, mat, ref Q1);
                stateB_w.matrix[i] -= perturbation;

                Jcolumn = (Q1 - Q0.matrix) * (-1 / perturbation);  // note sign change
                m_Jac.m_R.matrix.PasteMatrix(Jcolumn.matrix, 0, ndofA_w + i);
            }
        }

        /// Apply contact forces to the two objects.
        /// (new version, for interfacing to ChTimestepper and ChIntegrable)
        public override void ContIntLoadResidual_F(ref ChVectorDynamic<double> R, double c)
        {
            ChVector abs_force_scaled = new ChVector(m_force * c);

            ChBody oA = (this.objA as ChBody);
            ChBody oB = (this.objB as ChBody);

            if (oA.IsContactActive())
                oA.ContactForceLoadResidual_F(-abs_force_scaled, this.p1, ref R);

            if (oB.IsContactActive())
                oB.ContactForceLoadResidual_F(abs_force_scaled, this.p2, ref R);
        }

        /// Inject Jacobian blocks into the system descriptor.
        /// Tell to a system descriptor that there are item(s) of type ChKblock in this object
        /// (for further passing it to a solver)
        public override void ContInjectKRMmatrices(ref ChSystemDescriptor mdescriptor)
        {
            if (m_Jac != null)
                mdescriptor.InsertKblock(m_Jac.m_KRM);
        }

        /// Compute Jacobian of contact forces.
        public override void ContKRMmatricesLoad(double Kfactor, double Rfactor)
        {
            if (m_Jac != null)
            {
                m_Jac.m_KRM.Get_K().FillElem(0);

                m_Jac.m_KRM.Get_K().MatrInc(m_Jac.m_K.matrix * Kfactor);
                m_Jac.m_KRM.Get_K().MatrInc(m_Jac.m_R.matrix * Rfactor);
            }
        }
    }
}
