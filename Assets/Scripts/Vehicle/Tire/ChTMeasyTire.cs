using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;


namespace chrono
{
    namespace vehicle
    {

        /// @addtogroup vehicle_wheeled_tire
        /// @{

        /// TMeasy tire model.
        [RequireComponent(typeof(ChBody))]
        public class ChTMeasyTire : ChTire {

            public Vector3 tireForce;
            public Vector3 tireMoment;

            public double wheelDiametre;
            public double rimDiameter;

            public double m_mass;

            // TEST
            ChMatrix33<double> rot = new ChMatrix33<double>(0);


            public ChTMeasyTire() : base() {
                m_vnum = 0.01;
                m_gamma = 0;
                m_gamma_limit = 5;
                m_begin_start_transition = 0.1;
                m_end_start_transition = 0.5;
                m_tireforce.force = new ChVector(0, 0, 0);
                m_tireforce.point = new ChVector(0, 0, 0);
                m_tireforce.moment = new ChVector(0, 0, 0);

                m_TMeasyCoeff.pn = 0.0;
            }

            // Use this for initialization
            void Start()
            {
              //  m_system = GameObject.FindObjectOfType<ChSystem>();
                m_wheel = GetComponent<ChBody>();
                m_terrain = GameObject.FindObjectOfType<RigidTerrain>();

                m_data = new ContactData();
                m_data.in_contact = false;
                m_data.frame = new ChCoordsys();
                m_data.vel = new ChVector();
                m_data.normal_force = 0;
                m_data.depth = 0;

                SetStepsize(m_stepsize);

                Initialize(m_wheel, m_side);

            }

            // Update is called once per frame
            public override void FixedUpdate()
            {
                base.FixedUpdate();
               // m_tireforce = GetTireForce();

                tireForce.x = (float)m_tireforce.force.x;
                tireForce.y = (float)m_tireforce.force.y;
                tireForce.z = (float)m_tireforce.force.z;
                tireMoment.x = (float)m_tireforce.moment.x;
                tireMoment.y = (float)m_tireforce.moment.y;
                tireMoment.z = (float)m_tireforce.moment.z;


                // Advance simulation for one timestep for all modules
               // double softstep = ChSystem.system.m_softstep;

                Synchronize(ChSystem.system.GetChTime(), m_wheelState, m_terrain);
                Advance(m_stepsize);

            }

            /// Initialize this tire system.
            public override void Initialize(ChBody wheel, ChSubsysDefs.VehicleSide side)  ///< [in] associated wheel body
            {
                base.Initialize(wheel, side);

                SetTMeasyParams();

                // Initialize contact patch state variables to 0;
                m_states.sx = 0;
                m_states.sy = 0;
                m_states.vta = m_vnum;
                m_states.xe = 0;
                m_states.ye = 0;
                m_states.Fx_dyn = 0;
                m_states.Fy_dyn = 0;
                m_states.Mb_dyn = 0;
                m_consider_relaxation = true;
                m_use_Reff_fallback_calculation = false;
                m_integration_method = 2;

                if (!m_use_Reff_fallback_calculation)
                {
                    // calculate critical values
                    m_fz_rdynco_crit = (m_TMeasyCoeff.pn * (m_TMeasyCoeff.rdynco_p2n - 2.0 * m_TMeasyCoeff.rdynco_pn + 1.0)) /
                                       (2.0 * (m_TMeasyCoeff.rdynco_p2n - m_TMeasyCoeff.rdynco_pn));
                    m_rdynco_crit = InterpL(m_fz_rdynco_crit, m_TMeasyCoeff.rdynco_pn, m_TMeasyCoeff.rdynco_p2n);
                }
            }

            /// Add visualization assets for the rigid tire subsystem.
            // public override void AddVisualizationAssets(VisualizationType vis) override;

            /// Remove visualization assets for the rigid tire subsystem.
            // virtual void RemoveVisualizationAssets() override;

            /// Get the tire radius.
            public override double GetRadius() { return m_states.R_eff; }

            /// Get the tire force and moment.
            /// This represents the output from this tire system that is passed to the
            /// vehicle system.  Typically, the vehicle subsystem will pass the tire force
            /// to the appropriate suspension subsystem which applies it as an external
            /// force one the wheel body.
            public override ChSubsysDefs.TerrainForce GetTireForce() { return m_tireforce; }

            /// Report the tire force and moment.
            public override ChSubsysDefs.TerrainForce ReportTireForce(ChTerrain terrain) { return m_tireforce; }

            /// Update the state of this tire system at the current time.
            /// The tire system is provided the current state of its associated wheel.
            public override void Synchronize(double time,                    ///< [in] current time
                             ChSubsysDefs.WheelState wheel_state,  ///< [in] current state of associated wheel body
                             RigidTerrain terrain        ///< [in] reference to the terrain system
                             ) {
                // Invoke the base class function.
                base.Synchronize(time, wheel_state, terrain);

                m_time = time;

                //ChCoordsys contact_frame;
                // Clear the force accumulators and set the application point to the wheel
                // center.
                m_tireforce.force = new ChVector(0, 0, 0);
                m_tireforce.moment = new ChVector(0, 0, 0);
                m_tireforce.point = wheel_state.pos;
                m_mu = 0.89;// terrain.GetCoefficientFriction(m_tireforce.point.x, m_tireforce.point.y); // FIX THiS
                // Ensure that m_mu stays realistic and the formulae don't degenerate
                ChMaths.ChClampValue(ref m_mu, 0.1, 1.0);

                // Extract the wheel normal (expressed in global frame)
                ChMatrix33<double> A = new ChMatrix33<double>(wheel_state.rot);
                ChVector disc_normal = A.Get_A_Yaxis();

                // Assuming the tire is a disc, check contact with terrain
                switch (m_collision_type)
                {
                    case CollisionType.SINGLE_POINT:
                        m_data.in_contact = DiscTerrainCollision(terrain, wheel_state.pos, disc_normal, m_unloaded_radius,
                                                                 ref m_data.frame, ref m_data.depth);
                        m_gamma = GetCamberAngle();
                        break;
                    case CollisionType.FOUR_POINTS:
                        m_data.in_contact = DiscTerrainCollision4pt(terrain, wheel_state.pos, disc_normal, m_unloaded_radius,
                                                                    m_width, ref m_data.frame, ref m_data.depth, ref m_gamma);
                        break;
                    case CollisionType.ENVELOPE:
                       // m_data.in_contact = DiscTerrainCollisionEnvelope(terrain, wheel_state.pos, disc_normal, m_unloaded_radius,
                       //                                                  ref m_areaDep, m_data.frame, m_data.depth);
                        m_gamma = GetCamberAngle();
                        break;
                }
                UpdateVerticalStiffness();
                if (m_data.in_contact)
                {
                    // Wheel velocity in the ISO-C Frame
                    ChVector vel = wheel_state.lin_vel;
                    m_data.vel = m_data.frame.TransformDirectionParentToLocal(vel);

                    // Generate normal contact force (recall, all forces are reduced to the wheel
                    // center). If the resulting force is negative, the disc is moving away from
                    // the terrain so fast that no contact force is generated.
                    // The sign of the velocity term in the damping function is negative since
                    // a positive velocity means a decreasing depth, not an increasing depth
                    double Fn_mag = GetNormalStiffnessForce(m_data.depth) + GetNormalDampingForce(m_data.depth, -m_data.vel.z);

                    if (Fn_mag < 0)
                    {
                        Fn_mag = 0;
                        m_data.in_contact = false;  // Skip Force and moment calculations when the normal force = 0
                    }

                    m_data.normal_force = Fn_mag;
                    double r_stat = m_unloaded_radius - m_data.depth;
                    m_states.omega = wheel_state.omega;
                    if (m_use_Reff_fallback_calculation)
                    {
                        m_states.R_eff = (2.0 * m_unloaded_radius + r_stat) / 3.0;
                    }
                    else
                    {
                        if (Fn_mag <= m_fz_rdynco_crit)
                        {
                            m_rdynco = InterpL(Fn_mag, m_TMeasyCoeff.rdynco_pn, m_TMeasyCoeff.rdynco_p2n);
                            m_states.R_eff = m_rdynco * m_unloaded_radius + (1.0 - m_rdynco) * (r_stat);
                        }
                        else
                        {
                            m_rdynco = m_rdynco_crit;
                            m_states.R_eff = m_rdynco * m_unloaded_radius + (1.0 - m_rdynco) * (r_stat);
                        }
                    }
                    m_states.vta = m_states.R_eff * Mathfx.Abs(m_states.omega) + m_vnum;
                   // m_states.vx = Math.Abs(m_data.vel.x);
                    m_states.vsx = m_data.vel.x - m_states.omega * m_states.R_eff;
                    m_states.vsy = m_data.vel.y;
                    m_states.sx = -m_states.vsx / m_states.vta;
                    m_states.sy = -m_states.vsy / m_states.vta;
                    m_states.disc_normal = disc_normal;
                }
                else
                {
                    // Reset all states if the tire comes off the ground.
                    m_data.normal_force = 0;
                    m_states.R_eff = m_unloaded_radius;
                    m_states.sx = 0;
                    m_states.sy = 0;
                    m_states.vta = m_vnum;
                    m_states.vsx = 0;
                    m_states.vsy = 0;
                    m_states.omega = 0;
                    m_states.Fx_dyn = 0;
                    m_states.Fy_dyn = 0;
                    m_states.Mb_dyn = 0;
                    m_states.xe = 0;
                    m_states.ye = 0;
                    m_states.Fx = 0;
                    m_states.Fy = 0;
                    m_states.Mb = 0;
                    m_states.disc_normal = new ChVector(0, 0, 0);
                }
            }

            /// Advance the state of this tire by the specified time step.
            public override void Advance(double step)
            {
                // Return now if no contact.  Tire force and moment are already set to 0 in Synchronize().
                if (!m_data.in_contact)
                    return;

                double sc;              // combined slip
                double calpha, salpha;  // cos(alpha) rsp. sin(alpha), alpha = slip angle
                double muscale;         // factor for considering local friction

                muscale = m_mu / m_TMeasyCoeff.mu_0;

                // Clamp |gamma| to specified value: Limit due to tire testing, avoids erratic extrapolation.
                double gamma = ChMaths.ChClamp(GetCamberAngle(), -m_gamma_limit * ChMaths.CH_C_DEG_TO_RAD, m_gamma_limit * ChMaths.CH_C_DEG_TO_RAD);

                // Limit the effect of Fz on handling forces and torques to avoid nonsensical extrapolation of the curve coefficients
                // m_data.normal_force is nevertheless still taken as the applied vertical tire force
                double Fz = Math.Min(m_data.normal_force, m_TMeasyCoeff.pn_max);
                double Mx;// = 0;
                double My;// = 0;
                double Mz;// = 0;

                // Calculate Fz dependend Curve Parameters
                double dfx0 = InterpQ(Fz, m_TMeasyCoeff.dfx0_pn, m_TMeasyCoeff.dfx0_p2n);
                double dfy0 = InterpQ(Fz, m_TMeasyCoeff.dfy0_pn, m_TMeasyCoeff.dfy0_p2n);

                double fxm = muscale * InterpQ(Fz, m_TMeasyCoeff.fxm_pn, m_TMeasyCoeff.fxm_p2n);
                double fym = muscale * InterpQ(Fz, m_TMeasyCoeff.fym_pn, m_TMeasyCoeff.fym_p2n);

                double sxm = muscale * InterpL(Fz, m_TMeasyCoeff.sxm_pn, m_TMeasyCoeff.sxm_p2n);
                double sym = muscale * InterpL(Fz, m_TMeasyCoeff.sym_pn, m_TMeasyCoeff.sym_p2n);

                double fxs = muscale * InterpQ(Fz, m_TMeasyCoeff.fxs_pn, m_TMeasyCoeff.fxs_p2n);
                double fys = muscale * InterpQ(Fz, m_TMeasyCoeff.fys_pn, m_TMeasyCoeff.fys_p2n);

                double sxs = muscale * InterpL(Fz, m_TMeasyCoeff.sxs_pn, m_TMeasyCoeff.sxs_p2n);
                double sys = muscale * InterpL(Fz, m_TMeasyCoeff.sys_pn, m_TMeasyCoeff.sys_p2n);

                // slip normalizing factors
                double hsxn = sxm / (sxm + sym) + (fxm / dfx0) / (fxm / dfx0 + fym / dfy0);
                double hsyn = sym / (sxm + sym) + (fym / dfy0) / (fxm / dfx0 + fym / dfy0);

                double sxn = m_states.sx / hsxn;
                double syn = m_states.sy / hsyn;

                sc = ChMaths.hypot(sxn, syn);

                if (sc > 0)
                {
                    calpha = sxn / sc;
                    salpha = syn / sc;
                }
                else
                {
                    calpha = Math.Sqrt(2.0) / 2.0;
                    salpha = Math.Sqrt(2.0) / 2.0;
                }

                double nto0 = InterpL(Fz, m_TMeasyCoeff.nto0_pn, m_TMeasyCoeff.nto0_p2n);
                double synto0 = muscale * InterpL(Fz, m_TMeasyCoeff.synto0_pn, m_TMeasyCoeff.synto0_p2n);
                double syntoE = muscale * InterpL(Fz, m_TMeasyCoeff.syntoE_pn, m_TMeasyCoeff.syntoE_p2n);

                // Calculate resultant Curve Parameters
                double df0 = ChMaths.hypot(dfx0 * calpha * hsxn, dfy0 * salpha * hsyn);
                double fm = ChMaths.hypot(fxm * calpha, fym * salpha);
                double sm = ChMaths.hypot(sxm * calpha / hsxn, sym * salpha / hsyn);
                double fs = ChMaths.hypot(fxs * calpha, fys * salpha);
                double ss = ChMaths.hypot(sxs * calpha / hsxn, sys * salpha / hsyn);
                double f = 0.0;
                double fos = 0.0;

                // consider camber effects
                // Calculate length of tire contact patch
                double plen = 2.0 * Math.Sqrt(m_unloaded_radius * m_data.depth);

                // tire bore radius  (estimated from length l and width b of contact patch)
                double rb = 2.0 / 3.0 * 0.5 * ((plen / 2.0) + (m_width / 2.0));

                // bore slip due to camber
                double sb = -rb * m_states.omega * Math.Sin(gamma) / m_states.vta;

                // generalzed slip
                double sg = ChMaths.hypot(sc, sb);

                tmxy_combined(ref f, ref fos, sg, df0, sm, fm, ss, fs);
                if (sg > 0.0)
                {
                    m_states.Fx = f * m_states.sx / sg;
                    m_states.Fy = f * m_states.sy / sg;
                }
                else
                {
                    m_states.Fx = 0.0;
                    m_states.Fy = 0.0;
                }
                // Calculate dimensionless lever arm
                double levN = tmy_tireoff(m_states.sy, nto0, synto0, syntoE);

                // Bore Torque
                if (sg > 0.0)
                {
                    m_states.Mb = rb * f * sb / sg;
                }
                else
                {
                    m_states.Mb = 0.0;
                }

                //   camber slip and force
                double sy_c = -0.5 * plen * m_states.omega * Math.Sin(gamma) / m_states.vta;
                double fy_c = fos / 3.0 * sy_c;

                m_states.Fy += fy_c;

                // Overturning Torque
                {
                    double cg = Math.Pow(m_width, 2.0) * m_TMeasyCoeff.cz / 12.0;
                    Mx = -cg * gamma;
                }

                // Rolling Resistance, Ramp Like Signum inhibits 'switching' of My
                {
                    // smoothing interval for My
                    const double vx_min = 0.125;
                    const double vx_max = 0.5;

                    double Lrad = (m_unloaded_radius - m_data.depth);
                    m_rolling_resistance = InterpL(Fz, m_TMeasyCoeff.rrcoeff_pn, m_TMeasyCoeff.rrcoeff_p2n);
                    My = -ChMaths.ChSineStep(m_states.vta, vx_min, 0.0, vx_max, 1.0) * m_rolling_resistance * m_data.normal_force * Lrad * Math.Sign(m_states.omega);
                }

                double Ms = 0.0;

                double startup = ChMaths.ChSineStep(m_time, m_begin_start_transition, 0.0, m_end_start_transition, 1.0);

                if (m_consider_relaxation)
                {
                    double vtxs = m_states.vta * hsxn;
                    double vtys = m_states.vta * hsyn;
                    double relax = 0.66 * ChMaths.CH_C_2PI * m_states.R_eff;

                    //-------------------
                    // Self Alignment Torque
                    Ms = -plen * levN * m_states.Fy_dyn;
                    Mz = Ms + m_states.Mb_dyn;

                    // Take as many integration steps as needed to reach the value 'step'
                    double t = 0;
                    while (t < step)
                    {
                        // Ensure we integrate exactly to 'step'
                        double h = Math.Min(m_stepsize, step - t);
                        // limit delay time of bore torque Mb to realistic values
                        double tau = ChMaths.ChClamp(relax / m_states.vta, 1.0e-4, 0.25);
                        double gain = 1.0 / tau;

                        switch (m_integration_method)
                        {
                            case 1:
                                {
                                    // explicit Euler, may be unstable
                                    // 1. oder tire dynamics
                                    m_states.xe = m_states.xe + h * (-vtxs * m_TMeasyCoeff.cx * m_states.xe - fos * m_states.vsx) /
                                                                    (vtxs * m_TMeasyCoeff.dx + fos);
                                    m_states.ye = m_states.ye + h * (-vtys * m_TMeasyCoeff.cy * m_states.ye - fos * m_states.vsy) /
                                                                    (vtys * m_TMeasyCoeff.dy + fos);
                                    // 0. order tire dynamics
                                    m_states.Mb_dyn = m_states.Mb_dyn + h * (m_states.Mb - m_states.Mb_dyn) * m_states.vta / relax;
                                    break;
                                }
                            case 2:
                                {
                                    // semi-implicit Euler, absolutely stable
                                    // 1. oder tire dynamics
                                    double dFx = -vtxs * m_TMeasyCoeff.cx / (vtxs * m_TMeasyCoeff.dx + fos);
                                    m_states.xe = m_states.xe + h / (1.0 - h * dFx) *
                                                                    (-vtxs * m_TMeasyCoeff.cx * m_states.xe - fos * m_states.vsx) /
                                                                    (vtxs * m_TMeasyCoeff.dx + fos);
                                    double dFy = -vtys * m_TMeasyCoeff.cy / (vtys * m_TMeasyCoeff.dy + fos);
                                    m_states.ye = m_states.ye + h / (1.0 - h * dFy) *
                                                                    (-vtys * m_TMeasyCoeff.cy * m_states.ye - fos * m_states.vsy) /
                                                                    (vtys * m_TMeasyCoeff.dy + fos);
                                    // 0. order tire dynamics
                                    double dMb = -gain;
                                    m_states.Mb_dyn =
                                        m_states.Mb_dyn + h / (1.0 - h * dMb) * (m_states.Mb - m_states.Mb_dyn) * gain;
                                    break;
                                }
                        }
                        t += h;
                    }

                    m_states.Fx_dyn = m_TMeasyCoeff.dx * (-vtxs * m_TMeasyCoeff.cx * m_states.xe - fos * m_states.vsx) /
                                          (vtxs * m_TMeasyCoeff.dx + fos) +
                                      m_TMeasyCoeff.cx * m_states.xe;
                    m_states.Fy_dyn = m_TMeasyCoeff.dy * (-vtys * m_TMeasyCoeff.cy * m_states.ye - fos * m_states.vsy) /
                                          (vtys * m_TMeasyCoeff.dy + fos) +
                                      m_TMeasyCoeff.cy * m_states.ye;
                    // Calculate result of alignment torque and bore torque
                    // Compile the force and moment vectors so that they can be
                    // transformed into the global coordinate system.
                    m_tireforce.force = new ChVector(startup * m_states.Fx_dyn, startup * m_states.Fy_dyn, m_data.normal_force);
                    m_tireforce.moment = startup * new ChVector(Mx, My, Mz);
                    // Info data (not used in the algorithm)
                    m_tau_x = m_TMeasyCoeff.dx / m_TMeasyCoeff.cx + fos / (vtxs * m_TMeasyCoeff.cx);
                    m_tau_y = m_TMeasyCoeff.dy / m_TMeasyCoeff.cy + fos / (vtys * m_TMeasyCoeff.cy);
                    m_relaxation_lenght_x = m_states.R_eff * Mathfx.Abs(m_states.omega) * m_tau_x;
                    m_relaxation_lenght_y = m_states.R_eff * Mathfx.Abs(m_states.omega) * m_tau_y;
                    // could be changed to user input, if needed
                    m_relaxation_lenght_phi = relax;
                }
                else
                {
                    // Self Alignment Torque
                    m_tau_x = m_tau_y = 0;
                    m_relaxation_lenght_x = m_relaxation_lenght_y = 0;
                    Ms = -plen * levN * m_states.Fy;
                    // Calculate result of alignment torque and bore torque
                    Mz = Ms + m_states.Mb;
                    // Compile the force and moment vectors so that they can be
                    // transformed into the global coordinate system.
                    m_tireforce.force = new ChVector(startup * m_states.Fx, startup * m_states.Fy, m_data.normal_force);
                    m_tireforce.moment = startup * new ChVector(Mx, My, Mz);
                }

                // Rotate into global coordinates
                m_tireforce.force = m_data.frame.TransformDirectionLocalToParent(m_tireforce.force);
                m_tireforce.moment = m_data.frame.TransformDirectionLocalToParent(m_tireforce.moment);

                // Move the tire forces from the contact patch to the wheel center
                m_tireforce.moment +=
                    ChVector.Vcross((m_data.frame.pos + m_data.depth * m_data.frame.rot.GetYaxis()) - m_tireforce.point, m_tireforce.force);

            }

            /// Set the limit for camber angle (in degrees).  Default: 3 degrees.
            public void SetGammaLimit(double gamma_limit) { m_gamma_limit = gamma_limit; }

            /// Get the tire mass.
            /// Note that this should not include the mass of the wheel (rim).
            public override double GetMass() { return m_mass; }

            /// Get the width of the tire.
            public double GetWidth() { return m_width; }

            /// Get visualization width.
            // public virtual double GetVisualizationWidth() { return m_width; }

            /// Get the tire slip angle.
            public override double GetSlipAngle() { return Math.Atan(m_states.sy); }

            /// Get the tire longitudinal slip.
            public override double GetLongitudinalSlip() { return m_states.sx; }

            /// Get the camber angle used in the TMeasy model (expressed in radian).
            public double GetGamma() { return m_gamma; }

            /// Get Max. Tire Load from Load Index (LI) in N [0:279]
            public static double GetTireMaxLoad(int li) {
                double[] Weight_per_Tire = new double[]{
                45,    46.5,  47.5,   48.7,   50,     51.5,   53,     54.5,   56,     58,     60,     61.5,   63,     65,
                67,    69,    71,     73,     75,     77.5,   80.0,   82.5,   85.0,   87.5,   90.0,   92.5,   95.0,   97.5,
                100.0, 103,   106,    109,    112,    115,    118,    121,    125,    128,    132,    136,    140,    145,
                150,   155,   160,    165,    170,    175,    180,    185,    190,    195,    200,    206,    212,    218,
                224,   230,   236,    243,    250,    257,    265,    272,    280,    290,    300,    307,    315,    325,
                335,   345,   355,    365,    375,    387,    400,    412,    425,    437,    450,    462,    475,    487,
                500,   515,   530,    545,    560,    580,    600,    615,    630,    650,    670,    690,    710,    730,
                750,   775,   800,    825,    850,    875,    900,    925,    950,    975,    1000,   1030,   1060,   1090,
                1120,  1150,  1180,   1215,   1250,   1285,   1320,   1360,   1400,   1450,   1500,   1550,   1600,   1650,
                1700,  1750,  1800,   1850,   1900,   1950,   2000,   2060,   2120,   2180,   2240,   2300,   2360,   2430,
                2500,  2575,  2650,   2725,   2800,   2900,   3000,   3075,   3150,   3250,   3350,   3450,   3550,   3650,
                3750,  3875,  4000,   4125,   4250,   4375,   4500,   4625,   4750,   4875,   5000,   5150,   5300,   5450,
                5600,  5850,  6000,   6150,   6300,   6500,   6700,   6900,   7100,   7300,   7500,   7750,   8000,   8250,
                8500,  8750,  9000,   9250,   9500,   9750,   10000,  10300,  10600,  10900,  11200,  11500,  11800,  12150,
                12500, 12850, 13200,  13600,  14000,  14500,  15000,  15550,  16000,  16500,  17000,  17500,  18000,  18500,
                19000, 19500, 20000,  20600,  21200,  21800,  22400,  23000,  23600,  24300,  25000,  25750,  26500,  27250,
                28000, 29000, 30000,  30750,  31500,  32500,  33500,  34500,  35500,  36500,  37500,  38750,  40000,  41250,
                42500, 43750, 45000,  46250,  47500,  48750,  50000,  51500,  53000,  54500,  56000,  58000,  60000,  61500,
                63000, 65000, 67000,  69000,  71000,  73000,  75000,  77500,  80000,  82500,  85000,  87500,  90000,  92500,
                95000, 97500, 100000, 103000, 106000, 109000, 112000, 115000, 118000, 121000, 125000, 128500, 132000, 136000 };

                //uint nw = (uint)Marshal.SizeOf(Weight_per_Tire) / sizeof(double);
                // uint nw = sizeof(Weight_per_Tire) / sizeof(double);
                const double g = 9.81;
                double fmax;
                if (li < Weight_per_Tire.Length)
                {
                    fmax = Weight_per_Tire[li] * g;
                }
                else
                {
                    fmax = Weight_per_Tire[Weight_per_Tire.Length - 1] * g;
                }
                return fmax;
            }

            /// Guess Tire Parameters from characteristic truck tire parameter pattern (Ratio = 80%)
            public void GuessTruck80Par(int li,        //< tire load index
                         double tireWidth,       //< tire width [m]
                         double ratio,           //< use 0.75 meaning 75%
                         double rimDia,          //< rim diameter [m]
                         double pinfl_li = 1.0,  //< inflation pressure at load index
                         double pinfl_use = 1.0  //< inflation pressure in this configuration
    )
            {
                double tireLoad = GetTireMaxLoad(li);
                GuessTruck80Par(tireLoad, tireWidth, ratio, rimDia, pinfl_li, pinfl_use);
            }

            /// Get the tire moments of inertia.
            /// Note that these should not include the inertia of the wheel (rim).
            public override ChVector GetInertia() { return new ChVector(); }

            public void GuessTruck80Par(double tireLoad,   // tire load force [N]
                                   double tireWidth,  // [m]
                                   double ratio,      // [] = use 0.75 meaning 75%
                                   double rimDia,     // rim diameter [m]
                                   double pinfl_li = 1.0,   // inflation pressure at load index
                                   double pinfl_use = 1.0   // inflation pressure in this configuration
    )
            {
                const double N2kN = 0.001;
                double secth = tireWidth * ratio;  // tire section height
                double defl_max = 0.16 * secth;    // deflection at tire payload
                double xi = 0.05;                  // damping ratio

                m_TMeasyCoeff.pn = 0.5 * tireLoad * Math.Pow(pinfl_use / pinfl_li, 0.8);
                m_TMeasyCoeff.pn_max = 3.5 * m_TMeasyCoeff.pn;

                double CZ = m_TMeasyCoeff.pn / defl_max;
                double DZ = xi * Math.Sqrt(CZ * GetMass());

                SetVerticalStiffness(CZ);

                SetRollingResistanceCoefficients(0.015, 0.015);

                SetDynamicRadiusCoefficients(0.375, 0.75);

                m_TMeasyCoeff.dz = DZ;
                m_TMeasyCoeff.cx = 0.9 * CZ;
                m_TMeasyCoeff.dx = xi * Math.Sqrt(m_TMeasyCoeff.cx * GetMass());
                m_TMeasyCoeff.cy = 0.8 * CZ;
                m_TMeasyCoeff.dy = xi * Math.Sqrt(m_TMeasyCoeff.cy * GetMass());

                m_rim_radius = 0.5 * rimDia;
                m_roundness = 0.1;

                m_width = tireWidth;
                m_unloaded_radius = secth + rimDia / 2.0;
                m_TMeasyCoeff.mu_0 = 0.8;

                m_TMeasyCoeff.dfx0_pn = 17.6866 * m_TMeasyCoeff.pn * N2kN;
                m_TMeasyCoeff.sxm_pn = 0.12;
                m_TMeasyCoeff.fxm_pn = 0.88468 * m_TMeasyCoeff.pn * N2kN;
                m_TMeasyCoeff.sxs_pn = 0.9;
                m_TMeasyCoeff.fxs_pn = 0.54397 * m_TMeasyCoeff.pn * N2kN;

                m_TMeasyCoeff.dfx0_p2n = 13.8046 * 2.0 * m_TMeasyCoeff.pn * N2kN;
                m_TMeasyCoeff.sxm_p2n = 0.15;
                m_TMeasyCoeff.fxm_p2n = 0.7479 * 2.0 * m_TMeasyCoeff.pn * N2kN;
                m_TMeasyCoeff.sxs_p2n = 0.95;
                m_TMeasyCoeff.fxs_p2n = 0.50365 * 2.0 * m_TMeasyCoeff.pn * N2kN;

                m_TMeasyCoeff.dfy0_pn = 5.948 * m_TMeasyCoeff.pn * N2kN;
                m_TMeasyCoeff.sym_pn = 0.38786;
                m_TMeasyCoeff.fym_pn = 0.77253 * m_TMeasyCoeff.pn * N2kN;
                m_TMeasyCoeff.sys_pn = 0.82534;
                m_TMeasyCoeff.fys_pn = 0.71139 * m_TMeasyCoeff.pn * N2kN;

                m_TMeasyCoeff.dfy0_p2n = 5.506 * 2.0 * m_TMeasyCoeff.pn * N2kN;
                m_TMeasyCoeff.sym_p2n = 0.38786;
                m_TMeasyCoeff.fym_p2n = 0.73048 * 2.0 * m_TMeasyCoeff.pn * N2kN;
                m_TMeasyCoeff.sys_p2n = 0.91309;
                m_TMeasyCoeff.fys_p2n = 0.66823 * 2.0 * m_TMeasyCoeff.pn * N2kN;

                m_TMeasyCoeff.nto0_pn = 0.178;
                m_TMeasyCoeff.synto0_pn = 0.40726;
                m_TMeasyCoeff.syntoE_pn = 0.82534;

                m_TMeasyCoeff.nto0_p2n = 0.19;
                m_TMeasyCoeff.synto0_p2n = 0.40726;
                m_TMeasyCoeff.syntoE_p2n = 0.91309;
            }

            /// Guess Tire Parameters from characteristic passenger car tire parameter pattern (Ratio = 70%)
            public void GuessPassCar70Par(int li,        //< tire load index
                           double tireWidth,       //< tire width [m]
                           double ratio,           //< use 0.75 meaning 75%
                           double rimDia,          //< rim diameter [m]
                           double pinfl_li = 1.0,  //< inflation pressure at load index
                           double pinfl_use = 1.0  //< inflation pressure in this configuration
    )
            {
                double tireLoad = GetTireMaxLoad(li);
                GuessPassCar70Par(tireLoad, tireWidth, ratio, rimDia, pinfl_li, pinfl_use);
            }
            public void GuessPassCar70Par(double tireLoad,   // tire load force [N]
                                     double tireWidth,  // [m]
                                     double ratio,      // [] = use 0.75 meaning 75%
                                     double rimDia,     // rim diameter [m]
                                     double pinfl_li,   // inflation pressure at load index
                                     double pinfl_use   // inflation pressure in this configuration
    )
            {
                const double N2kN = 0.001;
                double secth = tireWidth * ratio;  // tire section height
                double defl_max = 0.16 * secth;    // deflection at tire payload
                double xi = 0.05;                  // damping ration

                m_TMeasyCoeff.pn = 0.5 * tireLoad * Math.Pow(pinfl_use / pinfl_li, 0.8);
                m_TMeasyCoeff.pn_max = 3.5 * m_TMeasyCoeff.pn;

                m_width = tireWidth;
                m_unloaded_radius = secth + rimDia / 2.0;
                m_TMeasyCoeff.mu_0 = 0.8;

                double CZ = m_TMeasyCoeff.pn / defl_max;
                double DZ = xi * Math.Sqrt(CZ * GetMass());

                SetVerticalStiffness(CZ);

                SetRollingResistanceCoefficients(0.015, 0.015);

                SetDynamicRadiusCoefficients(0.375, 0.75);

                m_TMeasyCoeff.dz = DZ;
                m_TMeasyCoeff.cx = 0.9 * CZ;
                m_TMeasyCoeff.dx = xi * Math.Sqrt(m_TMeasyCoeff.cx * GetMass());
                m_TMeasyCoeff.cy = 0.8 * CZ;
                m_TMeasyCoeff.dy = xi * Math.Sqrt(m_TMeasyCoeff.cy * GetMass());

                m_rim_radius = 0.5 * rimDia;
                m_roundness = 0.1;

                m_TMeasyCoeff.dfx0_pn = 18.6758 * m_TMeasyCoeff.pn * N2kN;
                m_TMeasyCoeff.sxm_pn = 0.17;
                m_TMeasyCoeff.fxm_pn = 1.1205 * m_TMeasyCoeff.pn * N2kN;
                m_TMeasyCoeff.sxs_pn = 0.9;
                m_TMeasyCoeff.fxs_pn = 0.8766 * m_TMeasyCoeff.pn * N2kN;

                m_TMeasyCoeff.dfx0_p2n = 20.1757 * 2.0 * m_TMeasyCoeff.pn * N2kN;
                m_TMeasyCoeff.sxm_p2n = 0.15;
                m_TMeasyCoeff.fxm_p2n = 1.072 * 2.0 * m_TMeasyCoeff.pn * N2kN;
                m_TMeasyCoeff.sxs_p2n = 0.95;
                m_TMeasyCoeff.fxs_p2n = 0.8245 * 2.0 * m_TMeasyCoeff.pn * N2kN;

                m_TMeasyCoeff.dfy0_pn = 14.9858 * m_TMeasyCoeff.pn * N2kN;
                m_TMeasyCoeff.sym_pn = 0.18197;
                m_TMeasyCoeff.fym_pn = 1.0084 * m_TMeasyCoeff.pn * N2kN;
                m_TMeasyCoeff.sys_pn = 0.82534;
                m_TMeasyCoeff.fys_pn = 0.83941 * m_TMeasyCoeff.pn * N2kN;

                m_TMeasyCoeff.dfy0_p2n = 10.0505 * 2.0 * m_TMeasyCoeff.pn * N2kN;
                m_TMeasyCoeff.sym_p2n = 0.24472;
                m_TMeasyCoeff.fym_p2n = 0.90003 * 2.0 * m_TMeasyCoeff.pn * N2kN;
                m_TMeasyCoeff.sys_p2n = 0.91309;
                m_TMeasyCoeff.fys_p2n = 0.76782 * 2.0 * m_TMeasyCoeff.pn * N2kN;

                m_TMeasyCoeff.nto0_pn = 0.178;
                m_TMeasyCoeff.synto0_pn = 0.19107;
                m_TMeasyCoeff.syntoE_pn = 0.82534;

                m_TMeasyCoeff.nto0_p2n = 0.19;
                m_TMeasyCoeff.synto0_p2n = 0.25695;
                m_TMeasyCoeff.syntoE_p2n = 0.91309;
            }

            /// Set vertical tire stiffness as linear function by coefficient [N/m].
            public void SetVerticalStiffness(double Cz) { SetVerticalStiffness(Cz, Cz); }

            /// Set vertical tire stiffness as nonlinear function by coefficients at nominal load 1 [N/m]
            /// and nominal load 2 [N/m].
            public void SetVerticalStiffness(double Cz1, double Cz2) {
                const double N2kN = 0.001;

                double cz1 = N2kN * Cz1;
                double cz2 = N2kN * Cz2;

                if (m_TMeasyCoeff.pn <= 0.0)
                {
                    Debug.Log("Fatal error in TMeasyTire: nominal tire load has not been set.");
                    //exit(99);
                }

                m_a1 = Math.Sqrt(2.0 * Math.Pow(cz1, 2) - Math.Pow(cz2, 2));
                m_a2 = (Math.Pow(cz2, 2) - Math.Pow(cz1, 2)) / (4.0 * N2kN * m_TMeasyCoeff.pn);
            }

            /// Set vertical tire stiffness as nonlinear function by calculation from tire test data (least squares).
            public void SetVerticalStiffness(ref List<double> defl, ref List<double> frc) {
                // for numerical reasons we scale down the force values form N to kN
                double N2kN = 0.001;
                // polynomial regression of the type y = a*t + b*t^2
                // at least 3 table entries, no identical pairs, no 0,0 pair
                double sat2 = 0.0, sat3 = 0.0, sbt3, sbt4 = 0.0, syt = 0.0, syt2 = 0.0;

                m_tire_test_defl.Resize(defl.Count);
                m_tire_test_frc.Resize(defl.Count);

                for (int i = 0; i < defl.Count; i++)
                {
                    m_tire_test_defl[i] = defl[i];  // needed for plotting
                    m_tire_test_frc[i] = frc[i];
                    sat2 += defl[i] * defl[i];
                    sat3 += defl[i] * defl[i] * defl[i];
                    sbt4 += defl[i] * defl[i] * defl[i] * defl[i];
                    syt += N2kN * frc[i] * defl[i];
                    syt2 += N2kN * frc[i] * defl[i] * defl[i];
                }
                sbt3 = sat3;

                double[,] A = new double[2,2];
                double[,] Aa = new double[2,2];
                double[,] Ab = new double[2,2];
                A[0,0] = sat2;
                A[0,1] = sbt3;
                A[1,0] = sat3;
                A[1,1] = sbt4;

                Aa[0,0] = syt;
                Aa[0,1] = sbt3;
                Aa[1,0] = syt2;
                Aa[1,1] = sbt4;

                Ab[0,0] = sat2;
                Ab[0,1] = syt;
                Ab[1,0] = sat3;
                Ab[1,1] = syt2;

                double dA = A[0,0] * A[1,1] - A[1,0] * A[0,1];

                if (dA == 0.0)
                {
                    Debug.Log( "There is a problem with the Force/Deflection Table of the Tire!");
                    return;
                }

                double a = (Aa[0,0] * Aa[1,1] - Aa[1,0] * Aa[0,1]) / dA;
                double b = (Ab[0,0] * Ab[1,1] - Ab[1,0] * Ab[0,1]) / dA;

                // set stiffness ploynomial coefficients
                m_a1 = a;
                m_a2 = b;
            }

            /// Set the tire reference coefficient of friction.
            public void SetFrictionCoefficient(double coeff) {
                m_TMeasyCoeff.mu_0 = coeff;
            }

            /// Set rolling resistance coefficients.
            public void SetRollingResistanceCoefficients(double rr_coeff_1, double rr_coeff_2) {
                m_TMeasyCoeff.rrcoeff_pn = rr_coeff_1;
                m_TMeasyCoeff.rrcoeff_p2n = rr_coeff_2;
            }

            /// Set dynamic radius coefficients.
            public void SetDynamicRadiusCoefficients(double rdyn_coeff_1, double rdyn_coeff_2) {
                m_TMeasyCoeff.rdynco_pn = rdyn_coeff_1;
                m_TMeasyCoeff.rdynco_p2n = rdyn_coeff_2;
            }

            /// Generate basic tire plots.
            /// This function creates a Gnuplot script file with the specified name.
            // public void WritePlots(const std::string& plFileName, const std::string& plTireFormat);

            /// Get the tire deflection
            public override double GetDeflection() { return m_data.depth; }

            /// Using tire relaxation, we have three tire deflections
            public ChVector GetDeflection2() { return new ChVector(m_states.xe, m_states.ye, m_data.depth); }

            /// Export a TMeasy Tire Parameter File
            // public void ExportParameterFile(std::string fileName);

            /// Export a TMeasy Tire Parameter File in JSON format
            //void ExportJSONFile(std::string jsonFileName);

            /// Simple parameter consistency test
            public bool CheckParameters() {
                bool isOk = false;

                // Nominal Load set?
                if (m_TMeasyCoeff.pn < GetTireMaxLoad(0))
                {
                    // std::cout << "TMeasyCheckParameters(): Tire Nominal Load Problem!" << std::endl;
                    Debug.Log("TMeasyCheckParameters(): Tire Nominal Load Problem!");
                    return isOk;
                }

                // Stiffness parameters, spring
                if (m_a1 <= 0.0)
                {
                    //std::cout << "TMeasyCheckParameters(): Tire Vertical Stiffness Problem!" << std::endl;
                    Debug.Log("TMeasyCheckParameters(): Tire Vertical Stiffness Problem!");
                    return isOk;
                }

                // Stiffness parameters, spring
                if (m_TMeasyCoeff.mu_0 <= 0.0)
                {
                    Debug.Log("TMeasyCheckParameters(): Friction Coefficien Mu_0 unset!");
                    return isOk;
                }

                if (m_TMeasyCoeff.dz <= 0.0)
                {
                    Debug.Log("TMeasyCheckParameters(): Tire Vertical Damping Problem!");
                    return isOk;
                }

                // Stiffness sx
                if (m_TMeasyCoeff.dfx0_pn <= 0.0 || m_TMeasyCoeff.dfx0_pn < (2.0 * m_TMeasyCoeff.fxm_pn / m_TMeasyCoeff.sxm_pn))
                {
                    Debug.Log("TMeasyCheckParameters(): fx(sx) slope at sx=0 too low, load level 1!");
                    Debug.Log(m_TMeasyCoeff.dfx0_pn + " < " + (2.0 * m_TMeasyCoeff.fxm_pn / m_TMeasyCoeff.sxm_pn));
                    return isOk;
                }
                if (m_TMeasyCoeff.dfx0_p2n <= 0.0 ||
                    m_TMeasyCoeff.dfx0_p2n < (2.0 * m_TMeasyCoeff.fxm_p2n / m_TMeasyCoeff.sxm_p2n))
                {
                    //std::cout << "TMeasyCheckParameters(): fx(sx) slope at sx=0 too low, load level 2!" << std::endl;
                    Debug.Log("TMeasyCheckParameters(): fx(sx) slope at sx=0 too low, load level 2!");
                    return isOk;
                }

                // Stiffness sy
                if (m_TMeasyCoeff.dfy0_pn <= 0.0 || m_TMeasyCoeff.dfy0_pn < (2.0 * m_TMeasyCoeff.fym_pn / m_TMeasyCoeff.sym_pn))
                {
                    // std::cout << "TMeasyCheckParameters(): fy(sy) slope at sy=0 too low, load level 1!" << std::endl;
                    Debug.Log("TMeasyCheckParameters(): fy(sy) slope at sy=0 too low, load level 1!");
                    return isOk;
                }
                if (m_TMeasyCoeff.dfy0_p2n <= 0.0 ||
                    m_TMeasyCoeff.dfy0_p2n < (2.0 * m_TMeasyCoeff.fym_p2n / m_TMeasyCoeff.sym_p2n))
                {
                    // std::cout << "TMeasyCheckParameters(): fy(sy) slope at sy=0 too low, load level 2!" << std::endl;
                    Debug.Log("TMeasyCheckParameters(): fy(sy) slope at sy=0 too low, load level 2!");
                    return isOk;
                }

                // Check single curve parameters
                if (m_TMeasyCoeff.sxm_pn <= 0.0)
                {
                    // std::cout << "TMeasyCheckParameters(): sxm load level 1 not set!" << std::endl;
                    Debug.Log("TMeasyCheckParameters(): sxm load level 1 not set!");
                    return isOk;
                }
                if (m_TMeasyCoeff.sxm_p2n <= 0.0)
                {
                    // std::cout << "TMeasyCheckParameters(): sxm load level 2 not set!" << std::endl;
                    Debug.Log("TMeasyCheckParameters(): sxm load level 2 not set!");
                    return isOk;
                }
                if (m_TMeasyCoeff.fxm_pn <= 0.0)
                {
                    // std::cout << "TMeasyCheckParameters(): fxm load level 1 not set!" << std::endl;
                    Debug.Log("TMeasyCheckParameters(): fxm load level 1 not set!");
                    return isOk;
                }
                if (m_TMeasyCoeff.fxm_p2n <= 0.0)
                {
                    // std::cout << "TMeasyCheckParameters(): fxm load level 2 not set!" << std::endl;
                    Debug.Log("TMeasyCheckParameters(): fxm load level 2 not set!");
                    return isOk;
                }

                if (m_TMeasyCoeff.sym_pn <= 0.0)
                {
                    //std::cout << "TMeasyCheckParameters(): sym load level 1 not set!" << std::endl;
                    Debug.Log("TMeasyCheckParameters(): sym load level 1 not set!");
                    return isOk;
                }
                if (m_TMeasyCoeff.sym_p2n <= 0.0)
                {
                    //std::cout << "TMeasyCheckParameters(): sym load level 2 not set!" << std::endl;
                    Debug.Log("TMeasyCheckParameters(): sym load level 2 not set!");
                    return isOk;
                }
                if (m_TMeasyCoeff.fym_pn <= 0.0)
                {
                    // std::cout << "TMeasyCheckParameters(): fym load level 1 not set!" << std::endl;
                    Debug.Log("TMeasyCheckParameters(): fym load level 1 not set!");
                    return isOk;
                }
                if (m_TMeasyCoeff.fym_p2n <= 0.0)
                {
                    // std::cout << "TMeasyCheckParameters(): fym load level 2 not set!" << std::endl;
                    Debug.Log("TMeasyCheckParameters(): fym load level 2 not set!");
                    return isOk;
                }

                if (m_TMeasyCoeff.sxm_pn >= m_TMeasyCoeff.sxs_pn)
                {
                    //std::cout << "TMeasyCheckParameters(): sxm >= sxs load level 1!" << std::endl;
                    Debug.Log("TMeasyCheckParameters(): sxm >= sxs load level 1!");
                    return isOk;
                }
                if (m_TMeasyCoeff.sxm_p2n >= m_TMeasyCoeff.sxs_p2n)
                {
                    // std::cout << "TMeasyCheckParameters(): sxm >= sxs load level 2!" << std::endl;
                    Debug.Log("TMeasyCheckParameters(): sxm >= sxs load level 2!");
                    return isOk;
                }

                if (m_TMeasyCoeff.sym_pn >= m_TMeasyCoeff.sys_pn)
                {
                    // std::cout << "TMeasyCheckParameters(): sym >= sys load level 1!" << std::endl;
                    Debug.Log("TMeasyCheckParameters(): sym >= sys load level 1!");
                    return isOk;
                }
                if (m_TMeasyCoeff.sym_p2n >= m_TMeasyCoeff.sys_p2n)
                {
                    // std::cout << "TMeasyCheckParameters(): sym >= sys load level 2!" << std::endl;
                    Debug.Log("TMeasyCheckParameters(): sym >= sys load level 2!");
                    return isOk;
                }

                isOk = true;

                return isOk;
            }


            /// Perform disc-terrain collision detection considering the curvature of the road
            /// surface. The surface normal is calculated based on 4 different height values below
            /// the wheel center. The effective height is calculated as average value of the four
            /// height values.
            /// This utility function checks for contact between a disc of specified
            /// radius with given position and orientation (specified as the location of
            /// its center and a unit vector normal to the disc plane) and the terrain
            /// system associated with this tire. It returns true if the disc contacts the
            /// terrain and false otherwise.  If contact occurs, it returns a coordinate
            /// system with the Z axis along the contact normal and the X axis along the
            /// "rolling" direction, as well as a positive penetration depth (i.e. the
            /// height below the terrain of the lowest point on the disc).
            protected bool disc_terrain_contact_3d(
                                            RigidTerrain terrain,       ///< [in] reference to terrain system
                                            ChVector disc_center,  ///< [in] global location of the disc center
                                            ChVector disc_normal,  ///< [in] disc normal, expressed in the global frame
                                            double disc_radius,             ///< [in] disc radius
                                            ref ChCoordsys contact,          ///< [out] contact coordinate system (relative to the global frame)
                                            ref double depth                   ///< [out] penetration depth (positive if contact occurred)
            )
            {
                double dx = 0.1 * m_unloaded_radius;
                double dy = 0.3 * m_width;

                // Find terrain height below disc center. There is no contact if the disc
                // center is below the terrain or farther away by more than its radius.
                double hc = terrain.GetHeight(disc_center.x, disc_center.z);
                if (disc_center.y <= hc || disc_center.y >= hc + disc_radius)
                    return false;

                // Find the lowest point on the disc. There is no contact if the disc is
                // (almost) horizontal.
                ChVector dir1 = ChVector.Vcross(disc_normal, new ChVector(0, 1, 0));
                double sinTilt2 = dir1.Length2();

                if (sinTilt2 < 1e-3)
                    return false;

                // Contact point (lowest point on disc).
                ChVector ptD = disc_center + disc_radius * ChVector.Vcross(disc_normal, dir1 / Math.Sqrt(sinTilt2));

                // Approximate the terrain with a plane. Define the projection of the lowest
                // point onto this plane as the contact point on the terrain.
                ChVector normal = terrain.GetNormal(ptD.x, ptD.z);
                ChVector longitudinal = ChVector.Vcross(disc_normal, normal);
                longitudinal.Normalize();
                ChVector lateral = ChVector.Vcross(normal, longitudinal);

                // Calculate four contact points in the contact patch
                ChVector ptQ1 = ptD + dx * longitudinal;
                ptQ1.z = terrain.GetHeight(ptQ1.x, ptQ1.z);

                ChVector ptQ2 = ptD - dx * longitudinal;
                ptQ2.z = terrain.GetHeight(ptQ2.x, ptQ2.z);

                ChVector ptQ3 = ptD + dy * lateral;
                ptQ3.z = terrain.GetHeight(ptQ3.x, ptQ3.z);

                ChVector ptQ4 = ptD - dy * lateral;
                ptQ4.z = terrain.GetHeight(ptQ4.x, ptQ4.z);

                // Calculate a smoothed road surface normal
                ChVector rQ2Q1 = ptQ1 - ptQ2;
                ChVector rQ4Q3 = ptQ3 - ptQ4;

                ChVector terrain_normal = ChVector.Vcross(rQ2Q1, rQ4Q3);
                terrain_normal.Normalize();

                // Find terrain height as average of four points. No contact if lowest point is above
                // the terrain.
                ptD = 0.25 * (ptQ1 + ptQ2 + ptQ3 + ptQ4);
                ChVector d = ptD - disc_center;
                double da = d.Length();

                if (da >= m_unloaded_radius)
                    return false;

                // Calculate an improved value for the camber angle
                m_gamma = Math.Sin(ChVector.Vdot(disc_normal, terrain_normal));

               // ChMatrix33<double> rot = new ChMatrix33<double>(0); // Nest for performance?
                rot.Set_A_axis(longitudinal, lateral, terrain_normal);

                contact.pos = ptD;
                contact.rot = rot.Get_A_quaternion();

                depth = m_unloaded_radius - da;
                Debug.Assert(depth > 0);

                return true;
            }

           

                /// Return the vertical tire stiffness contribution to the normal force.
                protected double GetNormalStiffnessForce(double depth) {
                return m_TMeasyCoeff.cz * depth;
            }

            /// Return the vertical tire damping contribution to the normal force.
            protected double GetNormalDampingForce(double depth, double velocity) {
                return m_TMeasyCoeff.dz * velocity;
            }

            /// Set the parameters in the TMeasy model.
            protected virtual void SetTMeasyParams() {
                // Tire Size = 37 x 12.5 x 16.5 Load Range D
                // Tire Load 3850 lbs at 50 psi (Goodyear Military Tire Brochure 6th Edition)

                const double lbs2N = 4.4482216153;
                int li = 108;  // guessed from load spec. of the vehicle
                const double in2m = 0.0254;
                double h = (wheelDiametre - rimDiameter) * in2m / 2.0;
                double w = 12.5 * in2m;
                double r = h / w;
                double rimdia = rimDiameter * in2m;

                double load = 3850.0 * lbs2N;

                GuessTruck80Par(load,   // tire load [N]
                                w,      // tire width [m]
                                r,      // aspect ratio []
                                rimdia  // rim diameter [m]
                );
            }

            protected bool m_consider_relaxation;

            protected bool m_use_Reff_fallback_calculation;

            protected int m_integration_method;

            protected double m_time;
            protected double m_begin_start_transition;
            protected double m_end_start_transition;

            protected double m_vnum;

            protected double m_gamma;  //< actual camber angle

            protected double m_gamma_limit;  //< limit camber angle (degrees!)

            // TMeasy tire model parameters
            protected double m_unloaded_radius;     //< reference tire radius
            protected double m_width;               //< tire width
            protected double m_rim_radius;          //< tire rim radius
            protected double m_roundness;           //< roundness factor for cross-section profile
            protected double m_rolling_resistance;  //< actual rolling friction coeff
            protected double m_mu;                  //< local friction coefficient of the road

            protected double m_a1;  ///< polynomial coefficient a1 in cz = a1 + 2.0*a2 * deflection
            double m_a2;  ///< polynomial coefficient a2 in cz = a1 + 2.0*a2 * deflection

            protected double m_tau_x;  ///< Longitudinal relaxation delay time
            protected double m_tau_y;  ///< Lateral relaxation delay time

            protected double m_relaxation_lenght_x;    //< Longitudinal relaxation length
            protected double m_relaxation_lenght_y;    //< Lateral relaxation length
            protected double m_relaxation_lenght_phi;  //< Relaxation length for bore movement

            protected double m_rdynco;          //< actual value of dynamic rolling radius weighting coefficient
            protected double m_rdynco_crit;     //< max. considered value of m_rdynco (local minimum of dynamic rolling radius)
            protected double m_fz_rdynco_crit;  //< Fz value r_dyn = r_dyn(m_fz_rdynco,m_rdynco_crit)

            //VehicleSide m_measured_side;

            public struct TMeasyCoeff
            {
                public double pn;      //< Nominal vertical force [N]
                public double pn_max;  //< Maximum vertical force [N]

                public double mu_0;  //< Local friction coefficient of the road for given parameters
                public double cx;    //< Linear stiffness x [N/m]
                public double cy;    //< Linear stiffness y [N/m]
                public double cz;    //< Stiffness, may vary with the vertical force [N/m]
                public double dx;    //< Linear damping coefficient x [Ns/m]
                public double dy;    //< Linear damping coefficient y [Ns/m]
                public double dz;    //< Linear damping coefficient z [Ns/m]

                public double dfx0_pn, dfx0_p2n;  //< Initial longitudinal slopes dFx/dsx [kN]
                public double fxm_pn, fxm_p2n;    //< Maximum longitudinal force [kN]
                public double fxs_pn, fxs_p2n;    //< Longitudinal load at sliding [kN]
                public double sxm_pn, sxm_p2n;    //< Slip sx at maximum longitudinal load Fx
                public double sxs_pn, sxs_p2n;    //< Slip sx where sliding begins

                public double dfy0_pn, dfy0_p2n;  //< Initial lateral slopes dFy/dsy [kN]
                public double fym_pn, fym_p2n;    //< Maximum lateral force [kN]
                public double fys_pn, fys_p2n;    //< Lateral load at sliding [kN]
                public double sym_pn, sym_p2n;    //< Slip sy at maximum lateral load Fy
                public double sys_pn, sys_p2n;    //< Slip sy where sliding begins

                public double nto0_pn, nto0_p2n;      //< Normalized pneumatic trail at sy=0
                public double synto0_pn, synto0_p2n;  //< Slip sy where trail changes sign
                public double syntoE_pn, syntoE_p2n;  //< Slip sy where trail tends to zero

                public double rrcoeff_pn, rrcoeff_p2n;  //< Rolling resistance coefficients
                public double rdynco_pn, rdynco_p2n;    //< Dynamic radius weighting coefficients

            }

            protected TMeasyCoeff m_TMeasyCoeff;

            // linear Interpolation
            protected double InterpL(double fz, double w1, double w2) { return w1 + (w2 - w1) * (fz / m_TMeasyCoeff.pn - 1.0); }
            // quadratic Interpolation
            protected double InterpQ(double fz, double w1, double w2)
            {
                return (fz / m_TMeasyCoeff.pn) * (2.0 * w1 - 0.5 * w2 - (w1 - 0.5 * w2) * (fz / m_TMeasyCoeff.pn));
            }


            private void UpdateVerticalStiffness() {
                const double kN2N = 1000.0;
                m_TMeasyCoeff.cz = kN2N * (m_a1 + 2.0 * m_a2 * m_data.depth);
            }

            private List<double> m_tire_test_defl = new List<double>();  // set, when test data are used for vertical
            private List<double> m_tire_test_frc = new List<double>();   // stiffness calculation

            private void tmxy_combined(ref double f, ref double fos, double s, double df0, double sm, double fm, double ss, double fs) {
                const double kN2N = 1000.0;
                double df0loc = 0.0;
                if (sm > 0.0)
                {
                    df0loc = Math.Max(2.0 * fm / sm, df0);
                }

                if (s > 0.0 && df0loc > 0.0)
                {  // normal operating conditions
                    if (s > ss)
                    {               // full sliding
                        f = fs;
                        fos = f / s;
                    }
                    else
                    {
                        if (s < sm)
                        {  // adhesion
                            double p = df0loc * sm / fm - 2.0;
                            double sn = s / sm;
                            double dn = 1.0 + (sn + p) * sn;
                            f = df0loc * sm * sn / dn;
                            fos = df0loc / dn;
                        }
                        else
                        {
                            double a = Math.Pow(fm / sm, 2.0) / (df0loc * sm);  // parameter from 2. deriv. of f @ s=sm
                            double sstar = sm + (fm - fs) / (a * (ss - sm));    // connecting point
                            if (sstar <= ss)
                            {                                  // 2 parabolas
                                if (s <= sstar)
                                {
                                    // 1. parabola sm < s < sstar
                                    f = fm - a * (s - sm) * (s - sm);
                                }
                                else
                                {
                                    // 2. parabola sstar < s < ss
                                    double b = a * (sstar - sm) / (ss - sstar);
                                    f = fs + b * (ss - s) * (ss - s);
                                }
                            }
                            else
                            {
                                // cubic fallback function
                                double sn = (s - sm) / (ss - sm);
                                f = fm - (fm - fs) * sn * sn * (3.0 - 2.0 * sn);
                            }
                            fos = f / s;
                        }
                    }
                }
                else
                {
                    f = 0.0;
                    fos = 0.0;
                }

                // scale up from kN to N
                f *= kN2N;
                fos *= kN2N;
            }
            private double tmy_tireoff(double sy, double nto0, double synto0, double syntoE) {
                double nto = 0.0;

                double sy_a = Mathfx.Abs(sy);  // absolute slip value

                double syntoE_loc = Math.Max(syntoE, synto0);  // ensure appropriate data

                if (sy_a >= syntoE_loc)
                {
                    nto = 0.0;  //   very high slip values --> pure sliding
                }
                else
                {
                    double wf = synto0 / syntoE_loc;  // weighting function for 2 approximations
                    if (sy_a <= synto0)
                    {             // low and moderate slip values
                        double sy_n = sy_a / synto0;
                        double nto1 = nto0 * (1.0 - sy_n);
                        double nto2 = nto0 * (1.0 - (3.0 - 2.0 * sy_n) * sy_n * sy_n);
                        nto = (1.0 - wf) * nto1 + wf * nto2;
                    }
                    else
                    {  //  high slip values
                        double sy_n = (syntoE_loc - sy_a) / (syntoE_loc - synto0);
                        nto = -nto0 * (1.0 - wf) * (sy_a - synto0) / synto0 * sy_n * sy_n;
                    }
                }

                return nto;
            }

          /*  public struct ContactData
            {
                public bool in_contact;      // true if disc in contact with terrain
                public ChCoordsys frame;   // contact frame (x: long, y: lat, z: normal)
                public ChVector vel;       // relative velocity expressed in contact frame
                public double normal_force;  // magnitude of normal contact force
                public double depth;         // penetration depth
            };*/

           /* public struct TireStates
            {
                public double sx;               // Contact Path - Longitudinal Slip State (Kappa)
                public double sy;               // Contact Path - Side Slip State (Alpha)
                public double vta;              // absolut transport velocity
                public double vsx;              // Longitudinal slip velocity
                public double vsy;              // Lateral slip velocity = Lateral velocity
                public double omega;            // Wheel angular velocity about its spin axis
                public double R_eff;            // Effective Rolling Radius
                public double Fx_dyn;           // Dynamic longitudinal fire force
                public double Fy_dyn;           // Dynamic lateral tire force
                public double Mb_dyn;           // Dynamic bore torque
                public double xe;               // Longitudinal tire deflection
                public double ye;               // Lateral tire deflection
                public double Fx;               // Steady state longitudinal tire force
                public double Fy;               // Steady state lateral tire force
                public double Mb;               // Steady state bore torque
                public ChVector disc_normal;  // (temporary for debug)
            };*/

            //private ContactData m_data;
            protected RigidTerrain m_terrain;
            // private TireStates m_states;

            // private ChSubsysDefs.TerrainForce m_tireforce;


        }
    }
}
