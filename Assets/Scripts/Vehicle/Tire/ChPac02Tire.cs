using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
using UnityEditor;
using UnityEngine.Assertions;

namespace chrono
{
    namespace vehicle
    {
        // Template for a tire model based on the Pacejka 2002 Tire Model
        public class ChPac02Tire : ChTire
        {
            public Vector3 tireForce = new Vector3();
            public Vector3 tireMoment = new Vector3();
            public double debug;
            public Quaternion rotationQuat;
            public Vector3 rotationEul;

            public double E;

            public ChPac02Tire() : base()
            {
               // m_areaDep = new ChFunction_Recorder();

                m_use_vert_map = false;

                m_data = new ContactData();
                m_data.in_contact = false;
                m_data.frame = new ChCoordsys();
                m_data.vel = new ChVector();
                m_data.normal_force = 0;
                m_data.depth = 0;

                m_kappa = 0;
                m_alpha = 0;
                m_gamma = 0;
                m_gamma_limit = 3.0 * ChMaths.CH_C_DEG_TO_RAD;
                m_mu = 0;

                m_use_mode = 1;
                m_Shf = 0;

                m_use_friction_ellipsis = false;
                m_allow_mirroring = false;
                m_measured_side = ChSubsysDefs.VehicleSide.LEFT;
                m_tireforce.force = new ChVector(0, 0, 0);
                m_tireforce.point = new ChVector(0, 0, 0);
                m_tireforce.moment = new ChVector(0, 0, 0);

                // standard settings for scaling factors
                m_PacScal.lfz0 = 1.0;
                m_PacScal.lcx = 1.0;
                m_PacScal.lex = 1.0;
                m_PacScal.lkx = 1.0;
                m_PacScal.lhx = 1.0;
                m_PacScal.lmux = 1.0;
                m_PacScal.lvx = 1.0;
                m_PacScal.lxal = 1.0;
                m_PacScal.lcy = 1.0;
                m_PacScal.ley = 1.0;
                m_PacScal.lhy = 1.0;
                m_PacScal.lky = 1.0;
                m_PacScal.lmuy = 1.0;
                m_PacScal.lvy = 1.0;
                m_PacScal.lyka = 1.0;
                m_PacScal.lvyka = 1.0;
                m_PacScal.ltr = 1.0;
                m_PacScal.lgax = 1.0;
                m_PacScal.lgay = 1.0;
                m_PacScal.lgaz = 1.0;
                m_PacScal.lres = 1.0;
                m_PacScal.ls = 1.0;
                m_PacScal.lsgkp = 1.0;
                m_PacScal.lsgal = 1.0;
                m_PacScal.lgyr = 1.0;

                m_PacCoeff.mu0 = 0.8;           // reference friction coefficient
                m_PacCoeff.R0 = 0.0;            // unloaded radius
                m_PacCoeff.width = 0.0;         // tire width = 0.0;
                m_PacCoeff.aspect_ratio = 0.8;  // actually unused
                m_PacCoeff.rim_radius = 0.0;    // actually unused
                m_PacCoeff.rim_width = 0.0;     // actually unused
                m_PacCoeff.FzNomin = 0.0;       // nominla wheel load
                m_PacCoeff.Cz = 0.0;            // vertical tire stiffness
                m_PacCoeff.Kz = 0.0;            // vertical tire damping

                // Longitudinal Coefficients
                m_PacCoeff.pcx1 = 0.0;
                m_PacCoeff.pdx1 = 0.0;
                m_PacCoeff.pdx2 = 0.0;
                m_PacCoeff.pdx3 = 0.0;
                m_PacCoeff.pex1 = 0.0;
                m_PacCoeff.pex2 = 0.0;
                m_PacCoeff.pex3 = 0.0;
                m_PacCoeff.pex4 = 0.0;
                m_PacCoeff.phx1 = 0.0;
                m_PacCoeff.phx2 = 0.0;
                m_PacCoeff.pkx1 = 0.0;
                m_PacCoeff.pkx2 = 0.0;
                m_PacCoeff.pkx3 = 0.0;
                m_PacCoeff.pvx1 = 0.0;
                m_PacCoeff.pvx2 = 0.0;
                m_PacCoeff.rcx1 = 0.0;
                m_PacCoeff.rbx1 = 0.0;
                m_PacCoeff.rbx2 = 0.0;
                m_PacCoeff.rbx3 = 0.0;
                m_PacCoeff.rex1 = 0.0;
                m_PacCoeff.rex2 = 0.0;
                m_PacCoeff.rhx1 = 0.0;
                m_PacCoeff.ptx1 = 0.0;
                m_PacCoeff.ptx2 = 0.0;
                m_PacCoeff.ptx3 = 0.0;
                m_PacCoeff.ptx4 = 0.0;

                // overturning coefficients
                m_PacCoeff.qsx1 = 0.0;
                m_PacCoeff.qsx2 = 0.0;
                m_PacCoeff.qsx3 = 0.0;
                m_PacCoeff.qsx4 = 0.0;
                m_PacCoeff.qsx5 = 0.0;
                m_PacCoeff.qsx6 = 0.0;
                m_PacCoeff.qsx7 = 0.0;
                m_PacCoeff.qsx8 = 0.0;
                m_PacCoeff.qsx9 = 0.0;
                m_PacCoeff.qsx10 = 0.0;
                m_PacCoeff.qsx11 = 0.0;

                // rolling coefficients
                m_PacCoeff.qsy1 = 0.0;
                m_PacCoeff.qsy2 = 0.0;
                m_PacCoeff.qsy3 = 0.0;
                m_PacCoeff.qsy4 = 0.0;
                m_PacCoeff.qsy5 = 0.0;
                m_PacCoeff.qsy6 = 0.0;
                m_PacCoeff.qsy7 = 0.0;
                m_PacCoeff.qsy8 = 0.0;

                // Lateral Coefficients
                m_PacCoeff.pcy1 = 0.0;
                m_PacCoeff.pdy1 = 0.0;
                m_PacCoeff.pdy2 = 0.0;
                m_PacCoeff.pdy3 = 0.0;
                m_PacCoeff.pey1 = 0.0;
                m_PacCoeff.pey2 = 0.0;
                m_PacCoeff.pey3 = 0.0;
                m_PacCoeff.pey4 = 0.0;
                m_PacCoeff.pey5 = 0.0;
                m_PacCoeff.phy1 = 0.0;
                m_PacCoeff.phy2 = 0.0;
                m_PacCoeff.phy3 = 0.0;
                m_PacCoeff.pky1 = 0.0;
                m_PacCoeff.pky2 = 0.0;
                m_PacCoeff.pky3 = 0.0;
                m_PacCoeff.pvy1 = 0.0;
                m_PacCoeff.pvy2 = 0.0;
                m_PacCoeff.pvy3 = 0.0;
                m_PacCoeff.pvy4 = 0.0;
                m_PacCoeff.rby1 = 0.0;
                m_PacCoeff.rby2 = 0.0;
                m_PacCoeff.rby3 = 0.0;
                m_PacCoeff.rby4 = 0.0;
                m_PacCoeff.rcy1 = 0.0;
                m_PacCoeff.rey1 = 0.0;
                m_PacCoeff.rey2 = 0.0;
                m_PacCoeff.rhy1 = 0.0;
                m_PacCoeff.rhy2 = 0.0;
                m_PacCoeff.rvy1 = 0.0;
                m_PacCoeff.rvy2 = 0.0;
                m_PacCoeff.rvy3 = 0.0;
                m_PacCoeff.rvy4 = 0.0;
                m_PacCoeff.rvy5 = 0.0;
                m_PacCoeff.rvy6 = 0.0;
                m_PacCoeff.pty1 = 0.0;
                m_PacCoeff.pty2 = 0.0;

                // alignment coefficients
                m_PacCoeff.qbz1 = 0.0;
                m_PacCoeff.qbz2 = 0.0;
                m_PacCoeff.qbz3 = 0.0;
                m_PacCoeff.qbz4 = 0.0;
                m_PacCoeff.qbz5 = 0.0;
                m_PacCoeff.qbz6 = 0.0;
                m_PacCoeff.qbz9 = 0.0;
                m_PacCoeff.qbz10 = 0.0;
                m_PacCoeff.qcz1 = 0.0;
                m_PacCoeff.qdz1 = 0.0;
                m_PacCoeff.qdz2 = 0.0;
                m_PacCoeff.qdz3 = 0.0;
                m_PacCoeff.qdz4 = 0.0;
                m_PacCoeff.qdz5 = 0.0;
                m_PacCoeff.qdz6 = 0.0;
                m_PacCoeff.qdz7 = 0.0;
                m_PacCoeff.qdz8 = 0.0;
                m_PacCoeff.qdz9 = 0.0;
                m_PacCoeff.qez1 = 0.0;
                m_PacCoeff.qez2 = 0.0;
                m_PacCoeff.qez3 = 0.0;
                m_PacCoeff.qez4 = 0.0;
                m_PacCoeff.qez5 = 0.0;
                m_PacCoeff.qhz1 = 0.0;
                m_PacCoeff.qhz2 = 0.0;
                m_PacCoeff.qhz3 = 0.0;
                m_PacCoeff.qhz4 = 0.0;
                m_PacCoeff.ssz1 = 0.0;
                m_PacCoeff.ssz2 = 0.0;
                m_PacCoeff.ssz3 = 0.0;
                m_PacCoeff.ssz4 = 0.0;
                m_PacCoeff.qtz1 = 0.0;
                m_PacCoeff.mbelt = 0.0;
            }

            // Use this for initialization
            void Start()
            {
                Assert.raiseExceptions = true;

              //  m_areaDep = gameObject.AddComponent<ChFunction_Recorder>();

               // m_system = GameObject.FindObjectOfType<ChSystem>();
                m_wheel = GetComponent<ChBody>();
                m_terrain = GameObject.FindObjectOfType<RigidTerrain>();

                Initialize(m_wheel, m_side);
            }

            void OnDrawGizmos()
            {

            }

            // Update is called once per frame
            public override void FixedUpdate()
            {
                base.FixedUpdate();
              //  m_tireforce = GetTireForce();

                tireForce.x = (float)m_tireforce.force.x;
                tireForce.y = (float)m_tireforce.force.y;
                tireForce.z = (float)m_tireforce.force.z;
                tireMoment.x = (float)m_tireforce.moment.x;
                tireMoment.y = (float)m_tireforce.moment.y;
                tireMoment.z = (float)m_tireforce.moment.x;

               /* rotationQuat.w = (float)GetSpindleRot().x;
                rotationQuat.x = (float)GetSpindleRot().y;
                rotationQuat.y = (float)GetSpindleRot().data[2];
                rotationQuat.z = (float)GetSpindleRot().data[3];*/
               // rotationEul = rotationQuat.eulerAngles;

                // Advance simulation for one timestep for all modules
                // double softstep = m_system.m_softstep;

                Synchronize(ChSystem.system.GetChTime(), m_wheelState, m_terrain);
                Advance(m_stepsize);

                debug = m_wheelState.omega;
            }


            /// Initialize this tire system.
            public override void Initialize(ChBody wheel,  //< [in] associated wheel body
                                ChSubsysDefs.VehicleSide side                //< [in] left/right vehicle side
                                )
            {
                SetPac02Params();

                // Build the lookup table for penetration depth as function of intersection area
                // (used only with the ChTire::ENVELOPE method for terrain-tire collision detection)
               // ConstructAreaDepthTable(m_PacCoeff.R0, ref m_areaDep);

                // all parameters are known now pepare mirroring
                if (m_allow_mirroring)
                {

                    if (GetSide() != m_measured_side)
                    {
                        // we flip the sign of some parameters
                        m_PacCoeff.rhx1 *= -1.0;
                        m_PacCoeff.qsx1 *= -1.0;
                        m_PacCoeff.pey3 *= -1.0;
                        m_PacCoeff.phy1 *= -1.0;
                        m_PacCoeff.phy2 *= -1.0;
                        m_PacCoeff.pvy1 *= -1.0;
                        m_PacCoeff.pvy2 *= -1.0;
                        m_PacCoeff.rby3 *= -1.0;
                        m_PacCoeff.rvy1 *= -1.0;
                        m_PacCoeff.rvy2 *= -1.0;
                        m_PacCoeff.qbz4 *= -1.0;
                        m_PacCoeff.qdz3 *= -1.0;
                        m_PacCoeff.qdz6 *= -1.0;
                        m_PacCoeff.qdz7 *= -1.0;
                        m_PacCoeff.qez4 *= -1.0;
                        m_PacCoeff.qhz1 *= -1.0;
                        m_PacCoeff.qhz2 *= -1.0;
                        m_PacCoeff.ssz1 *= -1.0;

                        if (m_measured_side == ChSubsysDefs.VehicleSide.LEFT)
                        {
                            Debug.Log("Tire is measured as left tire but mounted on the right vehicle side . mirroring.\n");
                        }
                        else
                        {
                            Debug.Log("Tire is measured as right tire but mounted on the left vehicle side . mirroring.\n");
                        }
                    }

                    // Initialize contact patch state variables to 0
                    m_data.normal_force = 0;
                    m_states.R_eff = m_PacCoeff.R0;
                    m_states.cp_long_slip = 0;
                    m_states.cp_side_slip = 0;
                    m_states.vx = 0;
                    m_states.vsx = 0;
                    m_states.vsy = 0;
                    m_states.omega = 0;
                    m_states.disc_normal = new ChVector(0, 0, 0);
                }
            }

            /// Get the tire moments of inertia.
            /// Note that these should not include the inertia of the wheel (rim).
            public override ChVector GetInertia() { return new ChVector(); }

            public override double GetRadius() { return m_states.R_eff; }
           // public override double GetWidth() { return m_PacCoeff.width; }

            /// Return the vertical tire stiffness contribution to the normal force.
            public virtual double GetNormalStiffnessForce(double depth)
            {
                if (m_use_vert_map)
                    return m_vert_map.Get_y(depth);
                else
                    return depth * m_PacCoeff.Cz;
            }

            /// Report the tire force and moment.
            public override ChSubsysDefs.TerrainForce ReportTireForce(ChTerrain terrain) { return m_tireforce; }

            /// Get the tire mass.
            /// Note that this should not include the mass of the wheel (rim).
            public override double GetMass() { return 0; }

            /// Return the vertical tire damping contribution to the normal force.
            public virtual double GetNormalDampingForce(double depth, double velocity)
            {
                return m_PacCoeff.Kz * velocity;
            }

            /// Get the tire force and moment.
            /// This represents the output from this tire system that is passed to the
            /// vehicle system.  Typically, the vehicle subsystem will pass the tire force
            /// to the appropriate suspension subsystem which applies it as an external
            /// force one the wheel body.
            public override ChSubsysDefs.TerrainForce GetTireForce() { return m_tireforce; }

            /// Set the parameters in the Pac89 model.
            public virtual void SetPac02Params()
            {

                // begin of variables set up
                m_use_mode = 3;
                m_measured_side = ChSubsysDefs.VehicleSide.LEFT;
                m_allow_mirroring = false;
                m_PacCoeff.R0 = 0.3175;
                m_PacCoeff.width = 0.225;
                m_PacCoeff.aspect_ratio = 0.5;
                m_PacCoeff.rim_radius = 0.2159;
                m_PacCoeff.rim_width = 0.2;
                m_PacCoeff.Cz = 261600;
                m_PacCoeff.Kz = 1750.99;
                m_PacCoeff.FzNomin = 4700;
                // begin scaling factors set up
                m_PacScal.lfz0 = 1;
                m_PacScal.lcx = 1;
                m_PacScal.lmux = 1;
                m_PacScal.lex = 1;
                m_PacScal.lkx = 1;
                m_PacScal.lhx = 1;
                m_PacScal.lvx = 1;
                m_PacScal.lgax = 1;
                m_PacScal.lcy = 1;
                m_PacScal.lmuy = 1;
                m_PacScal.ley = 1;
                m_PacScal.lky = 1;
                m_PacScal.lhy = 1;
                m_PacScal.lvy = 1;
                m_PacScal.lgay = 1;
                m_PacScal.ltr = 1;
                m_PacScal.lres = 1;
                m_PacScal.lgaz = 1;
                m_PacScal.lxal = 1;
                m_PacScal.lyka = 1;
                m_PacScal.lvyka = 1;
                m_PacScal.ls = 1;
                m_PacScal.lsgkp = 1;
                m_PacScal.lsgal = 1;
                m_PacScal.lgyr = 1;
                m_PacScal.lmx = 1;
                m_PacScal.lvmx = 1;
                m_PacScal.lmy = 1;
                // setting longitidunal coefficients
                m_PacCoeff.pcx1 = 1.3178;
                m_PacCoeff.pdx1 = 1.0455;
                m_PacCoeff.pdx2 = 0.063954;
                m_PacCoeff.pdx3 = 0;
                m_PacCoeff.pex1 = 0.15798;
                m_PacCoeff.pex2 = 0.41141;
                m_PacCoeff.pex3 = 0.1487;
                m_PacCoeff.pex4 = 3.0004;
                m_PacCoeff.pkx1 = 23.181;
                m_PacCoeff.pkx2 = -0.037391;
                m_PacCoeff.pkx3 = 0.80348;
                m_PacCoeff.phx1 = -0.00058264;
                m_PacCoeff.phx2 = -0.0037992;
                m_PacCoeff.pvx1 = 0.045118;
                m_PacCoeff.pvx2 = 0.058244;
                m_PacCoeff.rbx1 = 13.276;
                m_PacCoeff.rbx2 = -13.778;
                m_PacCoeff.rcx1 = 1;
                m_PacCoeff.rex1 = 0;
                m_PacCoeff.rex2 = 0;
                m_PacCoeff.rhx1 = 0;
                // setting lateral coefficients
                m_PacCoeff.pcy1 = 1.2676;
                m_PacCoeff.pdy1 = 0.90031;
                m_PacCoeff.pdy2 = -0.16748;
                m_PacCoeff.pdy3 = -0.43989;
                m_PacCoeff.pey1 = -0.3442;
                m_PacCoeff.pey2 = -0.10763;
                m_PacCoeff.pey3 = 0.11513;
                m_PacCoeff.pey4 = -6.9663;
                m_PacCoeff.pky1 = -25.714;
                m_PacCoeff.pky2 = 3.2658;
                m_PacCoeff.pky3 = -0.0054467;
                m_PacCoeff.phy1 = 0.0031111;
                m_PacCoeff.phy2 = 2.1666e-05;
                m_PacCoeff.phy3 = 0.036592;
                m_PacCoeff.pvy1 = 0.0064945;
                m_PacCoeff.pvy2 = -0.0052059;
                m_PacCoeff.pvy3 = 0.013713;
                m_PacCoeff.pvy4 = -0.0092737;
                m_PacCoeff.rby1 = 7.1433;
                m_PacCoeff.rby2 = 9.1916;
                m_PacCoeff.rby3 = -0.027856;
                m_PacCoeff.rby1 = 1;
                m_PacCoeff.rey1 = 0;
                m_PacCoeff.rey2 = 0;
                m_PacCoeff.rhy1 = 0;
                m_PacCoeff.rhy2 = 0;
                m_PacCoeff.rvy1 = 0;
                m_PacCoeff.rvy2 = 0;
                m_PacCoeff.rvy3 = 0;
                m_PacCoeff.rvy4 = 0;
                m_PacCoeff.rvy5 = 1.9;
                m_PacCoeff.rvy6 = 0;
                // setting alignment coefficients
                m_PacCoeff.qbz1 = 5.6008;
                m_PacCoeff.qbz2 = -1.9968;
                m_PacCoeff.qbz3 = -0.58685;
                m_PacCoeff.qbz4 = -0.20922;
                m_PacCoeff.qbz5 = 0.2973;
                m_PacCoeff.qbz9 = 3.2333;
                m_PacCoeff.qbz10 = 0;
                m_PacCoeff.qcz1 = 1.0913;
                m_PacCoeff.qdz1 = 0.082536;
                m_PacCoeff.qdz2 = -0.011631;
                m_PacCoeff.qdz3 = -0.18704;
                m_PacCoeff.qdz4 = 0.18698;
                m_PacCoeff.qdz6 = 0.00071228;
                m_PacCoeff.qdz7 = 0.0010419;
                m_PacCoeff.qdz8 = -0.11886;
                m_PacCoeff.qdz9 = -0.011967;
                m_PacCoeff.qez1 = -3.25;
                m_PacCoeff.qez2 = -3.746;
                m_PacCoeff.qez3 = 0;
                m_PacCoeff.qez4 = 0.62393;
                m_PacCoeff.qez5 = -2.6405;
                m_PacCoeff.qhz1 = 0.0023279;
                m_PacCoeff.qhz2 = -0.0010156;
                m_PacCoeff.qhz3 = 0.030508;
                m_PacCoeff.qhz4 = 0.058344;
                m_PacCoeff.ssz1 = 0.05897546;
                m_PacCoeff.ssz2 = 0.0543624;
                m_PacCoeff.ssz3 = 0;
                m_PacCoeff.ssz4 = 0;
                // setting overturning coefficients
                m_PacCoeff.qsx1 = 0;
                m_PacCoeff.qsx2 = 0;
                m_PacCoeff.qsx3 = 0;
                // setting rolling coefficients
                m_PacCoeff.qsy1 = 0.012;
                m_PacCoeff.qsy2 = 0;
                m_PacCoeff.qsy3 = 0;
                m_PacCoeff.qsy4 = 0;
            }

            /// Update the state of this tire system at the current time.
            /// The tire system is provided the current state of its associated wheel.
            public override void Synchronize(double time,                    //< [in] current time
                                 ChSubsysDefs.WheelState wheel_state,  //< [in] current state of associated wheel body
                                 RigidTerrain terrain        //< [in] reference to the rigid terrain system
                                                                    // ChTerrain  TODO, need to change to this 'evertually' for soft terrain.
                                 )
            {
                // SubsysDefs.WheelState wheel_state = m_wheel.GetState();
                CalculateKinematics(time, wheel_state, m_terrain);

                // Get mu at wheel location
                m_mu = 0.8;//m_terrain.GetCoefficientFriction(wheel_state.pos.data[0], wheel_state.pos.data[1]);

                // Extract the wheel normal (expressed in global frame)
                ChMatrix33<double> A = new ChMatrix33<double>(wheel_state.rot);
                ChVector disc_normal = A.Get_A_Yaxis();

                double dum_cam = 0;

                // Assuming the tire is a disc, check contact with terrain
               /* switch (m_collision_type)
                {

                    case ChTire.CollisionType.SINGLE_POINT:*/
                        m_data.in_contact =
                            DiscTerrainCollision(m_terrain, wheel_state.pos, disc_normal, m_PacCoeff.R0, ref m_data.frame, ref m_data.depth);
                      /*  break;
                    case ChTire.CollisionType.FOUR_POINTS:
                        m_data.in_contact = DiscTerrainCollision4pt(m_terrain, wheel_state.pos, disc_normal, m_PacCoeff.R0,
                                                                    m_PacCoeff.width, ref m_data.frame, ref m_data.depth, ref dum_cam);
                        break;
                    case ChTire.CollisionType.ENVELOPE:
                        m_data.in_contact = DiscTerrainCollisionEnvelope(m_terrain, wheel_state.pos, disc_normal, m_PacCoeff.R0,
                                                                         m_areaDep, ref m_data.frame, ref m_data.depth);
                        break;
                }*/

                if (m_data.in_contact)
                {
                    // Wheel velocity in the ISO-C Frame
                    ChVector vel = wheel_state.lin_vel;
                    m_data.vel = m_data.frame.TransformDirectionParentToLocal(vel);

                    // Generate normal contact force (recall, all forces are reduced to the wheel
                    // center). If the resulting force is negative, the disc is moving away from
                    // the terrain so fast that no contact force is generated.
                    // The sign of the velocity term in the damping function is negative Math.Since
                    // a positive velocity means a decreaMath.Sing depth, not an increaMath.Sing depth
                    double Fn_mag = GetNormalStiffnessForce(m_data.depth) + GetNormalDampingForce(m_data.depth, -m_data.vel.y);

                    if (Fn_mag < 0)
                    {
                        Fn_mag = 0;
                        m_data.in_contact = false;  // Skip Force and moment calculations when the normal force = 0
                    }

                    m_data.normal_force = Fn_mag;
                    m_states.R_eff = m_PacCoeff.R0 - m_data.depth;
                   // m_states.vx = Math.Abs(m_data.vel.x);
                   // if (m_system.ISO)
                   // {
                   //     m_states.vsx = m_data.vel.data[0] - wheel_state.omega * m_states.R_eff;
                    //}
                   // else
                  //  {
                        m_states.vsx = m_data.vel.x - wheel_state.omega * m_states.R_eff;
                   // }
                    m_states.vsy = -m_data.vel.y;  // PAC89 is defined in a modified SAE coordinate system
                    m_states.omega = wheel_state.omega;
                    m_states.disc_normal = disc_normal;
                }
                else
                {
                    // Reset all states if the tire comes off the ground.
                    m_data.normal_force = 0;
                    m_states.R_eff = m_PacCoeff.R0;
                    m_states.cp_long_slip = 0;
                    m_states.cp_side_slip = 0;
                    m_states.vx = 0;
                    m_states.vsx = 0;
                    m_states.vsy = 0;
                    m_states.omega = 0;
                    m_states.disc_normal = new ChVector(0, 0, 0);
                }
            }

            /// Advance the state of this tire by the specified time step.
            public override void Advance(double step)
            {

                // Set tire forces to zero.
                m_tireforce.point = m_wheel.BodyFrame.GetPos();
                m_tireforce.force = new ChVector(0, 0, 0);
                m_tireforce.moment = new ChVector(0, 0, 0);

                // Return now if no contact.
                if (!m_data.in_contact)
                    return;

                // prevent Math.Singularity for kappa, when vx == 0
                const double epsilon = 0.1;
                m_states.cp_long_slip = -m_states.vsx / (m_states.vx + epsilon);

                if (m_states.omega != 0)
                {
                    m_states.cp_side_slip = Math.Atan(m_states.vsy / Math.Abs(m_states.omega * (m_PacCoeff.R0 - m_data.depth)));
                }
                else
                {
                    m_states.cp_side_slip = 0;
                }

                // Ensure that cp_lon_slip stays between -1 & 1
                ChMaths.ChClampValue(ref m_states.cp_long_slip, -1.0, 1.0);

                // Ensure that cp_side_slip stays between -pi()/2 & pi()/2 (a little less to prevent tan from going to infinity)
                ChMaths.ChClampValue(ref m_states.cp_side_slip, -ChMaths.CH_C_PI_2 + 0.001, ChMaths.CH_C_PI_2 - 0.001);

                // Calculate the new force and moment values (normal force and moment have already been accounted for in
                // Synchronize()).
                // Express Fz in kN (note that all other forces and moments are in N and Nm).
                // See reference for details on the calculations.
                double Fx = 0;
                double Fy = 0;
                double Fz = m_data.normal_force;
                double Mx = 0;
                double My = 0;
                double Mz = 0;

                // Express alpha and gamma in rad. Express kappa as ratio.
                m_gamma = ChMaths.CH_C_PI_2 - Math.Acos(m_states.disc_normal.y);
                m_alpha = m_states.cp_side_slip;
                m_kappa = m_states.cp_long_slip;

                // Clamp |gamma| to specified value: Limit due to tire testing, avoids erratic extrapolation. m_gamma_limit is
                // in rad too.
                double gamma = ChMaths.ChClamp(m_gamma, -m_gamma_limit, m_gamma_limit);

                switch (m_use_mode)
                {
                    case 0:
                        // vertical spring & damper mode
                        break;
                    case 1:
                        // steady state pure longitudinal slip
                        Fx = CalcFx(m_kappa, Fz, gamma);
                        break;
                    case 2:
                        // steady state pure lateral slip
                        Fy = CalcFy(m_alpha, Fz, gamma);
                        break;
                    case 3:
                        // steady state pure lateral slip uncombined
                        Fx = CalcFx(m_kappa, Fz, gamma);
                        Fy = CalcFy(m_alpha, Fz, gamma);
                        Mx = CalcMx(Fy, Fz, gamma);
                        My = CalcMx(Fx, Fz, gamma);
                        Mz = CalcMz(m_alpha, Fz, gamma, Fy);
                        break;
                    case 4:
                        // steady state combined slip
                        if (m_use_friction_ellipsis)
                        {
                            double Fx_u = CalcFx(m_kappa, Fz, gamma);
                            double Fy_u = CalcFy(m_alpha, Fz, gamma);
                            double a_s = Math.Sin(m_alpha_c);
                            double beta = Math.Acos(Math.Abs(m_kappa_c) / Math.Sqrt(Math.Pow(m_kappa_c, 2) + Math.Pow(a_s, 2)));
                            double mux = 1.0 / Math.Sqrt(Math.Pow(1.0 / m_mu_x_act, 2) + Math.Pow(Math.Tan(beta) / m_mu_y_max, 2));
                            double muy = Math.Tan(beta) / Math.Sqrt(Math.Pow(1.0 / m_mu_x_max, 2) + Math.Pow(Math.Tan(beta) / m_mu_y_act, 2));
                            Fx = mux / m_mu_x_act * Fx_u;
                            Fy = muy / m_mu_y_act * Fy_u;
                            Mx = CalcMx(Fy, Fz, gamma);
                            My = CalcMx(Fx, Fz, gamma);
                            Mz = CalcMz(m_alpha, Fz, gamma, Fy);
                        }
                        else
                        {
                            Fx = CalcFxComb(m_kappa, m_alpha, Fz, gamma);
                            Fy = CalcFyComb(m_kappa, m_alpha, Fz, gamma);
                            Mx = CalcMx(Fy, Fz, gamma);
                            My = CalcMx(Fx, Fz, gamma);
                            Mz = CalcMzComb(m_kappa, m_alpha, Fz, gamma, Fx, Fy);
                        }
                        break;
                }

                // Compile the force and moment vectors so that they can be
                // transformed into the global coordinate system.
                // Convert from SAE to ISO Coordinates at the contact patch.
               // if (m_system.ISO)
               // {
                //    m_tireforce.force = new ChVector(Fx, -Fy, m_data.normal_force);
                //    m_tireforce.moment = new ChVector(Mx, -My, -Mz);
               // }
               // else
               // {
                    m_tireforce.force = new ChVector(-Fx, -Fy, m_data.normal_force);
                    m_tireforce.moment = new ChVector(-Mx, -My, -Mz);
               // }

                // Rotate into global coordinates  
                m_tireforce.force = m_data.frame.TransformDirectionLocalToParent(m_tireforce.force);
                m_tireforce.moment = m_data.frame.TransformDirectionLocalToParent(m_tireforce.moment);

                // Move the tire forces from the contact patch to the wheel center
               // if (m_system.ISO)
               // {
                //    m_tireforce.moment +=
                //           ChVector.Vcross((m_data.frame.pos + m_data.depth * m_data.frame.rot.GetZaxis()) - m_tireforce.point, m_tireforce.force);
               // }
               // else
               // {
                    m_tireforce.moment +=
                            ChVector.Vcross((m_data.frame.pos + m_data.depth * m_data.frame.rot.GetYaxis()) - m_tireforce.point, m_tireforce.force);
               // }

            }

            public double CalcFx(double kappa, double Fz, double gamma)
            {

                // calculates the longitudinal force based on a limited parameter set.
                // Pi is not considered
                double Fz0s = m_PacCoeff.FzNomin * m_PacScal.lfz0;
                double dFz = (Fz - Fz0s) / Fz0s;
                double C = m_PacCoeff.pcx1 * m_PacScal.lcx;
                double Mu = (m_PacCoeff.pdx1 + m_PacCoeff.pdx2 * dFz) * (1.0 - m_PacCoeff.pdx3 * Math.Pow(gamma, 2)) * m_PacScal.lmux;
                double D = Mu * Fz * m_mu / m_PacCoeff.mu0;
                double E = (m_PacCoeff.pex1 + m_PacCoeff.pex2 * dFz + m_PacCoeff.pex3 * dFz * dFz) * m_PacScal.lex;
                if (E > 1.0)
                    E = 1.0;

                double BCD = Fz * (m_PacCoeff.pkx1 + m_PacCoeff.pkx2) * m_PacScal.lkx;  // BCD = Kx
                double B = BCD / (C * D);
                double Sh = (m_PacCoeff.phx1 + m_PacCoeff.phx2 * dFz) * m_PacScal.lhx;
                double Sv = Fz * (m_PacCoeff.pvx1 + m_PacCoeff.pvx2 * dFz) * m_PacScal.lvx * m_PacScal.lmux;
                m_kappa_c = kappa + Sh + Sv / BCD;
                double X1 = B * (kappa + Sh);
                double Fx0 = D * Math.Sin(C * Math.Atan(X1 - E * (X1 - Math.Atan(X1)))) + Sv;
                m_mu_x_act = Math.Abs((Fx0 - Sv) / Fz);
                m_mu_x_max = Math.Abs(D / Fz);

                return Fx0;
            }

            public double CalcFy(double alpha, double Fz, double gamma)
            {

                double Fz0s = m_PacCoeff.FzNomin * m_PacScal.lfz0;
                double dFz = (Fz - Fz0s) / Fz0s;
                double C = m_PacCoeff.pcy1 * m_PacScal.lcy;
                m_Cy = C;
                double Mu = (m_PacCoeff.pdy1 + m_PacCoeff.pdy2 * dFz) * (1.0 - m_PacCoeff.pdy3 * Math.Pow(gamma, 2)) * m_PacScal.lmuy;
                double D = Mu * Fz * m_mu / m_PacCoeff.mu0;
                debug = Math.Sign(alpha);
                double E = (m_PacCoeff.pey1 + m_PacCoeff.pey2 * dFz) *
                           (1.0 + m_PacCoeff.pey5 * Math.Pow(gamma, 2) - (m_PacCoeff.pey3 + m_PacCoeff.pey4 * gamma) * Math.Sign(alpha)) *
                           m_PacScal.ley;
                if (E > 1.0)
                    E = 1.0;
                double Ky0 = m_PacCoeff.pky1 * m_PacCoeff.FzNomin * Math.Sin(2.0 * Math.Atan(Fz / (m_PacCoeff.pky2 * Fz0s))) *
                             m_PacScal.lfz0 * m_PacScal.lky;
                double BCD = Ky0;  // BCD = Ky
                double B = BCD / (C * D);
                m_By = B;
                double Sh = (m_PacCoeff.phy1 + m_PacCoeff.phy2 * dFz) * m_PacScal.lhy;
                double Sv = Fz * ((m_PacCoeff.pvy1 + m_PacCoeff.pvy2 * dFz) * m_PacScal.lvy) * m_PacScal.lmuy;
                m_alpha_c = alpha + Sh + Sv / BCD;
                double X1 = ChMaths.ChClamp(B * (alpha + Sh), -ChMaths.CH_C_PI_2 + 0.001,
                                    ChMaths.CH_C_PI_2 - 0.001);  // Ensure that X1 stays within +/-90 deg minus a little bit
                double Fy0 = D * Math.Sin(C * Math.Atan(X1 - E * (X1 - Math.Atan(X1)))) + Sv;
                m_Shf = Sh + Sv / BCD;
                m_mu_y_act = Math.Abs((Fy0 - Sv) / Fz);
                m_mu_y_max = Math.Abs(D / Fz);

                return Fy0;
            }

            // Oeverturning Couple
            public double CalcMx(double Fy, double Fz, double gamma)
            {
                double Mx = m_PacCoeff.R0 * Fz *
                            (m_PacCoeff.qsx1 * m_PacScal.lvx - m_PacCoeff.qsx2 * gamma + m_PacCoeff.qsx3 * Fy / m_PacCoeff.FzNomin +
                             m_PacCoeff.qsx4 * Math.Cos(m_PacCoeff.qsx5 * Math.Pow(Math.Atan(m_PacCoeff.qsx6 * Fz / m_PacCoeff.FzNomin), 2)) *
                                 Math.Sin(m_PacCoeff.qsx7 * gamma + m_PacCoeff.qsx8 * Math.Atan(m_PacCoeff.qsx9 * Fy / m_PacCoeff.FzNomin)) +
                             m_PacCoeff.qsx10 * Math.Atan(m_PacCoeff.qsx11 * Fz / m_PacCoeff.FzNomin) * gamma);
                return Mx;
            }

            // Rolling Resistance
            public double CalcMy(double Fx, double Fz, double gamma)
            {

                double v0 = Math.Sqrt(9.81 * m_PacCoeff.R0);
                double vstar = Math.Abs(m_states.vx / v0);
                double My = ChMaths.ChSineStep(Math.Abs(m_states.vx), 0.5, 0, 1.0, 1.0) * Math.Sign(m_states.vx) * Fz * m_PacCoeff.R0 *
                            (m_PacCoeff.qsy1 + m_PacCoeff.qsy2 * Fx / m_PacCoeff.FzNomin + m_PacCoeff.qsy3 * vstar +
                             m_PacCoeff.qsy4 * Math.Pow(vstar, 4) +
                             (m_PacCoeff.qsy5 + m_PacCoeff.qsy6 * Fz / m_PacCoeff.FzNomin) * Math.Pow(gamma, 2)) *
                            Math.Pow(Fz / m_PacCoeff.FzNomin, m_PacCoeff.qsy7) * m_PacScal.lmuy;
                return My;
            }

            public double CalcTrail(double alpha, double Fz, double gamma)
            {

                double Fz0s = m_PacCoeff.FzNomin * m_PacScal.lfz0;
                double dFz = (Fz - Fz0s) / Fz0s;
                double C = m_PacCoeff.qcz1;
                double gamma_z = gamma * m_PacScal.lgaz;
                double Sh = m_PacCoeff.qhz1 + m_PacCoeff.qhz2 * dFz + (m_PacCoeff.qhz3 + m_PacCoeff.qhz4 * dFz) * gamma_z;
                double alpha_t = alpha + Sh;
                double B = (m_PacCoeff.qbz1 + m_PacCoeff.qbz2 * dFz + m_PacCoeff.qbz3 * Math.Pow(dFz, 2)) *
                           (1.0 + m_PacCoeff.qbz4 * gamma_z + m_PacCoeff.qbz5 * Math.Abs(gamma_z)) * m_PacScal.lky / m_PacScal.lmuy;
                double D = Fz * (m_PacCoeff.qdz1 + m_PacCoeff.qdz2 * dFz) *
                           (1.0 + m_PacCoeff.qdz3 * gamma_z + m_PacCoeff.qdz4 * Math.Pow(gamma_z, 2)) * m_PacCoeff.R0 / Fz0s *
                           m_PacScal.ltr;
                double E = (m_PacCoeff.qez1 + m_PacCoeff.qez2 * dFz + m_PacCoeff.qez3 * Math.Pow(dFz, 2)) *
                           (1.0 + (m_PacCoeff.qez4 + m_PacCoeff.qez5 * gamma_z) * Math.Atan(B * C * alpha_t) / ChMaths.CH_C_PI_2);
                double X1 = B * alpha_t;
                return D * Math.Cos(C * Math.Atan(B * X1 - E * (B * X1 - Math.Atan(B * X1)))) * Math.Cos(alpha);
            }

            public double CalcMres(double alpha, double Fz, double gamma)
            {

                double Fz0s = m_PacCoeff.FzNomin * m_PacScal.lfz0;
                double dFz = (Fz - Fz0s) / Fz0s;
                double alpha_r = alpha + m_Shf;
                double gamma_z = gamma * m_PacScal.lgaz;
                double C = 1.0;
                double B = (m_PacCoeff.qbz9 * m_PacScal.lky / m_PacScal.lmuy + m_PacCoeff.qbz10 * m_By * m_Cy);
                double D = Fz *
                           ((m_PacCoeff.qdz6 + m_PacCoeff.qdz7 * dFz) * m_PacScal.ltr +
                            (m_PacCoeff.qdz8 + m_PacCoeff.qdz9 * dFz) * gamma_z) *
                           m_PacCoeff.R0 * m_PacScal.lmuy;
                return D * Math.Cos(C * Math.Atan(B * alpha_r)) * Math.Cos(alpha);
            }

            public double CalcFxComb(double kappa, double alpha, double Fz, double gamma)
            {

                double Fz0s = m_PacCoeff.FzNomin * m_PacScal.lfz0;
                double dFz = (Fz - Fz0s) / Fz0s;
                double C = m_PacCoeff.pcx1 * m_PacScal.lcx;
                double Mux = (m_PacCoeff.pdx1 + m_PacCoeff.pdx2 * dFz) * (1.0 - m_PacCoeff.pdx3 * Math.Pow(gamma, 2)) * m_PacScal.lmux;
                double D = Mux * Fz * m_mu / m_PacCoeff.mu0;
                double E = (m_PacCoeff.pex1 + m_PacCoeff.pex2 * dFz + m_PacCoeff.pex3 * dFz * dFz) * m_PacScal.lex;
                if (E > 1.0)
                    E = 1.0;
                double BCD = Fz * (m_PacCoeff.pkx1 + m_PacCoeff.pkx2) * m_PacScal.lkx;  // BCD = Kx
                double B = BCD / (C * D);
                double Sh = (m_PacCoeff.phx1 + m_PacCoeff.phx2 * dFz) * m_PacScal.lhx;
                double Sv = Fz * (m_PacCoeff.pvx1 + m_PacCoeff.pvx2 * dFz) * m_PacScal.lvx * m_PacScal.lmux;
                double X1 = B * (kappa + Sh);
                double Fx0 = D * Math.Sin(C * Math.Atan(X1 - E * (X1 - Math.Atan(X1)))) + Sv;
                double Shxa = m_PacCoeff.rhx1;
                double alpha_s = Math.Tan(alpha) * Math.Sign(m_data.vel.x) + Shxa;
                double Bxa =
                    m_PacCoeff.rbx1 + m_PacCoeff.rbx3 * Math.Pow(Math.Sin(gamma), 2) * Math.Cos(Math.Atan(m_PacCoeff.rbx2 * kappa)) * m_PacScal.lxal;
                double Cxa = m_PacCoeff.rcx1;
                double Exa = m_PacCoeff.rex1 + m_PacCoeff.rex2 * dFz;
                double Gxa = Math.Cos(Cxa * Math.Atan(Bxa * alpha_s) - Exa * (Bxa * alpha_s - Math.Atan(Bxa * alpha_s))) /
                             Math.Cos(Cxa * Math.Atan(Bxa * Shxa) - Exa * (Bxa * Shxa - Math.Atan(Bxa * Shxa)));
                return Fx0 * Gxa;
            }

            public double CalcFyComb(double kappa, double alpha, double Fz, double gamma)
            {

                double Fz0s = m_PacCoeff.FzNomin * m_PacScal.lfz0;
                double dFz = (Fz - Fz0s) / Fz0s;
                double C = m_PacCoeff.pcy1 * m_PacScal.lcy;
                double Muy = (m_PacCoeff.pdy1 + m_PacCoeff.pdy2 * dFz) * (1.0 - m_PacCoeff.pdy3 * Math.Pow(gamma, 2)) * m_PacScal.lmuy;
                double D = Muy * Fz * m_mu / m_PacCoeff.mu0;
                double E = (m_PacCoeff.pey1 + m_PacCoeff.pey2 * dFz) *
                           (1.0 + m_PacCoeff.pey5 * Math.Pow(gamma, 2) - (m_PacCoeff.pey3 + m_PacCoeff.pey4 * gamma) * Math.Sign(alpha)) *
                           m_PacScal.ley;
                if (E > 1.0)
                    E = 1.0;
                double Ky0 = m_PacCoeff.pky1 * m_PacCoeff.FzNomin * Math.Sin(2.0 * Math.Atan(Fz / (m_PacCoeff.pky2 * Fz0s))) *
                             m_PacScal.lfz0 * m_PacScal.lky;
                double BCD = Ky0;  // BCD = Ky
                double B = BCD / (C * D);
                double Sh = (m_PacCoeff.phy1 + m_PacCoeff.phy2 * dFz) * m_PacScal.lhy;
                double Sv = Fz * ((m_PacCoeff.pvy1 + m_PacCoeff.pvy2 * dFz) * m_PacScal.lvy) * m_PacScal.lmuy;
                double X1 = ChMaths.ChClamp(B * (alpha + Sh), -ChMaths.CH_C_PI_2 + 0.001,
                                    ChMaths.CH_C_PI_2 - 0.001);  // Ensure that X1 stays within +/-90 deg minus a little bit
                double Fy0 = D * Math.Sin(C * Math.Atan(X1 - E * (X1 - Math.Atan(X1)))) + Sv;
                double Shyk = m_PacCoeff.rhx1 + m_PacCoeff.rhy2 * dFz;
                m_Shf = Sh + Sv / BCD;
                double kappa_s = kappa + Shyk;
                double Byk = m_PacCoeff.rby1 * Math.Cos(Math.Atan(m_PacCoeff.rby2 * (Math.Tan(alpha) - m_PacCoeff.rby3)));
                double Cyk = m_PacCoeff.rcy1;
                double Dvyk = Muy * Fz * (m_PacCoeff.rvy1 + m_PacCoeff.rvy2 * dFz + m_PacCoeff.rvy3 * gamma) *
                              Math.Cos(Math.Atan(m_PacCoeff.rvy4 * Math.Tan(alpha)));
                double Svyk = Dvyk * Math.Sin(m_PacCoeff.rvy5 * Math.Atan(m_PacCoeff.rvy6 * kappa));
                double Gyk = Math.Cos(Cyk * Math.Atan(Byk * kappa_s)) / Math.Cos(Cyk * Math.Atan(Byk * Shyk));
                return Fy0 * Gyk + Svyk;
            }

            public double CalcMz(double alpha, double Fz, double gamma, double Fy)
            {

                double Fz0s = m_PacCoeff.FzNomin * m_PacScal.lfz0;
                double dFz = (Fz - Fz0s) / Fz0s;
                double C = m_PacCoeff.qcz1;
                double gamma_z = gamma * m_PacScal.lgaz;
                double Sh = m_PacCoeff.qhz1 + m_PacCoeff.qhz2 * dFz + (m_PacCoeff.qhz3 + m_PacCoeff.qhz4 * dFz) * gamma_z;
                double alpha_t = alpha + Sh;
                double B = (m_PacCoeff.qbz1 + m_PacCoeff.qbz2 * dFz + m_PacCoeff.qbz3 * Math.Pow(dFz, 2)) *
                           (1.0 + m_PacCoeff.qbz4 * gamma_z + m_PacCoeff.qbz5 * Math.Abs(gamma_z)) * m_PacScal.lky / m_PacScal.lmuy;
                double D = Fz * (m_PacCoeff.qdz1 + m_PacCoeff.qdz2 * dFz) *
                           (1.0 + m_PacCoeff.qdz3 * gamma_z + m_PacCoeff.qdz4 * Math.Pow(gamma_z, 2)) * m_PacCoeff.R0 / Fz0s *
                           m_PacScal.ltr;
                double E = (m_PacCoeff.qez1 + m_PacCoeff.qez2 * dFz + m_PacCoeff.qez3 * Math.Pow(dFz, 2)) *
                           (1.0 + (m_PacCoeff.qez4 + m_PacCoeff.qez5 * gamma_z) * Math.Atan(B * C * alpha_t) / ChMaths.CH_C_PI_2);
                double X1 = B * alpha_t;
                double t = D * Math.Cos(C * Math.Atan(B * X1 - E * (B * X1 - Math.Atan(B * X1)))) * Math.Cos(alpha);
                double Mz = -t * Fy + CalcMres(m_alpha, Fz, gamma);

                return Mz;
            }

            public double CalcMzComb(double kappa, double alpha, double Fz, double gamma, double Fx, double Fy)
            {

                double Mz = 0.0;
                double Fz0s = m_PacCoeff.FzNomin * m_PacScal.lfz0;
                double dFz = (Fz - Fz0s) / Fz0s;
                double Sht = m_PacCoeff.qhz1 + m_PacCoeff.qhz2 * dFz + (m_PacCoeff.qhz3 + m_PacCoeff.qhz4 * dFz) * Math.Sin(gamma);
                double alpha_s = alpha + (m_PacCoeff.phy1 + m_PacCoeff.phy2 * dFz) * m_PacScal.lhy;
                double alpha_t = alpha_s + Sht;
                double Ct = m_PacCoeff.qcz1;
                double gamma_s = Math.Sin(gamma);
                // pneumatic trail
                double Dt0 = Fz * (m_PacCoeff.R0 / Fz0s) * (m_PacCoeff.qdz1 + m_PacCoeff.qdz2 * dFz) * m_PacScal.ltr *
                             Math.Sign(m_data.vel.x);
                double Dt = Dt0 * (1.0 + m_PacCoeff.qdz3 * Math.Abs(gamma_s) + m_PacCoeff.qdz4 * Math.Pow(gamma_s, 2));
                double Bt = (m_PacCoeff.qbz1 + m_PacCoeff.qbz2 * dFz + m_PacCoeff.qbz3 * Math.Pow(dFz, 2)) *
                            (1.0 + m_PacCoeff.qbz5 * Math.Abs(gamma_s) + m_PacCoeff.qbz6 * Math.Pow(gamma_s, 2)) * m_PacScal.lky /
                            m_PacScal.lmuy;
                double Et = (m_PacCoeff.qez1 + m_PacCoeff.qez2 * dFz + m_PacCoeff.qez3 * Math.Pow(dFz, 2)) *
                            ((1.0 + m_PacCoeff.qez4 + m_PacCoeff.qez5 * gamma_s) * Math.Atan(Bt * Ct * alpha_t / ChMaths.CH_C_PI_2));
                double Kxk = Fz * (m_PacCoeff.pkx1 + m_PacCoeff.pkx2) * m_PacScal.lkx;
                double Kya = m_PacCoeff.pky1 * m_PacCoeff.FzNomin * Math.Sin(2.0 * Math.Atan(Fz / (m_PacCoeff.pky2 * Fz0s))) *
                             m_PacScal.lfz0 * m_PacScal.lky;
                double alpha_teq = Math.Sqrt(Math.Pow(alpha_t, 2) + Math.Pow(Kxk, 2) * Math.Pow(kappa, 2) / Math.Pow(Kya, 2)) * Math.Sign(alpha_t);
                double t = Dt * (Ct * Math.Atan(Bt * alpha_t - Et * (Bt * alpha_t - Math.Atan(Bt * alpha_t))));

                // residual moment
                double Shf = 0.0;  // todo!!
                double alpha_r = alpha + Shf;
                double alpha_req = Math.Sqrt(Math.Pow(alpha_r, 2) + Math.Pow(Kxk, 2) * Math.Pow(kappa, 2) / Math.Pow(Kya, 2)) * Math.Sign(alpha_r);
                double gamma_z = gamma * m_PacScal.lgaz;
                double Cr = 1.0;
                double Br = (m_PacCoeff.qbz9 * m_PacScal.lky / m_PacScal.lmuy + m_PacCoeff.qbz10 * m_By * m_Cy);
                double Dr = Fz *
                            ((m_PacCoeff.qdz6 + m_PacCoeff.qdz7 * dFz) * m_PacScal.ltr +
                             (m_PacCoeff.qdz8 + m_PacCoeff.qdz9 * dFz) * gamma_z) *
                            m_PacCoeff.R0 * m_PacScal.lmuy;
                double Mr = Dr * Math.Cos(Cr * Math.Atan(Br * alpha_req));
                // moment caused by longitudinal force
                double s = m_PacCoeff.R0 *
                           (m_PacCoeff.ssz1 + m_PacCoeff.ssz2 * (Fy / Fz0s) + (m_PacCoeff.ssz3 + m_PacCoeff.ssz4 * dFz) * gamma_s) *
                           m_PacScal.ls;
                Mz = -t * Fy + Mr + s * Fx;

                return Mz;
            }

            public bool DiscTerrainCollision(
                                      RigidTerrain terrain,       //< [in] reference to terrain system
                                      ChVector disc_center,  //< [in] global location of the disc center
                                      ChVector disc_normal,  //< [in] disc normal, expressed in the global frame
                                      double disc_radius,             //< [in] disc radius
                                      ref ChCoordsys contact,          //< [out] contact coordinate system (relative to the global frame)
                                      ref double depth                   //< [out] penetration depth (positive if contact occurred)
                                  )
            {
                // Find terrain height below disc center. There is no contact if the disc
                // center is below the terrain or farther away by more than its radius.
                double hc = terrain.GetHeight(disc_center.x, disc_center.z);
                if (disc_center.y <= hc || disc_center.y >= hc + disc_radius)
                    return false;

                // Find the lowest point on the disc. There is no contact if the disc is
                // (almost) horizontal.
                ChVector nhelp = terrain.GetNormal(disc_center.x, disc_center.z);
                ChVector dir1 = ChVector.Vcross(disc_normal, nhelp);
                double sinTilt2 = dir1.Length2();

                if (sinTilt2 < 1e-3)
                    return false;


                // Contact point (lowest point on disc).
                ChVector ptD = disc_center + disc_radius * ChVector.Vcross(disc_normal, dir1 / Math.Sqrt(sinTilt2));

                // Find terrain height at lowest point. No contact if lowest point is above
                // the terrain.
                double hp = terrain.GetHeight(ptD.x, ptD.z);

                if (ptD.y > hp)
                    return false;

                // Approximate the terrain with a plane. Define the projection of the lowest
                // point onto this plane as the contact point on the terrain.
                ChVector normal = terrain.GetNormal(ptD.x, ptD.z);
                // m_terrainNormal = normal;
                ChVector longitudinal = ChVector.Vcross(disc_normal, normal);
                longitudinal.Normalize();
                ChVector lateral = ChVector.Vcross(normal, longitudinal);
                ChMatrix33<double> rot = new ChMatrix33<double>(0);
                rot.Set_A_axis(longitudinal, lateral, normal);

                contact.pos = ptD;
                contact.rot = rot.Get_A_quaternion();

                depth = ChVector.Vdot(new ChVector(0, hp - ptD.y, 0), normal);
                Debug.Assert(depth > 0);

                return true;
            }

            public override double GetSlipAngle() { return m_alpha; }

            public override double GetLongitudinalSlip() { return m_kappa; }

            [System.Serializable]
            public struct TireStates
            {
                public double cp_long_slip;     // Contact Path - Longitudinal Slip State (Kappa)
                public double cp_side_slip;     // Contact Path - Side Slip State (Alpha)
                public double vx;               // Longitudinal speed
                public double vsx;              // Longitudinal slip velocity
                public double vsy;              // Lateral slip velocity = Lateral velocity
                public double omega;            // Wheel angular velocity about its spin axis
                public double R_eff;            // Effective Radius
                public ChVector disc_normal;  //(temporary for debug)
            };

          //  private ChFunction_Recorder m_areaDep = new ChFunction_Recorder();  // lookup table for estimation of penetration depth from intersection area


            public TireStates m_states;

            public double m_kappa;  //< longitudinal slip ratio
            public double m_alpha;  //< slip angle

            public double m_gamma;  //< camber angle
            private double m_gamma_limit;  //< limit camber angle
            private bool m_use_friction_ellipsis;

            /// Road friction
            private double m_mu;

            private double m_Shf;
            private double m_Cy;
            private double m_By;

            ChSubsysDefs.VehicleSide m_measured_side;
            private bool m_allow_mirroring;

            private uint m_use_mode;

            // combined forces calculation
            private double m_kappa_c;
            private double m_alpha_c;
            private double m_mu_x_act;
            private double m_mu_x_max;
            private double m_mu_y_act;
            private double m_mu_y_max;


            private struct Pac02ScalingFactors
            {
                public double lfz0;
                public double lcx;
                public double lex;
                public double lkx;
                public double lhx;
                public double lmux;
                public double lvx;
                public double lxal;
                public double lmx;
                public double lvmx;
                public double lmy;

                public double lcy;
                public double ley;
                public double lhy;
                public double lky;
                public double lmuy;
                public double lvy;
                public double lyka;
                public double lvyka;
                public double ltr;
                public double lgax;
                public double lgay;
                public double lgaz;
                public double lres;
                public double lsgkp;
                public double lsgal;
                public double lgyr;
                public double ls;
            };

            private struct Pac02Coeff
            {
                public double mu0;           // road friction coefficient at test conditions for the handling parameters
                public double R0;            // unloaded radius
                public double width;         // tire width
                public double aspect_ratio;  // actually unused
                public double rim_width;     // actually unused
                public double rim_radius;    // actually unused
                public double FzNomin;       // nominla wheel load
                public double Cz;            // vertical tire stiffness
                public double Kz;            // vertical tire damping

                // Longitudinal Coefficients
                public double pcx1;
                public double pdx1;
                public double pdx2;
                public double pdx3;
                public double pex1;
                public double pex2;
                public double pex3;
                public double pex4;
                public double phx1;
                public double phx2;
                public double pkx1;
                public double pkx2;
                public double pkx3;
                public double pvx1;
                public double pvx2;
                public double rbx1;
                public double rbx2;
                public double rbx3;
                public double rcx1;
                public double rex1;
                public double rex2;
                public double rhx1;
                public double ptx1;
                public double ptx2;
                public double ptx3;
                public double ptx4;

                // overturning coefficients
                public double qsx1;
                public double qsx2;
                public double qsx3;
                public double qsx4;
                public double qsx5;
                public double qsx6;
                public double qsx7;
                public double qsx8;
                public double qsx9;
                public double qsx10;
                public double qsx11;

                // rolling resistance coefficients
                public double qsy1;
                public double qsy2;
                public double qsy3;
                public double qsy4;
                public double qsy5;
                public double qsy6;
                public double qsy7;
                public double qsy8;

                // Lateral Coefficients
                public double pcy1;
                public double pdy1;
                public double pdy2;
                public double pdy3;
                public double pey1;
                public double pey2;
                public double pey3;
                public double pey4;
                public double pey5;
                public double phy1;
                public double phy2;
                public double phy3;
                public double pky1;
                public double pky2;
                public double pky3;
                public double pvy1;
                public double pvy2;
                public double pvy3;
                public double pvy4;
                public double rby1;
                public double rby2;
                public double rby3;
                public double rby4;
                public double rcy1;
                public double rey1;
                public double rey2;
                public double rhy1;
                public double rhy2;
                public double rvy1;
                public double rvy2;
                public double rvy3;
                public double rvy4;
                public double rvy5;
                public double rvy6;
                public double pty1;
                public double pty2;

                // alignment coefficients
                public double qbz1;
                public double qbz2;
                public double qbz3;
                public double qbz4;
                public double qbz5;
                public double qbz6;
                public double qbz9;
                public double qbz10;
                public double qcz1;
                public double qdz1;
                public double qdz2;
                public double qdz3;
                public double qdz4;
                public double qdz5;
                public double qdz6;
                public double qdz7;
                public double qdz8;
                public double qdz9;
                public double qez1;
                public double qez2;
                public double qez3;
                public double qez4;
                public double qez5;
                public double qhz1;
                public double qhz2;
                public double qhz3;
                public double qhz4;
                public double ssz1;
                public double ssz2;
                public double ssz3;
                public double ssz4;
                public double qtz1;
                public double mbelt;
            };
            protected RigidTerrain m_terrain;

            private ChFunction_Recorder m_vert_map;
            private bool m_use_vert_map;

            private Pac02ScalingFactors m_PacScal;
            private Pac02Coeff m_PacCoeff;
        }
    }
}
