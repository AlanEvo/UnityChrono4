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

        /// Base class for a tire system.
        /// A tire subsystem is a force element. It is passed position and velocity
        /// information of the wheel body and it produces ground reaction forces and
        /// moments to be applied to the wheel body.
        public abstract class ChTire : MonoBehaviour
        {
            public enum CollisionType { SINGLE_POINT, FOUR_POINTS, ENVELOPE }

            public struct ContactData
            {
                public bool in_contact;      // true if disc in contact with terrain
                public ChCoordsys frame;   // contact frame (x: long, y: lat, z: normal)
                public ChVector vel;       // relative velocity expressed in contact frame
                public double normal_force;  // magnitude of normal contact force
                public double depth;         // penetration depth
            };

            protected ChSubsysDefs.WheelState m_wheelState;
            protected ContactData m_data;

            // TEST
            ChMatrix33<double> rot = new ChMatrix33<double>(0);


            public ChTire()
            {
               /* m_wheel = wheel;

                // Increment mass and inertia of the wheel body.
                wheel.SetMass(wheel.GetMass() + GetMass());
                wheel.SetInertiaXX(wheel.GetInertiaXX() + GetInertia());*/
            }

            public void Start()
            {

            }

            /// Initialize this tire subsystem.
            /// Cache the associated wheel body and vehicle side flag, and add tire mass and
            /// inertia to the wheel body. A derived class must first call this base implementation.
            public virtual void Initialize(ChBody wheel, ChSubsysDefs.VehicleSide side)
            {
            }

            /// Update the state of this tire system at the current time.
            /// The tire system is provided the current state of its associated wheel and
            /// a handle to the terrain system.
            public virtual void Synchronize(double time,                    ///< [in] current time
                            ChSubsysDefs.WheelState wheel_state,  ///< [in] current state of associated wheel body
                             RigidTerrain terrain        ///< [in] reference to the terrain system
                             )
            {
                CalculateKinematics(time, wheel_state, terrain);
            }

            public virtual void FixedUpdate()
            {
                m_wheelState = GetWheelState();

                m_wheel.Empty_forces_accumulators();
                m_wheel.Accumulate_force(m_tireforce.force, m_tireforce.point, false);
                m_wheel.Accumulate_torque(m_tireforce.moment, false);
            }

            /// Set the value of the integration step size for the underlying dynamics (if applicable).
            /// Default value: 1ms.
            public void SetStepsize(double val) { m_stepsize = val; }

            /// Get the current value of the integration step size.
            public double GetStepsize() { return m_stepsize; }

            /// Advance the state of this tire by the specified time step.
            public virtual void Advance(double step) { }

            /// Get the tire radius.
            public abstract double GetRadius();

            /// Get the tire mass.
            /// Note that this should not include the mass of the wheel (rim).
            public abstract double GetMass();

            /// Report the tire mass.
            /// Certain tire models (e.g. those based on FEA) must return 0 in GetMass()
            /// so that the tire mass is not double counted in the underlying mechanical system.
            /// For reporting purposes, use this function instead.
            public virtual double ReportMass() {
                return GetMass();
            }

            /// Get the tire moments of inertia.
            /// Note that these should not include the inertia of the wheel (rim).
            public abstract ChVector GetInertia();

            /// Get the tire force and moment.
            /// This represents the output from this tire system that is passed to the
            /// vehicle system. Typically, the vehicle subsystem will pass the tire force
            /// to the appropriate suspension subsystem which applies it as an external
            /// force on the wheel body.
            /// NOTE: tire models that rely on underlying Chrono functionality (e.g., the
            /// Chrono contact system or Chrono constraints) must always return zero forces
            /// and moments, else tire forces are double counted.
            public abstract ChSubsysDefs.TerrainForce GetTireForce();

            /// Report the tire force and moment.
            /// This function can be used for reporting purposes or else to calculate tire
            /// forces in a co-simulation framework.
            public abstract ChSubsysDefs.TerrainForce ReportTireForce(ChTerrain terrain);

            /// Get the tire slip angle.
            /// Return the slip angle calculated based on the current state of the associated
            /// wheel body. A derived class may override this function with a more appropriate
            /// calculation based on its specific tire model.
            public virtual double GetSlipAngle() { return m_slip_angle; }

            /// Get the tire longitudinal slip.
            /// Return the longitudinal slip calculated based on the current state of the associated
            /// wheel body. A derived class may override this function with a more appropriate
            /// calculation based on its specific tire model.
            public virtual double GetLongitudinalSlip() { return m_longitudinal_slip; }

            /// Get the tire camber angle.
            /// Return the camber angle calculated based on the current state of the associated
            /// wheel body. A derived class may override this function with a more appropriate
            /// calculation based on its specific tire model.
            public virtual double GetCamberAngle() { return m_camber_angle; }

            /// Utility function for estimating the tire moments of inertia.
            /// The tire is assumed to be specified with the common scheme (e.g. 215/65R15)
            /// and the mass of the tire (excluding the wheel) provided.
           /* public static ChVector EstimateInertia(double tire_width,    //< tire width [mm]
                                                  double aspect_ratio,  //< aspect ratio: height to width [percentage]
                                                  double rim_diameter,  //< rim diameter [in]
                                                  double tire_mass,     //< mass of the tire [kg]
                                                  double t_factor = 2   //< tread to sidewall thickness factor
    )
            {

            }*/

            /// Report the tire deflection 
            public virtual double GetDeflection() { return 0; }

            // -----------------------------------------------------------------------------
            // Return the complete state (expressed in the global frame) for the specified
            // wheel body.
            // -----------------------------------------------------------------------------
            public ChSubsysDefs.WheelState GetWheelState()
            {
                ChSubsysDefs.WheelState state;

                state.pos = GetWheelPos();
                state.rot = GetSpindleRot();
                state.lin_vel = GetSpindleLinVel();
                state.ang_vel = GetSpindleAngVel();

                ChVector ang_vel_loc = state.rot.RotateBack(state.ang_vel);
                state.omega = ang_vel_loc.y;

                return state;

            }

            public ChVector GetWheelPos()
            {
                return m_wheel.BodyFrame.GetPos();
            }

            /// Get the orientation of the spindle body.
            /// The spindle body orientation is returned as a quaternion representing a
            /// rotation with respect to the global reference frame.
            public ChQuaternion GetSpindleRot() { return m_wheel.BodyFrame.GetRot(); }

            /// Get the linear velocity of the spindle body on the specified side.
            /// Return the linear velocity of the spindle center, expressed in the global
            /// reference frame.
            public ChVector GetSpindleLinVel() { return m_wheel.BodyFrame.GetPos_dt(); }

            /// Get the angular velocity of the spindle body on the specified side.
            /// Return the angular velocity of the spindle frame, expressed in the global
            /// reference frame.
            public ChVector GetSpindleAngVel() { return m_wheel.BodyFrame.GetWvel_par(); }

            /// Perform disc-terrain collision detection.
            /// This utility function checks for contact between a disc of specified
            /// radius with given position and orientation (specified as the location of
            /// its center and a unit vector normal to the disc plane) and the terrain
            /// system associated with this tire. It returns true if the disc contacts the
            /// terrain and false otherwise.  If contact occurs, it returns a coordinate
            /// system with the Z axis along the contact normal and the X axis along the
            /// "rolling" direction, as well as a positive penetration depth (i.e. the
            /// height below the terrain of the lowest point on the disc).
            protected static bool disc_terrain_contact(
                                            ChTerrain terrain,       ///< [in] reference to terrain system
                                            ChVector disc_center,  ///< [in] global location of the disc center
                                            ChVector disc_normal,  ///< [in] disc normal, expressed in the global frame
                                            double disc_radius,             ///< [in] disc radius
                                            ref ChCoordsys contact,          ///< [out] contact coordinate system (relative to the global frame)
                                            ref double depth                   ///< [out] penetration depth (positive if contact occurred)
        )
            {
                // Find terrain height below disc center. There is no contact if the disc
                // center is below the terrain or farther away by more than its radius.
                double hc = terrain.GetHeight(disc_center.x, disc_center.y);
                if (disc_center.z <= hc || disc_center.z >= hc + disc_radius)
                    return false;

                // Find the lowest point on the disc. There is no contact if the disc is
                // (almost) horizontal.
                ChVector nhelp = terrain.GetNormal(disc_center.x, disc_center.y);
                ChVector dir1 = ChVector.Vcross(disc_normal, nhelp);
                double sinTilt2 = dir1.Length2();

                if (sinTilt2 < 1e-3)
                    return false;

                // Contact point (lowest point on disc).
                ChVector ptD = disc_center + disc_radius * ChVector.Vcross(disc_normal, dir1 / Math.Sqrt(sinTilt2));

                // Find terrain height at lowest point. No contact if lowest point is above
                // the terrain.
                double hp = terrain.GetHeight(ptD.x, ptD.y);

                if (ptD.z > hp)
                    return false;

                // Approximate the terrain with a plane. Define the projection of the lowest
                // point onto this plane as the contact point on the terrain.
                ChVector normal = terrain.GetNormal(ptD.x, ptD.y);
                ChVector longitudinal = ChVector.Vcross(disc_normal, normal);
                longitudinal.Normalize();
                ChVector lateral = ChVector.Vcross(normal, longitudinal);
                ChMatrix33<double> rot = new ChMatrix33<double>(0); // Need to nest this.
                rot.Set_A_axis(longitudinal, lateral, normal);

                contact.pos = ptD;
                contact.rot = rot.Get_A_quaternion();

                depth = ChVector.Vdot(new ChVector(0, 0, hp - ptD.z), normal);
                //assert(depth > 0);

                return true;
            }

            /// Perform disc-terrain collision detection.
            /// This utility function checks for contact between a disc of specified
            /// radius with given position and orientation (specified as the location of
            /// its center and a unit vector normal to the disc plane) and the terrain
            /// system associated with this tire. It returns true if the disc contacts the
            /// terrain and false otherwise.  If contact occurs, it returns a coordinate
            /// system with the Z axis along the contact normal and the X axis along the
            /// "rolling" direction, as well as a positive penetration depth (i.e. the
            /// height below the terrain of the lowest point on the disc).
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
                // ChMatrix33<double> rot = new ChMatrix33<double>(0);
                rot.Set_A_axis(longitudinal, lateral, normal);

                contact.pos = ptD;
                contact.rot = rot.Get_A_quaternion();

                depth = ChVector.Vdot(new ChVector(0, hp - ptD.y, 0), normal);
                // Debug.Assert(depth > 0);

                return true;
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
            public bool DiscTerrainCollision4pt(
                                                    RigidTerrain terrain,       ///< [in] reference to terrain system
                                                    ChVector disc_center,  ///< [in] global location of the disc center
                                                    ChVector disc_normal,  ///< [in] disc normal, expressed in the global frame
                                                    double disc_radius,             ///< [in] disc radius
                                                    double width,                   ///< [in] tire width
                                                    ref ChCoordsys contact,          ///< [out] contact coordinate system (relative to the global frame)
                                                    ref double depth,                  ///< [out] penetration depth (positive if contact occurred)
                                                    ref double camber_angle            ///< [out] tire camber angle
    ) {
                double dx = 0.1 * disc_radius;
                double dy = 0.3 * width;

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
                ptQ1.y = terrain.GetHeight(ptQ1.x, ptQ1.z);

                ChVector ptQ2 = ptD - dx * longitudinal;
                ptQ2.y = terrain.GetHeight(ptQ2.x, ptQ2.z);

                ChVector ptQ3 = ptD + dy * lateral;
                ptQ3.y = terrain.GetHeight(ptQ3.x, ptQ3.z);

                ChVector ptQ4 = ptD - dy * lateral;
                ptQ4.y = terrain.GetHeight(ptQ4.x, ptQ4.z);

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

                if (da >= disc_radius)
                    return false;

                // Calculate an improved value for the camber angle
                camber_angle = Math.Asin(ChVector.Vdot(disc_normal, terrain_normal));

               // ChMatrix33<double> rot = new ChMatrix33<double>(0);
                rot.Set_A_axis(longitudinal, lateral, terrain_normal);

                contact.pos = ptD;
                contact.rot = rot.Get_A_quaternion();

                depth = disc_radius - da;
               // assert(depth > 0);

                return true;

            }

            /// Collsion algorithm based on a paper of J. Shane Sui and John A. Hirshey II:
            /// "A New Analytical Tire Model for Vehicle Dynamic Analysis" presented at 2001 MSC User Meeting
            public static bool DiscTerrainCollisionEnvelope(
                                                        ChTerrain terrain,            ///< [in] reference to terrain system
                                                        ChVector disc_center,       ///< [in] global location of the disc center
                                                        ChVector disc_normal,       ///< [in] disc normal, expressed in the global frame
                                                        double disc_radius,                  ///< [in] disc radius
                                                        ChFunction_Recorder areaDep,  ///< [in] lookup table to calculate depth from intersection area
                                                        ref ChCoordsys contact,               ///< [out] contact coordinate system (relative to the global frame)
                                                        ref double depth                        ///< [out] penetration depth (positive if contact occurred)
                                                    )
            {
                return false;
            }

            public ChSubsysDefs.VehicleSide GetSide() { return m_side; }

            public ChSubsysDefs.VehicleSide m_side;               //< tire mounted on left/right side
            protected ChBody m_wheel;  //< associated wheel body
            public double m_stepsize;                //< tire integration step size (if applicable)

            /// Utility function to construct a loopkup table for penetration depth as function of intersection area,
            /// for a given tire radius.  The return map can be used in DiscTerrainCollisionEnvelope.
            public static void ConstructAreaDepthTable(double disc_radius, ref ChFunction_Recorder areaDep)
            {
                const UInt64 n_lookup = 90;
                double depMax = disc_radius;  // should be high enough to avoid extrapolation
                double depStep = depMax / (double)(n_lookup - 1);

                for (UInt64 i = 0; i < n_lookup; i++)
                {
                    double dep = depStep * (double)(i);
                    double alpha = 2.0 * Math.Acos(1.0 - dep / disc_radius);
                    double area = 0.5 * disc_radius * disc_radius * (alpha - Math.Sin(alpha));
                    areaDep.AddPoint(area, dep);
                }
            }

            /// Calculate kinematics quantities based on the current state of the associated
            /// wheel body.
            public void CalculateKinematics(double time,                    ///< [in] current time
                             ChSubsysDefs.WheelState state,  ///< [in] current state of associated wheel body
                             RigidTerrain terrain        ///< [in] reference to the terrain system
                             )
            {
                // Wheel normal (expressed in global frame)
                ChVector wheel_normal = state.rot.GetYaxis();

                // Terrain normal at wheel location (expressed in global frame)
                ChVector Z_dir = terrain.GetNormal(state.pos.x, state.pos.y);

                // Longitudinal (heading) and lateral directions, in the terrain plane
                ChVector X_dir = ChVector.Vcross(wheel_normal, Z_dir);
                X_dir.Normalize();
                ChVector Y_dir = ChVector.Vcross(Z_dir, X_dir);

                // Tire reference coordinate system
               // ChMatrix33<double> rot = new ChMatrix33<double>(0); // Needs nesting
                rot.Set_A_axis(X_dir, Y_dir, Z_dir);
                ChCoordsys tire_csys = new ChCoordsys(state.pos, rot.Get_A_quaternion());

                // Express wheel linear velocity in tire frame
                ChVector V = tire_csys.TransformDirectionParentToLocal(state.lin_vel);
                // Express wheel normal in tire frame
                ChVector n = tire_csys.TransformDirectionParentToLocal(wheel_normal);

                // Slip angle
                double abs_Vx = Mathfx.Abs(V.x);
                double zero_Vx = 1e-4;
                m_slip_angle = (abs_Vx > zero_Vx) ? Math.Atan(V.y / abs_Vx) : 0;

                // Longitudinal slip
                m_longitudinal_slip = (abs_Vx > zero_Vx) ? -(V.x - state.omega * GetRadius()) / abs_Vx : 0;

                // Camber angle
                m_camber_angle = Math.Atan2(n.z, n.y);
            }

            public struct TireStates
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
            };

            public TireStates m_states;
            public ChSubsysDefs.TerrainForce m_tireforce;
            public CollisionType m_collision_type;    //< method used for tire-terrain collision

            private double m_slip_angle;
            private double m_longitudinal_slip;
            private double m_camber_angle;
        };

        /// Vector of handles to tire subsystems.
        //typedef std::vector<std::shared_ptr<ChTire>> ChTireList;

        /// @} vehicle_wheeled_tire

    }  // end namespace vehicle
}  // end namespace chrono
