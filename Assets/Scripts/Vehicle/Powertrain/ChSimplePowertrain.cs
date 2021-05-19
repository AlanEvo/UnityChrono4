using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono
{

    public class ChSimplePowertrain : ChPowertrain
    {
        public ChSimpleDriveline m_driveline;

        public double m_current_gear_ratio;
        public double m_motorSpeed;
        public double m_motorTorque;
        public double m_shaftTorque;

      //  public ChShaft leftshaft;
       // public ChShaft rightshaft;

        public double shaft_speed;
        public double throttle;

        public double m_max_torque;
        public double m_max_speed;
        public double m_fwd_gear_ratio = 0.3;
        public double m_rev_gear_ratio = -0.3;

        public double m_conicalgear_ratio = -0.2433;

        // Use this for initialization
        void Start()
        {
            SetDriveMode(DriveMode.FORWARD);
        }

        private void Update()
        {
            throttle = -Input.GetAxis("Accelerator");
            //shaft_speed = leftshaft.GetPos_dt();
        }

        /// Return the forward gear ratio (single gear transmission).
        private double GetForwardGearRatio() { return m_fwd_gear_ratio; }

        /// Return the reverse gear ratio.
        private double GetReverseGearRatio() { return m_rev_gear_ratio; }

        /// Return the maximum motor torque.
        private double GetMaxTorque() { return m_max_torque; }

        /// Return the maximum motor speed.
        private double GetMaxSpeed() { return m_max_speed; }

        public void SetDriveMode(DriveMode mode)
        {
            m_drive_mode = mode;
            switch (mode)
            {
                case DriveMode.FORWARD:
                    m_current_gear_ratio = GetForwardGearRatio();
                    break;
                case DriveMode.REVERSE:
                    m_current_gear_ratio = GetReverseGearRatio();
                    break;
                case DriveMode.NEUTRAL:
                    m_current_gear_ratio = 1e20;
                    break;
            }
        }

        // Update is called once per frame
        void FixedUpdate()
        {
            shaft_speed = m_driveline.GetDriveshaftSpeed();

            // The motor speed is the shaft speed multiplied by gear ratio inversed:
            m_motorSpeed = shaft_speed / m_current_gear_ratio;

            // The torque depends on speed-torque curve of the motor; here we assume a
            // very simplified model a bit like in DC motors.
            m_motorTorque = GetMaxTorque() - m_motorSpeed * (GetMaxTorque() / GetMaxSpeed());

            // Motor torque is linearly modulated by throttle gas value:
            m_motorTorque *= throttle;

            // The torque at motor shaft:
            m_shaftTorque = m_motorTorque / m_current_gear_ratio;

            // This will normally be in a DriveLine object class.
            //////////////////////////////////////////////////
            // Split the input torque front/back.
            double torque_drive = m_shaftTorque / m_conicalgear_ratio;

            // Apply to driveline
            m_driveline.torque = -torque_drive;

           // leftshaft.SetAppliedTorque(-torque_drive / 2);
           // rightshaft.SetAppliedTorque(-torque_drive / 2);
           // leftshaft.SetAppliedTorque(m_shaftTorque);
            //rightshaft.SetAppliedTorque(m_shaftTorque);

        }
    }
}

