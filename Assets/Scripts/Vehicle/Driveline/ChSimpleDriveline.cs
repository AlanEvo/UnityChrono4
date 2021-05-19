using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
using UnityEditor;

namespace chrono
{
    /// @addtogroup vehicle_wheeled_driveline
    /// @{

    /// Simple driveline model. This template can be used to model a 4WD driveline.
    /// It uses a constant front/rear torque split (a value between 0 and 1) and a
    /// simple model for Torsen limited-slip differentials.
    public class ChSimpleDriveline : MonoBehaviour
    {
        // public ChShaft m_driveshaft;  //< handle to the shaft connection to the powertrain

        // public List<int> m_driven_axles = new List<int>();  //< indexes of the driven vehicle axles

        public ChShaft m_front_left;
        public ChShaft m_front_right;
        public ChShaft m_rear_left;
        public ChShaft m_rear_right;

        public double m_front_torque_frac;
        public double m_front_diff_bias;
        public double m_rear_diff_bias;

        public double torque;

        // Use this for initialization
        void Start()
        {

        }

        // -----------------------------------------------------------------------------
        // This utility function implements a simple model of Torsen limited-slip
        // differential with a max_bias:1 torque bias ratio.
        // We hardcode the speed difference range over which the torque bias grows from
        // a value of 1 to a value of max_bias to the interval [0.25, 0.5].
        // -----------------------------------------------------------------------------
        void differentialSplit(double torque,
                               double max_bias,
                               double speed_left,
                               double speed_right,
                               ref double torque_left,
                               ref double torque_right)
        {
            double diff = Math.Abs(speed_left - speed_right);

            // The bias grows from 1 at diff=0.25 to max_bias at diff=0.5
            double bias = 1;
            if (diff > 0.5)
                bias = max_bias;
            else if (diff > 0.25)
                bias = 4 * (max_bias - 1) * diff + (2 - max_bias);

            // Split torque to the slow and fast wheels.
            double alpha = bias / (1 + bias);
            double slow = alpha * torque;
            double fast = torque - slow;

            if (Math.Abs(speed_left) < Math.Abs(speed_right))
            {
                torque_left = slow;
                torque_right = fast;
            }
            else
            {
                torque_left = fast;
                torque_right = slow;
            }
        }

        // Update is called once per frame
        void FixedUpdate()
        {
            // Split the input torque front/back.
            double torque_front = torque * GetFrontTorqueFraction();
            double torque_rear = torque - torque_front;

            // Split the axle torques for the corresponding left/right wheels and apply
            // them to the suspension wheel shafts.
            double torque_left = 0;
            double torque_right = 0;

            differentialSplit(torque_front, GetFrontDifferentialMaxBias(), m_front_left.GetPos_dt(),
                              m_front_right.GetPos_dt(), ref torque_left, ref torque_right);
            m_front_left.SetAppliedTorque(-torque_left);
            m_front_right.SetAppliedTorque(-torque_right);

            differentialSplit(torque_rear, GetRearDifferentialMaxBias(), m_rear_left.GetPos_dt(), m_rear_right.GetPos_dt(),
                              ref torque_left, ref torque_right);
            m_rear_left.SetAppliedTorque(-torque_left);
            m_rear_right.SetAppliedTorque(-torque_right);
        }

        // -----------------------------------------------------------------------------
        // -----------------------------------------------------------------------------
        public double GetDriveshaftSpeed()
        {
            double speed_front = 0.5 * (m_front_left.GetPos_dt() + m_front_right.GetPos_dt());
            double speed_rear = 0.5 * (m_rear_left.GetPos_dt() + m_rear_right.GetPos_dt());
            double alpha = GetFrontTorqueFraction();

            return alpha * speed_front + (1 - alpha) * speed_rear;
        }

        /// Return the number of driven axles.
        public virtual int GetNumDrivenAxles() { return 2; }

        /// Return the front torque fraction [0,1].
        protected virtual double GetFrontTorqueFraction() { return m_front_torque_frac; }

        /// Return the torque bias ratio for the front differential.
        /// This is a simple model of a Torsen limited-slip differential.
        protected virtual double GetFrontDifferentialMaxBias() { return m_front_diff_bias; }

        /// Return the torque bias ratio for the rear differential.
        /// This is a simple model of a Torsen limited-slip differential.
        protected virtual double GetRearDifferentialMaxBias() { return m_rear_diff_bias; }
    }
}

