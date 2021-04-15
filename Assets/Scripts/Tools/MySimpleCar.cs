using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono {

    public class MySimpleCar : MonoBehaviour
    {
        // THE DATA

        public ChLinkMotorRotationTorque link_engineL;
        public ChLinkMotorRotationTorque link_engineR;
        public ChLinkDistance link_distLSTEER;
        public ChLinkDistance link_distRSTEER;

        public double throttle;          // actual value 0...1 of gas throttle.
        public double conic_tau;         // the transmission ratio of the conic gears at the rear axle
        public double gear_tau;          // the actual tau of the gear
        public double max_motor_torque;  // the max torque of the motor [Nm];
        public double max_motor_speed;   // the max rotation speed of the motor [rads/s]
        public double horizontal;

        // Start is called before the first frame update
        void Start()
        {
            throttle = 0;  // initially, gas throttle is 0.
            conic_tau = 0.2;
            gear_tau = 0.3;
            max_motor_torque = 80;
            max_motor_speed = 800;
        }

        // Update is called once per frame
        void FixedUpdate()
        {

            horizontal = Input.GetAxis("Horizontal");
            throttle = -Input.GetAxis("Vertical");

            double newsteer = 0.18 * (((double)(horizontal)));
            // set the steering, moving horizontally the endpoints of the steer rod endpoint on truss.
            this.link_distRSTEER.SetEndPoint1Rel(new ChVector(0.5 + newsteer, 0.21, 1.4));
            this.link_distLSTEER.SetEndPoint1Rel(new ChVector(-0.5 + newsteer, 0.21, 1.4));

            ComputeWheelTorque();
        }

        // This can be used, at each time step, to compute the actual value of torque
        // transmitted to the wheels, according to gas throttle / speed / gear value.
        // The following is a very simplified model (the torque curve of the motor is linear
        // and no latency or inertial or clutch effects in gear train are considered.)
        double ComputeWheelTorque()
        {
            // Assume clutch is never used. Given the kinematics of differential,
            // the speed of the engine transmission shaft is the average of the two wheel speeds,
            // multiplied the conic gear transmission ratio inversed:
            double shaftspeed = (1.0 / this.conic_tau) * 0.5 *
                                (this.link_engineL.GetMotorRot_dt() + this.link_engineR.GetMotorRot_dt());
            // The motorspeed is the shaft speed multiplied by gear ratio inversed:
            double motorspeed = (1.0 / this.gear_tau) * shaftspeed;
            // The torque depends on speed-torque curve of the motor: here we assume a
            // very simplified model a bit like in DC motors:
            double motortorque = max_motor_torque - motorspeed * (max_motor_torque / max_motor_speed);
            // Motor torque is linearly modulated by throttle gas value:
            motortorque = motortorque * this.throttle;
            // The torque at motor shaft:
            double shafttorque = motortorque * (1.0 / this.gear_tau);
            // The torque at wheels - for each wheel, given the differential transmission,
            // it is half of the shaft torque  (multiplied the conic gear transmission ratio)
            double singlewheeltorque = 0.5 * shafttorque * (1.0 / this.conic_tau);
            // Set the wheel torque in both 'engine' links, connecting the wheels to the truss;
            ChFunction_Const mfun1 = (ChFunction_Const)link_engineL.GetTorqueFunction();
            ChFunction_Const mfun2 = (ChFunction_Const)link_engineR.GetTorqueFunction();
            //if (mfun == link_engineL as )
            mfun1.Set_yconst(singlewheeltorque);
            mfun2.Set_yconst(singlewheeltorque);
            // if (mfun = (ChFunction_Const)(link_engineR.Get_tor_funct()))
            // mfun.Set_yconst(singlewheeltorque);
            // debug:print infos on screen:
            // GetLog() << "motor torque="<< motortorque<< "  speed=" << motorspeed << "  wheel torqe=" << singlewheeltorque
            // <<"\n";
            // If needed, return also the value of wheel torque:
            return singlewheeltorque;
        }
    }
}