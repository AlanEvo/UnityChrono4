using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono
{

    public class CustomTorqueFunctor : ChLinkRotSpringCB.TorqueFunctor
    {
        public double spring_coef = 40;
        public double damping_coef = 2;
        public double rest_angle = ChMaths.CH_C_PI / 6;

        public override double this[double time,             //< current time
                                  double angle,            //< relative angle of rotation
                                  double vel,              //< relative angular speed
                                  ChLinkRotSpringCB link  //< back-pointer to associated link
                                  ]
        {
            get
            {
                double torque = -spring_coef * (angle - rest_angle) - damping_coef * vel;
                return torque;
            }
        }

    }
}
