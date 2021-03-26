using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono
{

    public class CustomSpringForce : ChLinkSpringCB.ForceFunctor
    {
        public double rest_length = 1.5;
        public double spring_coef = 50;
        public double damping_coef = 1;

        public override double this[double time,          // current time
                              double rest_length,   // undeformed length
                              double length,        // current length
                              double vel,           // current velocity (positive when extending)
                              ChLinkSpringCB link  // back-pointer to associated link
                              ]
        {
            get {
                double force = -spring_coef * (length - rest_length) - damping_coef * vel;
                return force;
            }
        }
}

}
