using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono
{

    /// Utility class for specifying a linear translational spring force.
    public class LinearSpringForce : ChLinkSpringCB.ForceFunctor
    {

       // public LinearSpringForce(double k) { m_k = k; }
        public override double this[double time,
                          double rest_length,
                          double length,
                          double vel,
                          ChLinkSpringCB link]
        {
            get
            {
                return -m_k * (length - rest_length);
            }
        }

        public double m_k;
    }
};



