using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono
{

    public abstract class ForceFunctor : MonoBehaviour
    {
        /// Calculate and return the general spring-damper force at the specified configuration.
        public virtual double this[double time,          //< current time
                              double rest_length,         //< undeformed length
                              double length,              //< current length
                              double vel,                 //< current velocity (positive when extending)
                              ChLinkSpringCB link         //< back-pointer to associated link
                              ]
        { get { return 0; } }

        public void Awake()
        {
            int a = 1;
        }
    }

}
