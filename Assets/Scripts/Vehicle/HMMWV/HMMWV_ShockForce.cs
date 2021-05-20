using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono
{

    public class HMMWV_ShockForce : ChLinkSpringCB.ForceFunctor
    {
       /* public HMMWV_ShockForce(double midstroke_compression_slope,
                 double midstroke_rebound_slope,
                 double bumpstop_compression_slope,
                 double bumpstop_rebound_slope,
                 double metalmetal_slope,
                 double min_bumpstop_compression_force,
                 double midstroke_lower_bound,
                 double midstroke_upper_bound,
                 double metalmetal_lower_bound,
                 double metalmetal_upper_bound)
        {
            m_ms_compr = midstroke_compression_slope;
            m_ms_rebound = midstroke_rebound_slope;
            m_bs_compr = bumpstop_compression_slope;
            m_bs_rebound = bumpstop_rebound_slope;
            m_metal_K = metalmetal_slope;
            m_F0 = min_bumpstop_compression_force;
            m_ms_min_length = midstroke_lower_bound;
            m_ms_max_length = midstroke_upper_bound;
            m_min_length = metalmetal_lower_bound;
            m_max_length = metalmetal_upper_bound;
        }*/

        public override double this[double time,          //< current time
                              double rest_length,         //< undeformed length
                              double length,              //< current length
                              double vel,                 //< current velocity (positive when extending)
                              ChLinkSpringCB link         //< back-pointer to associated link
                              ]
        { get {
                double force = 0;

                // Calculate Damping Force
                if (vel >= 0)
                {
                    force = (length >= m_ms_max_length) ? -m_bs_rebound * vel : -m_ms_rebound * vel;
                }
                else
                {
                    force = (length <= m_ms_min_length) ? -m_bs_compr * vel : -m_ms_compr * vel;
                }

                // Add in Shock metal to metal contact force
                if (length <= m_min_length)
                {
                    force = m_metal_K * (m_min_length - length);
                }
                else if (length >= m_max_length)
                {
                    force = -m_metal_K * (length - m_max_length);
                }

                return force;
            } 
        
        }

        public void Start()
        {
           
        }


        public double m_ms_compr;
        public double m_ms_rebound;
        public double m_bs_compr;
        public double m_bs_rebound;
        public double m_metal_K;
        public double m_F0;
        public double m_ms_min_length;
        public double m_ms_max_length;
        public double m_min_length;
        public double m_max_length;
    }

}
