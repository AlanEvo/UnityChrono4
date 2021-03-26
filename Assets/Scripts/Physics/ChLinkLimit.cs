using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;


namespace chrono
{
    /// Class for limits in link joints (for example limits on elbow or knee rotations, etc.)
    /// Old code. Must be improved.
    public class ChLinkLimit
    {

        private bool active = false;  // true|false
        private bool penalty_only = false;
        private bool polar = false;
        private bool rotation = false;
        private double max = 0;
        private double min = 0;
        private double maxCushion = 0;
        private double minCushion = 0;
        private double Kmax = 0;
        private double Kmin = 0;
        private double Rmax = 0;
        private double Rmin = 0;
        private double maxElastic = 0;
        private double minElastic = 0;
        private ChFunction modul_Kmax;
        private ChFunction modul_Kmin;
        private ChFunction modul_Rmax;
        private ChFunction modul_Rmin;
        private ChFunction polar_Max;


        public ChConstraintTwoBodies constr_upper = new ChConstraintTwoBodies();
        public ChConstraintTwoBodies constr_lower = new ChConstraintTwoBodies();

        public ChLinkLimit()
        {
            active = false;
            penalty_only = false;
            polar = false;
            rotation = false;
            max = 1;
            min = -1;
            maxCushion = 0;
            minCushion = 0;
            Kmax = 1000;
            Kmin = 1000;
            Rmax = 100;
            Rmin = 100;
            minElastic = 0;
            maxElastic = 0;
            modul_Kmax = new ChFunction_Const(1);  // default: const.modulation of K
            modul_Kmin = new ChFunction_Const(1);  // default: const.modulation of K
            modul_Rmax = new ChFunction_Const(1);  // default: const.modulation of K
            modul_Rmin = new ChFunction_Const(1);  // default: const.modulation of K
            polar_Max = new ChFunction_Const(1);   // default function for polar limits
            constr_upper.SetMode(eChConstraintMode.CONSTRAINT_UNILATERAL);
            constr_lower.SetMode(eChConstraintMode.CONSTRAINT_UNILATERAL);
        }
        public ChLinkLimit(ChLinkLimit other)
        {
            active = other.active;
            penalty_only = other.penalty_only;
            polar = other.polar;
            rotation = other.rotation;
            max = other.max;
            min = other.min;
            maxCushion = other.maxCushion;
            minCushion = other.minCushion;
            Kmax = other.Kmax;
            Kmin = other.Kmin;
            Rmax = other.Rmax;
            Rmin = other.Rmin;
            minElastic = other.minElastic;
            maxElastic = other.maxElastic;
            // replace functions:
            modul_Kmax = other.modul_Kmax.Clone();
            modul_Kmin = other.modul_Kmin.Clone();
            modul_Rmax = other.modul_Rmax.Clone();
            modul_Rmin = other.modul_Rmin.Clone();
            polar_Max = other.polar_Max.Clone();
        }

        public ChLinkLimit Clone() { return new ChLinkLimit(this); }

        public bool Get_active() { return active; }
        public bool Get_penalty() { return penalty_only; }
        public bool Get_polar() { return polar; }
        public bool Get_rotation() { return rotation; }
        public double Get_max() { return max; }
        public double Get_min() { return min; }
        public double Get_maxCushion() { return maxCushion; }
        public double Get_minCushion() { return minCushion; }
        public double Get_Kmax() { return Kmax; }
        public double Get_Kmin() { return Kmin; }
        public double Get_Rmax() { return Rmax; }
        public double Get_Rmin() { return Rmin; }
        public double Get_maxElastic() { return maxElastic; }
        public double Get_minElastic() { return minElastic; }
        public ChFunction GetModul_Kmax() { return modul_Kmax; }
        public ChFunction GetModul_Kmin() { return modul_Kmin; }
        public ChFunction GetModul_Rmax() { return modul_Rmax; }
        public ChFunction GetModul_Rmin() { return modul_Rmin; }
        public ChFunction GetPolar_Max() { return polar_Max; }
        public double Get_polar_max(double pol_ang)
        {
            if (polar_Max == null)
                return 0.001;
            return polar_Max.Get_y(pol_ang);
        }

        public void Set_active(bool m_active) { active = m_active; }
        public void Set_penalty(bool m_active) { penalty_only = m_active; }
        public void Set_polar(bool m_pol) { polar = m_pol; }
        public void Set_rotation(bool m_rot) { rotation = m_rot; }
        public void Set_max(double m_max)
        {
            max = m_max;
            if (max < min)
                min = max;
            if ((max - maxCushion) < min)
                maxCushion = max - min;
            if ((max - maxCushion) < (min + minCushion))
                minCushion = max - min - maxCushion;
            constr_upper.SetActive(true);
        }
        public void Set_min(double m_min)
        {
            min = m_min;
            if (min > max)
                max = min;
            if ((min + minCushion) > max)
                minCushion = max - min;
            if ((min + minCushion) > (max - maxCushion))
                maxCushion = max - min - minCushion;
            constr_lower.SetActive(true);
        }
        public void Set_maxCushion(double m_maxCushion)
        {
            maxCushion = m_maxCushion;
            if ((max - maxCushion) < min)
                maxCushion = max - min;
            if ((max - maxCushion) < (min + minCushion))
                minCushion = max - min - maxCushion;
        }
        public void Set_minCushion(double m_minCushion)
        {
            minCushion = m_minCushion;
            if ((min + minCushion) > max)
                minCushion = max - min;
            if ((min + minCushion) > (max - maxCushion))
                maxCushion = max - min - minCushion;
        }
        public void Set_Kmax(double m_K) { Kmax = m_K; }
        public void Set_Kmin(double m_K) { Kmin = m_K; }
        public void Set_Rmax(double m_R) { Rmax = m_R; }
        public void Set_Rmin(double m_R) { Rmin = m_R; }
        public void Set_maxElastic(double m_e) { maxElastic = m_e; }
        public void Set_minElastic(double m_e) { minElastic = m_e; }
        public void SetModul_Kmax(ChFunction m_funct)
        {
            if (modul_Kmax != null)
                modul_Kmax = null;
            modul_Kmax = m_funct;
        }
        public void SetModul_Kmin(ChFunction m_funct)
        {
            if (modul_Kmin != null)
                modul_Kmin = null;
            modul_Kmin = m_funct;
        }
        public void SetModul_Rmax(ChFunction m_funct)
        {
            if (modul_Rmax!= null)
                modul_Rmax = null;
            modul_Rmax = m_funct;
        }
        public void SetModul_Rmin(ChFunction m_funct) {
            if (modul_Rmin != null)
                modul_Rmin = null;
            modul_Rmin = m_funct;
        }
        public void SetPolar_Max(ChFunction m_funct) {
            if (polar_Max != null)
                polar_Max = null;
            polar_Max = m_funct;
        }

        /// Return negative violation when x<min, or positive if x>max
        public double GetViolation(double x)
        {
            if (!active || penalty_only)
            {
                return 0;
            }
            else
            {
                if ((x > min) && (x < max))
                    return 0;
                if (x <= min)
                    return (x - min);
                if (x >= max)
                    return (x - max);
            }
            return 0;
        }

        public double GetForce(double x, double x_dt)
        {
            double cush_coord;
            double cush_coord_norm;
            double force;
            double m_min, m_max;

            if (!penalty_only)
            {
                m_min = min;
                m_max = max;
            }
            else
            {
                m_min = -999999999;
                m_max = 999999999;
            }

            if ((x > m_min) && (x < (min + minCushion)))
            {
                cush_coord = (min + minCushion) - x;

                if (minCushion >= 0.0000001)
                    cush_coord_norm = cush_coord / minCushion;
                else
                    cush_coord_norm = 1;

                if (cush_coord_norm > 1)
                    cush_coord_norm = 1;  // clip cushion forces at stopper limit

                force = cush_coord * Kmin * modul_Kmin.Get_y(cush_coord_norm);
                force += (-x_dt) * Rmin * modul_Rmin.Get_y(cush_coord_norm);
                if (force < 0)
                {
                    force = 0;
                }  // damping could cause neg force while going away,
                   // so -as the limit is not "sticky"- clip force sign.

                return (force);
            }

            if ((x < m_max) && (x > (max - maxCushion)))
            {
                cush_coord = x - (max - maxCushion);

                if (maxCushion >= 0.0000001)
                    cush_coord_norm = cush_coord / maxCushion;
                else
                    cush_coord_norm = 1;

                if (cush_coord_norm > 1)
                    cush_coord_norm = 1;  // clip cushion forces at stopper limit

                force = (-cush_coord) * Kmax * modul_Kmax.Get_y(cush_coord_norm);
                force += (-x_dt) * Rmax * modul_Rmax.Get_y(cush_coord_norm);
                if (force > 0)
                {
                    force = 0;
                }  // damping could cause pos force while going away,
                   // so -as the limit is not "sticky"- clip force sign.
                return (force);
            }
            return 0;
        }
        public double GetPolarForce(double x, double x_dt, double pol_ang)
        {
            double cush_coord;
            double cush_coord_norm;
            double cushion_thick;
            double force;
            double m_max;
            double ang_max;

            if (polar_Max == null)
                return 0;

            if (!penalty_only)
            {
                m_max = max;
            }
            else
            {
                m_max = 999999999;
            }

            ang_max = polar_Max.Get_y(pol_ang);

            if ((x < m_max) && (x > (ang_max - maxCushion)))
            {
                cushion_thick = maxCushion;
                if (cushion_thick > ang_max)
                    cushion_thick = ang_max;

                cush_coord = x - (ang_max - maxCushion);

                if (cushion_thick >= 0.0000001)
                    cush_coord_norm = cush_coord / cushion_thick;
                else
                    cush_coord_norm = 1;

                if (cush_coord_norm > 1)
                    cush_coord_norm = 1;  // clip cushion forces at stopper limit

                force = (-cush_coord) * Kmax * modul_Kmax.Get_y(cush_coord_norm);
                force += (-x_dt) * Rmax * modul_Rmax.Get_y(cush_coord_norm);
                if (force > 0)
                {
                    force = 0;
                }  // damping could cause pos force while going away,
                   // so -as the limit is not "sticky"- clip force sign.
                return (force);
            }
            return 0;
        }

    }
}
