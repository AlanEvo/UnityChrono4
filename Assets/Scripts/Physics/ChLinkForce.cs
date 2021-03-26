using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;

namespace chrono
{
    /// Class for forces in link joints of type ChLink().
    public class ChLinkForce // : MonoBehaviour
    {

        private bool active;  //< true/false

        private double iforce;             //< impressed force
        private ChFunction modul_iforce;  //< time-modulation of imp. force

        private double K;             //< stiffness of the dof
        private ChFunction modul_K;  //< modulation of K along the dof coord

        private double R;             //< damping of the dof
        private ChFunction modul_R;  //< modulation of R along the dof coord


        public ChLinkForce()
        {
            active = false;
            iforce = 0;
            K = 0;
            R = 0;
            modul_iforce = new ChFunction_Const(1);  // default: const.modulation of iforce
            modul_K = new ChFunction_Const(1);       // default: const.modulation of K
            modul_R = new ChFunction_Const(1);       // default: const.modulation of R
        }

        public ChLinkForce(ChLinkForce other)
        {
            active = other.active;

            iforce = other.iforce;
            K = other.K;
            R = other.R;

            // replace functions:
            modul_iforce = null;
            modul_K = null;
            modul_R = null;
            modul_iforce = other.modul_iforce.Clone();
            modul_K = other.modul_K.Clone();
            modul_R = other.modul_R.Clone();
        }

        /// "Virtual" copy constructor (covariant return type).
        public ChLinkForce Clone() { return new ChLinkForce(this); }


        public bool Get_active() { return active; }
        public void Set_active(bool m_a) { active = m_a; }

        public double Get_iforce() { return iforce; }
        public void Set_iforce(double m_f) { iforce = m_f; }

        public double Get_K() { return K; }
        public void Set_K(double m_K) { K = m_K; }

        public double Get_R() { return R; }
        public void Set_R(double m_R) { R = m_R; }

        public ChFunction Get_modul_iforce() { return modul_iforce; }
        public ChFunction Get_modul_K() { return modul_K; }
        public ChFunction Get_modul_R() { return modul_R; }

        public void Set_modul_iforce(ChFunction m_funct) {
            if (modul_iforce != null)
                modul_iforce = null;
            modul_iforce = m_funct;
        }
        public void Set_modul_K(ChFunction m_funct) {
            if (modul_K != null)
                modul_K = null;
            modul_K = m_funct;
        }
        public void Set_modul_R(ChFunction m_funct) {
            if (modul_R != null)
                modul_R = null;
            modul_R = m_funct;
        }

        public double Get_Kcurrent(double x, double x_dt, double t) {
            double mK = 0;
            if (active)
            {
                double modulator;

                if (modul_K != null)
                {
                    modulator = modul_K.Get_y(x);
                }
                else
                {
                    modulator = 1;
                }
                mK = K * modulator;
            }
            return mK;
        }
        public double Get_Rcurrent(double x, double x_dt, double t) {
            double mR = 0;
            if (active)
            {
                double modulator;

                if (modul_R != null)
                {
                    modulator = modul_R.Get_y(x);
                }
                else
                {
                    modulator = 1;
                }
                mR = R * modulator;
            }
            return mR;
        }
        public double Get_iFcurrent(double x, double x_dt, double t) {
            double mforce = 0;
            if (active)
            {
                double modulator;

                // the internal force contribute = iforce
                if (modul_iforce != null)
                {
                    modulator = modul_iforce.Get_y(t);
                }
                else
                {
                    modulator = 1;
                }
                mforce = iforce * modulator;
            }
            return mforce;
        }

        // This is the most important function: it is called to evaluate
        // the internal force at instant t, position x and speed x_dt
        public double Get_Force(double x, double x_dt, double t) {
            double mforce = 0;
            if (active)
            {
                double modulator;

                // the internal force contribute = iforce
                if (modul_iforce != null)
                {
                    modulator = modul_iforce.Get_y(t);
                }
                else
                {
                    modulator = 1;
                }
                mforce = iforce * modulator;

                // the stiffness contribute =  - K x
                if (modul_K != null)
                {
                    modulator = modul_K.Get_y(x);
                }
                else
                {
                    modulator = 1;
                }
                mforce -= (K * modulator) * x;

                // the damping contribute =  - R x_dt
                if (modul_R != null)
                {
                    modulator = modul_R.Get_y(x);
                }
                else
                {
                    modulator = 1;
                }
                mforce -= (R * modulator) * x_dt;
            }

            return mforce;
        }

    };

}
