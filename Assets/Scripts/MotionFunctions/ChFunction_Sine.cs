using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;

namespace chrono
{

    /// SINE FUNCTION:
    /// y = sin (phase + w*x )     w=2*PI*freq
    public class ChFunction_Sine : ChFunction
    {

        public double amp;
        public double phase;
        public double freq;
        public double w;


        public ChFunction_Sine()
        {
            amp = 1; phase = 0; freq = 1; w = 2 * ChMaths.CH_C_PI;
        }
        public ChFunction_Sine(double m_phase, double m_freq, double m_amp)
        {
            amp = m_amp; phase = m_phase; freq = m_freq; w = 2 * ChMaths.CH_C_PI * m_freq;
        }
        public ChFunction_Sine(ChFunction_Sine other)
        {
            amp = other.amp;
            phase = other.phase;
            freq = other.freq;
            w = other.w;
        }

        /// "Virtual" copy constructor (covariant return type).
        public override ChFunction Clone(){ return new ChFunction_Sine(this);
    }

        public override FunctionType Get_Type()  { return FunctionType.FUNCT_SINE; }

        public override double Get_y(double x) {
            return amp * (Math.Sin(phase + w * x));
        }
        public override double Get_y_dx(double x) {
            return amp * w * (Math.Cos(phase + w * x));
        }
        public override double Get_y_dxdx(double x) {
            return amp * -w * w * (Math.Sin(phase + w * x));
        }

        public void Set_phase(double m_phase) { phase = m_phase; }
        public void Set_freq(double m_freq)
        {
            freq = m_freq;
            w = 2 * ChMaths.CH_C_PI * freq;
        }
        public void Set_w(double m_w)
        {
            w = m_w;
            freq = w / (2 * ChMaths.CH_C_PI);
        }
        public void Set_amp(double m_amp) { amp = m_amp; }

        public double Get_phase() { return phase; }
        public double Get_freq() { return freq; }
        public double Get_w() { return w; }
        public double Get_amp() { return amp; }
    }

}
