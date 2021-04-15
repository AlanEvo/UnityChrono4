using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono
{

    /// Linear function (like a straight ramp):
    /// y = y0 + x * speed
    public class ChFunction_Ramp : ChFunction
    {

        public double y0;
        public double ang;


        public ChFunction_Ramp() { y0 = 0; ang = 1; }
        public ChFunction_Ramp(double m_y0, double m_ang) { y0 = m_y0; ang = m_ang; }
        public ChFunction_Ramp(ChFunction_Ramp other)
        {
            y0 = other.y0;
            ang = other.ang;
        }

        /// "Virtual" copy constructor (covariant return type).
        public override ChFunction Clone() { return new ChFunction_Ramp(this); }

        public override FunctionType Get_Type() { return FunctionType.FUNCT_RAMP; }

        public override double Get_y(double x) { return (y0 + (x * ang)); }
        public override double Get_y_dx(double x) { return (ang); }
        public override double Get_y_dxdx(double x) { return 0; }

        /// The value for x=0;
        public void Set_y0(double m_y0) { y0 = m_y0; }
        public double Get_y0() { return y0; }

        /// The angular coefficient.
        public void Set_ang(double m_ang) { ang = m_ang; }
        public double Get_ang() { return ang; }

        public void Awake()
        {
            Set_y0(y0);
            Set_ang(ang);
        }
    }


}
