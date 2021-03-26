using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;


namespace chrono
{
    /// Constant function:  y = C
    public class ChFunction_Const : ChFunction
    {

        private double C;

        public double constant_val = 0;


        public ChFunction_Const() { C = 0; }
        public ChFunction_Const(double y_constant) { C = y_constant; }
        public ChFunction_Const(ChFunction_Const other) { C = other.C; }

        /// "Virtual" copy constructor (covariant return type).
        public override ChFunction Clone()
        {
            return new ChFunction_Const(this);
        }

        /// Returns the y value of the function, at position x.
        public override FunctionType Get_Type() { return FunctionType.FUNCT_CONST; }

        public override double Get_y(double x) { return C; }
        public override double Get_y_dx(double x) { return 0; }
        public override double Get_y_dxdx(double x) { return 0; }

        /// Set the constant C for the function, y=C.
        public void Set_yconst(double y_constant) { C = y_constant; }
        /// Get the constant C for the function, y=C.
        public double Get_yconst() { return C; }

        public void Awake()
        {
            Set_yconst(constant_val);
        }


    }
}
