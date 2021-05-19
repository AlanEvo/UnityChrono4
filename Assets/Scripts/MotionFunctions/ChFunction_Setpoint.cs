using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono
{

    /// Function that returns Y from an externally-provided value,
    /// as a ZOH (zero order hold) block. This means that the Y value
    /// does NOT change if you call Get_y(double x) with different values
    /// of x, unless you keep the setpoint Y updated via multiple
    /// calls to SetSetpoint(), for example calling SetSetpoint()
    /// at each timestep in the simulation loop.
    /// Also first two derivatives (speed, accel.) will persist until
    /// next SetSetpoint() call.
    /// Function of this class are most often functions of time.
    public class ChFunction_Setpoint : ChFunction
    {

        public double Y;
        public double Y_dx;
        public double Y_dxdx;
        public double last_x;
        public double last_Y;
        public double last_Y_dx;


        public ChFunction_Setpoint()
        {
            Y = 0;
            Y_dx = 0;
            Y_dxdx = 0;
            last_x = 0;
            last_Y = 0;
            last_Y_dx = 0;
        }

        public ChFunction_Setpoint(ChFunction_Setpoint other)
        {
            Y = other.Y;
            Y_dx = other.Y_dx;
            Y_dxdx = other.Y_dxdx;
            last_x = other.last_x;
            last_Y = other.last_Y;
            last_Y_dx = other.last_Y_dx;
        }

        /// "Virtual" copy constructor (covariant return type).
        public override ChFunction Clone()
        {
            return new ChFunction_Setpoint(this);
        }

        public override double Get_y(double x) { return Y; }
        public override double Get_y_dx(double x) { return Y_dx; }
        public override double Get_y_dxdx(double x) { return Y_dxdx; }

        /// Set the setpoint, and compute its derivatives (speed, acceleration) automatically
        /// by backward differentiation (only if x is called at increasing small steps).
        /// All values will persist indefinitely until next call.
        public virtual void SetSetpoint(double setpoint, double x)
        {
            Y = setpoint;
            if (x > this.last_x)
            {
                double dx = x - last_x;
                Y_dx = (Y - last_Y) / dx;
                Y_dxdx = (Y_dx - last_Y_dx) / dx;
            }
            last_x = x;
            last_Y = Y;
            last_Y_dx = Y_dx;
        }

        /// Set the setpoint, and also its derivatives.
        /// All values will persist indefinitely until next call.
        public virtual void SetSetpointAndDerivatives(double setpoint, double setpoint_dx, double setpoint_dxdx)
        {
            Y = setpoint;
            Y_dx = setpoint_dx;
            Y_dxdx = setpoint_dxdx;
        }

        /// Get the last set setpoint
        public double GetSetpoint() { return Y; }

        /// Update could be implemented by children classes, ex. to launch callbacks
        public override void update(double x) { }



    }
}