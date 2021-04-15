using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;

namespace chrono
{

    /// @addtogroup chrono_functions
    /// @{

    /// Interface base class for scalar functions of the type:
    ///    y= f(x)
    ///
    /// The ChFunction class defines the base class for all Chrono
    /// functions of type y=f(x), that is scalar functions of an
    /// input variable x (usually, the time). ChFunctions are often
    /// used to set time-dependent properties, for example to set
    /// motion laws in linear actuators, engines, etc.
    /// This base class just represent a constant function of
    /// the type y= C.  Inherited classes must override at least the
    /// Get_y() method, in order to represent more complex functions.
    public class ChFunction : MonoBehaviour
    {

        /// Enumeration of function types.
        public enum FunctionType
        {
            FUNCT_CUSTOM,
            FUNCT_CONST,
            FUNCT_CONSTACC,
            FUNCT_DERIVE,
            FUNCT_FILLET3,
            FUNCT_INTEGRATE,
            FUNCT_MATLAB,
            FUNCT_MIRROR,
            FUNCT_MOCAP,
            FUNCT_NOISE,
            FUNCT_OPERATION,
            FUNCT_OSCILLOSCOPE,
            FUNCT_POLY,
            FUNCT_POLY345,
            FUNCT_RAMP,
            FUNCT_RECORDER,
            FUNCT_REPEAT,
            FUNCT_SEQUENCE,
            FUNCT_SIGMA,
            FUNCT_SINE,
            FUNCT_LAMBDA
        };

        public FunctionType type;

        public ChFunction() { }
        public ChFunction(ChFunction other) { }

        /// "Virtual" copy constructor.
        public virtual ChFunction Clone() { return new ChFunction();  }

        /// Return the unique function type identifier.
        public virtual FunctionType Get_Type() { return FunctionType.FUNCT_CUSTOM; }

        // THE MOST IMPORTANT MEMBER FUNCTIONS
        // At least Get_y() should be overridden by derived classes.

        /// Return the y value of the function, at position x.
        public virtual double Get_y(double x) { return 0; }

        /// Return the dy/dx derivative of the function, at position x.
        /// Note that inherited classes may also avoid overriding this method,
        /// because this base method already provide a general-purpose numerical differentiation
        /// to get dy/dx only from the Get_y() function. (however, if the analytical derivative
        /// is known, it may better to implement a custom method).
        public virtual double Get_y_dx(double x) { return ((Get_y(x + ChMaths.BDF_STEP_LOW) - Get_y(x)) / ChMaths.BDF_STEP_LOW); }

        /// Return the ddy/dxdx double derivative of the function, at position x.
        /// Note that inherited classes may also avoid overriding this method,
        /// because this base method already provide a general-purpose numerical differentiation
        /// to get ddy/dxdx only from the Get_y() function. (however, if the analytical derivative
        /// is known, it may be better to implement a custom method).
        public virtual double Get_y_dxdx(double x) { return ((Get_y_dx(x + ChMaths.BDF_STEP_LOW) - Get_y_dx(x)) / ChMaths.BDF_STEP_LOW); }

        /// Return the weight of the function (useful for
        /// applications where you need to mix different weighted ChFunctions)
        public virtual double Get_weight(double x) { return 1.0; }

        /// Return an estimate of the range of the function argument.
        /// (Can be used for automatic zooming in a GUI)
        public virtual void Estimate_x_range(ref double xmin, ref double xmax) {
            xmin = 0.0;
            xmax = 1.2;
        }

        /// Return an estimate of the range of the function value.
        /// (Can be used for automatic zooming in a GUI)
        public virtual void Estimate_y_range(double xmin, double xmax, ref double ymin, ref double ymax, int derivate) {
            ymin = 10000;
            ymax = -10000;
            for (double mx = xmin; mx < xmax; mx += (xmax - xmin) / 100.0)
            {
                if (Get_y_dN(mx, derivate) < ymin)
                    ymin = Get_y_dN(mx, derivate);
                if (Get_y_dN(mx, derivate) > ymax)
                    ymax = Get_y_dN(mx, derivate);
            }
            if (Math.Abs(ymax - ymin) < 10e-12)
            {
                ymin = -0.5;
                ymax = +1.0;
            }
            ymax += 0.12 * (ymax - ymin);
            ymin -= 0.12 * (ymax - ymin);
        }

        /// Return the function derivative of specified order at the given point.
        /// Note that only order = 0, 1, or 2 is supported.
        public virtual double Get_y_dN(double x, int derivate) {
            switch (derivate)
            {
                case 0:
                    return Get_y(x);
                case 1:
                    return Get_y_dx(x);
                case 2:
                    return Get_y_dxdx(x);
                default:
                    return Get_y(x);
            }
        }

        /// Update could be implemented by children classes, ex. to launch callbacks
        public virtual void update(double x) { }

        //
        // Some analysis functions. If derivate=0, they are applied on y(x), if derivate =1, on dy/dx, etc.
        //

        /// Compute the maximum of y(x) in a range xmin-xmax, using a sampling method.
        public virtual double Compute_max(double xmin, double xmax, double sampling_step, int derivate) {
            double mret = -1E30;
            for (double mx = xmin; mx <= xmax; mx += sampling_step)
            {
                if (this.Get_y_dN(mx, derivate) > mret)
                    mret = this.Get_y_dN(mx, derivate);
            }
            return mret;
        }
        /// Compute the minimum of y(x) in a range xmin-xmax, using a sampling method.
        public virtual double Compute_min(double xmin, double xmax, double sampling_step, int derivate) {
            double mret = +1E30;
            for (double mx = xmin; mx <= xmax; mx += sampling_step)
            {
                if (this.Get_y_dN(mx, derivate) < mret)
                    mret = this.Get_y_dN(mx, derivate);
            }
            return mret;
        }
        /// Compute the mean value of y(x) in a range xmin-xmax, using a sampling method.
        public virtual double Compute_mean(double xmin, double xmax, double sampling_step, int derivate) {
            double mret = 0;
            int numpts = 0;
            for (double mx = xmin; mx <= xmax; mx = mx + sampling_step)
            {
                numpts++;
                mret += this.Get_y_dN(mx, derivate);
            }
            return mret / ((double)numpts);
        }
        /// Compute the square mean val. of y(x) in a range xmin-xmax, using sampling.
        public virtual double Compute_sqrmean(double xmin, double xmax, double sampling_step, int derivate) {
            double mret = 0;
            int numpts = 0;
            for (double mx = xmin; mx <= xmax; mx = mx + sampling_step)
            {
                numpts++;
                mret += Math.Pow(this.Get_y_dN(mx, derivate), 2.0);
            }
            return Math.Sqrt(mret / ((double)numpts));
        }
        /// Compute the integral of y(x) in a range xmin-xmax, using a sampling method.
        public virtual double Compute_int(double xmin, double xmax, double sampling_step, int derivate) {
            double mret = 0;
            double ya = this.Get_y_dN(xmin, derivate);
            double yb = 0;
            for (double mx = xmin + sampling_step; mx <= xmax; mx += sampling_step)
            {
                yb = this.Get_y_dN(mx, derivate);
                mret += sampling_step * (ya + yb) * 0.5;  // trapezoidal quadrature
                ya = yb;
            }
            return mret;
        }
        /// Computes the positive acceleration coefficient (inherited classes should customize this).
        public virtual double Get_Ca_pos() { return 0; }
        /// Compute the positive acceleration coefficient (inherited classes should customize this).
        public virtual double Get_Ca_neg() { return 0; }
        /// Compute the speed coefficient (inherited classes must customize this).
        public virtual double Get_Cv() { return 0; }

        // If the function has some handles (mouse-sensible markers on screen), implement these functions

        /// Return the number of handles of the function
        public virtual int HandleNumber() { return 0; }

        /// Get the x and y position of handle, given identifier.
        /// If set mode, x and y values are stored. Return false if handle not found.
        public virtual bool HandleAccess(int handle_id, double mx, double my, bool set_mode) { return true; }

    }
}