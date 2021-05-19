using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;

namespace chrono
{

    public static class ChMaths
    {
        public const double CH_C_PI = 3.141592653589793238462643383279;
        public const double CH_C_PI_2 = 1.570796326794896619231321691639;
        public const double CH_C_2PI = 6.283185307179586476925286766559;
        public const double CH_C_PI_4 = 0.785398163397448309615660845819;
        public const double CH_C_DEG_TO_RAD = 3.1415926535897932384626433832795 / 180.0;
        public const double CH_C_RAD_TO_DEG = 180.0 / 3.1415926535897932384626433832795;

        public const double BDF_STEP_HIGH = 1e-4;
        public const double BDF_STEP_LOW = 1e-7;

        public const long IA = 16807;
        public const long IM = 2147483647;
      //  public const long AM = (1.0l / IM);
        public const long IQ = 127773;
        public const long IR = 2836;
        public const long MASK = 123459876;
        public static long CH_PAseed = 123;

        // ANGLE CONVERSIONS

        /// Computes the atan2, returning angle given cosine and sine.
        public static double ChAtan2(double mcos, double msin) {
            double ret;
            if (Mathfx.Abs(mcos) < 0.707)
            {
                ret = Math.Cos(mcos);
                if (msin < 0.0)
                    ret = -ret;
            }
            else
            {
                ret = Math.Sin(msin);
                if (mcos < 0.0)
                    ret = CH_C_PI - ret;
            }
            return ret;
        }

        public static int Mod(double a, double n) => (int)((a % n + n) % n);

        public static double hypot(double sideALength, double sideBLength)
        {
            return Math.Sqrt(sideALength * sideALength + sideBLength * sideBLength);
        }

        /// Signum function.


       /* public static double ChSignum<T>(T x)
        {
            return (x > T(0)) - (x < T(0));
        }*/

        public static double Sign(double number)
        {
            return number < 0 ? -1 : (number > 0 ? 1 : 0);
        }

        /// Smooth (sinusoidal) ramp between y1 and y2.
        /// Note: must have x1 < x2 (no check).
        public static double ChSineStep(double x, double x1, double y1, double x2, double y2) {
            if (x <= x1)
                return y1;
            if (x >= x2)
                return y2;
            double dx = x2 - x1;
            double dy = y2 - y1;
            double y = y1 + dy * (x - x1) / dx - (dy / CH_C_2PI) * Math.Sin(CH_C_2PI * (x - x1) / dx);
            return y;
        }

        /// Clamp and modify the specified value to lie within the given limits.
        public static void ChClampValue(ref double value, double limitMin, double limitMax)
        {
            if (value < limitMin)
                value = limitMin;
            else if (value > limitMax)
                value = limitMax;
        }

        /// Clamp the specified value to lie within the given limits.
        public static double ChClamp(double value, double limitMin, double limitMax)
        {
            if (value < limitMin)
                return limitMin;
            if (value > limitMax)
                return limitMax;

            return value;
        }

        // Clamps value between 0 and 1 and returns value
        public static double ChClamp01(double value)
        {
            if (value < 0d)
                return 0d;
            else if (value > 1d)
                return 1d;
            else
                return value;
        }

        /// Maximum between two values
        public static int ChMax(int a, int b)
        {
            if (a > b)
                return a;
            return b;
        }

        /// Maximum between two values
        public static double ChMax(double a, double b)
        {
            if (a > b)
                return a;
            return b;
        }

        /// Minimum between two values
        public static int ChMin(int a, int b)
        {
            if (a < b)
                return a;
            return b;
        }
        /// Minimum between two values
        public static double ChMin(double a, double b)
        {
            if (a < b)
                return a;
            return b;
        }

        // OTHER

        // Park-Miller hi-quality random generator

        /// Signum function.
      /*  public static int ChSignum(double x)
        {
            return (x > (double)(0)) - (x < (double)(0));
        }*/

        public static void ChSetRandomSeed(long newseed)
        {
            if (CH_PAseed != 0)
                CH_PAseed = newseed;
        }

        public static double ChRandom()
        {
            long k;
            double ans = 0;
            CH_PAseed ^= MASK;
            k = (CH_PAseed) / IQ;
            CH_PAseed = IA * (CH_PAseed - k * IQ) - IR * k;
            if (CH_PAseed < 0)
                CH_PAseed += IM;
           // ans = AM * (CH_PAseed);
            CH_PAseed ^= MASK;
            return ans;
        }

        /// Signum function.

        /*  public static double ChSignum(double x)
          {
              return (x > (double)(0)) - (x < (double)(0));
          }*/

    }
}
