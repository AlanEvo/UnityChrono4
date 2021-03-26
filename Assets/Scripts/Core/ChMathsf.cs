using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;

namespace chrono
{

    public static class ChMathsf
    {
        public const float CH_C_PI = 3.141592653589793238462643383279f;
        public const float CH_C_PI_2 = 1.570796326794896619231321691639f;
        public const float CH_C_2PI = 6.283185307179586476925286766559f;
        public const float CH_C_PI_4 = 0.785398163397448309615660845819f;
        public const float CH_C_DEG_TO_RAD = 3.1415926535897932384626433832795f / 180.0f;
        public const float CH_C_RAD_TO_DEG = 180.0f / 3.1415926535897932384626433832795f;

        public static float hypot(float sideALength, float sideBLength)
        {
            return Mathf.Sqrt(sideALength * sideALength + sideBLength * sideBLength);
        }

        /// Signum function.


        /* public static float ChSignum<T>(T x)
         {
             return (x > T(0)) - (x < T(0));
         }*/

        public static float Sign(float number)
        {
            return number < 0 ? -1 : (number > 0 ? 1 : 0);
        }

        /// Smooth (sinusoidal) ramp between y1 and y2.
        /// Note: must have x1 < x2 (no check).
        public static float ChSineStep(float x, float x1, float y1, float x2, float y2)
        {
            if (x <= x1)
                return y1;
            if (x >= x2)
                return y2;
            float dx = x2 - x1;
            float dy = y2 - y1;
            float y = y1 + dy * (x - x1) / dx - (dy / CH_C_2PI) * Mathf.Sin(CH_C_2PI * (x - x1) / dx);
            return y;
        }

        /// Clamp and modify the specified value to lie within the given limits.
        public static void ChClampValue(ref float value, float limitMin, float limitMax)
        {
            if (value < limitMin)
                value = limitMin;
            else if (value > limitMax)
                value = limitMax;
        }

        /// Clamp the specified value to lie within the given limits.
        public static float ChClamp(float value, float limitMin, float limitMax)
        {
            if (value < limitMin)
                return limitMin;
            if (value > limitMax)
                return limitMax;

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
        public static float ChMax(float a, float b)
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
        public static float ChMin(float a, float b)
        {
            if (a < b)
                return a;
            return b;
        }

        /// Signum function.

        /*  public static float ChSignum(float x)
          {
              return (x > (float)(0)) - (x < (float)(0));
          }*/

    }
}
