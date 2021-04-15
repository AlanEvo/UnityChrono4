using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public static class UnitConverter
{
    /// <summary>
    /// m/s to km/h
    /// </summary>
    public static float MpsToKph(float value)
    {
        return value * 3.6f;
    }

    /// <summary>
    /// m/s to miles/h
    /// </summary>
    public static float MpsToMph(float value)
    {
        return value * 2.23694f;
    }

    /// <summary>
    /// miles/h to km/h
    /// </summary>
    public static float MphToKph(float value)
    {
        return value * 1.60934f;
    }

    /// <summary>
    /// km/l to l/100km
    /// </summary>
    public static float KmlToL100km(float kml)
    {
        return kml == 0 ? Mathf.Infinity : 100f / kml;
    }

    /// <summary>
    /// km/l to mpg
    /// </summary>
    public static float KmlToMpg(float kml)
    {
        return kml * 2.825f;
    }

    /// <summary>
    /// l/100km to mpg
    /// </summary>
    public static float L100kmToMpg(float l100km)
    {
        return l100km == 0 ? 0 : 282.5f / l100km;
    }

    /// <summary>
    /// mpg to l/100km
    /// </summary>
    public static float MpgToL100km(float mpg)
    {
        return mpg == 0 ? Mathf.Infinity : 282.5f / mpg;
    }

    /// <summary>
    /// mpg to km/l
    /// </summary>
    public static float MpgToKml(float mpg)
    {
        return mpg * 0.354f;
    }

    /// <summary>
    /// l/100km to km/l
    /// </summary>
    public static float L100kmToKml(float l100km)
    {
        return l100km == 0 ? 0 : 100f / l100km;
    }
}
