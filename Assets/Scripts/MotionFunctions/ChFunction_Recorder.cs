using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
using System.Linq;


namespace chrono
{

    public class ChRecPoint
    {

        public double x;  //< argument value
        public double y;  //< function value
        public double w;  //< weight

        public ChRecPoint() { }
        public ChRecPoint(double mx, double my, double mw)
        {
            x = mx;
            y = my;
            w = mw;
        }


    };

    /// Recorder function:
    ///
    /// y = interpolation of array of (x,y) data,
    ///     where (x,y) points can be inserted randomly.
    public class ChFunction_Recorder : ChFunction
    {

        private List<ChRecPoint> m_points = new List<ChRecPoint>();  //< the list of points
        private ChRecPoint m_last;

        // Use this for initialization
        public ChFunction_Recorder()
        {
            m_last = m_points[m_points.Count - 1];
        }

        public ChFunction_Recorder(ChFunction_Recorder other) {
            m_points = other.m_points;
            m_last = m_points[m_points.Count - 1];
        }

        /// "Virtual" copy constructor (covariant return type).
        public override ChFunction Clone() { return new ChFunction_Recorder(this); }

        public override FunctionType Get_Type() { return FunctionType.FUNCT_RECORDER; }

        private double Interpolate_y(double x, ChRecPoint p1, ChRecPoint p2)
        {
            return ((x - p1.x) * p2.y + (p2.x - x) * p1.y) / (p2.x - p1.x);
        }

    public override double Get_y(double x) {
            if (m_points.Count == 0)
            {
                return 0;
            }

            if (x <= m_points.First().x)
            {
                return m_points.First().y;
            }

            if (x >= m_points[m_points.Count - 1].x)
            {
                return m_points[m_points.Count - 1].y;
            }

            // At this point we are guaranteed that there are at least two records.

            if (m_last == m_points.Last())
            {
                m_last = m_points.First();
            }

            if (x > m_last.x)
            {
                // Search to the right
                foreach (ChRecPoint iter in m_points)
                {
                    if (x <= iter.x)
                    {
                        return Interpolate_y(x, m_last, iter);
                    }
                    m_last = iter;
                }
            }
            else
            {
                // Search to the left
                foreach (ChRecPoint iter in m_points)
                {
                    //--iter;
                    if (x >= iter.x)
                    {
                        return Interpolate_y(x, iter, m_last);
                    }
                    m_last = iter;
                }
            }

            return 0;
        }
        public override double Get_y_dx(double x) {
            //// TODO:  can we do better?            
            return base.Get_y_dx(x);
        }
        public override double Get_y_dxdx(double x) {
            //// TODO: can we do better?
            return base.Get_y_dxdx(x);
        }

        public void AddPoint(double mx, double my, double mw = 1)
        {
            foreach (ChRecPoint iter in m_points)
            {
                double dist = mx - iter.x;
                if (Math.Abs(dist) < double.Epsilon)
                {
                    // Overwrite current iterator
                    iter.x = mx;
                    iter.y = my;
                    iter.w = mw;
                    return;
                }
                else if (dist > 0)
                {
                    // Insert before current iterator
                    ChRecPoint rec = new ChRecPoint(mx, my, mw);
                    m_points.Insert(iter.GetHashCode(), rec);
                    return;
                }
            }

            // Insert in front of list
            m_points.Add(new ChRecPoint(mx, my, mw));
        }

        public void Reset()
        {
            m_points.Clear();
            m_last = m_points[m_points.Count - 1];
        }

        public List<ChRecPoint> GetPoints() { return m_points; }

        public override void Estimate_x_range(ref double xmin, ref double xmax) {
            if (m_points.Count == 0)
            {
                xmin = 0.0;
                xmax = 1.2;
                return;
            }

            xmin = m_points.First().x;
            xmax = m_points.Last().x;
            if (xmin == xmax)
                xmax = xmin + 0.5;
        }
    }

}
