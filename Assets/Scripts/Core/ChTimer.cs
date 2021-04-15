using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
//using System.Runtime.InteropServices;

namespace chrono
{
    public static class PerformanceTimer
    {
       /* [DllImport("KERNEL32")]
        public static extern bool QueryPerformanceCounter(out long lpPerformanceCount);
        [DllImport("Kernel32.dll")]
        public static extern bool QueryPerformanceFrequency(out long lpFrequency);*/
    }

    /// Class for high-resolution timing.
    /// Use the start() ... stop() commands, then request the elapsed time between 
    /// them using GetTimeMilliseconds() or GetTimeSeconds() etc.
    /// Note: the elapsed time is not computed at GetTime... functions, but only at stop().
    /// If you use multiple  start() ... stop()  pairs, each time the duration is accumulated,
    /// until a reset() is done.
    /// Note: since the Windows std::chrono has a limited resolution (1ms-15ms depending on the
    /// system) here we temporary provide a conditional compilation of another timer that does
    /// not use std::chrono for windows, until Microsoft will provide a better std::chrono. 

    public class ChTimer<seconds_type>
    {

        private long m_start;
        private long m_end;
        private long m_freq;
        private bool m_first;
        private seconds_type total;

        public ChTimer()
        {
            m_first = true;
            dynamic a = 0;
            total = a;
        }

        /// Start the timer
        public void start()
        {
          /*  if (m_first)
            {
                PerformanceTimer.QueryPerformanceFrequency(out m_freq);
                m_first = false;
            }
            PerformanceTimer.QueryPerformanceCounter(out m_start);*/
        }

        /// Stops the timer
        public void stop()
        {
           // PerformanceTimer.QueryPerformanceCounter(out m_end);
            double end = (m_end);
            double start = (m_start);
            double freq = (m_freq);
            dynamic a = total;
            a += (end - start) / freq;
        }

        /// Reset the total accumulated time (when repeating multiple start() stop() start() stop() )
        public void reset() {
            dynamic a = 0;
            total = a; }

        /// Returns the time in [ms]. 
        /// Use start()..stop() before calling this.
        public ulong GetTimeMilliseconds() {
            dynamic a = total;
            return (ulong)(a * 1000.0);
        }

        /// Returns the time in [ms] since start(). It does not require stop(). 
       /* public ulong GetTimeMillisecondsIntermediate() {
            return (ulong)(GetTimeSecondsIntermediate() * 1000.0);
        }*/

        /// Returns the time in [us]. 
        /// Use start()..stop() before calling this.
        public ulong GetTimeMicroseconds() {
            dynamic a = total;
            return (ulong)(a * 1000000.0);
        }

        /// Returns the time in [us] since start(). It does not require stop(). 
      /*  public ulong GetTimeMicrosecondsIntermediate() {
            return (ulong)(GetTimeSecondsIntermediate() * 1000000.0);
        }*/

        /// Returns the time in [s], with seconds_type precision
        /// Use start()..stop() before calling this.
        public seconds_type GetTimeSeconds() {
            return total;
        }

        /// Returns the time in [s] since start(). It does not require stop(). 
      /*  public double GetTimeSecondsIntermediate()
        {
            long m_inter;
           // PerformanceTimer.QueryPerformanceCounter(out m_inter);
           // double end = (double)(m_inter);
            double start = (double)(m_start);
            double freq = (double)(m_freq);
           // double intermediate = (end - start) / freq;
            return intermediate;
        }*/

        public seconds_type GetValue()
        {
            return total;
        }


    }
}
