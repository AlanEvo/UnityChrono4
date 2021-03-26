using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Threading;

namespace chrono
{
    public class LONG
    {
        public double value;
    }
    /// Class that wraps a spinlock, a very fast locking mutex
    /// that should be used only for short wait periods.
    /// This uses MSVC intrinsics to mimic a fast spinlock as
    /// in pthreads.h, but without the need of including
    /// the pthreads library for windows.
    /// See http://locklessinc.com/articles/pthreads_on_windows/
    public class ChSpinlock
    {
        public const double EBUSY = 16;
        public ChSpinlock() 
        {
            m_lock.value = 0; 
        }

        public void Lock()
        {
            while (Interlocked.Exchange(ref m_lock.value, EBUSY) != 0) {
                /* Don't lock the bus whilst waiting */
                while (m_lock != null)
                    {
                        Thread.Yield();
                    /* Compiler barrier.  Prevent caching of *l */
                    //Thread._ReadBarrier();
                    }
            }
        }
        public void Unlock()
        {
           // _ReadWriteBarrier();
            m_lock.value = 0;
        }

        class pseudo_pthread_spinlock_t : LONG { }
        pseudo_pthread_spinlock_t m_lock;
    }
}
