using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono
{

    /// Dummy mmutex that does nothing in case that no parallel
    /// multithreading via OpenMP is available.
    public class CHOMPmutex
    {

        public void Lock() { }
        public void Unlock() { }
    };

    /// Dummy functions that do nothing in case that no parallel
    /// multithreading via OpenMP is available.
    public static class CHOMPfunctions
    {

        public static void SetNumThreads(int mth) { }
        public static int GetNumThreads() { return 1; }
        public static int GetThreadNum() { return 0; }
        public static int GetNumProcs() { return 1; }
        public static int GetMaxThreads() { return 1; }
    };

}
