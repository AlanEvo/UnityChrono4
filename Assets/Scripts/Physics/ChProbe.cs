using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono
{
    /// Base class for probe objects, used to record data during simulations.
    public class ChProbe : ChObj
    {
        public ChProbe() { }
        public ChProbe(ChProbe other) : base(other) { }

        /// "Virtual" copy constructor.
        public override ChObj Clone() { return new ChProbe(this); }

        /// Record the value.
        /// Note that X and Y variables must be already set, using the probe data.
        /// Usually mtime is used for X var (default), while a script is used to
        /// specify the Y  variable. Record() is called for each integration step,
        /// by the system object which contains all probes.
        public virtual void Record(double mtime) { }

        // If some data is recorded, delete.
        public virtual void Reset() { }

        /// Method to allow serialization of transient data to archives.
        // public virtual void ArchiveOUT(ChArchiveOut& marchive) override;

        /// Method to allow de serialization of transient data from archives.
        // public virtual void ArchiveIN(ChArchiveIn& marchive) override;
    }
}
