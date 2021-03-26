using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono
{
    /// Basic interface class for 'controls', that are objects that change parameters
    /// during the simulation, ex. to simulate PIDs, etc.
    /// Must be inherited and implemented by user.
    public class ChControls : ChObj
    {

        public ChControls() { }
        public ChControls(ChControls other) : base(other) { }

        /// "Virtual" copy constructor.
        public override ChObj Clone() { return new ChControls(this);
        }

        public virtual bool ExecuteForUpdate() { return true; }
        public virtual bool ExecuteForStep() { return true; }


    }
}
