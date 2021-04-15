using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono
{
    /// Class representing the interface for containers of proximity pairs,
    /// that is pairs of collision models that have been obtained from the
    /// broadphase collision.
    /// There might be implementations of this interface
    /// in form of plain CPU linked lists of objects (ex. springs or similar
    /// forcefields for cloth simulation etc.) or highly optimized GPU buffers,
    /// etc. etc.
    /// This is only the basic interface with the features that are in common.
    public abstract class ChProximityContainer : ChPhysicsItem
    {

        public ChProximityContainer() {
            add_proximity_callback = null;
            report_proximity_callback = null;
        }
        public ChProximityContainer(ChProximityContainer other) { 
        
        }

        /// Tell the number of added proximity pairs. To be implemented by child classes.
        public abstract int GetNproximities();

        /// Remove (delete) all contained proximity pairs. To be implemented by child classes.
        public abstract void RemoveAllProximities();

        /// The collision system will call BeginAddProximity() before adding
        /// all proximity pairs (for example with AddProximity() or similar). By default
        /// it deletes all previous contacts. Custom more efficient implementations
        /// might reuse contacts if possible.
        public virtual void BeginAddProximities() { RemoveAllProximities(); }

        /// Add a proximity pair between two collision models, storing it into this container.
        /// To be implemented by child classes.
        /// Some specialized child classes (ex. one that uses GPU buffers)
        /// could implement also other more efficient functions to add many proximity pairs
        /// in a batch (so that, for example, a special GPU collision system can exploit it);
        /// yet most collision system might still fall back to this function if no other
        /// specialized add-functions are found.
        public abstract void AddProximity(collision.ChCollisionModel modA,  //< get contact model 1
                              collision.ChCollisionModel modB   //< get contact model 2
                              );

        /// The collision system will call this after adding
        /// all pairs (for example with AddProximity() or similar). By default
        /// it does nothing.
        public virtual void EndAddProximities() { }

        /// Class to be used as a callback interface for some user defined
        /// action to be taken each time a proximity info is added to the container.
        public abstract class AddProximityCallback {


            /// Callback used to process proximity pairs being added to the container.
            /// A derived user-provided callback class must implement this.
            public abstract void OnAddProximity(collision.ChCollisionModel modA,  //< contact model 1
                                    collision.ChCollisionModel modB   //< contact model 2
                                    );
        };

        /// Specify a callback object to be used each time a proximity pair is
        /// added to the container. Note that not all derived classes can support this.
        /// If supported, the OnAddProximity() method of the provided callback object
        /// will be called for each proximity pair as it is created.
        public void RegisterAddProximityCallback(AddProximityCallback mcallback) { add_proximity_callback = mcallback; }

        /// Class to be used as a callback interface for some user defined action to be taken
        /// for each proximity pair (already added to the container).
        /// It can be used to report or post-process proximity pairs.
        public abstract class ReportProximityCallback {


            /// Callback used to report contact points already added to the container.
            /// If it returns false, the contact scanning will be stopped.
            public abstract bool OnReportProximity(
                collision.ChCollisionModel modA,  //< model A (could be nullptr, if the container does not support it)
                collision.ChCollisionModel modB   //< model B (could be nullptr, if the container does not support it)
            );
        };

        /// Scans all the proximity pairs and, for each pair, executes the OnReportProximity()
        /// function of the provided callback object.
        /// Derived classes of ChProximityContainer should try to implement this.
        public abstract void ReportAllProximities(ReportProximityCallback mcallback);


        protected AddProximityCallback add_proximity_callback;
        protected ReportProximityCallback report_proximity_callback;
    }
}
