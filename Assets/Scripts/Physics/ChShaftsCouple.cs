using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono
{

    /// Base class for defining constraints between a couple of two one-degree-of-freedom
    /// parts; i.e., shafts that can be used to build 1D models of powertrains.

    public class ChShaftsCouple : ChPhysicsItem
    {

        public ChShaft shaft1;  //< first shaft
        public ChShaft shaft2;  //< second shaft


        protected ChShaftsCouple()
        {
            shaft1 = null;
            shaft2 = null;
        }
        protected ChShaftsCouple(ChShaftsCouple other) : base(other)
        {

        }


        /// "Virtual" copy constructor (covariant return type).
        public override ChObj Clone() { return new ChShaftsCouple(this); }

        /// Get the number of scalar variables affected by constraints in this link
        public virtual int GetNumCoords() { return 2; }

        /// Use this function after gear creation, to initialize it, given two shafts to join.
        /// Each shaft must belong to the same ChSystem.
        /// Derived classes might overload this (here, basically it only sets the two pointers)
        public virtual bool Initialize(ChShaft mshaft1,  //< first  shaft to join
                            ChShaft mshaft2  //< second shaft to join
                            )
        {
            ChShaft mm1 = mshaft1;
            ChShaft mm2 = mshaft2;
            // assert(mm1 && mm2);
            // assert(mm1 != mm2);
            // assert(mm1->GetSystem() == mm2->GetSystem());
            shaft1 = mm1;
            shaft2 = mm2;
            SetSystem(shaft1.GetSystem());
            return true;
        }

        /// Get the first (input) shaft
        public ChShaft GetShaft1() { return shaft1; }
        /// Get the second (output) shaft
        public ChShaft GetShaft2() { return shaft2; }

        /// Get the reaction torque exchanged between the two shafts,
        /// considered as applied to the 1st axis.
        /// Children classes might overload this.
        public virtual double GetTorqueReactionOn1() { return 0; }

        /// Get the reaction torque exchanged between the two shafts,
        /// considered as applied to the 2nd axis.
        /// Children classes might overload this.
        public virtual double GetTorqueReactionOn2() { return 0; }

        /// Get the actual relative angle in terms of phase of shaft 1 respect to 2.
        public double GetRelativeRotation() { return (this.shaft1.GetPos() - this.shaft2.GetPos()); }
        /// Get the actual relative speed in terms of speed of shaft 1 respect to 2.
        public double GetRelativeRotation_dt() { return (this.shaft1.GetPos_dt() - this.shaft2.GetPos_dt()); }
        /// Get the actual relative acceleration in terms of speed of shaft 1 respect to 2.
        public double GetRelativeRotation_dtdt() { return (this.shaft1.GetPos_dtdt() - this.shaft2.GetPos_dtdt()); }

    }
}
