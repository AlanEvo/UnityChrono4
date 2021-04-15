using System.Collections;
using System.Collections.Generic;
using UnityEngine;


namespace chrono
{

    /// Base class for all types of constraints that act like
    /// mechanical joints ('links') in 3D space.
    ///
    /// Note that there are many specializations of this base class,
    /// for example the ChLinkEngine class inherits this base class and
    /// implements specific functions to represent an engine between two
    /// bodies, etc. etc. (In fact, this base ChLink class does basically
    /// _nothing_ unless it is specialized by some child class).
    public abstract class ChLinkBase : ChPhysicsItem
    {
        protected bool disabled;  //< all constraints of link disabled because of user needs
        protected bool valid;     //< link data is valid
        protected bool broken;    //< link is broken because of excessive pulling/pushing.

        public ChLinkBase()
        {
            broken = false;
            valid = true;
            disabled = false;
        }

        public ChLinkBase(ChLinkBase other)
        {

        }

        /// Tells if the link data is currently valid.
        /// (i.e. pointers to other items are correct)
        public bool IsValid() { return valid; }
        /// Set the status of link validity
        public void SetValid(bool mon) { valid = mon; }

        /// Tells if all constraints of this link are currently turned on or off by the user.
        public bool IsDisabled() { return disabled; }
        /// User can use this to enable/disable all the constraint of the link as desired.
        public virtual void SetDisabled(bool mdis) { disabled = mdis; }

        /// Tells if the link is broken, for excess of pulling/pushing.
        public bool IsBroken() { return broken; }
        /// Set the 'broken' status vof this link.
        public virtual void SetBroken(bool mon) { broken = mon; }

        /// An important function!
        /// Tells if the link is currently active, in general,
        /// that is tells if it must be included into the system solver or not.
        /// This method cumulates the effect of various flags (so a link may
        /// be not active either because disabled, or broken, or not valid)
        public bool IsActive() { return (valid && !disabled && !broken); }

        /// Get the number of scalar variables affected by constraints in this link
        public abstract int GetNumCoords();

        /// Get the link coordinate system in absolute reference.
        /// This represents the 'main' reference of the link: reaction forces
        /// and reaction torques are expressed in this coordinate system.
        /// Child classes should implement this.
        public virtual ChCoordsys GetLinkAbsoluteCoords() { return new ChCoordsys(new ChVector(0, 0, 0), new ChQuaternion(0, 0, 0, 0)); }

        /// Get the master coordinate system for the assets, in absolute reference.
        /// (should be implemented by children classes)
        // public override ChFrame<double> GetAssetsFrame(int nclone = 0) { return new ChFrame<double>(GetLinkAbsoluteCoords()); }

        /// To get reaction force, expressed in link coordinate system:
        public virtual ChVector Get_react_force() { return new ChVector(0, 0, 0); }
        /// To get reaction torque,  expressed in link coordinate system:
        public virtual ChVector Get_react_torque() { return new ChVector(0, 0, 0); }
        // (Note, functions above might fit better in a specialized subclass, but here for easier GUI interface)

        /// Tells if this link requires that the connected ChBody objects
        /// must be waken if they are sleeping. By default =true, i.e. always keep awaken, but
        /// child classes might return false for optimizing sleeping, in case no time-dependant.
        public virtual bool IsRequiringWaking() { return true; }
    }
}
