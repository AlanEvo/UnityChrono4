using System.Collections;
using System.Collections.Generic;
using System;
using UnityEditor;
using UnityEngine;


namespace chrono
{

    /// Base class for joints betwenn two ChBodyFrame objects.
    ///
    /// Links are objects which can be created to constrain two rigid
    /// bodies (i.e. objects from the ChBody class) in 3D space, like with
    /// revolute joints, guides, etc.
    ///
    /// Note that there are many specializations of this base class,
    /// for example the ChLinkEngine class inherits this base class and
    /// implements specific functions to represent an engine between two
    /// bodies, etc. etc. (In fact, this base ChLink class does basically
    /// _nothing_ unless it is specialized by some child class)
    public class ChLink : ChLinkBase
    {

        protected ChBodyFrame Body1 = new ChBodyFrame();       //< first connected body
        protected ChBodyFrame Body2 = new ChBodyFrame();       //< second connected body
        protected ChVector react_force = new ChVector();   //< store the xyz reactions, expressed in local coordinate system of link;
        protected ChVector react_torque = new ChVector();  //< store the torque reactions, expressed in local coordinate system of link;

       public ChLink() {
            //Body1 = null;
            //Body2 = null;
            react_force = new ChVector();
            react_torque = new ChVector();
        }

        public ChLink(ChLink other) : base(other)
        {
            Body1 = null;
            Body2 = null;

            react_force = other.react_force;
            react_torque = other.react_torque;
        }

        /// "Virtual" copy constructor (covariant return type).
        public override ChObj Clone() { return new ChLink(this); }

        /// Get the number of free degrees of freedom left by this link, between two bodies.
        public int GetLeftDOF() { return 6 - GetDOC(); }

        /// Get the number of scalar variables affected by constraints in this link
        public override int GetNumCoords() { return 12; }

        /// Get the constrained body '1', the 'slave' body.
        public ChBodyFrame GetBody1() { return Body1; }
        /// Get the constrained body '2', the 'master' body.
        public ChBodyFrame GetBody2() { return Body2; }

        /// Get the link coordinate system, expressed relative to Body2 (the 'master'
        /// body). This represents the 'main' reference of the link: reaction forces
        /// and reaction torques are expressed in this coordinate system.
        /// By default is in the origin of Body2, but child classes should implement this.
        public virtual ChCoordsys<double> GetLinkRelativeCoords() { return new ChCoordsys<double>(new ChVector(), new ChQuaternion()); }

        /// Get the link coordinate system in absolute reference.
        /// This represents the 'main' reference of the link: reaction forces
        /// and reaction torques are expressed in this coordinate system.
        /// Child classes should implement this.
        public override ChCoordsys<double> GetLinkAbsoluteCoords() { return ChCoordsys<double>.BitShiftRight(GetLinkRelativeCoords() , Body2.GetCoord()); }

        /// To get reaction force, expressed in link coordinate system:
        public override ChVector Get_react_force() { return react_force; }
        /// To get reaction torque,  expressed in link coordinate system:
        public override ChVector Get_react_torque() { return react_torque; }

        /// If some constraint is redundant, return to normal state  //***OBSOLETE***
        public virtual int RestoreRedundant() { return 0; }

        //
        // UPDATING FUNCTIONS
        //

        /// Given new time, current body state, update time-dependent quantities in link state,
        /// for example motion laws, moving markers, etc.
        /// Default: do nothing except setting new time.
        public virtual void UpdateTime(double time) { ChTime = time; }

        /// This is an important function, which is called by the
        /// owner ChSystem at least once per integration step.
        /// It may update all auxiliary data of the link, such as
        /// matrices if any, etc.
        /// The inherited classes, for example the ChLinkMask, often
        /// implement specialized versions of this Update(time) function,
        /// because they might need to update inner states, forces, springs, etc.
        /// This base version, by default, simply updates the time.
        public override void update(double time, bool update_assets = true) {
            // 1 -
            UpdateTime(time);

            // This will update assets
            base.update(ChTime, update_assets);
        }

        /// As above, but with current time
        public override void update(bool update_assets = true) {
           update(ChTime, update_assets);  // use the same time 
        }

        /// Called from a external package (i.e. a plugin, a CAD app.) to report
        /// that time has changed. Most often you can leave this unimplemented.
        public virtual void UpdatedExternalTime(double prevtime, double time) { }


    };
}
