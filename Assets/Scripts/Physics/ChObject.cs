using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono
{
    /// Base class for items which can be named, deleted, copied. etc. as in the editor of a 3d modeler.
    public abstract class ChObj : MonoBehaviour
    {
        private string name;  //< name of object
        private int identifier;    //< object identifier

        protected double ChTime;  //< the time of simulation for the object

        public ChObj() {
            ChTime = 0;
           // identifier = ChGlobal.GetUniqueIntID();
        }

        public ChObj(ChObj other)
        {
           // identifier = ChGlobal.GetUniqueIntID();

            name = other.name;
            ChTime = other.ChTime;

        }

        /// "Virtual" copy constructor.
        /// Concrete derived classes must implement this.
        public abstract ChObj Clone();

        /// Gets the numerical identifier of the object.
        public int GetIdentifier() { return identifier; }
        /// Sets the numerical identifier of the object.
        public void SetIdentifier(int id) { identifier = id; }

        /// Gets the simulation time of this object
        public double GetChTime() { return ChTime; }
        /// Sets the simulation time of this object.
        public void SetChTime(double m_time) { ChTime = m_time; }

        /// Gets the name of the object as C Ascii null-terminated string -for reading only!
        public string GetName() { return name.ToString(); }
        /// Sets the name of this object, as ascii string
        public void SetName(string myname) { name = myname; }

        /// Gets the name of the object as C Ascii null-terminated string.
        public string GetNameString() { return name; }
        /// Sets the name of this object, as std::string
        public void SetNameString(string myname) { name = myname; }

        // Set-get generic LONG flags, passed as reference

        public void MFlagsSetAllOFF(int mflag) { mflag = 0; }
        public void MFlagsSetAllON(int mflag)
        {
            mflag = 0;
            mflag = ~mflag;
        }
        public void MFlagSetON(int mflag, int mask) { mflag |= mask; }
        public void MFlagSetOFF(int mflag, int mask) { mflag &= ~mask; }
        public int MFlagGet(int mflag, int mask) { return (mflag & mask); }
        /// Method to allow serialization of transient data to archives.
       // public virtual void ArchiveOUT(ChArchiveOut marchive) { }

        /// Method to allow de-serialization of transient data from archives.
       // public virtual void ArchiveIN(ChArchiveIn marchive) { }

        // Method to allow mnemonic names in (de)serialization of containers (std::vector, arrays, etc.)
        public virtual string ArchiveContainerName() { return name; }


    }
}
