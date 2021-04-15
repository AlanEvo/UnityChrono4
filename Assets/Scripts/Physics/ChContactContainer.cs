using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
using UnityEngine.Serialization;
using System.Linq;

namespace chrono
{
    /// Class representing a container of many contacts.
    /// There might be implementations of this interface in form of plain CPU linked lists of contact objects,
    /// or highly optimized GPU buffers, etc. This is only the basic interface with the features that are in common.
    /// Struct to store resultant contact force/torque applied on rigid body
    public abstract class ChContactContainer : ChPhysicsItem
    {
        public ChContactContainer()
        {
            add_contact_callback = null;
            report_contact_callback = null;
        }
        public ChContactContainer(ChContactContainer other)
        {
            add_contact_callback = other.add_contact_callback;
            report_contact_callback = other.report_contact_callback;
        }

        /// Get the number of added contacts. To be implemented by child classes.
        public abstract int GetNcontacts();

        /// Remove (delete) all contained contact data. To be implemented by child classes.
        public abstract void RemoveAllContacts();

        /// The collision system will call BeginAddContact() before adding
        /// all contacts (for example with AddContact() or similar). By default
        /// it deletes all previous contacts. Custom more efficient implementations
        /// might reuse contacts if possible.
        public virtual void BeginAddContact() { RemoveAllContacts(); }

        /// Add a contact between two models, storing it into this container.
        /// To be implemented by child classes.
        /// Some specialized child classes (ex. one that uses GPU buffers)
        /// could implement also other more efficient functions to add many contacts
        /// in a batch (so that, for example, a special GPU collision system can exploit it);
        /// yet most collision system might still fall back to this function if no other
        /// specialized add-functions are found.
        public abstract void AddContact(collision.ChCollisionInfo mcontact);

        /// The collision system will call EndAddContact() after adding
        /// all contacts (for example with AddContact() or similar). By default
        /// it does nothing.
        public virtual void EndAddContact() { }

        /// Class to be used as a callback interface for some user defined action to be taken
        /// each time a contact is added to the container.
        /// It can be used to modify the composite material properties for the contact pair.
        public abstract class AddContactCallback
        {

            /// Callback used to process contact points being added to the container.
            /// A derived user-provided callback class must implement this. The provided
            /// composite material should be downcast to the appropriate type.
            public abstract void OnAddContact(
                    collision.ChCollisionInfo contactinfo,  //< information about the collision pair
                    ChMaterialComposite material             //< composite material can be modified
            );
        };

        /// Specify a callback object to be used each time a contact point is added to the container.
        /// Note that not all derived classes can support this. If supported, the OnAddContact() method
        /// of the provided callback object will be called for each contact pair to allow modifying the
        /// composite material properties.
        public virtual void RegisterAddContactCallback(AddContactCallback mcallback) { add_contact_callback = mcallback; }

        /// Get the callback object to be used each time a contact point is added to the container.
        public virtual AddContactCallback GetAddContactCallback() { return add_contact_callback; }

        /// Class to be used as a callback interface for some user defined action to be taken
        /// for each contact (already added to the container, maybe with already computed forces).
        /// It can be used to report or post-process contacts.
        public abstract class ReportContactCallback
        {

            /// Callback used to report contact points already added to the container.
            /// If it returns false, the contact scanning will be stopped.
            public abstract bool OnReportContact(
                    ChVector pA,             //< contact pA
                    ChVector pB,             //< contact pB
                    ChMatrix33<double> plane_coord,  //< contact plane coordsystem (A column 'X' is contact normal)
                    double distance,           //< contact distance
                    double eff_radius,         //< effective radius of curvature at contact
                    ChVector react_forces,   //< react.forces (if already computed). In coordsystem 'plane_coord'
                    ChVector react_torques,  //< react.torques, if rolling friction (if already computed).
                    ChContactable contactobjA,  //< model A (note: some containers may not support it and could be nullptr)
                    ChContactable contactobjB   //< model B (note: some containers may not support it and could be nullptr)
            );
        };

        /// Scans all the contacts and for each contact executes the OnReportContact()
        /// function of the provided callback object.
        /// Derived classes of ChContactContainer should try to implement this.
        public virtual void ReportAllContacts(ReportContactCallback mcallback) { }

        /// Compute contact forces on all contactable objects in this container.
        /// If implemented by a derived class, these forces must be stored in the hash table
        /// contact_forces (with key a pointer to ChContactable and value a ForceTorque structure).
        public virtual void ComputeContactForces() { }

        /// Return the resultant contact force acting on the specified contactable object.
        public ChVector GetContactableForce(ChContactable contactable)
        {
            //Dictionary<IChContactable, ForceTorque> it = new Dictionary<IChContactable, ForceTorque>();
            /*  foreach (KeyValuePair<IChContactable, ForceTorque> it in contact_forces)
              {
                  if(it.Key == contactable)
                  {
                      return it[1].force;
                  }
              }
              Dictionary<IChContactable, ForceTorque> Iterator = contact_forces.Find(contactable);
              if (Iterator != contact_forces.Last())
              {
                  return Iterator[1].force;
              }*/
            return new ChVector(0);
        }

        /// Return the resultant contact torque acting on the specified contactable object.
        public ChVector GetContactableTorque(ChContactable contactable)
        {
            /* Dictionary<IChContactable, ForceTorque>.Enumerator Iterator = contact_forces.find(contactable);
             if (Iterator != contact_forces.end())
             {
                 return Iterator.second.torque;
             }*/
            return new ChVector(0);
        }

        /// Method for serialization of transient data to archives.
       // public virtual void ArchiveOUT(ChArchiveOut marchive);

        /// Method for de-serialization of transient data from archives.
       // public virtual void ArchiveIN(ChArchiveIn marchive);

        protected struct ForceTorque
        {
            public ChVector force;
            public ChVector torque;
        };

        protected SortedList<ChContactable, ForceTorque> contact_forces = new SortedList<ChContactable, ForceTorque>();
        protected AddContactCallback add_contact_callback;       
        protected ReportContactCallback report_contact_callback;

        protected void SumAllContactForces<Tcont>(ref List<Tcont> contactlist,
                            ref SortedList<ChContactable, ForceTorque> contactforces)
        {
            /* List<Tcont>.Enumerator contact = contactlist.GetEnumerator();
             while (contact.MoveNext())
             {
               /*  for (contact.Current == (dynamic)contactlist.FirstOrDefault(); contact.Current != (dynamic)contactlist.LastOrDefault(); contact.MoveNext())
                 {

                     // itercontact.MoveNext();
                 }
             }*/
            // List<Tcont>.Enumerator contact = contactlist.GetEnumerator();
            // while (contact.MoveNext())
            // {
            /* for (var contact = contactlist.GetEnumerator(); contact != (dynamic)contactlist.LastOrDefault(); contact.MoveNext())
             {

             }*/
            //for (int i = 0; contact.Current != (dynamic)contactlist.LastOrDefault(); i++)
            /*IEnumerator contact = contactlist.GetEnumerator();
           // List<Tcont>.Enumerator contact = contactlist.GetEnumerator();
            IEnumerator iterator = contactforces.GetEnumerator();
            //foreach (var contact in contactlist)
            if (contactlist.Count > 0)
            {
            
                while(contact.MoveNext())
                //foreach(contact in contactlist)
                {

                    // contact.MoveNext();
                    // Extract information for current contact (expressed in global frame)

                    ChMatrix33<double> A = ((List<ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>.Enumerator)contact).Current.GetContactPlane();
                    ChVector force_loc = ((List<ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>.Enumerator)contact).Current.GetContactForce();
                    ChVector force = A.Matr_x_Vect(force_loc);
                    ChVector p1 = ((List<ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>.Enumerator)contact).Current.GetContactP1();
                    ChVector p2 = ((List<ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>.Enumerator)contact).Current.GetContactP2();
                    

                    // Calculate contact torque for first object (expressed in global frame).
                    // Recall that -force is applied to the first object.
                    ChVector torque1 = new ChVector(0);
                    //ChBody bodyA = gameObject.AddComponent<ChBody>(); //new ChBody();                    
                    ChBody bodyA = (ChBody)((List<ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>.Enumerator)contact).Current.GetObjA(); //new ChBody();

                    // Alan // this seems to work, for now.
                    if (bodyA == ((List<ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>.Enumerator)contact).Current.GetObjA())
                    {
                        torque1 = Vector.Vcross(p1 - bodyA.BodyFrame.GetPos(), -force);
                    }

                    // If there is already an entry for the first object, accumulate.
                    // Otherwise, insert a new entry.
                    //var entry1 = contactforces.Values.Cast<ChBody>().FirstOrDefault(((ChContactTuple<double, double>)contact).GetObjA());//Find((contact).GetObjA());
                   // ForceTorque entry1 = contactforces.FirstOrDefault(pair => { return ((ChContactNSC<ChBody, ChBody>)contact).GetObjA(); }).Value;//Find((contact).GetObjA());
                    ForceTorque entry1 = contactforces.FirstOrDefault().Value; // date of 1/1/2016
                    if (contactforces.Count != 0 && entry1.Equals(contactforces.LastOrDefault().Value))
                    {
                        entry1.force -= force;
                        entry1.torque += torque1;

                    }
                    else
                    {
                        ForceTorque ft = new ForceTorque();
                        ft.force = -force;
                        ft.torque = torque1;
                        contactforces.Add(((List<ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>.Enumerator)contact).Current.GetObjA(), ft);
                    }

                    // Calculate contact torque for second object (expressed in global frame).
                    // Recall that +force is applied to the second object.
                    ChVector torque2 = new ChVector(0);
                    ChBody bodyB = (ChBody)((List<ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>.Enumerator)contact).Current.GetObjB();
                    if (bodyB == ((List<ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>.Enumerator)contact).Current.GetObjB())
                    {
                        torque2 = Vector.Vcross(p2 - bodyB.BodyFrame.GetPos(), force);
                    }

                    // If there is already an entry for the first object, accumulate.
                    // Otherwise, insert a new entry.
                    ForceTorque entry2 = contactforces.FirstOrDefault().Value; // date of 1/1/2016
                    if (contactforces.Count != 0 && entry2.Equals(contactforces.LastOrDefault().Value))
                    {
                        entry2.force += force;
                        entry2.torque += torque1;

                    }
                    else
                    {
                        ForceTorque ft = new ForceTorque();
                        ft.force = force;
                        ft.torque = torque2;
                        contactforces.Add(((List<ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>.Enumerator)contact).Current.GetObjB(), ft);
                    }
                }
            }*/
            //}
        }
    }


}

