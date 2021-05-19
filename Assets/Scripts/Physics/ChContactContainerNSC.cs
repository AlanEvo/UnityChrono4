using System.Collections;
using System.Collections.Generic;
using System;
using System.Linq;
using UnityEngine;
using UnityEngine.Serialization;
using System.Reflection;

namespace chrono
{

    /// Class representing a container of many non-smooth contacts.
    /// This is implemented as a typical linked list of ChContactNSC objects
    /// (that is, contacts between two ChContactable objects, with 3 reactions).
    /// It might also contain ChContactNSCrolling objects (extended versions of ChContactNSC,
    /// with 6 reactions, that account also for rolling and spinning resistance), but also
    /// for '6dof vs 6dof' contactables.
    public unsafe class ChContactContainerNSC : ChContactContainer
    {

        public class ChContactNSC_6_6 : ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>> { }
        public class ChContactNSC_6_3 : ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Three>> { }
        public class ChContactNSC_3_3 : ChContactNSC<ChContactable_1vars<IntInterface.Three>, ChContactable_1vars<IntInterface.Three>> { }
        public class ChContactNSC_333_3 : ChContactNSC<ChContactable_3vars<IntInterface.Three, IntInterface.Three, IntInterface.Three>, ChContactable_1vars<IntInterface.Three>> { }
        public class ChContactNSC_333_6 : ChContactNSC<ChContactable_3vars<IntInterface.Three, IntInterface.Three, IntInterface.Three>, ChContactable_1vars<IntInterface.Six>> { }
        public class ChContactNSC_333_333 : ChContactNSC<ChContactable_3vars<IntInterface.Three, IntInterface.Three, IntInterface.Three>, ChContactable_3vars<IntInterface.Three, IntInterface.Three, IntInterface.Three>> { }
        public class ChContactNSC_666_3 : ChContactNSC<ChContactable_3vars<IntInterface.Six, IntInterface.Six, IntInterface.Six>, ChContactable_1vars<IntInterface.Three>> { }
        public class ChContactNSC_666_6 : ChContactNSC<ChContactable_3vars<IntInterface.Six, IntInterface.Six, IntInterface.Six>, ChContactable_1vars<IntInterface.Six>> { }
        public class ChContactNSC_666_333 : ChContactNSC<ChContactable_3vars<IntInterface.Six, IntInterface.Six, IntInterface.Six>, ChContactable_3vars<IntInterface.Three, IntInterface.Three, IntInterface.Three>> { }
        public class ChContactNSC_666_666 : ChContactNSC<ChContactable_3vars<IntInterface.Six, IntInterface.Six, IntInterface.Six>, ChContactable_3vars<IntInterface.Six, IntInterface.Six, IntInterface.Six>> { }

        public class ChContactNSCrolling_6_6 : ChContactNSCrolling<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>> { }


        protected List<ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>> contactlist_6_6 = new List<ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>();
        protected List<ChContactNSC_6_3> contactlist_6_3 = new List<ChContactNSC_6_3>();
        protected List<ChContactNSC_3_3> contactlist_3_3 = new List<ChContactNSC_3_3>();
        protected List<ChContactNSC_333_3> contactlist_333_3 = new List<ChContactNSC_333_3>();
        protected List<ChContactNSC_333_6> contactlist_333_6 = new List<ChContactNSC_333_6>();
        protected List<ChContactNSC_333_333> contactlist_333_333 = new List<ChContactNSC_333_333>();
        protected List<ChContactNSC_666_3> contactlist_666_3 = new List<ChContactNSC_666_3>();
        protected List<ChContactNSC_666_6> contactlist_666_6 = new List<ChContactNSC_666_6>();
        protected List<ChContactNSC_666_333> contactlist_666_333 = new List<ChContactNSC_666_333>();
        protected List<ChContactNSC_666_666> contactlist_666_666 = new List<ChContactNSC_666_666>();

        protected List<ChContactNSCrolling<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>> contactlist_6_6_rolling = new List<ChContactNSCrolling<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>();

        protected int n_added_6_6;
        protected int n_added_6_3;
        protected int n_added_3_3;
        protected int n_added_333_3;
        protected int n_added_333_6;
        protected int n_added_333_333;
        protected int n_added_666_3;
        protected int n_added_666_6;
        protected int n_added_666_333;
        protected int n_added_666_666;
        protected int n_added_6_6_rolling;

        // protected IEnumerator<ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>> lastcontact_6_6;
        // protected IEnumerator<ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>> lastcontact_6_6;
        //protected List<ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>> lastcontact_6_6 = new List<ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>();
        protected IEnumerator<ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>> lastcontact_6_6;
        protected IEnumerator<ChContactNSC_6_3> lastcontact_6_3;
        protected IEnumerator<ChContactNSC_3_3> lastcontact_3_3;
        protected IEnumerator<ChContactNSC_333_3> lastcontact_333_3;
        protected IEnumerator<ChContactNSC_333_6> lastcontact_333_6;
        protected IEnumerator<ChContactNSC_333_333> lastcontact_333_333;
        protected IEnumerator<ChContactNSC_666_3> lastcontact_666_3;
        protected IEnumerator<ChContactNSC_666_6> lastcontact_666_6;
        protected IEnumerator<ChContactNSC_666_333> lastcontact_666_333;
        protected IEnumerator<ChContactNSC_666_666> lastcontact_666_666;

        protected IEnumerator<ChContactNSCrolling<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>> lastcontact_6_6_rolling;


        public ChContactContainerNSC() : base()
        {
            n_added_6_6 = 0;
            n_added_6_3 = 0;
            n_added_3_3 = 0;
            n_added_333_3 = 0;
            n_added_333_6 = 0;
            n_added_333_333 = 0;
            n_added_666_3 = 0;
            n_added_666_6 = 0;
            n_added_666_333 = 0;
            n_added_666_666 = 0;
            n_added_6_6_rolling = 0;
        }
        public ChContactContainerNSC(ChContactContainerNSC other) : base(other)
        {
            n_added_6_6 = 0;
            n_added_6_3 = 0;
            n_added_3_3 = 0;
            n_added_333_3 = 0;
            n_added_333_6 = 0;
            n_added_333_333 = 0;
            n_added_666_3 = 0;
            n_added_666_6 = 0;
            n_added_666_333 = 0;
            n_added_666_666 = 0;
            n_added_6_6_rolling = 0;
        }

        public void FixedUpdate()
        {
            // Debug.Log("contacts " + contactlist_6_6.Count);
        }

        /// Tell the number of added contacts
        public override int GetNcontacts()
        {
            return n_added_3_3 + n_added_6_3 + n_added_6_6 + n_added_333_3 + n_added_333_6 + n_added_333_333 +
                   n_added_666_3 + n_added_666_6 + n_added_666_333 + n_added_666_666 + n_added_6_6_rolling;
        }

        public void _RemoveAllContacts<Tcont, Titer>(ref List<Tcont> contactlist, ref Titer lastcontact, ref int n_added)
        {
            //typename std::list < Tcont *>::iterator itercontact = contactlist.begin();
            IEnumerator<Tcont> itercontact = contactlist.GetEnumerator();

            /*while (itercontact != contactlist.Last())
            {
               // delete (itercontact);
                (itercontact) = null;
                ++itercontact;
            }*/
            contactlist.Clear();
            //lastcontact = contactlist.First<Tcont>();
            n_added = 0;

        }

        /// Remove (delete) all contained contact data.
        public override void RemoveAllContacts()
        {
            _RemoveAllContacts(ref contactlist_6_6, ref lastcontact_6_6, ref n_added_6_6);
            _RemoveAllContacts(ref contactlist_6_3, ref lastcontact_6_3, ref n_added_6_3);
            _RemoveAllContacts(ref contactlist_3_3, ref lastcontact_3_3, ref n_added_3_3);
            _RemoveAllContacts(ref contactlist_333_3, ref lastcontact_333_3, ref n_added_333_3);
            _RemoveAllContacts(ref contactlist_333_6, ref lastcontact_333_6, ref n_added_333_6);
            _RemoveAllContacts(ref contactlist_333_333, ref lastcontact_333_333, ref n_added_333_333);
            _RemoveAllContacts(ref contactlist_666_3, ref lastcontact_666_3, ref n_added_666_3);
            _RemoveAllContacts(ref contactlist_666_6, ref lastcontact_666_6, ref n_added_666_6);
            _RemoveAllContacts(ref contactlist_666_333, ref lastcontact_666_333, ref n_added_666_333);
            _RemoveAllContacts(ref contactlist_666_666, ref lastcontact_666_666, ref n_added_666_666);
            _RemoveAllContacts(ref contactlist_6_6_rolling, ref lastcontact_6_6_rolling, ref n_added_6_6_rolling);
        }


        /// The collision system will call BeginAddContact() before adding
        /// all contacts (for example with AddContact() or similar). Instead of
        /// simply deleting all list of the previous contacts, this optimized implementation
        /// rewinds the link iterator to begin and tries to reuse previous contact objects
        /// until possible, to avoid too much allocation/deallocation.
        public override void BeginAddContact()
        {
            //lastcontact_6_6 = (IEnumerator<ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>)Enumerable.Empty<ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>();//(IEnumerator<ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>)contactlist_6_6.GetEnumerator().;//(List<ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>)Enumerable.Empty<ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>();// = (IEnumerator<ChContactNSC_6_6>)contactlist_6_6.FirstOrDefault();
            //contactlist_6_6.GetEnumerator().MoveNext();
            //for (int i = 0; i <= 1; i++, contactlist_6_6.GetEnumerator().MoveNext())
            // {
            //iter.Current;
           // contactlist_6_6.Clear();
            lastcontact_6_6 = contactlist_6_6.GetEnumerator();
            n_added_6_6 = 0;
            //}

            /* lastcontact_6_3 = (IEnumerator<ChContactNSC_6_3>)Enumerable.Empty<ChContactNSC_6_3>();// = (IEnumerator<ChContactNSC_6_3>)contactlist_6_3.FirstOrDefault();
             n_added_6_3 = 0;
             lastcontact_3_3 = (IEnumerator<ChContactNSC_3_3>)Enumerable.Empty<ChContactNSC_3_3>();// = (IEnumerator<ChContactNSC_3_3>)contactlist_3_3.FirstOrDefault();
             n_added_3_3 = 0;
             lastcontact_333_3 = (IEnumerator<ChContactNSC_333_3>)Enumerable.Empty<ChContactNSC_333_3>();// = (IEnumerator<ChContactNSC_333_3>)contactlist_333_3.FirstOrDefault();
             n_added_333_3 = 0;
             lastcontact_333_6 = (IEnumerator<ChContactNSC_333_6>)Enumerable.Empty<ChContactNSC_333_6>();// = (IEnumerator<ChContactNSC_333_6>)contactlist_333_6.FirstOrDefault();
             n_added_333_6 = 0;
             lastcontact_333_333 = (IEnumerator<ChContactNSC_333_333>)Enumerable.Empty<ChContactNSC_333_333>();// = (IEnumerator<ChContactNSC_333_333>)contactlist_333_333.FirstOrDefault();
             n_added_333_333 = 0;
             lastcontact_666_3 = (IEnumerator<ChContactNSC_666_3>)Enumerable.Empty<ChContactNSC_666_3>();// = (IEnumerator<ChContactNSC_666_3>)contactlist_666_3.FirstOrDefault();
             n_added_666_3 = 0;
             lastcontact_666_6 = (IEnumerator<ChContactNSC_666_6>)Enumerable.Empty<ChContactNSC_666_6>();// = (IEnumerator<ChContactNSC_666_6>)contactlist_666_6.FirstOrDefault();
             n_added_666_6 = 0;
             lastcontact_666_333 = (IEnumerator<ChContactNSC_666_333>)Enumerable.Empty<ChContactNSC_666_333>();// = (IEnumerator<ChContactNSC_666_333>)contactlist_666_333.FirstOrDefault();
             n_added_666_333 = 0;
             lastcontact_666_666 = (IEnumerator<ChContactNSC_666_666>)Enumerable.Empty<ChContactNSC_666_666>();// = (IEnumerator<ChContactNSC_666_666>)contactlist_666_666.FirstOrDefault();
             n_added_666_666 = 0;*/
           // contactlist_6_6_rolling.Clear();
            lastcontact_6_6_rolling = contactlist_6_6_rolling.GetEnumerator();// = (IEnumerator<ChContactNSCrolling_6_6>)contactlist_6_6_rolling.FirstOrDefault();
            n_added_6_6_rolling = 0;
        }

        /// The collision system will call BeginAddContact() after adding
        /// all contacts (for example with AddContact() or similar). This optimized version
        /// purges the end of the list of contacts that were not reused (if any).
        public override void EndAddContact()
        {
            // remove contacts that are beyond last contact            
            while (lastcontact_6_6 != null && lastcontact_6_6.Current != contactlist_6_6.LastOrDefault()) // This is working correctly, Alan
            {
                lastcontact_6_6.MoveNext();
                contactlist_6_6.Remove(lastcontact_6_6.Current);
                lastcontact_6_6 = contactlist_6_6.GetEnumerator();
            }
            while (lastcontact_6_3 != contactlist_6_3.LastOrDefault())
            {
                lastcontact_6_3 = null;
                contactlist_6_3.Remove((ChContactNSC_6_3)lastcontact_6_3);
            }
            while (lastcontact_3_3 != contactlist_3_3.LastOrDefault())
            {
                lastcontact_3_3 = null;
                contactlist_3_3.Remove((ChContactNSC_3_3)lastcontact_3_3);
            }
            while (lastcontact_333_3 != contactlist_333_3.LastOrDefault())
            {
                lastcontact_333_3 = null;
                contactlist_333_3.Remove((ChContactNSC_333_3)lastcontact_333_3);
            }
            while (lastcontact_333_6 != contactlist_333_6.LastOrDefault())
            {
                lastcontact_333_6 = null;
                contactlist_333_6.Remove((ChContactNSC_333_6)lastcontact_333_6);
            }
            while (lastcontact_333_333 != contactlist_333_333.LastOrDefault())
            {
                lastcontact_333_333 = null;
                contactlist_333_333.Remove((ChContactNSC_333_333)lastcontact_333_333);
            }
            while (lastcontact_666_3 != contactlist_666_3.LastOrDefault())
            {
                lastcontact_666_3 = null;
                contactlist_666_3.Remove((ChContactNSC_666_3)lastcontact_666_3);
            }
            while (lastcontact_666_6 != contactlist_666_6.LastOrDefault())
            {
                lastcontact_666_6 = null;
                contactlist_666_6.Remove((ChContactNSC_666_6)lastcontact_666_6);
            }
            while (lastcontact_666_333 != contactlist_666_333.LastOrDefault())
            {
                lastcontact_666_333 = null;
                contactlist_666_333.Remove((ChContactNSC_666_333)lastcontact_666_333);
            }
            while (lastcontact_666_666 != contactlist_666_666.LastOrDefault())
            {
                lastcontact_666_666 = null;
                contactlist_666_666.Remove((ChContactNSC_666_666)lastcontact_666_666);
            }

            while (lastcontact_6_6_rolling != null && lastcontact_6_6_rolling.Current != contactlist_6_6_rolling.LastOrDefault())
            {
                lastcontact_6_6_rolling.MoveNext();
                contactlist_6_6_rolling.Remove(lastcontact_6_6_rolling.Current);
                lastcontact_6_6_rolling = null;
                
            }
        }

        /// Add a contact between two frames.
        public override void AddContact(collision.ChCollisionInfo mcontact)
        {
            // Debug.Assert(mcontact.modelA.GetContactable() != null);
            //  Debug.Assert(mcontact.modelB.GetContactable() != null);

            var contactableA = mcontact.modelA.GetContactable();
            var contactableB = mcontact.modelB.GetContactable();

            // See if both collision models use NSC i.e. 'non-smooth dynamics' material
            // of type ChMaterialSurfaceNSC, trying to downcast from ChMaterialSurface.
            // If not NSC vs NSC, just bailout (ex it could be that this was a SMC vs SMC contact)

            var mmatA = (ChMaterialSurfaceNSC)(contactableA.GetMaterialSurfaceBase());
            var mmatB = (ChMaterialSurfaceNSC)(contactableB.GetMaterialSurfaceBase());

            if (!mmatA || !mmatB)
                return;

            // Bail out if any of the two contactable objects is
            // not contact-active:

            bool inactiveA = !contactableA.IsContactActive();
            bool inactiveB = !contactableB.IsContactActive();

            if ((inactiveA && inactiveB))
                return;

            // CREATE THE CONTACTS
            //
            // Switch among the various cases of contacts: i.e. between a 6-dof variable and another 6-dof variable,
            // or 6 vs 3, etc.
            // These cases are made distinct to exploit the optimization coming from templates and static data sizes
            // in contact types.

            ChBody mmboA = (ChBody)contactableA;
            ChBody mmboB = (ChBody)contactableB;

            if ((mmatA.rolling_friction != 0 && mmatB.rolling_friction != 0) ||
               (mmatA.spinning_friction != 0 && mmatB.spinning_friction != 0))
            {
                _OptimalContactInsertRolling(ref contactlist_6_6_rolling, ref lastcontact_6_6_rolling, ref n_added_6_6_rolling, this,
                                     mmboA, mmboB, mcontact);
            }
            else
            {
                _OptimalContactInsert(ref contactlist_6_6, ref lastcontact_6_6, ref n_added_6_6, this, mmboA, mmboB, mcontact);

            }

            /*
            if (mmboA == (contactableA as ChContactable_1vars<IntInterface.Three>)) {
                 if (mmboB == (contactableB as ChContactable_1vars<IntInterface.Three>)) {
                     // 3_3
                     _OptimalContactInsert(ref contactlist_3_3, ref lastcontact_3_3, ref n_added_3_3, this, mmboA, mmboB, mcontact);         
               } else if (mmboB == (contactableB as ChContactable_1vars<IntInterface.Six>)) {
                   // 3_6 . 6_3
                   collision.ChCollisionInfo swapped_contact = new collision.ChCollisionInfo(mcontact, true);
                   _OptimalContactInsert(ref contactlist_6_3, ref lastcontact_6_3, ref n_added_6_3, this, mmboB, mmboA, swapped_contact);
               } else if (mmboB == (contactableB as ChContactable_3vars<IntInterface.Three, IntInterface.Three, IntInterface.Three>)) {
                   // 3_333 . 333_3
                   collision.ChCollisionInfo swapped_contact = new collision.ChCollisionInfo(mcontact, true);
                   _OptimalContactInsert(ref contactlist_333_3, ref lastcontact_333_3, ref n_added_333_3, this, mmboB, mmboA, swapped_contact);
               } else if (mmboB == (contactableB as ChContactable_3vars<IntInterface.Six, IntInterface.Six, IntInterface.Six>)) {
                   // 3_666 . 666_3
                   collision.ChCollisionInfo swapped_contact = new collision.ChCollisionInfo(mcontact, true);
                   _OptimalContactInsert(ref contactlist_666_3, ref lastcontact_666_3, ref n_added_666_3, this, mmboB, mmboA,
                                         swapped_contact);
               }
           }
           else if (mmboA == (contactableA as ChContactable_1vars<IntInterface.Six>)) {
               if (mmboB == (contactableB as ChContactable_1vars<IntInterface.Three>)) {
                   // 6_3
                   _OptimalContactInsert(ref contactlist_6_3, ref lastcontact_6_3, ref n_added_6_3, this, mmboA, mmboB, mcontact);
               } else if (mmboB == (contactableB as ChContactable_1vars<IntInterface.Six>)) {
                   // 6_6    ***NOTE: for body-body one could have rolling friction: ***
                  // if ((mmatA.rolling_friction != 0 && mmatB.rolling_friction != 0) ||
                    //   (mmatA.spinning_friction != 0 && mmatB.spinning_friction != 0))
                  // {
                      // _OptimalContactInsert(ref contactlist_6_6_rolling, ref lastcontact_6_6_rolling, ref n_added_6_6_rolling, this,
                       //                      mmboA, mmboB, mcontact);
                   //}
                   //else
                  // {
                       _OptimalContactInsert(ref contactlist_6_6, ref lastcontact_6_6, ref n_added_6_6, this, mmboA, mmboB, mcontact);
                  // }
               } else if (mmboB == (contactableB as ChContactable_3vars<IntInterface.Three, IntInterface.Three, IntInterface.Three>)) {
                   // 6_333 . 333_6
                   collision.ChCollisionInfo swapped_contact = new collision.ChCollisionInfo(mcontact, true);
                   _OptimalContactInsert(ref contactlist_333_6, ref lastcontact_333_6, ref n_added_333_6, this, mmboB, mmboA,
                                         swapped_contact);
               } else if (mmboB == (contactableB as ChContactable_3vars<IntInterface.Six, IntInterface.Six, IntInterface.Six>)) {
                   // 6_666 . 666_6
                   collision.ChCollisionInfo swapped_contact = new collision.ChCollisionInfo(mcontact, true);
                   _OptimalContactInsert(ref contactlist_666_6, ref lastcontact_666_6, ref n_added_666_6, this, mmboB, mmboA,
                                         swapped_contact);
               }
           }
           else if (mmboA == (contactableA as ChContactable_3vars<IntInterface.Three, IntInterface.Three, IntInterface.Three>)) {
               if (mmboB == (contactableB as ChContactable_1vars<IntInterface.Three>)) {
                   // 333_3
                   _OptimalContactInsert(ref contactlist_333_3, ref lastcontact_333_3, ref n_added_333_3, this, mmboA, mmboB, mcontact);
               } else if (mmboB == (contactableB as ChContactable_1vars<IntInterface.Six>)) {
                   // 333_6
                   _OptimalContactInsert(ref contactlist_333_6, ref lastcontact_333_6, ref n_added_333_6, this, mmboA, mmboB, mcontact);
               } else if (mmboB == (contactableB as ChContactable_3vars<IntInterface.Three, IntInterface.Three, IntInterface.Three>)) {
                   // 333_333
                   _OptimalContactInsert(ref contactlist_333_333, ref lastcontact_333_333, ref n_added_333_333, this, mmboA, mmboB,
                                         mcontact);
               } else if (mmboB == (contactableB as ChContactable_3vars<IntInterface.Six, IntInterface.Six, IntInterface.Six>)) {
                   // 333_666 . 666_333
                   collision.ChCollisionInfo swapped_contact = new collision.ChCollisionInfo(mcontact, true);
                   _OptimalContactInsert(ref contactlist_666_333, ref lastcontact_666_333, ref n_added_666_333, this, mmboB, mmboA,
                                         swapped_contact);
               }
           }
           else if (mmboA == (contactableA as ChContactable_3vars<IntInterface.Six, IntInterface.Six, IntInterface.Six>)) {
               if (mmboB == (contactableB as ChContactable_1vars<IntInterface.Three>)) {
                   // 666_3
                   _OptimalContactInsert(ref contactlist_666_3, ref lastcontact_666_3, ref n_added_666_3, this, mmboA, mmboB, mcontact);
               } else if (mmboB == (contactableB as ChContactable_1vars<IntInterface.Six>)) {
                   // 666_6
                   _OptimalContactInsert(ref contactlist_666_6, ref lastcontact_666_6, ref n_added_666_6, this, mmboA, mmboB, mcontact);
               } else if (mmboB == (contactableB as ChContactable_3vars<IntInterface.Three, IntInterface.Three, IntInterface.Three>)) {
                   // 666_333
                   _OptimalContactInsert(ref contactlist_666_333, ref lastcontact_666_333, ref n_added_666_333, this, mmboA, mmboB,
                                         mcontact);
               } else if (mmboB == (contactableB as ChContactable_3vars<IntInterface.Six, IntInterface.Six, IntInterface.Six>)) {
                   // 666_666
                   _OptimalContactInsert(ref contactlist_666_666, ref lastcontact_666_666, ref n_added_666_666, this, mmboA, mmboB,
                                         mcontact);
               }
           }*/

            // ***TODO*** Fallback to some dynamic-size allocated constraint for cases that were not trapped by the switch
        }

        //template<class Tcont, class Titer, class Ta, class Tb>
        public void _OptimalContactInsert<Ta, Tb, Tcont, Titer>(ref List<Tcont> contactlist,          //< contact list
                                                                       ref IEnumerator<Titer> lastcontact,                      //< last contact acquired
                                                                                                                                // ref Titer lastcontact,                      //< last contact acquired
                                                                                                                                // ref IEnumerator<ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>> lastcontact,
                                                                       ref int n_added,                            //< number of contact inserted
                                                                       ChContactContainer mcontainer,          //< contact container
                                                                       Ta objA,                                //< collidable object A
                                                                       Tb objB,                                //< collidable object B
                                                                       collision.ChCollisionInfo cinfo  //< collision informations
        )
           // where Titer: object
           //
           where Tcont : ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>
           where Titer : ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>
           where Ta : ChContactable_1vars<IntInterface.Six>
           where Tb : ChContactable_1vars<IntInterface.Six>
        {

            if (/*lastcontact != null &&*/ lastcontact.Current != contactlist.LastOrDefault())
            {
                // reuse old contacts
                lastcontact.MoveNext();
                ((IEnumerator<ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>)lastcontact).Current.Reset(objA, objB, cinfo);
            }
            else
            {
                ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>> mc = new ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>(mcontainer, objA, objB, cinfo);
                Tcont coont = (Tcont)mc;

                contactlist.Add(coont);

                // move to last element in list
                IEnumerator<Tcont> iter = contactlist.GetEnumerator();
                for (int i = 0; i < contactlist.Count; i++)
                {
                    iter.MoveNext();
                }
                lastcontact = (IEnumerator<Titer>)iter;

                // lastcontact = (dynamic)contactlist.GetEnumerator();

            }
            n_added++;
        }

        public void _OptimalContactInsertRolling<Ta, Tb, Tcont, Titer>(ref List<Tcont> contactlist,          //< contact list
                                                                      ref IEnumerator<Titer> lastcontact,                      //< last contact acquired
                                                                                                                               // ref Titer lastcontact,                      //< last contact acquired
                                                                                                                               // ref IEnumerator<ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>> lastcontact,
                                                                      ref int n_added,                            //< number of contact inserted
                                                                      ChContactContainer mcontainer,          //< contact container
                                                                      Ta objA,                                //< collidable object A
                                                                      Tb objB,                                //< collidable object B
                                                                      collision.ChCollisionInfo cinfo  //< collision informations
       )
          // where Titer: object
          //
          where Tcont : ChContactNSCrolling<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>
          where Titer : ChContactNSCrolling<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>
          where Ta : ChContactable_1vars<IntInterface.Six>
          where Tb : ChContactable_1vars<IntInterface.Six>
        {

            if (/*lastcontact != null && */lastcontact.Current != contactlist.LastOrDefault())
            {
                // reuse old contacts
                lastcontact.MoveNext();
                ((IEnumerator<ChContactNSCrolling<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>)lastcontact).Current.Reset(objA, objB, cinfo);
            }
            else
            {
                ChContactNSCrolling<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>> mc = new ChContactNSCrolling<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>(mcontainer, objA, objB, cinfo);
                Tcont coont = (Tcont)mc;

                contactlist.Add(coont);

                // move to last element in list
                IEnumerator<Tcont> iter = contactlist.GetEnumerator();
                for (int i = 0; i < contactlist.Count; i++)
                {
                    iter.MoveNext();
                }
                lastcontact = (IEnumerator<Titer>)iter;

                // lastcontact = (dynamic)contactlist.GetEnumerator();

            }
            n_added++;
        }

        public void _ReportAllContacts<Tcont>(ref List<Tcont> contactlist, ChContactContainer.ReportContactCallback mcallback)
        where Tcont : ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>
        {
            List<Tcont>.Enumerator itercontact = contactlist.GetEnumerator();
            while (itercontact.Current != contactlist.LastOrDefault())
            {
                itercontact.MoveNext();
                bool proceed = mcallback.OnReportContact(
                    ((IEnumerator<ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>)itercontact).Current.GetContactP1(), ((IEnumerator<ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>)itercontact).Current.GetContactP2(), ((IEnumerator<ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>)itercontact).Current.GetContactPlane(),
                    ((IEnumerator<ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>)itercontact).Current.GetContactDistance(), ((IEnumerator<ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>)itercontact).Current.GetEffectiveCurvatureRadius(),
                    ((IEnumerator<ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>)itercontact).Current.GetContactForce(), new ChVector(0, 0, 0), ((IEnumerator<ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>)itercontact).Current.GetObjA(), ((IEnumerator<ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>)itercontact).Current.GetObjB());
                if (!proceed)
                    break;
            }
        }

        //  template<class Tcont>
        public void _ReportAllContactsRolling<Tcont>(ref List<Tcont> contactlist, ChContactContainer.ReportContactCallback mcallback)
        where Tcont : ChContactNSCrolling<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>
        {
            List<Tcont>.Enumerator itercontact = contactlist.GetEnumerator();
            while (itercontact.Current != contactlist.LastOrDefault())
            {
                itercontact.MoveNext();
                bool proceed = mcallback.OnReportContact(
                    ((IEnumerator<ChContactNSCrolling<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>)itercontact).Current.GetContactP1(), ((IEnumerator<ChContactNSCrolling<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>)itercontact).Current.GetContactP2(), ((IEnumerator<ChContactNSCrolling<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>)itercontact).Current.GetContactPlane(),
                    ((IEnumerator<ChContactNSCrolling<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>)itercontact).Current.GetContactDistance(), ((IEnumerator<ChContactNSCrolling<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>)itercontact).Current.GetEffectiveCurvatureRadius(),
                    ((IEnumerator<ChContactNSCrolling<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>)itercontact).Current.GetContactForce(), ((ChBody)(dynamic)itercontact).GetContactTorque(), ((IEnumerator<ChContactNSCrolling<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>)itercontact).Current.GetObjA(),
                    ((IEnumerator<ChContactNSCrolling<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>)itercontact).Current.GetObjB());
                if (!proceed)
                    break;
            }
        }

        /// Scans all the contacts and for each contact executes the OnReportContact()
        /// function of the provided callback object.
        public override void ReportAllContacts(ReportContactCallback mcallback)
        {
            _ReportAllContacts(ref contactlist_6_6, mcallback);
            /*  _ReportAllContacts(ref contactlist_6_3, mcallback);
              _ReportAllContacts(ref contactlist_3_3, mcallback);
              _ReportAllContacts(ref contactlist_333_3, mcallback);
              _ReportAllContacts(ref contactlist_333_6, mcallback);
              _ReportAllContacts(ref contactlist_333_333, mcallback);
              _ReportAllContacts(ref contactlist_666_3, mcallback);
              _ReportAllContacts(ref contactlist_666_6, mcallback);
              _ReportAllContacts(ref contactlist_666_333, mcallback);
              _ReportAllContacts(ref contactlist_666_666, mcallback);*/
            _ReportAllContactsRolling(ref contactlist_6_6_rolling, mcallback);
        }

        /// Tell the number of scalar bilateral constraints (actually, friction
        /// constraints aren't exactly as unilaterals, but count them too)
        public override int GetDOC_d()
        {
            return 3 * (n_added_3_3 + n_added_6_3 + n_added_6_6 + n_added_333_3 + n_added_333_6 + n_added_333_333 +
                        n_added_666_3 + n_added_666_6 + n_added_666_333 + n_added_666_666) +
                   6 * (n_added_6_6_rolling);
        }

        /// In detail, it computes jacobians, violations, etc. and stores
        /// results in inner structures of contacts.
        public override void update(double mytime, bool update_assets = true)
        {
            // Inherit time changes of parent class, basically doing nothing :)
            base.update(mytime, update_assets);
        }

        /// Compute contact forces on all contactable objects in this container.
        public override void ComputeContactForces()
        {
            contact_forces.Clear();
            SumAllContactForces(ref contactlist_3_3, ref contact_forces);
            SumAllContactForces(ref contactlist_6_3, ref contact_forces);
            SumAllContactForces(ref contactlist_6_6, ref contact_forces);
            SumAllContactForces(ref contactlist_333_3, ref contact_forces);
            SumAllContactForces(ref contactlist_333_6, ref contact_forces);
            SumAllContactForces(ref contactlist_333_333, ref contact_forces);
            SumAllContactForces(ref contactlist_666_3, ref contact_forces);
            SumAllContactForces(ref contactlist_666_6, ref contact_forces);
            SumAllContactForces(ref contactlist_666_333, ref contact_forces);
            SumAllContactForces(ref contactlist_666_666, ref contact_forces);
            SumAllContactForces(ref contactlist_6_6_rolling, ref contact_forces);
        }

        //
        // STATE FUNCTIONS
        //

        //template<class Tcont>
        public void _IntStateGatherReactions<Tcont>(ref int coffset,
                                      ref List<Tcont> contactlist,
                                      int off_L,
                                      ref ChVectorDynamic<double> L,
                                      int stride)
        where Tcont : ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>
        {

            List<Tcont>.Enumerator itercontact = contactlist.GetEnumerator();

            //itercontact.MoveNext();
            while (itercontact.Current != contactlist.LastOrDefault())
            {
                itercontact.MoveNext();
                ((IEnumerator<ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>)itercontact).Current.ContIntStateGatherReactions(off_L + coffset, ref L);
                coffset += stride;

            }
        }

        public override void IntStateGatherReactions(int off_L, ref ChVectorDynamic<double> L)
        {
            int coffset = 0;
            _IntStateGatherReactions(ref coffset, ref contactlist_6_6, off_L, ref L, 3);
            /* _IntStateGatherReactions(ref coffset, ref contactlist_6_3, off_L, ref L, 3);
             _IntStateGatherReactions(ref coffset, ref contactlist_3_3, off_L, ref L, 3);
             _IntStateGatherReactions(ref coffset, ref contactlist_333_3, off_L, ref L, 3);
             _IntStateGatherReactions(ref coffset, ref contactlist_333_6, off_L, ref L, 3);
             _IntStateGatherReactions(ref coffset, ref contactlist_333_333, off_L, ref L, 3);
             _IntStateGatherReactions(ref coffset, ref contactlist_666_3, off_L, ref L, 3);
             _IntStateGatherReactions(ref coffset, ref contactlist_666_6, off_L, ref L, 3);
             _IntStateGatherReactions(ref coffset, ref contactlist_666_333, off_L, ref L, 3);
             _IntStateGatherReactions(ref coffset, ref contactlist_666_666, off_L, ref L, 3);*/
            _IntStateGatherReactions(ref coffset, ref contactlist_6_6_rolling, off_L, ref L, 6);
        }

        //  template<class Tcont>
        public void _IntStateScatterReactions<Tcont>(ref int coffset,
                                       ref List<Tcont> contactlist,
                                       int off_L,
                                       ChVectorDynamic<double> L,
                                       int stride)
        where Tcont : ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>
        {
            /*IEnumerator<Tcont> itercontact = (IEnumerator<Tcont>)contactlist.FirstOrDefault();
            while (itercontact != (IEnumerator<Tcont>)contactlist.LastOrDefault())
            {
                ((ChContactTuple<ChBody, ChBody>)itercontact).ContIntStateScatterReactions(off_L + coffset, L);
                coffset += stride;
                itercontact.MoveNext();
            }*/
            List<Tcont>.Enumerator itercontact = contactlist.GetEnumerator();

            while (itercontact.Current != contactlist.LastOrDefault())
            {
                itercontact.MoveNext();
                ((IEnumerator<ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>)itercontact).Current.ContIntStateScatterReactions(off_L + coffset, L);
                coffset += stride;
                // itercontact.MoveNext();
            }
        }

        public override void IntStateScatterReactions(int off_L, ChVectorDynamic<double> L)
        {
            int coffset = 0;
            _IntStateScatterReactions(ref coffset, ref contactlist_6_6, off_L, L, 3);
            /*_IntStateScatterReactions(ref coffset, ref contactlist_6_3, off_L, L, 3);
            _IntStateScatterReactions(ref coffset, ref contactlist_3_3, off_L, L, 3);
            _IntStateScatterReactions(ref coffset, ref contactlist_333_3, off_L, L, 3);
            _IntStateScatterReactions(ref coffset, ref contactlist_333_6, off_L, L, 3);
            _IntStateScatterReactions(ref coffset, ref contactlist_333_333, off_L, L, 3);
            _IntStateScatterReactions(ref coffset, ref contactlist_666_3, off_L, L, 3);
            _IntStateScatterReactions(ref coffset, ref contactlist_666_6, off_L, L, 3);
            _IntStateScatterReactions(ref coffset, ref contactlist_666_333, off_L, L, 3);
            _IntStateScatterReactions(ref coffset, ref contactlist_666_666, off_L, L, 3);*/
            _IntStateScatterReactions(ref coffset, ref contactlist_6_6_rolling, off_L, L, 6);
        }

        //   template<class Tcont>
        public void _IntLoadResidual_CqL<Tcont>(ref int coffset,           //< offset of the contacts
                                  ref List<Tcont> contactlist,  //< list of contacts
                                  int off_L,        //< offset in L multipliers
                                  ref ChVectorDynamic<double> R,            //< result: the R residual, R += c*Cq'*L
                                  ChVectorDynamic<double> L,      //< the L vector
                                  double c,                  //< a scaling factor
                                  int stride                 //< stride
        )
        where Tcont : ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>
        {
            List<Tcont>.Enumerator itercontact = contactlist.GetEnumerator();

            while (itercontact.Current != contactlist.LastOrDefault())
            {
                itercontact.MoveNext();
                ((IEnumerator<ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>)itercontact).Current.ContIntLoadResidual_CqL(off_L + coffset, ref R, L, c);
                coffset += stride;
                // itercontact.MoveNext();
            }
        }

        public override void IntLoadResidual_CqL(int off_L,
                                             ref ChVectorDynamic<double> R,
                                             ChVectorDynamic<double> L,
                                             double c)
        {
            int coffset = 0;
            _IntLoadResidual_CqL(ref coffset, ref contactlist_6_6, off_L, ref R, L, c, 3);
            /*  _IntLoadResidual_CqL(ref coffset, ref contactlist_6_3, off_L, ref R, L, c, 3);
              _IntLoadResidual_CqL(ref coffset, ref contactlist_3_3, off_L, ref R, L, c, 3);
              _IntLoadResidual_CqL(ref coffset, ref contactlist_333_3, off_L, ref R, L, c, 3);
              _IntLoadResidual_CqL(ref coffset, ref contactlist_333_6, off_L, ref R, L, c, 3);
              _IntLoadResidual_CqL(ref coffset, ref contactlist_333_333, off_L, ref R, L, c, 3);
              _IntLoadResidual_CqL(ref coffset, ref contactlist_666_3, off_L, ref R, L, c, 3);
              _IntLoadResidual_CqL(ref coffset, ref contactlist_666_6, off_L, ref R, L, c, 3);
              _IntLoadResidual_CqL(ref coffset, ref contactlist_666_333, off_L, ref R, L, c, 3);
              _IntLoadResidual_CqL(ref coffset, ref contactlist_666_666, off_L, ref R, L, c, 3);*/
            _IntLoadResidual_CqL(ref coffset, ref contactlist_6_6_rolling, off_L, ref R, L, c, 6);
        }

        //  template<class Tcont>
        public void _IntLoadConstraint_C<Tcont>(ref int coffset,           //< contact offset
                                  ref List<Tcont> contactlist,  //< contact list
                                  int off,          //< offset in Qc residual
                                  ref ChVectorDynamic<double> Qc,           //< result: the Qc residual, Qc += c*C
                                  double c,                  //< a scaling factor
                                  bool do_clamp,                   //< apply clamping to c*C?
                                  double recovery_clamp,           //< value for min/max clamping of c*C
                                  int stride                 //< stride
        )
        where Tcont : ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>
        {
            List<Tcont>.Enumerator itercontact = contactlist.GetEnumerator();

            while (itercontact.Current != contactlist.LastOrDefault())
            {
                itercontact.MoveNext();
                ((IEnumerator<ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>)itercontact).Current.ContIntLoadConstraint_C(off + coffset, ref Qc, c, do_clamp, recovery_clamp);
                coffset += stride;
            }
        }

        public override void IntLoadConstraint_C(int off,
                                             ref ChVectorDynamic<double> Qc,
                                             double c,
                                             bool do_clamp,
                                             double recovery_clamp)
        {
            int coffset = 0;
            _IntLoadConstraint_C(ref coffset, ref contactlist_6_6, off, ref Qc, c, do_clamp, recovery_clamp, 3);
            /* _IntLoadConstraint_C(ref coffset, ref contactlist_6_3, off, ref Qc, c, do_clamp, recovery_clamp, 3);
             _IntLoadConstraint_C(ref coffset, ref contactlist_3_3, off, ref Qc, c, do_clamp, recovery_clamp, 3);
             _IntLoadConstraint_C(ref coffset, ref contactlist_333_3, off, ref Qc, c, do_clamp, recovery_clamp, 3);
             _IntLoadConstraint_C(ref coffset, ref contactlist_333_6, off, ref Qc, c, do_clamp, recovery_clamp, 3);
             _IntLoadConstraint_C(ref coffset, ref contactlist_333_333, off, ref Qc, c, do_clamp, recovery_clamp, 3);
             _IntLoadConstraint_C(ref coffset, ref contactlist_666_3, off, ref Qc, c, do_clamp, recovery_clamp, 3);
             _IntLoadConstraint_C(ref coffset, ref contactlist_666_6, off, ref Qc, c, do_clamp, recovery_clamp, 3);
             _IntLoadConstraint_C(ref coffset, ref contactlist_666_333, off, ref Qc, c, do_clamp, recovery_clamp, 3);
             _IntLoadConstraint_C(ref coffset, ref contactlist_666_666, off, ref Qc, c, do_clamp, recovery_clamp, 3);*/
            _IntLoadConstraint_C(ref coffset, ref contactlist_6_6_rolling, off, ref Qc, c, do_clamp, recovery_clamp, 6);
        }

        //   template<class Tcont>
        public void _IntToDescriptor<Tcont>(ref int coffset,
                              ref List<Tcont> contactlist,
                              int off_v,
                              ChStateDelta v,
                              ChVectorDynamic<double> R,
                              int off_L,
                              ChVectorDynamic<double> L,
                              ChVectorDynamic<double> Qc,
                              int stride)
        where Tcont : ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>
        {
            List<Tcont>.Enumerator itercontact = contactlist.GetEnumerator();

            while (itercontact.Current != contactlist.LastOrDefault())
            {
                itercontact.MoveNext();
                ((IEnumerator<ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>)itercontact).Current.ContIntToDescriptor(off_L + coffset, L, Qc);
                coffset += stride;
            }
        }

        public override void IntToDescriptor(int off_v,
                                         ChStateDelta v,
                                         ChVectorDynamic<double> R,
                                         int off_L,
                                         ChVectorDynamic<double> L,
                                         ChVectorDynamic<double> Qc)
        {
            int coffset = 0;
            _IntToDescriptor(ref coffset, ref contactlist_6_6, off_v, v, R, off_L, L, Qc, 3);
            /*_IntToDescriptor(ref coffset, ref contactlist_6_3, off_v, v, R, off_L, L, Qc, 3);
            _IntToDescriptor(ref coffset, ref contactlist_3_3, off_v, v, R, off_L, L, Qc, 3);
            _IntToDescriptor(ref coffset, ref contactlist_333_3, off_v, v, R, off_L, L, Qc, 3);
            _IntToDescriptor(ref coffset, ref contactlist_333_6, off_v, v, R, off_L, L, Qc, 3);
            _IntToDescriptor(ref coffset, ref contactlist_333_333, off_v, v, R, off_L, L, Qc, 3);
            _IntToDescriptor(ref coffset, ref contactlist_666_3, off_v, v, R, off_L, L, Qc, 3);
            _IntToDescriptor(ref coffset, ref contactlist_666_6, off_v, v, R, off_L, L, Qc, 3);
            _IntToDescriptor(ref coffset, ref contactlist_666_333, off_v, v, R, off_L, L, Qc, 3);
            _IntToDescriptor(ref coffset, ref contactlist_666_666, off_v, v, R, off_L, L, Qc, 3);*/
            _IntToDescriptor(ref coffset, ref contactlist_6_6_rolling, off_v, v, R, off_L, L, Qc, 6);
        }

        //    template<class Tcont>
        public void _IntFromDescriptor<Tcont>(ref int coffset,
                                ref List<Tcont> contactlist,
                                int off_v,
                                ref ChStateDelta v,
                                int off_L,
                                ref ChVectorDynamic<double> L,
                                int stride)
        where Tcont : ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>
        {

            List<Tcont>.Enumerator itercontact = contactlist.GetEnumerator();

            while (itercontact.Current != contactlist.LastOrDefault())
            {
                itercontact.MoveNext();
                ((IEnumerator<ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>)itercontact).Current.ContIntFromDescriptor(off_L + coffset, ref L);

            }
        }

        public override void IntFromDescriptor(int off_v,
                                           ref ChStateDelta v,
                                           int off_L,
                                           ref ChVectorDynamic<double> L)
        {
            int coffset = 0;
            _IntFromDescriptor(ref coffset, ref contactlist_6_6, off_v, ref v, off_L, ref L, 3);
            /* _IntFromDescriptor(ref coffset, ref contactlist_6_3, off_v, ref v, off_L, ref L, 3);
             _IntFromDescriptor(ref coffset, ref contactlist_3_3, off_v, ref v, off_L, ref L, 3);
             _IntFromDescriptor(ref coffset, ref contactlist_333_3, off_v, ref v, off_L, ref L, 3);
             _IntFromDescriptor(ref coffset, ref contactlist_333_6, off_v, ref v, off_L, ref L, 3);
             _IntFromDescriptor(ref coffset, ref contactlist_333_333, off_v, ref v, off_L, ref L, 3);
             _IntFromDescriptor(ref coffset, ref contactlist_666_3, off_v, ref v, off_L, ref L, 3);
             _IntFromDescriptor(ref coffset, ref contactlist_666_6, off_v, ref v, off_L, ref L, 3);
             _IntFromDescriptor(ref coffset, ref contactlist_666_333, off_v, ref v, off_L, ref L, 3);
             _IntFromDescriptor(ref coffset, ref contactlist_666_666, off_v, ref v, off_L, ref L, 3);*/
            _IntFromDescriptor(ref coffset, ref contactlist_6_6_rolling, off_v, ref v, off_L, ref L, 6);
        }

        //
        // SOLVER INTERFACE
        //

        //   template<class Tcont>
        public void _InjectConstraints<Tcont>(ref List<Tcont> contactlist, ref ChSystemDescriptor mdescriptor)
        where Tcont : ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>
        {
            List<Tcont>.Enumerator itercontact = contactlist.GetEnumerator();

            while (itercontact.Current != contactlist.LastOrDefault())
            {
                itercontact.MoveNext();
                ((IEnumerator<ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>)itercontact).Current.InjectConstraints(ref mdescriptor);
            }

        }

        public override void InjectConstraints(ref ChSystemDescriptor mdescriptor)
        {
            _InjectConstraints(ref contactlist_6_6, ref mdescriptor);
            /*_InjectConstraints(ref contactlist_6_3, ref mdescriptor);
            _InjectConstraints(ref contactlist_3_3, ref mdescriptor);
            _InjectConstraints(ref contactlist_333_3, ref mdescriptor);
            _InjectConstraints(ref contactlist_333_6, ref mdescriptor);
            _InjectConstraints(ref contactlist_333_333, ref mdescriptor);
            _InjectConstraints(ref contactlist_666_3, ref mdescriptor);
            _InjectConstraints(ref contactlist_666_6, ref mdescriptor);
            _InjectConstraints(ref contactlist_666_333, ref mdescriptor);
            _InjectConstraints(ref contactlist_666_666, ref mdescriptor);*/
            _InjectConstraints(ref contactlist_6_6_rolling, ref mdescriptor);
        }

        //  template<class Tcont>
        public void _ConstraintsBiReset<Tcont>(ref List<Tcont> contactlist)
        where Tcont : ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>
        {
            /* IEnumerator<Tcont> itercontact = (IEnumerator<Tcont>)contactlist.FirstOrDefault();
             while (itercontact != (IEnumerator<Tcont>)contactlist.LastOrDefault())
             {
                 ((ChContactTuple<ChBody, ChBody>)itercontact).ConstraintsBiReset();
                 itercontact.MoveNext();
             }*/
            List<Tcont>.Enumerator itercontact = contactlist.GetEnumerator();

            while (itercontact.Current != contactlist.LastOrDefault())
            {
                itercontact.MoveNext();
                ((IEnumerator<ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>)itercontact).Current.ConstraintsBiReset();
                // itercontact.MoveNext();
            }

        }

        public override void ConstraintsBiReset()
        {
            _ConstraintsBiReset(ref contactlist_6_6);
            /*_ConstraintsBiReset(ref contactlist_6_3);
            _ConstraintsBiReset(ref contactlist_3_3);
            _ConstraintsBiReset(ref contactlist_333_3);
            _ConstraintsBiReset(ref contactlist_333_6);
            _ConstraintsBiReset(ref contactlist_333_333);
            _ConstraintsBiReset(ref contactlist_666_3);
            _ConstraintsBiReset(ref contactlist_666_6);
            _ConstraintsBiReset(ref contactlist_666_333);
            _ConstraintsBiReset(ref contactlist_666_666);*/
            _ConstraintsBiReset(ref contactlist_6_6_rolling);
        }

        // template<class Tcont>
        public void _ConstraintsBiLoad_C<Tcont>(ref List<Tcont> contactlist, double factor, double recovery_clamp, bool do_clamp)
        where Tcont : ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>
        {
            /*  IEnumerator<Tcont> itercontact = (IEnumerator<Tcont>)contactlist.FirstOrDefault();
              while (itercontact != (IEnumerator<Tcont>)contactlist.LastOrDefault())
              {
                  ((ChContactTuple<ChBody, ChBody>)itercontact).ConstraintsBiLoad_C(factor, recovery_clamp, do_clamp);
                  itercontact.MoveNext();
              }*/
            List<Tcont>.Enumerator itercontact = contactlist.GetEnumerator();

            while (itercontact.Current != contactlist.LastOrDefault())
            {
                itercontact.MoveNext();
                ((IEnumerator<ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>)itercontact).Current.ConstraintsBiLoad_C(factor, recovery_clamp, do_clamp);
                // itercontact.MoveNext();
            }

        }

        public override void ConstraintsBiLoad_C(double factor = 1, double recovery_clamp = 0.1, bool do_clamp = false)
        {
            _ConstraintsBiLoad_C(ref contactlist_6_6, factor, recovery_clamp, do_clamp);
            /*_ConstraintsBiLoad_C(ref contactlist_6_3, factor, recovery_clamp, do_clamp);
            _ConstraintsBiLoad_C(ref contactlist_3_3, factor, recovery_clamp, do_clamp);
            _ConstraintsBiLoad_C(ref contactlist_333_3, factor, recovery_clamp, do_clamp);
            _ConstraintsBiLoad_C(ref contactlist_333_6, factor, recovery_clamp, do_clamp);
            _ConstraintsBiLoad_C(ref contactlist_333_333, factor, recovery_clamp, do_clamp);
            _ConstraintsBiLoad_C(ref contactlist_666_3, factor, recovery_clamp, do_clamp);
            _ConstraintsBiLoad_C(ref contactlist_666_6, factor, recovery_clamp, do_clamp);
            _ConstraintsBiLoad_C(ref contactlist_666_333, factor, recovery_clamp, do_clamp);
            _ConstraintsBiLoad_C(ref contactlist_666_666, factor, recovery_clamp, do_clamp);*/
            _ConstraintsBiLoad_C(ref contactlist_6_6_rolling, factor, recovery_clamp, do_clamp);
        }

        public override void ConstraintsLoadJacobians()
        {
            // already loaded when contact objects are created
        }

        // template<class Tcont>
        public void _ConstraintsFetch_react<Tcont>(ref List<Tcont> contactlist, double factor)
        where Tcont : ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>
        {
            // From constraints to react vector:
            /* IEnumerator<Tcont> itercontact = (IEnumerator<Tcont>)contactlist.FirstOrDefault();
             while (itercontact != (IEnumerator<Tcont>)contactlist.LastOrDefault())
             {
                 ((ChContactTuple<ChBody, ChBody>)itercontact).ConstraintsFetch_react(factor);
                 itercontact.MoveNext();
             }*/
            List<Tcont>.Enumerator itercontact = contactlist.GetEnumerator();

            while (itercontact.Current != contactlist.LastOrDefault())
            {
                itercontact.MoveNext();
                ((IEnumerator<ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>)itercontact).Current.ConstraintsFetch_react(factor);
                // itercontact.MoveNext();
            }

        }

        public override void ConstraintsFetch_react(double factor = 1)
        {
            _ConstraintsFetch_react(ref contactlist_6_6, factor);
            /* _ConstraintsFetch_react(ref contactlist_6_3, factor);
             _ConstraintsFetch_react(ref contactlist_3_3, factor);
             _ConstraintsFetch_react(ref contactlist_333_3, factor);
             _ConstraintsFetch_react(ref contactlist_333_6, factor);
             _ConstraintsFetch_react(ref contactlist_333_333, factor);
             _ConstraintsFetch_react(ref contactlist_666_3, factor);
             _ConstraintsFetch_react(ref contactlist_666_6, factor);
             _ConstraintsFetch_react(ref contactlist_666_333, factor);
             _ConstraintsFetch_react(ref contactlist_666_666, factor);*/
            _ConstraintsFetch_react(ref contactlist_6_6_rolling, factor);
        }
    }
}