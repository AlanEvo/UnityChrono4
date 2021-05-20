using System.Collections;
using System.Collections.Generic;
using System;
using System.Linq;
using UnityEngine;
using UnityEngine.Serialization;
using System.Reflection;

namespace chrono
{
    /// Class representing a container of many smooth (penalty) contacts.
    /// This is implemented as a typical linked list of ChContactSMC objects
    /// (that is, contacts between two ChContactable objects).
    public class ChContactContainerSMC : ChContactContainer
    {
        /* public class ChContactNSC_6_6 : ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>> { }
         public class ChContactNSC_6_3 : ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Three>> { }
         public class ChContactNSC_3_3 : ChContactNSC<ChContactable_1vars<IntInterface.Three>, ChContactable_1vars<IntInterface.Three>> { }
         public class ChContactNSC_333_3 : ChContactNSC<ChContactable_3vars<IntInterface.Three, IntInterface.Three, IntInterface.Three>, ChContactable_1vars<IntInterface.Three>> { }
         public class ChContactNSC_333_6 : ChContactNSC<ChContactable_3vars<IntInterface.Three, IntInterface.Three, IntInterface.Three>, ChContactable_1vars<IntInterface.Six>> { }
         public class ChContactNSC_333_333 : ChContactNSC<ChContactable_3vars<IntInterface.Three, IntInterface.Three, IntInterface.Three>, ChContactable_3vars<IntInterface.Three, IntInterface.Three, IntInterface.Three>> { }
         public class ChContactNSC_666_3 : ChContactNSC<ChContactable_3vars<IntInterface.Six, IntInterface.Six, IntInterface.Six>, ChContactable_1vars<IntInterface.Three>> { }
         public class ChContactNSC_666_6 : ChContactNSC<ChContactable_3vars<IntInterface.Six, IntInterface.Six, IntInterface.Six>, ChContactable_1vars<IntInterface.Six>> { }
         public class ChContactNSC_666_333 : ChContactNSC<ChContactable_3vars<IntInterface.Six, IntInterface.Six, IntInterface.Six>, ChContactable_3vars<IntInterface.Three, IntInterface.Three, IntInterface.Three>> { }
         public class ChContactNSC_666_666 : ChContactNSC<ChContactable_3vars<IntInterface.Six, IntInterface.Six, IntInterface.Six>, ChContactable_3vars<IntInterface.Six, IntInterface.Six, IntInterface.Six>> { }

         public class ChContactNSCrolling_6_6 : ChContactNSCrolling<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>> { }*/

        protected List<ChContactSMC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>> contactlist_6_6 = new List<ChContactSMC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>();
        /*protected LinkedList<ChContactNSC_6_3> contactlist_6_3 = new LinkedList<ChContactNSC_6_3>();
        protected LinkedList<ChContactNSC_3_3> contactlist_3_3 = new LinkedList<ChContactNSC_3_3>();
        protected LinkedList<ChContactNSC_333_3> contactlist_333_3 = new LinkedList<ChContactNSC_333_3>();
        protected LinkedList<ChContactNSC_333_6> contactlist_333_6 = new LinkedList<ChContactNSC_333_6>();
        protected LinkedList<ChContactNSC_333_333> contactlist_333_333 = new LinkedList<ChContactNSC_333_333>();
        protected LinkedList<ChContactNSC_666_3> contactlist_666_3 = new LinkedList<ChContactNSC_666_3>();
        protected LinkedList<ChContactNSC_666_6> contactlist_666_6 = new LinkedList<ChContactNSC_666_6>();
        protected LinkedList<ChContactNSC_666_333> contactlist_666_333 = new LinkedList<ChContactNSC_666_333>();
        protected LinkedList<ChContactNSC_666_666> contactlist_666_666 = new LinkedList<ChContactNSC_666_666>();

        protected LinkedList<ChContactNSCrolling_6_6> contactlist_6_6_rolling = new LinkedList<ChContactNSCrolling_6_6>();*/

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

        protected IEnumerator<ChContactSMC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>> lastcontact_6_6;
        /* protected IEnumerator<ChContactNSC_6_3> lastcontact_6_3;
         protected IEnumerator<ChContactNSC_3_3> lastcontact_3_3;
         protected IEnumerator<ChContactNSC_333_3> lastcontact_333_3;
         protected IEnumerator<ChContactNSC_333_6> lastcontact_333_6;
         protected IEnumerator<ChContactNSC_333_333> lastcontact_333_333;
         protected IEnumerator<ChContactNSC_666_3> lastcontact_666_3;
         protected IEnumerator<ChContactNSC_666_6> lastcontact_666_6;
         protected IEnumerator<ChContactNSC_666_333> lastcontact_666_333;
         protected IEnumerator<ChContactNSC_666_666> lastcontact_666_666;

         protected IEnumerator<ChContactNSCrolling_6_6> lastcontact_6_6_rolling;*/

        public ChContactContainerSMC() : base()
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

        public ChContactContainerSMC(ChContactContainerSMC other) : base(other)
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
           // contactlist.Clear();
            //lastcontact = contactlist.First<Tcont>();
            n_added = 0;

        }

        public override void RemoveAllContacts()
        {
            _RemoveAllContacts(ref contactlist_6_6, ref lastcontact_6_6, ref n_added_6_6);
            /* _RemoveAllContacts(ref contactlist_6_3, ref lastcontact_6_3, ref n_added_6_3);
             _RemoveAllContacts(ref contactlist_3_3, ref lastcontact_3_3, ref n_added_3_3);
             _RemoveAllContacts(ref contactlist_333_3, ref lastcontact_333_3, ref n_added_333_3);
             _RemoveAllContacts(ref contactlist_333_6, ref lastcontact_333_6, ref n_added_333_6);
             _RemoveAllContacts(ref contactlist_333_333, ref lastcontact_333_333, ref n_added_333_333);
             _RemoveAllContacts(ref contactlist_666_3, ref lastcontact_666_3, ref n_added_666_3);
             _RemoveAllContacts(ref contactlist_666_6, ref lastcontact_666_6, ref n_added_666_6);
             _RemoveAllContacts(ref contactlist_666_333, ref lastcontact_666_333, ref n_added_666_333);
             _RemoveAllContacts(ref contactlist_666_666, ref lastcontact_666_666, ref n_added_666_666);
             _RemoveAllContacts(ref contactlist_6_6_rolling, ref lastcontact_6_6_rolling, ref n_added_6_6_rolling);*/
        }

        /// The collision system will call BeginAddContact() before adding
        /// all contacts (for example with AddContact() or similar). Instead of
        /// simply deleting all list of the previous contacts, this optimized implementation
        /// rewinds the link iterator to begin and tries to reuse previous contact objects
        /// until possible, to avoid too much allocation/deallocation.
        public override void BeginAddContact()
        {
            //lastcontact_6_6 = (IEnumerator<ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>)Enumerable.Empty<ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>();//(IEnumerator<ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>)contactlist_6_6.GetEnumerator().;//(LinkedList<ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>)Enumerable.Empty<ChContactNSC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>();// = (IEnumerator<ChContactNSC_6_6>)contactlist_6_6.FirstOrDefault();
            //contactlist_6_6.GetEnumerator().MoveNext();
            //for (int i = 0; i <= 1; i++, contactlist_6_6.GetEnumerator().MoveNext())
            // {
            //iter.Current;
            //contactlist_6_6.GetEnumerator().MoveNext();
            lastcontact_6_6 = contactlist_6_6.GetEnumerator();
            n_added_6_6 = 0;


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

        }

        /// Add a contact between two frames.
        public override void AddContact(collision.ChCollisionInfo mcontact)
        {
           // Debug.Assert(mcontact.modelA.GetContactable() != null);
           // Debug.Assert(mcontact.modelB.GetContactable() != null);

            var contactableA = mcontact.modelA.GetContactable();
            var contactableB = mcontact.modelB.GetContactable();

            // Check that the two collision models are compatible with penalty contact.
            // If either one has a contact material for complementarity, skip processing this contact.
            var mmatA = (ChMaterialSurfaceSMC)(contactableA.GetMaterialSurfaceBase());
            var mmatB = (ChMaterialSurfaceSMC)(contactableB.GetMaterialSurfaceBase());

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

           /* if ((mmatA.rolling_friction != 0 && mmatB.rolling_friction != 0) ||
                (mmatA.spinning_friction != 0 && mmatB.spinning_friction != 0))
            {
                _OptimalContactInsert(ref contactlist_6_6_rolling, ref lastcontact_6_6_rolling, ref n_added_6_6_rolling, this,
                                     mmboA, mmboB, mcontact);
            }
            else
            {*/
                _OptimalContactInsert(ref contactlist_6_6, ref lastcontact_6_6, ref n_added_6_6, this, mmboA, mmboB, mcontact);

           // }


        }

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
        where Tcont : ChContactSMC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>
        where Titer : ChContactSMC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>
        where Ta : ChContactable_1vars<IntInterface.Six>
        where Tb : ChContactable_1vars<IntInterface.Six>
        {

            if (/*lastcontact != null && */lastcontact.Current != contactlist.LastOrDefault())
            {
                // reuse old contacts
                lastcontact.MoveNext();
                ((IEnumerator<ChContactSMC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>)lastcontact).Current.Reset(objA, objB, cinfo);
            }
            else
            {
                ChContactSMC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>> mc = new ChContactSMC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>(mcontainer, objA, objB, cinfo);
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
            where Tcont : ChContactSMC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>
        {
            List<Tcont>.Enumerator itercontact = contactlist.GetEnumerator();
            while (itercontact.Current != contactlist.LastOrDefault())
            {
                itercontact.MoveNext();
                bool proceed = mcallback.OnReportContact(
                    ((IEnumerator<ChContactSMC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>)itercontact).Current.GetContactP1(), ((IEnumerator<ChContactSMC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>)itercontact).Current.GetContactP2(), ((IEnumerator<ChContactSMC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>)itercontact).Current.GetContactPlane(),
                    ((IEnumerator<ChContactSMC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>)itercontact).Current.GetContactDistance(), ((IEnumerator<ChContactSMC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>)itercontact).Current.GetEffectiveCurvatureRadius(),
                    ((IEnumerator<ChContactSMC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>)itercontact).Current.GetContactForce(), new ChVector(0, 0, 0), ((IEnumerator<ChContactSMC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>)itercontact).Current.GetObjA(), ((IEnumerator<ChContactSMC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>)itercontact).Current.GetObjB());
                if (!proceed)
                    break;
            }
        }

        public override void ReportAllContacts(ReportContactCallback mcallback)
        {
            //  _ReportAllContacts(ref contactlist_3_3, mcallback);
            //  _ReportAllContacts(ref contactlist_6_3, mcallback);
            _ReportAllContacts(ref contactlist_6_6, mcallback);
            /*_ReportAllContacts(ref contactlist_333_3, mcallback);
            _ReportAllContacts(ref contactlist_333_6, mcallback);
            _ReportAllContacts(ref contactlist_333_333, mcallback);
            _ReportAllContacts(ref contactlist_666_3, mcallback);
            _ReportAllContacts(ref contactlist_666_6, mcallback);
            _ReportAllContacts(ref contactlist_666_333, mcallback);
            _ReportAllContacts(ref contactlist_666_666, mcallback);*/
        }

        public void _IntLoadResidual_F<Tcont>(ref List<Tcont> contactlist, ref ChVectorDynamic<double> R, double c)
            where Tcont : ChContactSMC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>
        {
            List<Tcont>.Enumerator itercontact = contactlist.GetEnumerator();

            //itercontact.MoveNext();
            while (itercontact.Current != contactlist.LastOrDefault())
            {
                itercontact.MoveNext();
                ((IEnumerator<ChContactSMC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>)itercontact).Current.ContIntLoadResidual_F(ref R, c);

            }
        }

        public override void IntLoadResidual_F(int off, ref ChVectorDynamic<double> R, double c)
        {
            // _IntLoadResidual_F(contactlist_3_3, R, c);
            // _IntLoadResidual_F(contactlist_6_3, R, c);
            _IntLoadResidual_F(ref contactlist_6_6, ref R, c);
            /*_IntLoadResidual_F(contactlist_333_3, R, c);
            _IntLoadResidual_F(contactlist_333_6, R, c);
            _IntLoadResidual_F(contactlist_333_333, R, c);
            _IntLoadResidual_F(contactlist_666_3, R, c);
            _IntLoadResidual_F(contactlist_666_6, R, c);
            _IntLoadResidual_F(contactlist_666_333, R, c);
            _IntLoadResidual_F(contactlist_666_666, R, c);*/
        }

        public void _KRMmatricesLoad<Tcont>(List<Tcont> contactlist, double Kfactor, double Rfactor)
            where Tcont : ChContactSMC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>
        {
            List<Tcont>.Enumerator itercontact = contactlist.GetEnumerator();

            //itercontact.MoveNext();
            while (itercontact.Current != contactlist.LastOrDefault())
            {
                itercontact.MoveNext();
                ((IEnumerator<ChContactSMC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>)itercontact).Current.ContKRMmatricesLoad(Kfactor, Rfactor);

            }
        }

        public override void KRMmatricesLoad(double Kfactor, double Rfactor, double Mfactor)
        {
            // _KRMmatricesLoad(contactlist_3_3, Kfactor, Rfactor);
            // _KRMmatricesLoad(contactlist_6_3, Kfactor, Rfactor);
            _KRMmatricesLoad(contactlist_6_6, Kfactor, Rfactor);
            /* _KRMmatricesLoad(contactlist_333_3, Kfactor, Rfactor);
             _KRMmatricesLoad(contactlist_333_6, Kfactor, Rfactor);
             _KRMmatricesLoad(contactlist_333_333, Kfactor, Rfactor);
             _KRMmatricesLoad(contactlist_666_3, Kfactor, Rfactor);
             _KRMmatricesLoad(contactlist_666_6, Kfactor, Rfactor);
             _KRMmatricesLoad(contactlist_666_333, Kfactor, Rfactor);
             _KRMmatricesLoad(contactlist_666_666, Kfactor, Rfactor);*/
        }

        public void _InjectKRMmatrices<Tcont>(List<Tcont> contactlist, ref ChSystemDescriptor mdescriptor)
            where Tcont : ChContactSMC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>
        {
            List<Tcont>.Enumerator itercontact = contactlist.GetEnumerator();

            //itercontact.MoveNext();
            while (itercontact.Current != contactlist.LastOrDefault())
            {
                itercontact.MoveNext();
                ((IEnumerator<ChContactSMC<ChContactable_1vars<IntInterface.Six>, ChContactable_1vars<IntInterface.Six>>>)itercontact).Current.ContInjectKRMmatrices(ref mdescriptor);

            }
        }
        public override void InjectKRMmatrices(ref ChSystemDescriptor mdescriptor)
        {
           // _InjectKRMmatrices(contactlist_3_3, mdescriptor);
           // _InjectKRMmatrices(contactlist_6_3, mdescriptor);
            _InjectKRMmatrices(contactlist_6_6, ref mdescriptor);
            /*_InjectKRMmatrices(contactlist_333_3, mdescriptor);
            _InjectKRMmatrices(contactlist_333_6, mdescriptor);
            _InjectKRMmatrices(contactlist_333_333, mdescriptor);
            _InjectKRMmatrices(contactlist_666_3, mdescriptor);
            _InjectKRMmatrices(contactlist_666_6, mdescriptor);
            _InjectKRMmatrices(contactlist_666_333, mdescriptor);
            _InjectKRMmatrices(contactlist_666_666, mdescriptor);*/
        }

    }

}