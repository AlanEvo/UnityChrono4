using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Reflection;

namespace chrono
{
    ///
    /// This is a base class for name-value pairs
    ///

    public class ChNameValue<T>
    {

        public ChNameValue(char mname, T mvalue, char mflags)
        {
            _name = mname;
            _value = ((T)(mvalue));
            _flags = ((char)mflags);
        }

        public ChNameValue(ChNameValue<T> other)
        {
            _name = other._name;
            _value = other._value;
            _flags = other._flags;
        }


        public char name()
        {
            return this._name;
        }

        public char flags()
        {
            return this._flags;
        }

        public T value()
        {
            return (this._value);
        }

        public T const_value()
        {
            return (this._value);
        }


        protected T _value;
        protected char _name;
        char _flags;
    };

    ///////////////////////////////

    /// Class that handle C++ values of generic type using type erasure and functors.
    /// For example used to call the ArchiveOUT function for unrelated classes that
    /// implemented them. This helps stripping out the templating, to make ChArchiveOut
    /// easier and equippable with virtual functions.

    public abstract class ChValue
    {


        public abstract ChValue new_clone();

        // 
        // Type helpers
        // 
        /// Get registered name in class factory. If type is not previously registered, 
        /// returns a "" string.
        /// This is platform-independent.
        public abstract string GetClassRegisteredName();

        /// Get class version, if class version is registered, otherwise defaults 0
        public abstract int GetClassRegisteredVersion();

        /// Get platform-dependent typeid name of referenced data
        public abstract char GetTypeidName();

        /// Get platform-dependent typeid of referenced data
        public abstract TypeInfo GetTypeid();

        /// Tell if it is a null pointer    
        public abstract bool IsNull();

        /// Tell if the underlying original type is polymorphic
        public abstract bool IsPolymorphic();
        /// Tell if the underlying original type is an array
        public abstract bool IsArray();
        /// Tell if the underlying original type is a class
        public abstract bool IsClass();
        /// Tell if the underlying original type is a pointer
        public abstract bool IsPointer();

        /// Access the data by a raw pointer, given as static_cast
        public abstract object GetRawPtr();

        /// Get name of property
        public string name()
        {
            return this._name.ToString();
        }

        /// Get flags of property
        public char flags()
        {
            return this._flags;
        }


        //    
        // Casting:
        // 

        /// Use this to do safe dynamic cast. 
        /// This uses the cast/throw trick for dynamic cast after type-erasure; note that this 
        /// has performance penalty respect to usual dynamic_cast<>, which is not possible here.
        /// Note: can only upcast, but no downcast (i.e. U must be higher the TClass used
        /// when instantiating ChValueSpecific); otherwise use a later dynamic_cast<>.

        /* U* PointerUpCast()
         {
             try { this->thrower(); }
             catch (U* ptr) { return static_cast<U*>(ptr); }
             catch (...) { }
             return 0;
             }*/



        // 
        // Call members from names (without needing to have them in base classes)
        // 

        /// Use this to call ArchiveOut member function.
        public abstract void CallArchiveOut(ref ChArchiveOut marchive);

        /// Use this to call (optional) member function ArchiveOUTconstructor. This is
        /// expected to serialize constructor parameters if any. 
        /// If ArchiveOUTconstructor is not provided, simply does nothing.
        public abstract void CallArchiveOutConstructor(ref ChArchiveOut marchive);

        /// Tell if the object has the ArchiveContainerName() function; if so you might call CallArchiveContainerName
        public abstract bool HasArchiveContainerName();

        /// Use this to call ArchiveContainerName member function, if present
        public abstract string CallArchiveContainerName();

        public abstract void CallOut(ref ChArchiveOut marchive);


        protected abstract void thrower();

        //const char* _name;
        protected string _name;
        protected char _flags;

    };


    /// Class for mapping enums to ChNameValue pairs that contain a 'readable' ascii string
    /// of the selected enum. This could be used when streaming from/to human readable formats
    /// such as JSON or XML or ascii dumps.

    public abstract class ChEnumMapperBase
    {

        public ChEnumMapperBase() { }

        public abstract int GetValueAsInt();
        public abstract void SetValueAsInt(int mval);

        public abstract string GetValueAsString();
        public abstract bool SetValueAsString(string mname);
    };

    ///
    /// This is a base class for archives with pointers to shared objects 
    ///
    public class ChArchive : MonoBehaviour
    {

        protected bool cluster_class_versions;

        protected Dictionary<int, int> class_versions;

        protected bool use_versions;

        public ChArchive()
        {
            use_versions = true;
            cluster_class_versions = true;
        }

        /// By default, version numbers are saved in archives
        /// Use this to turn off version info in archives (either save/load both
        /// with version info, or not, do not mix because it could give problems in binary archives.).
        public void SetUseVersions(bool muse) { this.use_versions = muse; }

        /// If true, the version number is not saved in each class: rather, 
        /// it is saved only the first time that class is encountered. 
        /// The same setting must be used for both serialization and deserialization.
        public void SetClusterClassVersions(bool mcl) { this.cluster_class_versions = mcl; }

    }

    ///
    /// This is a base class for serializing into archives
    ///

    public abstract class ChArchiveOut : ChArchive
    {


        protected Dictionary<object, int> internal_ptr_id;

        protected int currentID;

        protected Dictionary<object, int> external_ptr_id;

        protected HashSet<object> cut_pointers;

        protected bool cut_all_pointers;


        public ChArchiveOut()
        {

            cut_all_pointers = false;

            internal_ptr_id.Clear();
            internal_ptr_id[0] = (0);  // ID=0 -> null pointer.
            currentID = 0;
        }

        /// If you enable  SetCutAllPointers(true), no serialization happens for 
        /// objects referenced via pointers. This can be useful to save a single object, 
        /// regardless of the fact that it contains pointers to other 'children' objects.
        /// Cut pointers are turned into null pointers.
        public void SetCutAllPointers(bool mcut) { this.cut_all_pointers = mcut; }

        /// Access the container of pointers that must not be serialized.
        /// This is in case SetCutAllPointers(true) is too extreme. So you can 
        /// selectively 'cut' the network of pointers when serializing an object that
        /// has a network of sub objects. Works also for shared pointers, but remember to store
        /// the embedded pointer, not the shared pointer itself. For instance:
        ///    myarchive.CutPointers().insert(my_raw_pointer); // normal pointers
        ///    myarchive.CutPointers().insert(my_shared_pointer.get());  // shared pointers
        /// The cut pointers are serialized as null pointers.
        public HashSet<object> CutPointers() { return cut_pointers; }




        /// Use the following to declare pointer(s) that must not be de-serialized
        /// but rather be 'unbind' and be saved just as unique IDs.
        /// Note, the IDs can be whatever integer > 0. Use unique IDs per each pointer. 
        /// Note, the same IDs must be used when de-serializing pointers in ArchiveIN.
        public void UnbindExternalPointer(object mptr, int ID)
        {
            external_ptr_id[mptr] = ID;
        }
        /// Use the following to declare pointer(s) that must not be de-serialized
        /// but rather be 'unbind' and be saved just as unique IDs.
        /// Note, the IDs can be whatever integer > 0. Use unique IDs per each pointer. 
        /// Note, the same IDs must be used when de-serializing pointers in ArchiveIN.
        /* public void UnbindExternalPointer(object mptr, int ID)
         {
             external_ptr_id[mptr] = ID;
         }*/

        /// Find a pointer in pointer map: eventually add it to map if it
        /// was not previously inserted. Returns already_stored=false if was
        /// already inserted. Return 'obj_ID' offset in vector in any case.
        /// For null pointers, always return 'already_stored'=true, and 'obj_ID'=0.
        public void PutPointer(object obj, ref bool already_stored, ref int obj_ID)
        {
            /* if (this.internal_ptr_id.TryGetValue((object)(obj)) != this.internal_ptr_id.Values.end())
             {
                 already_stored = true;
                 obj_ID = internal_ptr_id[static_cast<void*>(obj)];
                 return;
             }

             // wasn't in list.. add to it
             ++currentID;
             obj_ID = currentID;
             internal_ptr_id[static_cast<void*>(obj)] = obj_ID;
             already_stored = false;
             return;*/
        }

        //---------------------------------------------------
        // INTERFACES - to be implemented by children classes
        //

        // for integral types:
        public abstract void outs(ChNameValue<bool> bVal);
        public abstract void outs(ChNameValue<int> bVal);
        public abstract void outs(ChNameValue<double> bVal);
        public abstract void outs(ChNameValue<float> bVal);
        public abstract void outs(ChNameValue<char> bVal);
        public abstract void outs(ChNameValue<uint> bVal);
        public abstract void outs(ChNameValue<string> bVal);
        public abstract void outs(ChNameValue<ulong> bVal);
        // public abstract void outs(ChNameValue<ulong ulong> bVal);
        public abstract void outs(ChNameValue<ChEnumMapperBase> bVal);

        // for custom C++ objects - see 'wrapping' trick below
        public abstract void outs(ref ChValue bVal, bool tracked, int obj_ID);

        // for pointed objects
        public abstract void out_ref(ref ChValue bVal, bool already_inserted, int obj_ID, int ext_ID);

        // for wrapping arrays and lists
        public abstract void out_array_pre(ref ChValue bVal, int msize);
        public abstract void out_array_between(ref ChValue bVal, int msize);
        public abstract void out_array_end(ref ChValue bVal, int msize);


        // trick to apply 'virtual out..' on remaining C++ object, that has a function "ArchiveOUT" 
        public void outs<T>(ChNameValue<T> bVal)
        {
            bool tracked = false;
            int obj_ID = 0;
            /* if (bVal.flags() & NVP_TRACK_OBJECT)
             {
                 bool already_stored = false;
                 T mptr = bVal.value();
                 PutPointer(mptr, ref already_stored, ref obj_ID);
                 if (already_stored)
                 { throw (ChExceptionArchive("Cannot serialize tracked object '" + string(bVal.name()) + "' by value, AFTER already serialized by pointer.")); }
                 tracked = true;
             }
             ChValueSpecific<T> specVal(bVal.value(), bVal.name(), bVal.flags());
             this.outs(
                 specVal,
                 tracked,
                 obj_ID);*/
        }



        /// Operator to allow easy serialization as   myarchive << mydata;

        public ChArchiveOut shiftleft<T>(ChNameValue<T> bVal) where T : unmanaged
        {
            this.outs(bVal);
            return (this);
        }
    }

    ///
    /// This is a base class for serializing into archives
    ///

    public abstract class ChArchiveIn : ChArchive
    {
    }
}
