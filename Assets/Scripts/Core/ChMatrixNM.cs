using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Reflection;
using System.Runtime.InteropServices;
using System.Linq;

namespace chrono
{
  
    ///
    /// ChMatrixNM
    ///
    /// Specialized matrix class having a pre-allocated NxM buffer of elements on stack.
    ///  This means that elements are put the stack when the matrix is created, and this
    /// can be more efficient than heap allocation (but, please, do not use too large N or M
    /// sizes, this is meant to work up to 10x10, for example; prefer ChMatrixDyamic for larger).
    ///  The NxM size must be known 'statically', at compile-time; however later at run-time
    /// this matrix can be resized anyway (but for larger size than NxM, it falls back to
    /// heap allocation). Note that if resizing is often required, it may be better
    /// to create a ChMatrixDyamic instead, from the beginning.
    public class ChMatrixNM<A, B> : ChMatrix       
       // where Real : unmanaged
        where A : IntInterface.IBase, new()
        where B : IntInterface.IBase, new()
    {
        // private dynamic preall_rows = typeof(A);
        // private dynamic preall_columns = typeof(B);
        protected double[] buffer;// = new double[0];

        //
        // CONSTRUCTORS
        //

        public static void GetPropertyValues(object obj)
        {
           // Type t = obj.GetType();
           // Console.WriteLine("Type is: {0}", t.Name);
          //  PropertyInfo[] props = t.GetProperties();
            //foreach (var prop in props)
              //  Debug.Log("result " + prop.GetValue(obj));

           // Console.WriteLine("Properties (N = {0}):",
            //                  props.Length);
           /* foreach (var prop in props)
                if (prop.GetIndexParameters().Length == 0)
                    Debug.LogErrorFormat("   {0} ({1}): {2}", prop.Name,
                                      prop.PropertyType.Name,
                                      prop.GetValue(obj));
                else
                    Debug.LogErrorFormat("   {0} ({1}): <Indexed>", prop.Name,
                                      prop.PropertyType.Name);*/
        }


        /// The default constructor builds a NxN matrix
        public ChMatrixNM()     
        {
            // Type t = preall_rows.GetType();
            //PropertyInfo prop = preall_rows.GetType.GetProperty("Masta");
            //var val = prop.GetValue(preall_rows, null).Value;
            //Type t = preall_rows.GetType();
            //  var a = new A().Masta;
            // var a = new A().GetType().GetProperty("Masta");
          //  var a = new A().GetType().IsGenericType;
            //var b = new B().GetType().IsGenericType;
           // var b = new B().GetType().GetGenericTypeDefinition();

           // var a2 = preall_rows.GetFields(BindingFlags.Instance | BindingFlags.Public);
           // var b2 = preall_columns.GetFields(BindingFlags.Instance | BindingFlags.Public);
            
            // var val = a.GetType().GetProperty("Masta");

            // var val = prop.GetValue(preall_rows);

            // PropertyInfo[] props3 = GetPublicProperties(preall_rows);
            //  FieldInfo[] field = preall_rows.GetFields(BindingFlags.Instance | BindingFlags.Public);
            // var vals = field.GetValue(preall_rows);
            /* foreach (PropertyInfo prop in prop)
             {
                 int num;
                 var val = (dynamic)prop.GetValue(preall_rows);
                 bool isNum = int.TryParse(val.ToString(), out num);
                 if (isNum)
                 {
                     this.rows = val;
                 }
                 else
                 {
                     FieldInfo[] fields = val.GetFields(BindingFlags.Static | BindingFlags.Public);
                     foreach (var prop2 in fields)
                     {
                         var val2 = prop2.GetValue(val);
                         this.rows = val2;
                     }
                 }
             }*/
            /* FieldInfo[] props2 = preall_columns.GetFields(BindingFlags.Static | BindingFlags.Public);
             foreach (var prop in props2)
             {
                 int num;
                 var val = prop.GetValue(preall_columns);
                 bool isNum = int.TryParse(val.ToString(), out num);
                 if (isNum)
                 {
                     this.columns = val;
                 }
                 else
                 {
                     FieldInfo[] fields = val.GetFields(BindingFlags.Static | BindingFlags.Public);
                     foreach (var prop2 in fields)
                     {
                         var val2 = prop2.GetValue(val);
                         bool isNum2 = int.TryParse(val2.ToString(), out num);
                         if (isNum2)
                         {
                             this.columns = val2;
                         }
                         else
                         {
                             FieldInfo[] fields2 = val2.GetFields(BindingFlags.Static | BindingFlags.Public);
                             foreach (var prop3 in fields)
                             {
                                 var val3 = prop3.GetValue(val);
                                 this.columns = val3;
                             }
                         }
                     }
                 }
             }*/

            //FieldInfo[] props = new A().Value.GetFields(BindingFlags.Static | BindingFlags.Public);
            /*  int num;
              dynamic a = new A().Value;
              bool isNum = int.TryParse(a.ToString(), out num);
              if (isNum)
              {
                  this.rows = a;
              }
              else
              {
                  FieldInfo[] props = new A().Value.GetFields(BindingFlags.Static | BindingFlags.Public);
                  foreach (var prop in props)
                  {
                      var val = prop.GetValue(preall_rows);
                      this.rows = val;
                  }
              }
              int num2;
              dynamic b = new B().Value;
              bool isNum2 = int.TryParse(b.ToString(), out num2);
              if (isNum2)
              {
                  this.columns = b;
              }
              else
              {
                  //FieldInfo[] props = new B().Value.GetFields(BindingFlags.Instance | BindingFlags.Public);
                  PropertyInfo props = preall_columns.GetProperty("Value", BindingFlags.Instance | BindingFlags.Public);
                  var val = props.GetValue(preall_columns);
                 /* foreach (var prop in props)
                   {
                       var val = prop.GetValue(preall_columns);
                       this.columns = val;
                   }*/
            //}
            //if(props )

            /*  foreach (var prop in props)
              {
                  var val = prop.GetValue(preall_rows);
                  this.rows = val;
              }

              FieldInfo[] props2 = new B().Value.GetFields(BindingFlags.Static | BindingFlags.Public);
              foreach (var prop in props2)
              {
                  var val2 = prop.GetValue(preall_rows);
                  this.columns = val2;
              }   */
            this.rows = new A().Masta;
            this.columns = new B().Masta;
            buffer = new double[this.rows * this.columns + 3];
            this.address = buffer;
           /* this.address = (double*)Marshal.AllocHGlobal(buffer.Length * sizeof(double));
            for (int i = 0; i < this.buffer.Length; i++)
            {
                this.address[i] = this.buffer[i];
            }*/
            //for (int i = 0; i < buffer.Length; ++i)
            //  this.address[i] = buffer[i];
            SetZero(this.rows * this.columns);
        }

      

        /// Copy constructor
        public ChMatrixNM(ChMatrixNM<A, B> msource)
        {
            //int a = 0;
            // this.address = (Real*)&a;
            /*            FieldInfo[] props = preall_rows.GetFields(BindingFlags.Static | BindingFlags.Public);
                        foreach (var prop in props)
                        {
                            //var value = prop.GetValue(preall_rows);
                            //this.rows = (dynamic)value;// prop.GetValue(preall_rows); //preall_rows; 
                            int num;
                            var val = prop.GetValue(preall_rows);
                            bool isNum = int.TryParse(val.ToString(), out num);
                            if (isNum)
                            {
                                this.rows = val;
                            }
                            else
                            {
                                FieldInfo[] fields = val.GetFields(BindingFlags.Static | BindingFlags.Public);
                                foreach (var prop2 in fields)
                                {
                                    var val2 = prop2.GetValue(val);
                                    this.rows = val2;
                                }
                            }
                        }
                        FieldInfo[] props2 = preall_columns.GetFields(BindingFlags.Static | BindingFlags.Public);
                        foreach (var prop in props2)
                        {
                            //var value = prop.GetValue(preall_columns);
                            //this.columns = (int)value;// prop.GetValue(preall_columns);
                            int num;
                            var val = prop.GetValue(preall_columns);
                            bool isNum = int.TryParse(val.ToString(), out num);
                            if (isNum)
                            {
                                this.columns = val;
                            }
                            else
                            {
                                FieldInfo[] fields = val.GetFields(BindingFlags.Static | BindingFlags.Public);
                                foreach (var prop2 in fields)
                                {
                                    var val2 = prop2.GetValue(val);
                                    this.columns = val2;
                                }
                            }
                        }*/
            this.rows = new A().Masta;
            this.columns = new B().Masta;
            buffer = new double[this.rows * this.columns + 3];
            // for (int i = 0; i < buffer.Length; ++i)
            this.address = buffer;
           /* this.address = (double*)Marshal.AllocHGlobal(buffer.Length * sizeof(double));
             for (int i = 0; i < this.buffer.Length; i++)
             {
                 this.address[i] = this.buffer[i];
             }*/
            //this.address = buffer;
            ElementsCopy(this.address, msource.address, this.rows * this.columns);
        }



        public void SetZero(int els)
        {
            for (int i = 0; i < els; ++i)
            {
                this.address[i] = 0;
            }
        }

        public void ElementsCopy(double[] to, double[] from, int els)
        {
            for (int i = 0; i < els; ++i)
                to[i] = from[i];
        }

        /// Copy constructor from all types of base matrices (only with same size)
        public ChMatrixNM(ChMatrix msource)
        {
           // Debug.Assert(msource.GetColumns() == preall_columns && msource.GetRows() == preall_rows);
           // this.rows = preall_rows;
           // this.columns = preall_columns;
            this.address = buffer;
           /* this.address = (double*)Marshal.AllocHGlobal(buffer.Length * sizeof(double));
            for (int i = 0; i < this.buffer.Length; i++)
            {
                this.address[i] = this.buffer[i];
            }*/
            /*   for (int i = 0; i < buffer.Length; ++i)
                   this.address[i] = buffer[i];*/
            //  ElementsCopy(this.address, msource.GetAddress(), preall_rows * preall_columns);
            ElementsCopy(this.address, msource.address, this.rows * this.columns);
        }


        //
        // FUNCTIONS
        //

        /// Resize for this matrix is NOT SUPPORTED ! DO NOTHING!
        public override void Resize(int nrows, int ncols) { /*Debug.Assert((nrows == this.rows) && (ncols == this.columns));*/ }

    }
}
