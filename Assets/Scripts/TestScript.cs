using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Diagnostics;
using System.Linq;
using System.Runtime.InteropServices;
using System.Reflection;
using chrono;




public class MyMatrix
{
    double address;

    public void Add()
    {
        for(int i = 0; i < 999999; i++)
        {
            address += Math.Sqrt(10);
        }
    }
}

public class MyMatrix<T>
{
    T address;

    public void Add()
    {
        for (int i = 0; i < 999999; i++)
        {
            address += (dynamic)Math.Sqrt(10);
        }
    }
}

public class Base<T1, T2> {
    public int a = 1;

    public Base()
    {

    }
}
public class Derived<T1, T2> : Base<T1, T2> {
    public int b = 2;
    public Derived() : base()
    {

    }
}
public class ManagedListClass
{
    public List<double> address = new List<double>();
    public int rows = 1;
    public int columns = 1;

    public void Element(int index, double elem)
    {
        address.Add(elem);
    }
}

public class ManagedArrayClass
{
    public Array address = Array.CreateInstance(typeof(double), 90000000);
    public int rows = 1;
    public int columns = 1;

    public void Element(int index, double elem)
    {
        address.SetValue(elem, index);
    }
}

public class ManagedClass
{
    public double[] address = new double[90000000];
    public int rows = 1;
    public int columns = 1;

    public void Element(int index, double elem)
    {
        address[index] = elem;
    }
}

public unsafe class UnManagedClass
{
    public double* address = (double*)Marshal.AllocHGlobal(90000000 * sizeof(double));//= new double[90000000];

   /* public UnManagedClass()
    {
        address = (double*)Marshal.AllocHGlobal(90000000 * sizeof(double));
    }*/
    public void Element(int index, double elem)
    {
       *(address + index) = elem;
        
    }
}

public class TestCreatingObjects // create loads of flapps
{

}

public class MyClass
{
    double[] address;

    public void Add(double a, double b, double c) {
        double[] address = new double[3];
        address[0] = a;
        address[1] = b;
        address[2] = c;
    }

}


public class TestScript : MonoBehaviour
{
    public double step;

    // Start is called before the first frame update
    void Start()
    {
      

        /* ChVector vtraslA = new ChVector(5, 6, 7);         // origin of local frame w.r.t. global frame
         ChQuaternion qrotA = new ChQuaternion(1, 3, 4, 5);    // rotation of local frame w.r.t. global frame
         qrotA.Normalize();                   // quaternion must be normalized to represent a rotation
         ChMatrix33<double> mrotA = new ChMatrix33<double>(qrotA);           // corresponding rotation matrix
         ChCoordsys<double> csysA = new ChCoordsys<double>(vtraslA, qrotA);  // corresponding coordinate system (translation + rotation)
         ChFrame<double> frameA = new ChFrame<double>(vtraslA, qrotA);    // corresponding frame

         ChVector vG_ref = new ChVector(7.5882352941, 9.0000000000, 10.6470588235);  // reference
         ChVector vL = new ChVector(2, 3, 4);                                        // point in local frame
         ChVector vG;                                                 // point in global frame (computed)

         vG = vtraslA + mrotA * vL;  // like:  v2 = t + [A]*v1*/


        /* ManagedClass managed = new ManagedClass();

         Stopwatch sw = new Stopwatch();
         sw = Stopwatch.StartNew();
         for (int i = 0; i < 90000000; i++)
         {
             managed.Element(i, 256);
         }
         UnityEngine.Debug.Log("time " + sw.ElapsedMilliseconds);
         sw.Stop();*/

        /* UnManagedClass unmanaged = new UnManagedClass();

         Stopwatch sw = new Stopwatch();
         sw = Stopwatch.StartNew();
         for (int i = 0; i < 90000000; i++)
         {
             unmanaged.Element(i, 256);
         }
         UnityEngine.Debug.Log("time " + sw.ElapsedMilliseconds);
         sw.Stop();*/

        /*  ManagedListClass managed = new ManagedListClass();

          Stopwatch sw = new Stopwatch();
          sw = Stopwatch.StartNew();
          for (int i = 0; i < 90000000; i++)
          {
              managed.Element(i, 256);
          }
          UnityEngine.Debug.Log("time " + sw.ElapsedMilliseconds);
          sw.Stop();*/

        /*ManagedArrayClass managed = new ManagedArrayClass();

        Stopwatch sw = new Stopwatch();
        sw = Stopwatch.StartNew();
        for (int i = 0; i < 90000000; i++)
        {
            managed.Element(i, 256);
        }
        UnityEngine.Debug.Log("time " + sw.ElapsedMilliseconds);
        sw.Stop();*/

        // }

        //testenum = list.LastOrDefault();
        /*  chrono.ChVectorDynamic<double> mat1 = new chrono.ChVectorDynamic<double>();
          chrono.ChVectorDynamic<double> mat2 = new chrono.ChVectorDynamic<double>();
          mat1[0] = 2.0f;
          mat2[0] = 1.5;
          mat1[0] += mat2[0];
          Debug.Log("result " + mat1[0]);*/
        //MeClass<chrono.IntInterface.Six> flapps = new MeClass<chrono.IntInterface.Six>();
        // Matrix<IntInterface.VariableTupleCarrier_1vars<IntInterface.Six>.nvars1> matrix = new Matrix<IntInterface.VariableTupleCarrier_1vars<IntInterface.Six>.nvars1>();
        // Matrix<IntInterface.Three> matrix = new Matrix<IntInterface.Three>();
        // ChMatrixNM<double, IntInterface.Three, IntInterface.ChVariableTupleCarrier_1vars<IntInterface.Three>.nvars1> Flappa = new ChMatrixNM<double, IntInterface.Three, IntInterface.ChVariableTupleCarrier_1vars<IntInterface.Three>.nvars1>();
        // ConstraintTuple_1vars<IntInterface.Three> constraint = new ConstraintTuple_1vars<IntInterface.Three>();
        //VariableTupleCarrier_1vars<IntInterface.Six>.type_constraint_tuple tuple_a = new VariableTupleCarrier_1vars<IntInterface.Six>.type_constraint_tuple();

        /* Planet jupiter = new Planet();
         Type t = jupiter.GetType();
         PropertyInfo prop = t.GetProperty("Distance");
         var val = prop.GetValue(jupiter);*/

    }

    public void FixedUpdate()
    {
        Time.fixedDeltaTime = (float)step;

        for (int i = 0; i < 900000; i++)
        {
            ChMatrixNM<IntInterface.Three, IntInterface.Four> body1Gl = new ChMatrixNM<IntInterface.Three, IntInterface.Four>();
            ChMatrixNM<IntInterface.Three, IntInterface.Four> body2Gl = new ChMatrixNM<IntInterface.Three, IntInterface.Four>();

            ChFrame<double>.SetMatrix_Gl(ref body1Gl, new ChQuaternion(0, 0, 0, 0));
            ChFrame<double>.SetMatrix_Gl(ref body2Gl, new ChQuaternion(0, 0, 0, 0));
        }
    }

    public static void GetPropertyValues(object obj)
    {
      /*  Type t = obj.GetType();
        Console.WriteLine("Type is: {0}", t.Name);
        PropertyInfo[] props = t.GetProperties();
        Console.WriteLine("Properties (N = {0}):",
                          props.Length);
        foreach (var prop in props)
            if (prop.GetIndexParameters().Length == 0)
                Debug.LogErrorFormat("   {0} ({1}): {2}", prop.Name,
                                  prop.PropertyType.Name,
                                  prop.GetValue(obj));
            else
                Debug.LogErrorFormat("   {0} ({1}): <Indexed>", prop.Name,
                                  prop.PropertyType.Name);*/
    }
}


