using System;
using UnityEngine.Scripting;
using System.Runtime.InteropServices;
using UnityEngine.Bindings;
using scm = System.ComponentModel;
using uei = UnityEngine.Internal;
using System.Globalization;
using System.Runtime.CompilerServices;


namespace chrono
{
    /// Definition of general purpose 3d vector variables, such as points in 3D.
    /// This class implements the vectorial algebra in 3D (Gibbs products).
    /// ChVector is templated by precision, with default 'double'.
    [StructLayout(LayoutKind.Sequential)]
    public partial struct ChVector
    {

        private double[] data;// = new double[3];

        public static ChVector VNULL = new ChVector(0.0, 0.0, 0.0);
        public static ChVector VECT_X = new ChVector(1.0, 0.0, 0.0);
        public static ChVector VECT_Y = new ChVector(0.0, 1.0, 0.0);
        public static ChVector VECT_Z = new ChVector(0.0, 0.0, 1.0);

      /*  public ChVector(0, 0, 0) 
        {
            data = new double[3];
            data[0] = 0;
            data[1] = 0;
            data[2] = 0;
        }*/

        public ChVector(double a)
        {
            data = new double[3];
            data[0] = a;
            data[1] = a;
            data[2] = a;
        }

        public ChVector(double x, double y, double z)
        {
            data = new double[3];
            data[0] = x;
            data[1] = y;
            data[2] = z;
        }

        /// Copy constructor with type change.
        public ChVector(ChVector other)
        {
            data = new double[3];
            data[0] = other.data[0];
            data[1] = other.data[1];
            data[2] = other.data[2];
        }

        /// Access to components
        public double x
        {
            get { return data[0]; }
            set { data[0] = value; }
        }
        public double y { 
            get { return data[1]; }
            set { data[1] = value; }
        }
        public double z
        {
            get { return data[2]; }
            set { data[2] = value; }
        }

        // SET FUNCTIONS

        /// Set the three values of the vector at once.
        public void Set(double x, double y, double z) {
            data[0] = x;
            data[1] = y;
            data[2] = z;
        }

        /// Set the vector as a copy of another vector.
        public void Set(ChVector v) {
            data[0] = v.data[0];
            data[1] = v.data[1];
            data[2] = v.data[2];
        }

        /// Set all the vector components ts to the same scalar.
        public void Set(double s) {
            data[0] = s;
            data[1] = s;
            data[2] = s;
        }

        /// Set the vector to the null vector.
        public void SetNull() {
            data[0] = 0;
            data[1] = 0;
            data[2] = 0;
        }

        /// Return true if this vector is the null vector.
        public bool IsNull() {
            return data[0] == 0 && data[1] == 0 && data[2] == 0;
        }

        /// Return true if this vector is equal to another vector.
        public bool Equals(ChVector other) {
            return (other.data[0] == data[0]) && (other.data[1] == data[1]) && (other.data[2] == data[2]);
        }

        /// Return true if this vector is equal to another vector, within a tolerance 'tol'.
        public bool Equals(ChVector other, double tol) {
            return (Math.Abs(other.data[0] - data[0]) < tol) && (Math.Abs(other.data[1] - data[1]) < tol) &&
                   (Math.Abs(other.data[2] - data[2]) < tol);
        }

        // VECTOR NORMS

        /// Compute the euclidean norm of the vector, that is its length or magnitude.
        public double Length() {
            return Math.Sqrt(Length2());
        }

        /// Compute the squared euclidean norm of the vector.
        public double Length2() {
            return this.Dot(this);
        }

        /// Compute the infinity norm of the vector, that is the maximum absolute value of one of its elements.
        public double LengthInf() {
            return Math.Max(Math.Max(Math.Abs(data[0]), Math.Abs(data[1])), Math.Abs(data[2]));
        }


        // OPERATORS OVERLOADING
        //
        // Note: c++ automatically creates temporary objects to store intermediate
        // results in long formulas, such as a= b*c*d, so the usage of operators
        // may give slower results than a wise (less readable however) usage of
        // Dot(), Cross() etc.. Also pay attention to C++ operator precedence rules!

        /*public static ChVector operator -(ChVector other)
        {
            double aX = other.data[0];
            double aY = other.data[1];
            double aZ = other.data[2];

            aX = -aX;
            aY = -aY;
            aZ = -aZ;
            return new ChVector(aX, aY, aZ);

        }*/

        // -----------------------------------------------------------------------------
        // Arithmetic operations

        public static ChVector operator +(ChVector a, ChVector other)
        {
             //ChVector<Real> v = new ChVector<Real>();
            /** a.data[0] = a.data[0] + other.data[0];
             a.data[1] = a.data[1] + other.data[1];
             a.data[2] = a.data[2] + other.data[2];
             return a;*/
             return new ChVector(a.data[0] + other.data[0], a.data[1] + other.data[1], a.data[2] + other.data[2]);
        }

        public static ChVector operator -(ChVector a, ChVector other)
        {
          /*  ChVector<Real> v = new ChVector<Real>();
            a.data[0] = a.data[0] - other.data[0];
            a.data[1] = a.data[1] - other.data[1];
            a.data[2] = a.data[2] - other.data[2];
            return a;*/
            return new ChVector(a.data[0] - other.data[0], a.data[1] - other.data[1], a.data[2] - other.data[2]);
        }



        public static ChVector operator *(ChVector a, double s)
        {
           // ChVector<Real> v = new ChVector<Real>();
          /*  a.data[0] = a.data[0] * s;
            a.data[1] = a.data[1] * s;
            a.data[2] = a.data[2] * s;
            return a;*/
            return new ChVector(a.data[0] * s, a.data[1] * s, a.data[2] * s);
        }

        public static ChVector operator *(double a, ChVector s)
        {
            // ChVector<Real> v = new ChVector<Real>();
            /*  a.data[0] = a.data[0] * s;
              a.data[1] = a.data[1] * s;
              a.data[2] = a.data[2] * s;
              return a;*/
            return new ChVector(a * s.data[0], a * s.data[1], a * s.data[2]);
        }

        public static ChVector operator /(ChVector a, double s)
        {
            //ChVector<Real> v = new ChVector<Real>();
          /*  a.data[0] = a.data[0] / s;
            a.data[1] = a.data[1] / s;
            a.data[2] = a.data[2] / s;
            return a;*/
            return new ChVector(a.data[0] / s, a.data[1] / s, a.data[2] / s);
        }

        // -----------------------------------------------------------------------------
        // Vector operations

        public static ChVector operator %(ChVector a, ChVector other)
        {
            ChVector v = new ChVector(0, 0, 0);
            v.Cross(a, other);
            return v;
        }

        public static ChVector operator -(ChVector a)
        {
            return new ChVector(-a.data[0], -a.data[1], -a.data[2]);
        }

        public static bool operator ==(ChVector a, ChVector other)
        {
            return other.data[0] == a.data[0] && other.data[1] == a.data[1] && other.data[2] == a.data[2];
        }

        public static bool operator !=(ChVector a, ChVector other)
        {
            return !(a == other);
        }

        // -----------------------------------------------------------------------------
        // STATIC VECTOR MATH OPERATIONS
        //
        // These functions are here for people which prefer to use static
        // functions instead of ChVector class' member functions.
        // NOTE: sometimes a wise adoption of the following functions may
        // give faster results rather than using overloaded operators +/-/* in
        // the vector class.
        // For best readability of our code, it is suggested not to use
        // these functions - use the member functions or operators of
        // the ChVector class instead.

        public static double Vdot(ChVector va, ChVector vb)
        {
            return ((va.x * vb.x) + (va.y * vb.y) + (va.z * vb.z));
        }

    /* public static ChVector operator -(double a, ChVector b)
     {
         double bX = b.data[0];
         double bY = b.data[1];
         double bZ = b.data[2];

         double resultX = (a - bX);
         double resultY = (a - bY);
         double resultZ = (a - bZ);

         return new ChVector(resultX, resultY, resultZ);
     }
     public static ChVector operator *(double a, ChVector b)
     {
         /*double bX = b.GetX();
         double bY = b.GetY();
         double bZ = b.GetZ();*/
    /* double bX = b.data[0];
     double bY = b.data[1];
     double bZ = b.data[2];

     double resultX = (a * bX);
     double resultY = (a * bY);
     double resultZ = (a * bZ);

     return new ChVector(resultX, resultY, resultZ);
 }
 public static ChVector operator *(ChVector a, double b)
 {
     /*double bX = b.GetX();
     double bY = b.GetY();
     double bZ = b.GetZ();*/
    /*  double bX = a.data[0];
      double bY = a.data[1];
      double bZ = a.data[2];

      double resultX = (b * bX);
      double resultY = (b * bY);
      double resultZ = (b * bZ);

      return new ChVector(resultX, resultY, resultZ);
  }
  public static ChVector operator /(ChVector a, double b)
  {
      double bX = a.data[0];
      double bY = a.data[1];
      double bZ = a.data[2];

      double resultX = (bX / b);
      double resultY = (bY / b);
      double resultZ = (bZ / b);

      return new ChVector(resultX, resultY, resultZ);
  }

/*  public static ChVector operator *(ChMatrix33 A, ChVector v)
  {
      return new ChVector(A.GetElementMultiply(0, 0, v.data[0]) + A.GetElementMultiply(0, 1, v.data[2]) + A.GetElementMultiply(0, 2, v.data[2]),
                A.GetElementMultiply(1, 0, v.data[0]) + A.GetElementMultiply(1, 1, v.data[1]) + A.GetElementMultiply(1, 2, v.data[2]),
                A.GetElementMultiply(2, 0, v.data[0]) + A.GetElementMultiply(2, 1, v.data[1]) + A.GetElementMultiply(2, 2, v.data[2]));
  }*/

    /* public static double Vdot(ChVector va, ChVector vb)
     {
         /*double aX = va.GetX();
         double aY = va.GetY();
         double aZ = va.GetZ();
         double bX = vb.GetX();
         double bY = vb.GetY();
         double bZ = vb.GetZ();*/
    /*   double aX = va.data[0];
       double aY = va.data[1];
       double aZ = va.data[2];
       double bX = vb.data[0];
       double bY = vb.data[1];
       double bZ = vb.data[2];

       return (double)((aX * bX) + (aY * bY) + (aZ * bZ));
   }

   /// Access to components
   public double GetX()
   {
       data[0] = ChVector_GetX(m_ChVector);
       return data[0];
   }
   public double GetY()
   {
       data[1] = ChVector_GetY(m_ChVector);
       return data[1];
   }
   public double GetZ()
   {
       data[2] = ChVector_GetZ(m_ChVector);
       return data[2];
   }
   public void SetX()
   {
      // data[0] = x;
       ChVector_SetX(m_ChVector, data[0]);
   }
   public void SetY()
   {
       //data[1] = y;
       ChVector_SetY(m_ChVector, data[1]);
   }
   public void SetZ()
   {
       //data[2] = z;
       ChVector_SetZ(m_ChVector, data[2]);
   }

   /*public void SetX(double x)
   {
       data[0] = x;
       ChVector_SetX(m_ChVector, data[0]);
   }
   public void SetY(double y)
   {
       data[1] = y;
       ChVector_SetY(m_ChVector, data[1]);
   }
   public void SetZ(double z)
   {
       data[2] = z;
       ChVector_SetZ(m_ChVector, data[2]);
   }   */

    /// Return the dot product with another vector: result = this ^ B
    public double Dot(ChVector B) {
            return (data[0] * B.data[0]) + (data[1] * B.data[1]) + (data[2] * B.data[2]);
        }

        /// Return a normalized copy of this vector, with euclidean length = 1.
        /// Not to be confused with Normalize() which normalizes in place.
        public ChVector GetNormalized()
        {
            ChVector v = new ChVector(this);
            v.Normalize();
            return v;
        }

        /// Normalize this vector in place, so that its euclidean length is 1.
        /// Return false if the original vector had zero length (in which case the vector
        /// is set to [1,0,0]) and return true otherwise.
        public bool Normalize()
        {
            double length = this.Length();
            double min = 2.2250738585072014e-308;
            if (length < min)
            {
                data[0] = 1;
                data[1] = 0;
                data[2] = 0;
                return false;
            }
            this.Scale(1 / length);
            return true;
        }

        /* public static ChVector Vcross(ChVector va, ChVector vb)
         {
             ChVector result = new ChVector(0, 0, 0);
             result.data[0] = (va.data[1] * vb.data[2]) - (va.data[2] * vb.data[1]);
             result.data[1] = (va.data[2] * vb.data[0]) - (va.data[0] * vb.data[2]);
             result.data[2] = (va.data[0] * vb.data[1]) - (va.data[1] * vb.data[0]);
             return result;
         }*/

        // Gets the zenith angle of a unit vector respect to YZ plane  ***OBSOLETE
        public static double VangleYZplaneNorm(ChVector va)
        {
            return Math.Acos(Vdot(va, new ChVector(1, 0, 0)));
        }
        public static ChVector Vcross(ChVector va, ChVector vb)
        {
            ChVector result = new ChVector(0, 0, 0);
            result.x = (va.y * vb.z) - (va.z * vb.y);
            result.y = (va.z * vb.x) - (va.x * vb.z);
            result.z = (va.x * vb.y) - (va.y * vb.x);
            return result;
        }

        // Gets the angle of the projection on the YZ plane respect to
        // the Y vector, as spinning about X.
        public static double VangleRX(ChVector va)
        {
            ChVector vproj = new ChVector(0, 0, 0);
            vproj.x = 0;
            vproj.y = va.y;
            vproj.z = va.z;
            vproj = Vnorm(vproj);
            if (vproj.x == 1)
                return 0;
            return Math.Acos(vproj.y);
        }

    /// Scale this vector by a scalar: this *= s
    public void Scale(double s){
            data[0] *= s;
            data[1] *= s;
            data[2] *= s;
        }

        public void Cross(ChVector A, ChVector B) {
            data[0] = (A.data[1] * B.data[2]) - (A.data[2] * B.data[1]);
            data[1] = (A.data[2] * B.data[0]) - (A.data[0] * B.data[2]);
            data[2] = (A.data[0] * B.data[1]) - (A.data[1] * B.data[0]);
        }

        public ChVector Cross(ChVector other)
        {
            ChVector v = new ChVector(0, 0, 0);
            v.Cross(this, other);
            return v;
        }

        public static bool Vequal(ChVector va, ChVector vb)
        {
            return (va == vb);
        }

        public static bool Vnotnull(ChVector va)
        {
            return (va.x != 0 || va.y != 0 || va.z != 0);
        }

        public static ChVector Vnorm(ChVector va)
        {
            ChVector result = new ChVector(va);
            result.Normalize();
            return result;
        }
        public static double Vlength(ChVector va)
        {
            return va.Length();
        }
        public static ChVector Vmul(ChVector va, double fact)
        {
            ChVector result = new ChVector(0, 0, 0);
            result.x = va.x * fact;
            result.y = va.y * fact;
            result.z = va.z * fact;
            return result;
        }

        public static ChVector Vadd(ChVector va, ChVector vb)
        {
            ChVector result = new ChVector(0, 0, 0);
            result.x = va.x + vb.x;
            result.y = va.y + vb.y;
            result.z = va.z + vb.z;
            return result;
        }
    public static ChVector Vsub(ChVector va, ChVector vb)
        {
            ChVector result = new ChVector(0, 0, 0);
            result.x = va.x - vb.x;
            result.y = va.y - vb.y;
            result.z = va.z - vb.z;
            return result;
        }

        // From non-normalized x direction, to versors DxDyDz.
        // Vsingular sets the normal to the plane on which Dz must lie.
        public static void XdirToDxDyDz(ChVector Vxdir,
                                    ChVector Vsingular,
                                    ref ChVector Vx,
                                    ref ChVector Vy,
                                    ref ChVector Vz) {

            ChVector mVnull = new ChVector(0, 0, 0);
            double zlen;            

            if (Vequal(Vxdir, mVnull))
                Vx = new ChVector(1, 0, 0);
            else
                Vx = Vnorm(Vxdir);
            
            Vz = Vcross(Vx, Vsingular);
            zlen = Vlength(Vz);

            // If close to singularity, change reference vector
            if (zlen < 0.0001) {
                ChVector mVsingular = new ChVector(0, 0, 0);
                if (Math.Abs(Vsingular.x) < 0.9)
                    mVsingular = new ChVector(0, 0, 1);
                if (Math.Abs(Vsingular.y) < 0.9)
                    mVsingular = new ChVector(0, 1, 0);
                if (Math.Abs(Vsingular.z) < 0.9)
                    mVsingular = new ChVector(1, 0, 0);
                Vz = Vcross(Vx, mVsingular);
                zlen = Vlength(Vz);  // now should be nonzero length.
            }

            // normalize Vz
            Vz = Vmul(Vz, 1.0 / zlen);
            // compute Vy
            Vy = Vcross(Vz, Vx);
        }

        public void DirToDxDyDz(ref ChVector Vx,
                                        ref ChVector Vy,
                                        ref ChVector Vz,
                                        ChVector Vsingular)
        {
            // set Vx.
            if (this.IsNull())
                Vx = new ChVector(1, 0, 0);
            else
                Vx = this.GetNormalized();

            Vz.Cross(Vx, Vsingular);
            double zlen = Vz.Length();

            // if near singularity, change the singularity reference vector.
            if (zlen < 0.0001)
            {
                ChVector mVsingular = new ChVector(0, 0, 0);

                if (Math.Abs(Vsingular.data[0]) < 0.9)
                    mVsingular = new ChVector(1, 0, 0);
                else if (Math.Abs(Vsingular.data[1]) < 0.9)
                    mVsingular = new ChVector(0, 1, 0);
                else if (Math.Abs(Vsingular.data[2]) < 0.9)
                    mVsingular = new ChVector(0, 0, 1);

                Vz.Cross(Vx, mVsingular);
                zlen = Vz.Length();  // now should be nonzero length.
            }

            // normalize Vz.
            Vz.Scale(1 / zlen);

            // compute Vy.
            Vy.Cross(Vz, Vx);
        }

        public int GetMaxComponent()
        {
            int idx = 0;
            double max = Math.Abs(data[0]);
            if (Math.Abs(data[1]) > max)
            {
                idx = 1;
                max = data[1];
            }
            if (Math.Abs(data[2]) > max)
            {
                idx = 2;
                max = data[2];
            }
            return idx;
        }

        public ChVector GetOrthogonalVector()
        {
            int idx1 = this.GetMaxComponent();
            int idx2 = (idx1 + 1) % 3;  // cycle to the next component
            int idx3 = (idx2 + 1) % 3;  // cycle to the next component

            // Construct v2 by rotating in the plane containing the maximum component
            ChVector v2 = new ChVector(-data[idx2], data[idx1], data[idx3]);

            // Construct the normal vector
            ChVector ortho = Cross(v2);
            ortho.Normalize();
            return ortho;
        }


    };



    // -----------------------------------------------------------------------------

    /// Shortcut for faster use of typical double-precision vectors.
    /// <pre>
    /// Instead of writing
    ///    ChVector<double> foo;
    /// or
    ///    ChVector<> foo;
    /// you can use the shorter version
    ///    Vector foo;
    /// </pre>
  //  public class Vector : ChVector { }

    /// Shortcut for faster use of typical single-precision vectors.
    /// <pre>
    /// Instead of writing
    ///    ChVector<float> foo;
    /// you can use the shorter version
    ///    Vector foo;
    /// </pre>
    //public class VectorF : ChVector { }
}