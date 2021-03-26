using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;


namespace chrono
{
    /// Definitions of various angle sets for conversions.
    public enum AngleSet
    {
        ANGLE_AXIS,
        EULERO,
        CARDANO,
        HPB,
        RXYZ,
        RODRIGUEZ,
        QUATERNION,
    }

    /// Class defining quaternion objects, that is four-dimensional numbers, also known as Euler parameters.
    /// Quaternions are very useful when used to represent rotations in 3d.
    [System.Serializable]
    public class ChQuaternion
    {


        /// Default constructor.
        /// Note that this constructs a null quaternion {0,0,0,0}, not a {1,0,0,0} unit quaternion.
        public ChQuaternion()
        {
            data[0] = 0;
            data[1] = 0;
            data[2] = 0;
            data[3] = 0;
        }

        /// Constructor from four scalars. The first is the real part, others are i,j,k imaginary parts
        public ChQuaternion(double e0, double e1, double e2, double e3)
        {
            data[0] = e0;
            data[1] = e1;
            data[2] = e2;
            data[3] = e3;
        }

        /// Constructor from real part, and vector with i,j,k imaginary part.
        public ChQuaternion(double s, ChVector v) {
            data[0] = s;
            data[1] = v.x;
            data[2] = v.y;
            data[3] = v.z;
        }

        /// Copy constructor
        public ChQuaternion(ChQuaternion other)
        {
            data[0] = other.data[0];
            data[1] = other.data[1];
            data[2] = other.data[2];
            data[3] = other.data[3];
        }

        // NOTE
        // The following * and *= operators had a different behaviour prior to 13/9/2014,
        // but we assume no one used * *= in that previous form (element-by-element product).
        // Now * operator will be used for classical quaternion product, as the old % operator.
        // This is to be more consistent with the * operator for ChFrames etc.

        // -----------------------------------------------------------------------------
        // Sign operators

        public static ChQuaternion operator +(ChQuaternion q, ChQuaternion s)
        {
           // dynamic a = s;
            return new ChQuaternion(q.data[0] + s.data[0], q.data[1] + s.data[1], q.data[2] + s.data[2], q.data[3] + s.data[3]);
        }

        public static ChQuaternion operator -(ChQuaternion q, ChQuaternion s)
        {
            //dynamic a = s;
            return new ChQuaternion(q.data[0] - s.data[0], q.data[1] - s.data[1], q.data[2] - s.data[2], q.data[3] - s.data[3]);
        }

        // -----------------------------------------------------------------------------
        // Arithmetic & quaternion operations

        public static ChQuaternion operator *(ChQuaternion a, double s)
        {
            return new ChQuaternion(a.data[0] * s, a.data[1] * s, a.data[2] * s, a.data[3] * s);
        }

        public static ChQuaternion operator /(ChQuaternion q, double s)
        {
           // dynamic a = s;
            return new ChQuaternion(q.data[0] / s, q.data[1] / s, q.data[2] / s, q.data[3] / s);

        }

        public static ChQuaternion operator *(ChQuaternion a, ChQuaternion other)
        {
            ChQuaternion q = new ChQuaternion();
            q.Cross(a, other);
            return q;

        }



        /// Operator for quaternion product: A%B means the typical quaternion product AxB.
        public static ChQuaternion operator %(ChQuaternion q, ChQuaternion other) // works fine
        {
            ChQuaternion a = new ChQuaternion();
            a.Cross(q, other);
            return a;

        }

        /// Access to components
        public double e0
        {
            get { return data[0]; }
            set { data[0] = value; }
        }
        public double e1
        {
            get { return data[1]; }
            set { data[1] = value; }
        }
        public double e2
        {
            get { return data[2]; }
            set { data[2] = value; }
        }
        public double e3
        {
            get { return data[3]; }
            set { data[3] = value; }
        }

        /// Set this quaternion to the sum of A and B: this = A + B.
        public void Add(ChQuaternion A, ChQuaternion B){
            data[0] = A.data[0] + B.data[0];
            data[1] = A.data[1] + B.data[1];
            data[2] = A.data[2] + B.data[2];
            data[3] = A.data[3] + B.data[3];
        }

        /// Conjugate this quaternion in place (its vectorial part changes sign).
        public void Conjugate()
        {
            Conjugate(this);
        }

        public void Conjugate(ChQuaternion A)
        {
           // dynamic a = A;
            data[0] = +A.data[0];
            data[1] = -A.data[1];
            data[2] = -A.data[2];
            data[3] = -A.data[3];
        }

        public ChQuaternion GetConjugate() {
           // dynamic d = data;
            return new ChQuaternion(data[0], -data[1], -data[2], -data[3]);
        }

        public ChQuaternion GetInverse()
        {
            ChQuaternion invq = this.GetConjugate();
           // dynamic l = this.Length();
            invq.Scale(1 / this.Length());
            return invq;
        }

        public static double Qlength(ChQuaternion q)
        {
            return (Math.Sqrt(Math.Pow(q.e0, 2) + Math.Pow(q.e1, 2) + Math.Pow(q.e2, 2) + Math.Pow(q.e3, 2)));
        }

        // -----------------------------------------------------------------------------
        // Functions
        public void Set(double e0, double e1, double e2, double e3)
        {
            data[0] = e0;
            data[1] = e1;
            data[2] = e2;
            data[3] = e3;
        }

        public void Set(ChQuaternion q)
        {
            data[0] = q.data[0];
            data[1] = q.data[1];
            data[2] = q.data[2];
            data[3] = q.data[3];
        }

        public void Set(double s)
        {
            data[0] = s;
            data[1] = s;
            data[2] = s;
            data[3] = s;
        }

        public ChVector GetVector()
        {
           // dynamic a = data;
            return new ChVector(data[1], data[2], data[3]);
        }

        public bool Equals(ChQuaternion other) {
           // dynamic a = other;
            return (other.data[0] == data[0]) && (other.data[1] == data[1]) && (other.data[2] == data[2]) &&
                   (other.data[3] == data[3]);
        }

        public bool Equals(ChQuaternion other, double tol)
        {
           // dynamic a = other;
            return (Math.Abs(other.data[0] - data[0]) < tol) && (Math.Abs(other.data[1] - data[1]) < tol) &&
                   (Math.Abs(other.data[2] - data[2]) < tol) && (Math.Abs(other.data[3] - data[3]) < tol);
        }


        public static ChQuaternion Qadd(ChQuaternion qa, ChQuaternion qb)
        {
            ChQuaternion result = new ChQuaternion();
            result.e0 = qa.e0 + qb.e0;
            result.e1 = qa.e1 + qb.e1;
            result.e2 = qa.e2 + qb.e2;
            result.e3 = qa.e3 + qb.e3;
            return result;
        }

    // QUATERNION NORMS

    /// Compute the euclidean norm of the quaternion, that is its length or magnitude.
    public double Length() {
            return Math.Sqrt(Length2());
        }

        /// Compute the squared euclidean norm of the quaternion.
        public double Length2() {
            return this.Dot(this);
        }

        /// Compute the infinity norm of the quaternion, that is the maximum absolute value of one of its elements.
        public double LengthInf() {
            double e0e1 = Math.Max(Math.Abs(data[0]), Math.Abs(data[1]));
            double e0e1e2 = Math.Max(e0e1, Math.Abs(data[2]));

            return Math.Max(e0e1e2, Math.Abs(data[3]));
        }

        public static ChQuaternion Qnorm(ChQuaternion q)
        {
            double invlength;
            invlength = 1 / (Qlength(q));
            return Qscale(q, invlength);
        }

        /// Normalize this quaternion in place, so that its euclidean length is 1.
        /// Return false if the original quaternion had zero length (in which case the quaternion
        /// is set to [1,0,0,0]) and return true otherwise.
        public bool Normalize() {
            double length = this.Length();
            double min = 2.2250738585072014e-308;
            if (length < min)
            {
                data[0] = 1;
                data[1] = 0;
                data[2] = 0;
                data[3] = 0;
                return false;
            }
            this.Scale(1 / length);
            return true;
        }

        /// Check if quaternion is not null.
        public static bool Qnotnull(ChQuaternion qa) {
            return (qa.e0 != 0) || (qa.e1 != 0) || (qa.e2 != 0) || (qa.e3 != 0);
        }

        public double Dot(ChQuaternion B)
        {
           // dynamic a = data;
            return (data[0] * B.data[0]) + (data[1] * B.data[1]) + (data[2] * B.data[2]) + (data[3] * B.data[3]);
        }
        public void Scale(double s)
        {
           // dynamic a = s;
            data[0] *= s;
            data[1] *= s;
            data[2] *= s;
            data[3] *= s;
        }

        public void Cross(ChQuaternion qa, ChQuaternion qb)
        {
           // dynamic q1 = qa;
            double w = qa.data[0] * qb.data[0] - qa.data[1] * qb.data[1] - qa.data[2] * qb.data[2] - qa.data[3] * qb.data[3];
            double x = qa.data[0] * qb.data[1] + qa.data[1] * qb.data[0] - qa.data[3] * qb.data[2] + qa.data[2] * qb.data[3];
            double y = qa.data[0] * qb.data[2] + qa.data[2] * qb.data[0] + qa.data[3] * qb.data[1] - qa.data[1] * qb.data[3];
            double z = qa.data[0] * qb.data[3] + qa.data[3] * qb.data[0] - qa.data[2] * qb.data[1] + qa.data[1] * qb.data[2];
            data[0] = w;
            data[1] = x;
            data[2] = y;
            data[3] = z;
        }

        /// Set the quaternion from an angle of rotation and an axis, defined in absolute coords.
        /// The axis is supposed to be fixed, i.e. it is constant during rotation!
        /// NOTE, axis must be normalized!
        /// If you need directly the rotation vector=axis * angle, use Q_from_Rotv().
        public static ChQuaternion Q_from_AngAxis2(double angle, ChVector axis)
        {
            ChQuaternion quat = new ChQuaternion();
            double halfang;
            double sinhalf;

            //dynamic a = angle;
            halfang = angle * 0.5;
            sinhalf = Math.Sin(halfang);

            quat.e0 = Math.Cos(halfang);
            quat.e1 = axis.x * sinhalf;
            quat.e2 = axis.y * sinhalf;
            quat.e3 = axis.z * sinhalf;
            return (quat);
        }

        /// Set the quaternion from an angle of rotation about X axis.
        public void Q_from_AngX(double angleX) { Q_from_AngAxis(angleX, new ChVector(1, 0, 0)); }

        /// Set the quaternion from an angle of rotation about Y axis.
        void Q_from_AngY(double angleY) { Q_from_AngAxis(angleY, new ChVector(0, 1, 0)); }

        /// Set the quaternion from an angle of rotation about Z axis.
        void Q_from_AngZ(double angleZ) { Q_from_AngAxis(angleZ, new ChVector(0, 0, 1)); }

        public void Q_from_AngAxis(double angle, ChVector axis)
        {
            double halfang = (angle / 2); 
            double sinhalf = Math.Sin(halfang);
            data[0] = Math.Cos(halfang);
            data[1] = axis.x * halfang;
            data[2] = axis.y * halfang;
            data[3] = axis.z * halfang;
        }



        public void Q_to_AngAxis(ref double a_angle, ref ChVector a_axis)
        {
            double sin_squared = data[1] * data[1] + data[2] * data[2] + data[3] * data[3];
            // For non-zero rotation
            if (sin_squared > 0)
            {
                double sin_theta = Math.Sqrt(sin_squared);
                a_angle = 2.0 * Math.Atan2(sin_theta, data[0]);
                double k = 1.0 / sin_theta;
                a_axis.x = data[1] * k;
                a_axis.y = data[2] * k;
                a_axis.z = data[3] * k;
                a_axis.Normalize();
            }
            else
            {
                // For almost zero rotation
                a_angle = 0.0;
                a_axis.x = 1;  // data[1] * 2.0;
                a_axis.y = 0;  // data[2] * 2.0;
                a_axis.z = 0;  // data[3] * 2.0;
            }
            // Ensure that angle is always in  [-PI...PI] range
            var PI = (ChMaths.CH_C_PI);
            if (a_angle > PI)
            {
                a_angle -= 2 * PI;
            }
            else if (a_angle < -PI)
            {
                a_angle += 2 * PI;
            }
        }

        public static void Q_to_AngAxis(ChQuaternion quat, ref double angle, ref ChVector axis)
        {
            if (Math.Abs(quat.e0) < 0.99999999)
            {
                double arg = Math.Acos(quat.e0);
                double invsine = 1 / Math.Sin(arg);
                ChVector vtemp = new ChVector();
                vtemp.x = invsine * quat.e1;
                vtemp.y = invsine * quat.e2;
                vtemp.z = invsine * quat.e3;
                angle = 2 * arg;
                axis = ChVector.Vnorm(vtemp);
            }
            else
            {
                axis.x = 1;
                axis.y = 0;
                axis.z = 0;
                angle = 0;
            }
        }

        public ChVector Rotate(ChVector A) {
            //dynamic a = data;
            double e0e0 = data[0] * data[0];
            double e1e1 = data[1] * data[1];
            double e2e2 = data[2] * data[2];
            double e3e3 = data[3] * data[3];
            double e0e1 = data[0] * data[1];
            double e0e2 = data[0] * data[2];
            double e0e3 = data[0] * data[3];
            double e1e2 = data[1] * data[2];
            double e1e3 = data[1] * data[3];
            double e2e3 = data[2] * data[3];

            return new ChVector(((e0e0 + e1e1) * 2 - 1) * A.x + ((e1e2 - e0e3) * 2) * A.y + ((e1e3 + e0e2) * 2) * A.z,
                                  ((e1e2 + e0e3) * 2) * A.x + ((e0e0 + e2e2) * 2 - 1) * A.y + ((e2e3 - e0e1) * 2) * A.z,
                                  ((e1e3 - e0e2) * 2) * A.x + ((e2e3 + e0e1) * 2) * A.y + ((e0e0 + e3e3) * 2 - 1) * A.z);
        }

        public ChVector RotateBack(ChVector A)
        {
            //dynamic a = data;
            double e0e0 = +data[0] * data[0];
            double e1e1 = +data[1] * data[1];
            double e2e2 = +data[2] * data[2];
            double e3e3 = +data[3] * data[3];
            double e0e1 = -data[0] * data[1];
            double e0e2 = -data[0] * data[2];
            double e0e3 = -data[0] * data[3];
            double e1e2 = +data[1] * data[2];
            double e1e3 = +data[1] * data[3];
            double e2e3 = +data[2] * data[3];

            return new ChVector(((e0e0 + e1e1) * 2 - 1) * A.x + ((e1e2 - e0e3) * 2) * A.y + ((e1e3 + e0e2) * 2) * A.z,
                                  ((e1e2 + e0e3) * 2) * A.x + ((e0e0 + e2e2) * 2 - 1) * A.y + ((e2e3 - e0e1) * 2) * A.z,
                                  ((e1e3 - e0e2) * 2) * A.x + ((e2e3 + e0e1) * 2) * A.y + ((e0e0 + e3e3) * 2 - 1) * A.z);
        }


        /// Data in the order e0, e1, e2, e3
        [SerializeField]
        public double[] data = new double[4];

        // -----------------------------------------------------------------------------



        // -----------------------------------------------------------------------------
        // CONSTANTS

        /// Constant null quaternion: {0, 0, 0, 0}
        public static ChQuaternion QNULL = new ChQuaternion(0.0, 0.0, 0.0, 0.0); // PROBLEM These do not work, suspected c# 8 bug

        public static ChQuaternion QUNIT = new ChQuaternion();

        // -----------------------------------------------------------------------------
        // STATIC QUATERNION MATH OPERATIONS
        //
        // These functions are here for people which prefer to use static functions
        // instead of ChQuaternion class' member functions.
        // NOTE: sometimes a wise adoption of the following functions may give faster
        // results than using overloaded operators +/-/* in the quaternion class.

        // Return the conjugate of the quaternion [s,v1,v2,v3] is [s,-v1,-v2,-v3]
        public static ChQuaternion Qconjugate(ChQuaternion q)
        {
            ChQuaternion res = new ChQuaternion();
            res.e0 = q.e0;
            res.e1 = -q.e1;
            res.e2 = -q.e2;
            res.e3 = -q.e3;
            return (res);
        }

        /// Return the product of two quaternions. It is non-commutative (like cross product in vectors).
        public static ChQuaternion Qcross(ChQuaternion qa, ChQuaternion qb)
        {
            ChQuaternion res = new ChQuaternion();
            res.e0 = qa.e0 * qb.e0 - qa.e1 * qb.e1 - qa.e2 * qb.e2 - qa.e3 * qb.e3;
            res.e1 = qa.e0 * qb.e1 + qa.e1 * qb.e0 - qa.e3 * qb.e2 + qa.e2 * qb.e3;
            res.e2 = qa.e0 * qb.e2 + qa.e2 * qb.e0 + qa.e3 * qb.e1 - qa.e1 * qb.e3;
            res.e3 = qa.e0 * qb.e3 + qa.e3 * qb.e0 - qa.e2 * qb.e1 + qa.e1 * qb.e2;
            return (res);
        }


        public static ChQuaternion Qsub(ChQuaternion qa, ChQuaternion qb)
        {
            ChQuaternion result = new ChQuaternion();
            result.e0 = qa.e0 - qb.e0;
            result.e1 = qa.e1 - qb.e1;
            result.e2 = qa.e2 - qb.e2;
            result.e3 = qa.e3 - qb.e3;
            return result;
        }

        public static ChQuaternion Qscale(ChQuaternion q, double fact)
        {
            ChQuaternion result = new ChQuaternion();
            result.e0 = q.e0 * fact;
            result.e1 = q.e1 * fact;
            result.e2 = q.e2 * fact;
            result.e3 = q.e3 * fact;
            return result;
        }

        // Get the quaternion time derivative from the vector of angular speed, with w specified in _absolute_ coords.
        public static ChQuaternion Qdt_from_Wabs(ChVector w, ChQuaternion q)
        {
            ChQuaternion qw = new ChQuaternion();
            double half = 0.5;

            qw.e0 = 0;
            qw.e1 = w.x;
            qw.e2 = w.y;
            qw.e3 = w.z;

            return Qscale(Qcross(qw, q), half);  // {q_dt} = 1/2 {0,w}*{q}
        }

        public ChVector Q_to_Rotv() {
            ChVector angle_axis = new ChVector();
            double sin_squared = data[1] * data[1] + data[2] * data[2] + data[3] * data[3];
            // For non-zero rotation
            if (sin_squared > 0)
            {
                double sin_theta = Math.Sqrt(sin_squared);
                double k = 2 * Math.Atan2(sin_theta, data[0]) / sin_theta;
                angle_axis.x = data[1] * k;
                angle_axis.y = data[2] * k;
                angle_axis.z = data[3] * k;
            }
            else
            {
                // For almost zero rotation
                double k = 2.0;
                angle_axis.x = data[1] * k;
                angle_axis.y = data[2] * k;
                angle_axis.z = data[3] * k;
            }
            return angle_axis;
        }

        // Get the time derivative from a quaternion, a speed of rotation and an axis, defined in _abs_ coords.
        public static ChQuaternion Qdt_from_AngAxis(ChQuaternion quat, double angle_dt, ChVector axis)
        {
            ChVector W;

            W = ChVector.Vmul(axis, angle_dt);

            return Qdt_from_Wabs(W, quat);
        }


        // Get the second time derivative from a quaternion, an angular acceleration and an axis, defined in _abs_ coords.
        public static ChQuaternion Qdtdt_from_AngAxis(double angle_dtdt,
                                        ChVector axis,
                                        ChQuaternion q,
                                        ChQuaternion q_dt)
        {
            ChVector Acc;

            Acc = ChVector.Vmul(axis, angle_dtdt);

            return Qdtdt_from_Aabs(Acc, q, q_dt);
        }

        /// Set the quaternion ddq/dtdt. Inputs: the axis of ang. acceleration 'axis' (assuming it is already
        /// normalized and expressed in absolute coords), the angular acceleration 'angle_dtdt' (scalar value),
        /// the rotation expressed as a quaternion 'quat' and th rotation speed 'q_dt'.
        public void Qdtdt_from_AngAxis(ChQuaternion q,
                                                   ChQuaternion q_dt,
                                                   double angle_dtdt,
                                                   ChVector axis)
        {
            this.Qdtdt_from_Aabs2(angle_dtdt * axis, q, q_dt);
        }

        // Get the quaternion first derivative from the vector of angular acceleration with a specified in _absolute_ coords.
        public static ChQuaternion Qdtdt_from_Aabs(ChVector a,
                                                    ChQuaternion q,
                                                    ChQuaternion q_dt)
        {
            ChQuaternion ret = new ChQuaternion();
            ret.Qdtdt_from_Aabs2(a, q, q_dt);
            return ret;
        }

        // Get the quaternion first derivative from the vector of angular acceleration with a specified in _absolute_ coords.
        public void Qdtdt_from_Aabs2(ChVector a,
                                    ChQuaternion q,
                                    ChQuaternion q_dt)
        {
            ChQuaternion qao = new ChQuaternion(0, a);
            ChQuaternion qwo = new ChQuaternion();
            ChQuaternion qtmpa = new ChQuaternion();
            ChQuaternion qtmpb = new ChQuaternion();
            qwo.Cross(q_dt, q.GetConjugate());
            qtmpb.Cross(qwo, q_dt);
            qtmpa.Cross(qao, q);
            qtmpa.Scale(0.5);
            this.Add(qtmpa, qtmpb);
        }

        public ChVector Q_to_Euler123() {
            // Angles {phi;theta;psi} aka {roll;pitch;yaw} rotation XYZ
            ChVector euler = new ChVector();
            double sq0 = data[0] * data[0];
            double sq1 = data[1] * data[1];
            double sq2 = data[2] * data[2];
            double sq3 = data[3] * data[3];
            // roll
            euler.x = Math.Atan2(2 * (data[2] * data[3] + data[0] * data[1]), sq3 - sq2 - sq1 + sq0);
            // pitch
            euler.y = -Math.Asin(2 * (data[1] * data[3] - data[0] * data[2]));
            // yaw
            euler.z = Math.Atan2(2 * (data[1] * data[2] + data[3] * data[0]), sq1 + sq0 - sq3 - sq2);
            return euler;
        }

        // Get the quaternion first derivative from the vector of angular acceleration with a specified in _absolute_ coords.
        /*  public static ChQuaternion Qdtdt_from_Aabs(ChVector a,
                                       ChQuaternion q,
                                       ChQuaternion q_dt) {
              ChQuaternion ret;
              ret.Qdtdt_from_Aabs(a, q, q_dt);
              return ret;
          }*/

        // Check if two quaternions are equal
        public static bool Qequal(ChQuaternion qa, ChQuaternion qb)
        {
            return qa == qb;
        }

        // Get the X axis of a coordsystem, given the quaternion which
        // represents the alignment of the coordsystem.
        public static ChVector VaxisXfromQuat(ChQuaternion quat)
        {
            ChVector res = new ChVector();
            res.x = (Math.Pow(quat.e0, 2) + Math.Pow(quat.e1, 2)) * 2 - 1;
            res.y = ((quat.e1 * quat.e2) + (quat.e0 * quat.e3)) * 2;
            res.z = ((quat.e1 * quat.e3) - (quat.e0 * quat.e2)) * 2;
            return res;
        }
};

    /// Shortcut for faster use of typical double-precision quaternion.
    /// <pre>
    /// Instead of writing
    ///    ChQuaternion foo;
    /// or
    ///    ChQuaternion<> foo;
    /// you can use the shorter version
    ///    Quaternion foo;
    /// </pre>
    public class Quaternion : ChQuaternion { };

    /// Shortcut for faster use of typical single-precision quaternion.
    /// <pre>
    /// Instead of writing
    ///    ChQuaternion<float> foo;
    /// you can use the shorter version
    ///    Quaternion foo;
    /// </pre>
    public class QuaternionF : ChQuaternion { }
}