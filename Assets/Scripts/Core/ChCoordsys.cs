using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
using System.Runtime.InteropServices;

namespace chrono
{
    ///
    /// COORDSYS:
    ///
    ///  This class contains both translational variable
    /// (the origin of the axis) and the rotational variable
    /// (that is the unitary quaternion which represent the
    /// special-orthogonal transformation matrix).
    ///   Basic features for point-coordinate transformations
    /// are provided. However, for more advanced features, the
    /// heavier classes ChFrame() or ChFrameMoving() may suit better.
    ///  The coordsys object comes either with the template "ChCoordsys<type>" mode,
    /// either in the 'shortcut' flavor, that is "Coordsys", which assumes
    /// the type of the four scalars is double precision, so it is faster to type.
    ///
    /// Further info at the @ref coordinate_transformations manual page.
    [System.Serializable]
    public class ChCoordsys<Real> where Real : notnull
    {
        public ChVector pos = new ChVector(0, 0, 0);
        public ChQuaternion rot = new ChQuaternion(0, 0, 0, 0);

        // Default constructor (identity frame)
        public ChCoordsys()
        {
            pos = new ChVector(0, 0, 0);
            rot = new ChQuaternion(1, 0, 0, 0);
        }
        // Construct from position and rotation (as quaternion)
        public ChCoordsys(ChVector mv, ChQuaternion mq)
        {
            pos = mv;
            rot = mq;
            
        }

        // Construct from position mv and rotation of angle alpha around unit vector mu
        public ChCoordsys(ChVector mv, double alpha, ChVector mu)
        {
            pos = new ChVector(mv);
            rot.Q_from_AngAxis(alpha, mu);
        }

        /// Copy constructor
        public ChCoordsys(ChCoordsys<Real> other)
        {
            pos = other.pos;
            rot = other.rot;
        }

        public ChVector GetPos()
        {            
            return pos;
        }

        public ChQuaternion GetRot()
        {

            return rot;
        }

        public void SetPos(ChVector vec)
        {
            pos = vec;
        }

        public void SetRot(ChQuaternion quat) {
            rot = quat;
 
        }

        /// Returns true if coordsys is identical to other coordsys
        public bool Equals(ChCoordsys<Real> other) 
        { 
            return rot.Equals(other.rot) && pos.Equals(other.pos); 
        }

        public bool Equals(ChCoordsys<Real> other, double tol)
        {
            return rot.Equals(other.rot, tol) && pos.Equals(other.pos, tol);
        }

        /// Sets to no translation and no rotation
        public void SetIdentity()
        {
            pos = new ChVector(0, 0, 0);
            rot = new ChQuaternion(1, 0, 0, 0);
        }

        /// Returns true if coordsys is equal to other coordsys, within a tolerance 'tol'
       /* public bool Equals(ChCoordsys<Real> other, double tol)
        {
            return rot.Equals(other.rot, tol) && pos.Equals(other.pos, tol);
        }*/

        public static ChCoordsys<Real> BitShiftRight(ChCoordsys<Real> a, ChCoordsys<Real> Fb)
        {
            return Fb.TransformLocalToParent(a);
        }

        // public void SetRot(double e0, double e1, double e2, double e3)
        //{
        /* rot.Sete0(e0);
         rot.Sete1(e1);
         rot.Sete2(e2);
         rot.Sete3(e3);
         ChCoordsys_SetRot(m_ChCoordsys, rot.m_ChQuaternion);*/
        // }
        /*   public ChVector TransformLocalToParent(ChVector vec)
           {
               ChVector temp = new ChVector(0, 0, 0);
               temp.m_ChVector = ChCoordsys_TransformLocalToParent(m_ChCoordsys, vec.m_ChVector);
               temp.data[0] = temp.GetX();
               temp.data[1] = temp.GetY();
               temp.data[2] = temp.GetZ();

               return temp;
           }*/

        /*   public ChVector TransformDirectionParentToLocal(ChVector vec)
           {
               ChVector temp = new ChVector(0, 0, 0);
               temp.m_ChVector = ChCoordsys_TransformDirectionParentToLocal(m_ChCoordsys, vec.m_ChVector);
               temp.data[0] = temp.GetX();
               temp.data[1] = temp.GetY();
               temp.data[2] = temp.GetZ();

               return temp;
           }*/

        /*  public ChVector TransformDirectionLocalToParent(ChVector vec)
          {
              ChVector temp = new ChVector(0, 0, 0);
              temp.m_ChVector = ChCoordsys_TransformDirectionLocalToParent(m_ChCoordsys, vec.m_ChVector);
              temp.data[0] = temp.GetX();
              temp.data[1] = temp.GetY();
              temp.data[2] = temp.GetZ();

              return temp;
          }*/

        // FUNCTIONS FOR COORDINATE TRANSFORMATIONS

        /// This function transforms a point from the local coordinate
        /// system to the parent coordinate system. Relative position of local respect
        /// to parent is given by this coordys, i.e. 'origin' translation and 'alignment' quaternion.
        /// \return The point in parent coordinate, as parent=origin +q*[0,(local)]*q'
        public ChVector TransformLocalToParent(ChVector local) { return pos + rot.Rotate(local); }

        public ChVector TransformPointLocalToParent(ChVector local) { return pos + rot.Rotate(local); }

        /// This function transforms a direction from the parent coordinate system to
        /// 'this' local coordinate system.
        public ChVector TransformDirectionParentToLocal(ChVector parent) { return rot.RotateBack(parent); }

        /// This function transforms a point from the parent coordinate
        /// system to a local coordinate system, whose relative position
        /// is given by this coodsys, i.e. 'origin' translation and 'alignment' quaternion.
        /// \return The point in local coordinate, as local=q'*[0,(parent-origin)]*q
        public ChVector TransformParentToLocal( ChVector parent)  { return rot.RotateBack(parent - pos); }

        public ChVector TransformPointParentToLocal(ChVector parent)
        {
            return rot.RotateBack(parent - pos);
        }

        /// This function transforms a coordsys given in 'this' coordinate system to
        /// the parent coordinate system
        public ChCoordsys<Real> TransformLocalToParent(ChCoordsys<Real> local)
        {
            return new ChCoordsys<Real>(TransformLocalToParent(local.pos), rot % local.rot);
        }

    /// This function transforms a direction from 'this' local coordinate system
    /// to the parent coordinate system.
    public ChVector TransformDirectionLocalToParent(ChVector local) { return rot.Rotate(local); }



     /// This function transforms a coordsys given in 'this' coordinate system to
     /// the parent coordinate system
   /*  public ChCoordsys TransformLocalToParent(ChCoordsys local) {
         return new ChCoordsys(TransformLocalToParent(local.pos), rot % local.rot);
     }*/

    //
    // STATIC COORDSYS OPERATIONS
    //

    /// Force 3d coordsys to lie on a XY plane (note: no normaliz. on quat)
    public static Coordsys Force2Dcsys(Coordsys cs)
        {
            Coordsys res = new Coordsys();
            res = cs;
            res.pos.z = 0;
            res.rot.e1 = 0;
            res.rot.e2 = 0;
            return (res);
        }

        // CONSTANTS

       // public static ChCoordsys<double> CSYSNULL = new ChCoordsys<double>(new ChVector(0, 0, 0), ChQuaternion.QNULL);
       // public static ChCoordsys<double> CSYSNORM = new ChCoordsys<double>(new ChVector(0, 0, 0), ChQuaternion.QUNIT);

    }

    /// Shortcut for faster use of typical double-precision coordsys.
    ///  Instead of writing    "ChCoordsys<double> foo;"   you can write
    ///  the shorter version   "Coordsys foo;"
    ///
    public class Coordsys : ChCoordsys<double> { }

    /// Shortcut for faster use of typical single-precision coordsys.
    ///
    public class CoordsysF : ChCoordsys<float> { }


}
