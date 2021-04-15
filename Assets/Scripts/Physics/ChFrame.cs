using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
using System.Runtime.InteropServices;


namespace chrono
{

    /// ChFrame: a class for coordinate systems in 3D space.
    ///
    ///  A 'frame' coordinate system has a translation and
    /// a rotation respect to a 'parent' coordinate system,
    /// usually the absolute (world) coordinates.
    ///
    ///  Differently from a simple ChCoordsys object, however,
    /// the ChFrame implements some optimizations because
    /// each ChFrame stores also a 3x3 rotation matrix, which
    /// can speed up coordinate transformations when a large
    /// amount of vectors must be transformed by the same
    /// coordinate frame.
    ///
    /// Further info at the @ref coordinate_transformations manual page.
    public class ChFrame<Real> where Real : unmanaged, IConvertible
    {
        public ChCoordsys coord = new ChCoordsys(new ChVector(0, 0, 0), new ChQuaternion(1, 0, 0, 0));  //< Rotation and position, as vector+quaternion

        // Auxiliary, for faster transformation of many coordinates      
        public ChMatrix33<Real> Amatrix = new ChMatrix33<Real>();  //< 3x3 orthogonal rotation matrix

        public static ChFrame<Real> FNULL = new ChFrame<Real>();

        public ChFrame() {
        }

        /// Default constructor, or construct from pos and rot (as a quaternion)
        public ChFrame(ChVector mv,
                        ChQuaternion mq)
        {
            coord = new ChCoordsys(mv, mq);
            Amatrix = new ChMatrix33<Real>(mq);
        }

        /// Construct from pos and rotation (as a 3x3 matrix)
        public ChFrame(ChVector mv, ChMatrix33<Real> ma)
        {
            coord = new ChCoordsys(mv, ma.Get_A_quaternion());
            Amatrix = new ChMatrix33<Real>(ma);
        }

        /// Construct from a coordsys
        public ChFrame(ChCoordsys mc)
        {
            coord = new ChCoordsys(mc);
            Amatrix = new ChMatrix33<Real>(mc.rot);
        }

        /// Construct from position mv and rotation of angle alpha around unit vector mu
        public ChFrame(ChVector mv, double alpha, ChVector mu)
        {
            coord = new ChCoordsys(mv, alpha, mu);
            Amatrix.Set_A_quaternion(coord.rot);
        }

        /// Copy constructor, build from another frame
        public ChFrame(ChFrame<Real> other)
        {
            coord = other.coord;
            Amatrix = other.Amatrix;
        }

        //
        // OPERATORS OVERLOADING
        //


        /// The '*' operator transforms a vector, so
        /// transformations can be represented with this syntax:
        ///  new_v = tr_frame * old_v;
        /// For a sequence of transformations, i.e. a chain of coordinate
        /// systems, you can also write this (just like you would do with
        /// a sequence of Denavitt-Hartemberg matrix multiplications!)
        ///  new_v = frame1to0 * frame2to1 * frame3to2 * old_v;
        /// This operation is not commutative.
        ///  NOTE: since c++ operator execution is from left to right, in
        /// case of multiple transformations like w=A*B*C*v, the >> operator
        /// performs faster, like  w=v>>C>>B>>A;
        public static ChVector operator *(ChFrame<Real> frame, ChVector V) { return frame.TransformLocalToParent(V); }

        /// The '/' is like the '*' operator (see), but uses the inverse
        /// transformation for A, in A/b. (with A ChFrame, b ChVector)
        /// That is: c=A*b ; b=A/c;
        public static ChVector operator /(ChFrame<Real> frame, ChVector V) { return frame.TransformParentToLocal(V); }

        public static ChFrame<double> BitShiftRight(ChFrame<double> Fa, ChFrame<double> Fb)
        {
            ChFrame<double> res = new ChFrame<double>();
            Fb.TransformLocalToParent(Fa, res);
            return res;
        }


        /// The transformation is inverted in place.
        /// That is if w=A*v, then A.Invert();v=A*w;
        public virtual void Invert()
        {
            coord.rot.Conjugate();
            Amatrix.MatrTranspose();
            coord.pos = -Amatrix.Matr_x_Vect(coord.pos);
        }

        /// Return the current translation as a 3d vector
        //  public ChVector GetPos() { return coord.pos; }

        /// Return the current rotation as a quaternion
        // public ChQuaternion GetRot() { return coord.rot; }

        // FUNCTIONS FOR COORDINATE TRANSFORMATIONS

        /// This function transforms a point from the local frame coordinate
        /// system to the parent coordinate system.
        /// OPTIMIZED FOR SPEED.
        /// Since it will use the auxiliary rotation matrix of the ChFrame
        /// object, this function is about 50% faster than TransformParentToLocal
        /// of a ChCoordsys.
        /// \return The point in parent coordinate
        public virtual ChVector TransformLocalToParent(ChVector local) {
            return ChTransform<Real>.TransformLocalToParent(local, coord.pos, Amatrix);
        }

        /// This function transforms a point from the parent coordinate
        /// system to local frame coordinate system.
        /// OPTIMIZED FOR SPEED.
        /// Since it will use the auxiliary rotation matrix of the ChFrame
        /// object, this function is about 50% faster than TransformParentToLocal
        /// method of a ChCoordsys.
        /// \return The point in local frame coordinate
        public virtual ChVector TransformParentToLocal(ChVector parent) {
            return ChTransform<Real>.TransformParentToLocal(parent, coord.pos, Amatrix);
        }

        public virtual ChVector TransformPointLocalToParent(ChVector local)
        {
            return ChTransform<Real>.TransformLocalToParent(local, coord.pos, Amatrix);
        }

        public ChVector TransformPointParentToLocal(ChVector parent) {
            return ChTransform<Real>.TransformParentToLocal(parent, coord.pos, Amatrix);
        }

        /// This function transforms a frame from 'this' local coordinate
        /// system to parent frame coordinate system.
        /// \return The frame in parent frame coordinate
        public void TransformLocalToParent(
        ChFrame<Real> local,  //< frame to transform, given in local frame coordinates
        ChFrame<Real> parent        //< transformed frame, in parent coordinates, will be stored here
        )
        {
            parent.SetCoord(TransformLocalToParent(local.coord.pos), coord.rot % local.coord.rot);
        }

        /// This function transforms a frame from the parent coordinate
        /// system to 'this' local frame coordinate system.
        /// \return The frame in local frame coordinate
        public virtual void TransformParentToLocal(
        ChFrame<Real> parent,  //< frame to transform, given in parent coordinates
        ChFrame<Real> local          //< transformed frame, in local coordinates, will be stored here
        )
        {
            local.SetCoord(TransformParentToLocal(parent.coord.pos), coord.rot.GetConjugate() % parent.coord.rot);
        }

        public virtual void TransformParentToLocal2(
        ChFrame<Real> parent,  //< frame to transform, given in parent coordinates
        ChFrameMoving<Real> local          //< transformed frame, in local coordinates, will be stored here
)
        {
            local.SetCoord(TransformParentToLocal(parent.coord.pos), coord.rot.GetConjugate() % parent.coord.rot);
        }

        /// This function transforms a direction from the parent frame coordinate system
        /// to 'this' local coordinate system.
        /// \return The direction in parent frame coordinate
        public virtual ChVector TransformDirectionLocalToParent(ChVector mdirection)
        {
            return Amatrix.Matr_x_Vect(mdirection);
        }

    //
    // FUNCTIONS
    //

    // GET-FUNCTIONS

    /// Return both current rotation and translation as
    /// a coordsystem object, with vector and quaternion
    public ChCoordsys GetCoord() { return coord; }

        /// Return the current translation as a 3d vector
        public ChVector GetPos() { return coord.pos; }

        /// Return the current rotation as a quaternion
        public  ChQuaternion GetRot() { return coord.rot; }

        /// Return the current rotation as a 3x3 matrix
        public ChMatrix33<Real> GetA() { return Amatrix; }

        /// Get axis of finite rotation, in parent space
        public ChVector GetRotAxis()
        {
            ChVector vtmp = new ChVector(0, 0, 0);
            double angle = 0;
            coord.rot.Q_to_AngAxis(ref angle, ref vtmp);
            return vtmp;
        }

        /// Get angle of rotation about axis of finite rotation
        public double GetRotAngle()
        {
            ChVector vtmp = new ChVector(0, 0, 0);
            double angle = 0;
            coord.rot.Q_to_AngAxis(ref angle, ref vtmp);
            return angle;
        }

        // SET-FUNCTIONS

        /// Impose both translation and rotation as a
        /// single ChCoordsys. Note: the quaternion part must be
        /// already normalized!
        public virtual void SetCoord(ChCoordsys mcoord)
        {
            coord = mcoord;
            //Debug.Log("Debug " + coord.pos.y);
            Amatrix.Set_A_quaternion(mcoord.rot);
        }


        /// Impose both translation and rotation.
        /// Note: the quaternion part must be already normalized!
        public virtual void SetCoord(ChVector mv,  ChQuaternion mq)
        {
            coord.pos = mv;
            coord.rot = mq;
            Amatrix.Set_A_quaternion(mq);
        }

        /// Impose the rotation as a quaternion.
        /// Note: the quaternion must be already normalized!
        public virtual void SetRot( ChQuaternion mrot) {
            coord.rot = mrot;
            Amatrix.Set_A_quaternion(mrot);
        }

        /// Impose the rotation as a 3x3 matrix.
        /// Note: the rotation matrix must be already orthogonal!
        public virtual void SetRot(ChMatrix33<Real> mA)
        {
            coord.rot = mA.Get_A_quaternion();
            Amatrix.CopyFromMatrix(mA);
        }

        /// Impose the translation
        public virtual void SetPos(ChVector mpos) { 
            coord.pos = mpos;     
        }

        /// Sets to no translation and no rotation
        public virtual void SetIdentity()
        {
            coord.SetIdentity();
            Amatrix.SetIdentity();
        }

        public ChFrameMoving<Real> GetInverse() {
            ChFrameMoving<Real> tmp = new ChFrameMoving<Real>(this);
            tmp.Invert();
            return tmp;
        }

    /// Fast fill a 3x4 matrix [Gl(q)], as in local angular speed conversion
    /// Wl=[Gl]*q_dt   (btw: [Gl(q)] = 2*[Fp(q')] = 2*[G] with G matrix as in Shabana)
    public static void SetMatrix_Gl(ref ChMatrixNM<IntInterface.Three, IntInterface.Four> Gl,  ChQuaternion mq) {
           // Debug.Assert((Gl.GetRows() == 3) && (Gl.GetColumns() == 4));
            double de0 = 2 * mq.e0;
            double de1 = 2 * mq.e1;
            double de2 = 2 * mq.e2;
            double de3 = 2 * mq.e3;
            Gl[0] = -de1;
            Gl[1] = de0;
            Gl[2] = de3;
            Gl[3] = -de2;
            Gl[4] = -de2;
            Gl[5] = -de3;
            Gl[6] = de0;
            Gl[7] = de1;
            Gl[8] = -de3;
            Gl[9] = de2;
            Gl[10] = -de1;
            Gl[11] = de0;
        }

        /// Fast fill a 3x4 matrix [Gl(q)], as in local angular speed conversion
        /// Wl=[Gl]*q_dt   (btw: [Gl(q)] = 2*[Fp(q')] = 2*[G] with G matrix as in Shabana)
       /* public static void SetMatrix_Gl(ref ChMatrixNM<3, 4> Gl, ChQuaternion mq) {
            // assert((Gl.GetRows() == 3) && (Gl.GetColumns() == 4));
            double de0 = 2 * mq.e0;
            double de1 = 2 * mq.e1;
            double de2 = 2 * mq.e2;
            double de3 = 2 * mq.e3;
            Gl[0] = -de1;
            Gl[1] = de0;
            Gl[2] = de3;
            Gl[3] = -de2;
            Gl[4] = -de2;
            Gl[5] = -de3;
            Gl[6] = de0;
            Gl[7] = de1;
            Gl[8] = -de3;
            Gl[9] = de2;
            Gl[10] = -de1;
            Gl[11] = de0;
        }*/

    /// Fast fill a 3x4 matrix [Gw(q)], as in absolute angular speed conversion
    /// Ww=[Gw]*q_dt   (btw: [Gw(q)] = 2*[Fm(q')] = 2*[E] with E matrix as in Shabana)
    public static void SetMatrix_Gw(ref ChMatrixNM<IntInterface.Three, IntInterface.Four> Gw,  ChQuaternion mq)
        {
           // Debug.Assert((Gw.GetRows() == 3) && (Gw.GetColumns() == 4));
            double de0 = 2 * mq.e0;
            double de1 = 2 * mq.e1;
            double de2 = 2 * mq.e2;
            double de3 = 2 * mq.e3;
            Gw[0] = -de1;
            Gw[1] = de0;
            Gw[2] = -de3;
            Gw[3] = de2;
            Gw[4] = -de2;
            Gw[5] = de3;
            Gw[6] = de0;
            Gw[7] = -de1;
            Gw[8] = -de3;
            Gw[9] = -de2;
            Gw[10]= de1;
            Gw[11] = de0;
        }

        /// Fills a 3x4 matrix [Fp(q)], as in  [Fp(q)]*[Fm(q)]' = [A(q)]
        public static void SetMatrix_Fp(ref ChMatrixNM<IntInterface.Three, IntInterface.Four> Fp,  ChQuaternion mq) {
            //Debug.Assert((Fp.GetRows() == 3) && (Fp.GetColumns() == 4));
            Fp[0] = mq.e1;
            Fp[1] = mq.e0;
            Fp[2] = -mq.e3;
            Fp[3] = mq.e2;
            Fp[4] = mq.e2;
            Fp[5] = mq.e3;
            Fp[6] = mq.e0;
            Fp[7] = -mq.e1;
            Fp[8] = mq.e3;
            Fp[9] = -mq.e2;
            Fp[10] = mq.e1;
            Fp[11] = mq.e0;
        }

        /// Fills a 3x4 matrix [Fm(q)], as in  [Fp(q)]*[Fm(q)]' = [A(q)]
        public static void SetMatrix_Fm(ref ChMatrixNM<IntInterface.Three, IntInterface.Four> Fp,  ChQuaternion mq)
        {
           // Debug.Assert((Fp.GetRows() == 3) && (Fp.GetColumns() == 4));
            Fp[0] = mq.e1;
            Fp[1] = mq.e0;
            Fp[2] = mq.e3;
            Fp[3] = -mq.e2;
            Fp[4] = mq.e2;
            Fp[5] = -mq.e3;
            Fp[6] = mq.e0;
            Fp[7] = mq.e1;
            Fp[8] = mq.e3;
            Fp[9] = mq.e2;
            Fp[10] = -mq.e1;
            Fp[11] = mq.e0;
        }


        /// Computes the product q=[Gl(mq)]*v  without the need of having
        /// the [Gl] matrix (just pass the mq quaternion, since Gl is function of mq)
        public  ChQuaternion GlT_x_Vect(ChQuaternion mq, ChVector v)
        {
            double de0 = 2 * mq.e0;
            double de1 = 2 * mq.e1;
            double de2 = 2 * mq.e2;
            double de3 = 2 * mq.e3;
            return new  ChQuaternion(-de1 * v.x - de2 * v.y - de3 * v.z, +de0 * v.x - de3 * v.y + de2 * v.z,
                                      +de3 * v.x + de0 * v.y - de1 * v.z, -de2 * v.x + de1 * v.y + de0 * v.z);
        }

        /// This function transforms a direction from 'this' local coordinate
        /// system to parent frame coordinate system.
        /// \return The direction in local frame coordinate
        public virtual ChVector TransformDirectionParentToLocal(
                                        ChVector mdirection  //< direction to transform, given in parent coordinates
                                        )
        {
            return Amatrix.MatrT_x_Vect(mdirection);
        }

    /// The transformation is inverted in place.
    /// That is if w=A*v, then A.Invert();v=A*w;
    /* public void Invert()
     {
         coord.rot.Conjugate();
         Amatrix.transposeInPlace();
         coord.pos = -(Amatrix * coord.pos);
     }*/

    /* public ChFrame GetInverse() {
         ChFrame tmp = new ChFrame(this);
         tmp.Invert();
         return tmp;
     }*/

    /// This function transforms a direction from 'this' local coordinate
    /// system to parent frame coordinate system.
    /// \return The direction in local frame coordinate
    /* public ChVector TransformDirectionParentToLocal(ChVector mdirection)
     {

        // ChMatrix33 mat = new ChMatrix33();
        // return mat.transpose() * mdirection;
         //return Amatrix.transpose() * mdirection;

         ChVector vec = new ChVector(0, 0, 0);
         vec.m_ChVector = ChFrame_TransformDirectionParentToLocal(m_ChFrame, mdirection.m_ChVector);
         vec.data[0] = vec.GetX();
         vec.data[1] = vec.GetY();

         vec.data[2] = vec.GetZ();
         return vec;
     }*/

    /// This function transforms a direction from the parent frame coordinate system
    /// to 'this' local coordinate system.
    /// \return The direction in parent frame coordinate
    /* public ChVector TransformDirectionLocalToParent(ChVector mdirection)
     {
         //return Amatrix * mdirection;

         ChVector vec = new ChVector(0, 0, 0);
         vec.m_ChVector = ChFrame_TransformDirectionLocalToParent(m_ChFrame, mdirection.m_ChVector);
         vec.data[0] = vec.GetX();
         vec.data[1] = vec.GetY();
         vec.data[2] = vec.GetZ();
         return vec;
     }*/

}
}
