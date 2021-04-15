using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
using System.Runtime.InteropServices;

namespace chrono
{
    /// ChFrameMoving: a class for coordinate systems in 3D space.
    ///
    ///  A 'frame' coordinate system has a translation and
    /// a rotation respect to a 'parent' coordinate system,
    /// usually the absolute (world) coordinates.
    ///
    /// Differently from a simple ChCoordsys() object, however,
    /// the ChFrame implements some optimizations because
    /// each ChFrame stores also a 3x3 rotation matrix, which
    /// can speed up coordinate transformations when a large
    /// amount of vectors must be transformed by the same
    /// coordinate frame.
    ///
    /// Further info at the @ref coordinate_transformations manual page.

    public class ChFrameMoving<Real> : ChFrame<Real> where Real : unmanaged, IConvertible
    {
        public ChCoordsys coord_dt = new ChCoordsys(new ChVector(0, 0, 0), new ChQuaternion(0, 0, 0, 0));    //< Rotation and position speed, as vector+quaternion
        public ChCoordsys coord_dtdt = new ChCoordsys(new ChVector(0, 0, 0), new ChQuaternion(0, 0, 0, 0));  //< Rotation and position acceleration, as vector+quaternion

        //Test
        ChMatrix33<double> mr = new ChMatrix33<double>();

        public ChFrameMoving(): base(new ChVector(0, 0, 0), new ChQuaternion(1, 0, 0, 0)){
            coord_dt.rot = coord_dtdt.rot = new ChQuaternion(0, 0, 0, 0);
        }
        /// Construct from pos and rot (as a quaternion)
        public ChFrameMoving(ChVector mv,
                             ChQuaternion mq)
        : base(mv, mq)
        {
            coord_dt.rot = coord_dtdt.rot = new ChQuaternion(0, 0, 0, 0);
        }

        /// Construct from pos and rotation (as a 3x3 matrix)
        public ChFrameMoving(ChVector mv, ChMatrix33<Real> ma) : base(mv, ma)
        {
            coord_dt.rot = coord_dtdt.rot = new ChQuaternion(0, 0, 0, 0);
        }

        /// Construct from a coordsys
        public ChFrameMoving(ChCoordsys mc) : base(mc)
        {
            coord_dt.rot = coord_dtdt.rot = new ChQuaternion(0, 0, 0, 0);
        }

        /// Construct from a frame
        public ChFrameMoving(ChFrame<Real> mc) : base(mc)
        {
            coord_dt.rot = coord_dtdt.rot = new ChQuaternion(0, 0, 0, 0);
        }

        /// Copy constructor, build from another moving frame
        public ChFrameMoving(ChFrameMoving<Real> other)
            : base(other)
        {
            coord_dt = other.coord_dt;
            coord_dtdt = other.coord_dtdt;
        }

        /// The '>>' operator transforms a coordinate system, so
        /// transformations can be represented with this syntax:
        ///  new_frame = old_frame >> tr_frame;
        /// For a sequence of transformations, i.e. a chain of coordinate
        /// systems, you can also write this (like you would do with
        /// a sequence of Denavitt-Hartemberg matrix multiplications,
        /// but in the _opposite_ order...)
        ///  new_frame = old_frame >> frame3to2 >> frame2to1 >> frame1to0;
        /// This operation is not commutative.
        /// Also speeds and accelerations are transformed.
        public static ChFrameMoving<Real> BitShiftRight(ChFrameMoving<Real> Fa, ChFrameMoving<Real> Fb)
        {
            ChFrameMoving<Real> res = new ChFrameMoving<Real>();
            Fb.TransformLocalToParent(Fa, ref res);
            return res;
        }

        
        //
        // FUNCTIONS
        //

        // GET-FUNCTIONS

        /// Return both current rotation and translation speeds as
        /// a coordsystem object, with vector and quaternion
        public ChCoordsys GetCoord_dt() { return coord_dt; }

        /// Return both current rotation and translation accelerations as
        /// a coordsystem object, with vector and quaternion
        public ChCoordsys GetCoord_dtdt() { return coord_dtdt; }


        /// Return the current speed as a 3d vector
        public ChVector GetPos_dt() { return coord_dt.pos; }

        /// Return the current acceleration as a 3d vector
        public ChVector GetPos_dtdt() { return coord_dtdt.pos; }

        /// Return the current rotation speed as a quaternion
        public ChQuaternion GetRot_dt() { return coord_dt.rot; }

        /// Return the current rotation acceleration as a quaternion
        public ChQuaternion GetRot_dtdt() { return coord_dtdt.rot; }

        /// Computes the actual angular speed (expressed in local coords)
        public ChVector GetWvel_loc()
        {
            ChMatrixNM<IntInterface.Three, IntInterface.Four> tempGl = ChMatrixNM<IntInterface.Four, IntInterface.Four>.NMNULL3_4;// new ChMatrixNM<IntInterface.Three, IntInterface.Four>();
            // ChFrame<Real> gl = new ChFrame<Real>();
            ChFrame<Real>.SetMatrix_Gl(ref tempGl, this.coord.rot);

            return tempGl.Matr34_x_Quat(coord_dt.rot);  // wl=[Gl]*q_dt
        }

        /// Computes the actual angular speed (expressed in parent coords)
        public ChVector GetWvel_par()
        {
            ChMatrixNM<IntInterface.Three, IntInterface.Four> tempGw = ChMatrixNM<IntInterface.Four, IntInterface.Four>.NMNULL3_4;//new ChMatrixNM<IntInterface.Three, IntInterface.Four>();
            ChFrame<Real>.SetMatrix_Gw(ref tempGw, this.coord.rot);
            return tempGw.Matr34_x_Quat(coord_dt.rot);  // ww=[Gw]*q_dt
        }

        /// Computes the actual angular acceleration (expressed in local coords)
        public ChVector GetWacc_loc()
        {
            ChMatrixNM<IntInterface.Three, IntInterface.Four> tempGl = ChMatrixNM<IntInterface.Four, IntInterface.Four>.NMNULL3_4;//new ChMatrixNM<IntInterface.Three, IntInterface.Four>();
            //ChFrame<Real> gl = new ChFrame<Real>();
            ChFrame<Real>.SetMatrix_Gl(ref tempGl, this.coord.rot);
            return tempGl.Matr34_x_Quat(coord_dtdt.rot);  // al=[Gl]*q_dtdt
        }

        /// Computes the actual angular acceleration (expressed in parent coords)
        public ChVector GetWacc_par()
        {
            ChMatrixNM<IntInterface.Three, IntInterface.Four> tempGw = ChMatrixNM<IntInterface.Four, IntInterface.Four>.NMNULL3_4;//new ChMatrixNM<IntInterface.Three, IntInterface.Four>();
            ChFrame<Real>.SetMatrix_Gw(ref tempGw, this.coord.rot);
            return tempGw.Matr34_x_Quat(coord_dtdt.rot);  // aw=[Gw]*q_dtdt

        }

        // SET-FUNCTIONS

        /// Set both linear speed and rotation speed as a
        /// single ChCoordsys derivative.
        public virtual void SetCoord_dt(ChCoordsys mcoord_dt) { coord_dt = mcoord_dt; }

        /// Set the linear speed
        public virtual void SetPos_dt(ChVector mvel) { coord_dt.pos = mvel; }

        /// Set the rotation speed as a quaternion.
        /// Note: the quaternion must already satisfy  dot(q,q_dt)=0
        public virtual void SetRot_dt( ChQuaternion mrot_dt) { coord_dt.rot = mrot_dt; }

        /// Set the rotation speed from given angular speed
        /// (expressed in local csys)
        public virtual void SetWvel_loc(ChVector wl)
        {
            coord_dt.rot.Cross(this.coord.rot, new ChQuaternion(0, wl));
            coord_dt.rot *= 0.5;  // q_dt = 1/2 * q * (0,wl)
        }

        /// Set the rotation speed from given angular speed
        /// (expressed in parent csys)
        public virtual void SetWvel_par(ChVector wp)
        {
            coord_dt.rot.Cross(new ChQuaternion(0, wp), this.coord.rot);           
            coord_dt.rot *= 0.5;  // q_dt = 1/2 * (0,wp) * q
        }

        /// Set both linear acceleration and rotation acceleration as a
        /// single ChCoordsys derivative.
        public virtual void SetCoord_dtdt(ChCoordsys mcoord_dtdt) { coord_dtdt = mcoord_dtdt; }

        /// Set the linear acceleration
        public virtual void SetPos_dtdt(ChVector macc) { coord_dtdt.pos = macc; }

        /// Set the rotation acceleration as a quaternion derivative.
        /// Note: the quaternion must already satisfy  dot(q,q_dt)=0
        public virtual void SetRot_dtdt( ChQuaternion mrot_dtdt) { coord_dtdt.rot = mrot_dtdt; }

        /// Set the rotation acceleration from given angular acceleration
        /// (expressed in local csys)
        public virtual void SetWacc_loc(ChVector al)
        {
            // q_dtdt = q_dt * q' * q_dt + 1/2 * q * (0,al)
            coord_dtdt.rot = (coord_dt.rot % this.coord.rot.GetConjugate() % coord_dt.rot) +
                             (this.coord.rot % new ChQuaternion(0, al) * 0.5);
        }

        /// Set the rotation speed from given angular speed
        /// (expressed in parent csys)
        public virtual void SetWacc_par(ref ChVector ap)
        {
            // q_dtdt = q_dt * q' * q_dt + 1/2 * (0,ap) * q
            coord_dtdt.rot = (coord_dt.rot % this.coord.rot.GetConjugate() % coord_dt.rot) +
                             (new ChQuaternion(0, ap) % this.coord.rot * 0.5);
        }

        /// Computes the time derivative of rotation matrix, mAdt.
        public void Compute_Adt(ref ChMatrix33<Real> mA_dt)
        {
            //  [A_dt]=2[dFp/dt][Fm]'=2[Fp(q_dt)][Fm(q)]'
            ChMatrixNM<IntInterface.Three, IntInterface.Four> Fpdt = new ChMatrixNM<IntInterface.Three, IntInterface.Four>(); //ChMatrixNM<IntInterface.Four, IntInterface.Four>.NMNULL3_4;
            ChMatrixNM<IntInterface.Three, IntInterface.Four> Fm = new ChMatrixNM<IntInterface.Three, IntInterface.Four>();
            ChFrame<Real>.SetMatrix_Fp(ref Fpdt, coord_dt.rot);
            ChFrame<Real>.SetMatrix_Fm(ref Fm, this.coord.rot);
            mA_dt.MatrMultiplyT(Fpdt, Fm);
            mA_dt.MatrScale(2);
        }

        /// Computes the 2nd time derivative of rotation matrix, mAdtdt.
        public void Compute_Adtdt(ref ChMatrix33<Real> mA_dtdt)
        {
            //  [A_dtdt]=2[Fp(q_dtdt)][Fm(q)]'+2[Fp(q_dt)][Fm(q_dt)]'
            ChMatrixNM<IntInterface.Three, IntInterface.Four> ma = new ChMatrixNM<IntInterface.Three, IntInterface.Four>();
            ChMatrixNM<IntInterface.Three, IntInterface.Four> mb = new ChMatrixNM<IntInterface.Three, IntInterface.Four>();
            ChMatrix33<double> mr = new ChMatrix33<double>();

            ChFrame<Real>.SetMatrix_Fp(ref ma, coord_dtdt.rot);
            ChFrame<Real>.SetMatrix_Fm(ref mb, this.coord.rot);
            mr.MatrMultiplyT(ma, mb);
            ChFrame<Real>.SetMatrix_Fp(ref ma, coord_dt.rot);
            ChFrame<Real>.SetMatrix_Fm(ref mb, coord_dt.rot);
            mA_dtdt.MatrMultiplyT(ma, mb);
            mA_dtdt.MatrInc(mr);
            mA_dtdt.MatrScale(2);
        }

        /// Computes and returns an Adt matrix (-note: prefer using
        /// Compute_Adt() directly for better performance)
        public ChMatrix33<Real> GetA_dt()
        {
            ChMatrix33<Real> res = new ChMatrix33<Real>();
            Compute_Adt(ref res);
            return res;
        }

        /// Computes and returns an Adt matrix (-note: prefer using
        /// Compute_Adtdt() directly for better performance)
        public ChMatrix33<Real> GetA_dtdt()
        {
            ChMatrix33<Real> res = new ChMatrix33<Real>();
            Compute_Adtdt(ref res);
            return res;
        }

        // FUNCTIONS TO TRANSFORM THE FRAME ITSELF

        /// Apply a transformation (rotation and translation) represented by
        /// another ChFrameMoving T. This is equivalent to pre-multiply this frame
        /// by the other frame T:   this'= T * this;  or this' = this >> T
        public void ConcatenatePreTransformation(ChFrameMoving<Real> T)
        {
            ChFrameMoving<Real> res = new ChFrameMoving<Real>();
            T.TransformLocalToParent(this, ref res);
            //this = res;
        }

        /// Apply a transformation (rotation and translation) represented by
        /// another ChFrameMoving T in local coordinate. This is equivalent to
        /// post-multiply this frame by the other frame T:   this'= this * T; or this' = T >> this
        public void ConcatenatePostTransformation(ChFrameMoving<Real> T)
        {
            ChFrameMoving<Real> res = new ChFrameMoving<Real>();
            this.TransformLocalToParent(T, ref res);
            //*this = res;
        }

        // FUNCTIONS FOR COORDINATE TRANSFORMATIONS

        /// Given the position of a point in local frame coords, and
        /// assuming it is sticky to frame, return the speed in parent coords.
        public ChVector PointSpeedLocalToParent(ChVector localpos)
        {
            return coord_dt.pos +
                   ((coord_dt.rot % new ChQuaternion(0, localpos) % this.coord.rot.GetConjugate()).GetVector() * 2);
        }

        /// Given the position localpos of a point in the local reference frame, assuming
        /// that the point moves in the local reference frame with localspeed,
        /// return the speed in the parent reference frame.
        public ChVector PointSpeedLocalToParent(ChVector localpos, ChVector localspeed)
        {
            return coord_dt.pos + this.Amatrix.Matr_x_Vect(localspeed) +
                   ((coord_dt.rot % new ChQuaternion(0, localpos) % this.coord.rot.GetConjugate()).GetVector() * 2);
        }

        /// Given the position of a point in local frame coords, and
        /// assuming it is sticky to frame, return the acceleration in parent coords.
        public ChVector PointAccelerationLocalToParent(ChVector localpos)
        {
            return coord_dtdt.pos +
                   ((coord_dtdt.rot % new ChQuaternion(0, localpos) % this.coord.rot.GetConjugate()).GetVector() * 2) +
                   ((coord_dt.rot % new ChQuaternion(0, localpos) % coord_dt.rot.GetConjugate()).GetVector() * 2);
        }

        /// Given the position of a point in local frame coords, and
        /// assuming it has a frame-relative speed localspeed and frame-relative
        /// acceleration localacc, return the acceleration in parent coords.
        public ChVector PointAccelerationLocalToParent(ChVector localpos,
                                                  ChVector localspeed,
                                                  ChVector localacc) {

            return coord_dtdt.pos + this.Amatrix.Matr_x_Vect(localacc) +
                   ((coord_dtdt.rot % new ChQuaternion(0, localpos) % this.coord.rot.GetConjugate()).GetVector() * 2) +
                   ((coord_dt.rot % new ChQuaternion(0, localpos) % coord_dt.rot.GetConjugate()).GetVector() * 2) +
                   ((coord_dt.rot % new ChQuaternion(0, localspeed) % this.coord.rot.GetConjugate()).GetVector() * 4);
        }

        /// Given the position of a point in parent frame coords, and
        /// assuming it has an absolute speed parentspeed,
        /// return the speed in local coords.
        public ChVector PointSpeedParentToLocal(ChVector parentpos, ChVector parentspeed) {
            ChFrame<Real> f = new ChFrame<Real>();
            ChVector localpos = f.TransformParentToLocal(parentpos);

            return this.Amatrix.MatrT_x_Vect(
                parentspeed - coord_dt.pos -
                ((coord_dt.rot % new ChQuaternion(0, localpos) % this.coord.rot.GetConjugate()).GetVector() * 2));
        }

        /// Given the position of a point in parent frame coords, and
        /// assuming it has an absolute speed parentspeed and absolute
        /// acceleration parentacc, return the acceleration in local coords.
        public ChVector PointAccelerationParentToLocal(ChVector parentpos,
                                                  ChVector parentspeed,
                                                  ChVector parentacc) {
            ChFrame<Real> f = this;// new ChFrame<Real>();
            ChVector localpos = f.TransformParentToLocal(parentpos);
            ChVector localspeed = PointSpeedParentToLocal(parentpos, parentspeed);

            return this.Amatrix.MatrT_x_Vect(
                parentacc - coord_dtdt.pos -
                (coord_dtdt.rot % new ChQuaternion(0, localpos) % this.coord.rot.GetConjugate()).GetVector() * 2 -
                (coord_dt.rot % new ChQuaternion(0, localpos) % coord_dt.rot.GetConjugate()).GetVector() * 2 -
                (coord_dt.rot % new ChQuaternion(0, localspeed) % this.coord.rot.GetConjugate()).GetVector() * 4);
        }

        /// This function transforms a frame from 'this' local coordinate
        /// system to parent frame coordinate system, and also transforms the speed
        /// and acceleration of the frame.
        public void TransformLocalToParent(
            ChFrameMoving<Real> local,  //< frame to transform, given in local frame coordinates
            ref ChFrameMoving<Real> parent        //< transformed frame, in parent coordinates, will be stored here
            ) {
            // pos & rot
            //ChFrame<Real> f = this;
            TransformLocalToParent(local, parent); // Problem here?

            // pos_dt
            parent.coord_dt.pos = PointSpeedLocalToParent(local.coord.pos, local.coord_dt.pos);

            // pos_dtdt
            parent.coord_dtdt.pos =
                    PointAccelerationLocalToParent(local.coord.pos, local.coord_dt.pos, local.coord_dtdt.pos);

            // rot_dt
            parent.coord_dt.rot = coord_dt.rot % local.coord.rot + this.coord.rot % local.coord_dt.rot;

            // rot_dtdt
            parent.coord_dtdt.rot = coord_dtdt.rot % local.coord.rot + (coord_dt.rot % local.coord_dt.rot) * 2 +
                                    this.coord.rot % local.coord_dtdt.rot;
        }

        /// This function transforms a frame from the parent coordinate
        /// system to 'this' local frame coordinate system.
        public void TransformParentToLocal(
        ChFrameMoving<Real> parent,  //< frame to transform, given in parent coordinates
        //ChFrameMoving<Real> local          //< transformed frame, in local coordinates, will be stored here
        ChFrameMoving<Real> local          //< transformed frame, in local coordinates, will be stored here
        ) {
            // pos & rot
            // ChFrame<Real> f = this;// new ChFrame<Real>();

            TransformParentToLocal2(parent, local);

            // pos_dt
            local.coord_dt.pos = PointSpeedParentToLocal(parent.coord.pos, parent.coord_dt.pos);

            // pos_dtdt
            local.coord_dtdt.pos =
                PointAccelerationParentToLocal(parent.coord.pos, parent.coord_dt.pos, parent.coord_dtdt.pos);

            // rot_dt
            local.coord_dt.rot = this.coord.rot.GetConjugate() % (parent.coord_dt.rot - coord_dt.rot % local.coord.rot);

            // rot_dtdt
            local.coord_dtdt.rot =
                this.coord.rot.GetConjugate() %
                (parent.coord_dtdt.rot - coord_dtdt.rot % local.coord.rot - (coord_dt.rot % local.coord_dt.rot) * 2);
        }

        // OTHER FUNCTIONS

        /// Returns true if coordsys is identical to other coordsys
        public bool Equals(ChFrameMoving<Real> other) {
            return this.coord.Equals(other.coord) && coord_dt.Equals(other.coord_dt) &&
                   coord_dtdt.Equals(other.coord_dtdt);
        }

        /// Returns true if coordsys is equal to other coordsys, within a tolerance 'tol'
        public bool Equals(ChFrameMoving<Real> other, double tol) {
            return this.coord.Equals(other.coord, tol) && coord_dt.Equals(other.coord_dt, tol) &&
                   coord_dtdt.Equals(other.coord_dtdt, tol);
        }

        /// The transformation (also for speeds, accelerations) is
        /// inverted in place.
        /// That is if w=A*v, then A.Invert();v=A*w;
        public override void Invert() {
            ChFrameMoving<Real> tmp;// = new ChFrameMoving<Real>();
            ChFrameMoving<Real> unit = new ChFrameMoving<Real>();
            tmp = this;
            tmp.TransformParentToLocal(unit, this);
        }

        public ChFrameMoving<Real> GetInverse() {
            ChFrameMoving<Real> tmp = new ChFrameMoving<Real>(this);
            tmp.Invert();
            return tmp;
        }
    }
}
