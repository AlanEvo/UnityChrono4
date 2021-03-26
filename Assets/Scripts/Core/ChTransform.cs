using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

namespace chrono
{

    /// ChTransform: a class for fast coordinate transformations
    /// in 3D space.
    ///
    ///  A coordinate system (a 'frame') has a translation and
    /// a rotation respect to a 'parent' coordinate system,
    /// usually the absolute (world) coordinates.
    ///  This class implements useful static functions
    /// to perform the typical local->parent or parent->local
    /// transformations of points. Such functions are static,
    /// so you don't even need to instantiate a ChTransform object,
    /// you can just call functions in this way:
    ///   ChTransform<>::SomeFunction(..)
    ///
    public class ChTransform<Real> where Real : unmanaged, IConvertible
    {
        //
        // STATIC FUNCTIONS
        //

        // TRANSFORMATIONS, USING POSITION AND ROTATION MATRIX [A]

        /// This function transforms a point from the parent coordinate
        /// system to a local coordinate system, whose relative position
        /// is given by the 'origin' translation and 'alignment' rotation matrix.
        ///  Since the function is static, you do not need a ChTransform object, for example
        /// use it as: mresult=ChTransform<>::TransformParentToLocal(mpar, morig, malign)
        ///  This function is optimized for fast execution.
        /// \return The point in local coordinate, as local=[A]'*(parent-origin)
        public static ChVector TransformParentToLocal(
            ChVector parent,  //< point to transform, given in parent coordinates;
            ChVector origin,  //< location of local frame with respect to parent, expressed in parent ref frame;
            ChMatrix33<Real> alignment  //< rotation of local frame with respect to parent, expressed in parent coords.
        )
        {
            double mx = parent.x - origin.x;
            double my = parent.y - origin.y;
            double mz = parent.z - origin.z;

            return new ChVector(((alignment.Get33Element(0, 0)) * mx) + ((alignment.Get33Element(1, 0)) * my) +
                                ((alignment.Get33Element(2, 0)) * mz),
                                ((alignment.Get33Element(0, 1)) * mx) + ((alignment.Get33Element(1, 1)) * my) +
                                ((alignment.Get33Element(2, 1)) * mz),
                                ((alignment.Get33Element(0, 2)) * mx) + ((alignment.Get33Element(1, 2)) * my) +
                                ((alignment.Get33Element(2, 2)) * mz));
        }

        /// This function transforms a point from the local reference
        /// frame to the parent reference frame. The relative attitude of
        /// the local reference frame with respect to the parent reference frame
        /// is given by the 'origin' translation and the 'alignment' rotation matrix.
        /// Since the function is static, you do not need a ChTransform object. For example,
        /// use it as: mresult=ChTransform<>::TransformLocalToParent(mloc, morig, malign).
        /// This function is optimized for fast execution.
        /// \return The point in the parent reference frame, as parent = origin + [A]*(local)
        public static ChVector TransformLocalToParent(
            ChVector local,       //< point to transform, given in local coordinates
            ChVector origin,      //< origin of frame respect to parent, in parent coords,
            ChMatrix33<Real> alignment  //< rotation of frame respect to parent, in parent coords.
        )
        {
            return new ChVector(((alignment.Get33Element(0, 0)) * local.x) + ((alignment.Get33Element(0, 1)) * local.y) +
                            ((alignment.Get33Element(0, 2)) * local.z) + origin.x,
                        ((alignment.Get33Element(1, 0)) * local.x) + ((alignment.Get33Element(1, 1)) * local.y) +
                            ((alignment.Get33Element(1, 2)) * local.z) + origin.y,
                        ((alignment.Get33Element(2, 0)) * local.x) + ((alignment.Get33Element(2, 1)) * local.y) +
                            ((alignment.Get33Element(2, 2)) * local.z) + origin.z);
        }

        // TRANSFORMATIONS, USING POSITION AND ROTATION QUATERNION

        /// This function transforms a point from the parent coordinate
        /// system to a local coordinate system, whose relative position
        /// is given by the 'origin' translation and 'alignment' quaternion q.
        ///  Since the function is static, you do not need a ChTransform object, for example
        /// use it as: mresult=ChTransform<>::TransformParentToLocal(mpar, morig, malign)
        /// \return The point in local coordinate, as local=q*[(parent-origin)]*q

        public static ChVector TransformParentToLocal(
            ChVector parent,        //< point to transform, given in parent coordinates
            ChVector origin,        //< origin of frame respect to parent, in parent coords,
            ChQuaternion alignment  //< rotation of frame respect to parent, in parent coords.
        ) 
        {            
            // It could be simply "return alignment.RotateBack(parent-origin);"
            // but for faster execution do this:
            double e0e0 = alignment.e0 * alignment.e0;
            double e1e1 = alignment.e1 * alignment.e1;
            double e2e2 = alignment.e2 * alignment.e2;
            double e3e3 = alignment.e3 * alignment.e3;
            double e0e1 = -alignment.e0 * alignment.e1;
            double e0e2 = -alignment.e0 * alignment.e2;
            double e0e3 = -alignment.e0 * alignment.e3;
            double e1e2 = alignment.e1 * alignment.e2;
            double e1e3 = alignment.e1 * alignment.e3;
            double e2e3 = alignment.e2 * alignment.e3;

            double dx = parent.x - origin.x;
            double dy = parent.y - origin.y;
            double dz = parent.z - origin.z;
            return new ChVector(((e0e0 + e1e1) * 2.0 - 1.0) * dx + ((e1e2 - e0e3) * 2.0) * dy + ((e1e3 + e0e2) * 2.0) * dz,
                                  ((e1e2 + e0e3) * 2.0) * dx + ((e0e0 + e2e2) * 2.0 - 1.0) * dy + ((e2e3 - e0e1) * 2.0) * dz,
                                  ((e1e3 - e0e2) * 2.0) * dx + ((e2e3 + e0e1) * 2.0) * dy + ((e0e0 + e3e3) * 2.0 - 1.0) * dz);
        }
    }
}
