using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono
{

    /// Namespace for classes which represent basic geometric objects
    namespace geometry
    {

        /// @addtogroup chrono_geometry
        /// @{

        /// Base class for geometric objects used for collisions and visualization.
        public abstract class ChGeometry
        {

            /// Enumeration of geometric objects
            public enum GeometryType
            {
                NONE,
                SPHERE,
                BOX,
                CYLINDER,
                TRIANGLE,
                CAPSULE,
                CONE,
                LINE,
                LINE_ARC,
                LINE_BEZIER,
                LINE_CAM,
                LINE_PATH,
                LINE_POLY,
                LINE_SEGMENT,
                ROUNDED_BOX,
                ROUNDED_CYLINDER,
                ROUNDED_CONE,
                TRIANGLEMESH,
                TRIANGLEMESH_CONNECTED,
                TRIANGLEMESH_SOUP
            };


            public ChGeometry() { }
            public ChGeometry(ChGeometry source) { }


            /// "Virtual" copy constructor.
            public abstract ChGeometry Clone();

            /// Get the class type as unique numerical ID (faster
            /// than using ChronoRTTI mechanism).
            /// Each inherited class must return an unique ID.
            public virtual GeometryType GetClassType() { return GeometryType.NONE; }

            /// Compute bounding box.
            /// If a matrix Rot is not null, it should compute bounding box along
            /// the rotated directions represented by that transformation matrix Rot.
            /// It must be overridden by inherited classes.
            public virtual void GetBoundingBox(ref double xmin,
                                ref double xmax,
                                ref double ymin,
                                ref double ymax,
                                ref double zmin,
                                ref double zmax,
                                ChMatrix33<double> Rot)
            {
                xmin = xmax = ymin = ymax = zmin = zmax = 0.0;
            }

            /// Enlarge a previous existing bounding box.
            /// Usually it does not need to be overridden: base function uses GetBoundingBox()
            /// If Rot is not null, the bounding box axes are considered rotated.
            public virtual void InflateBoundingBox(ref double xmin,
                                ref double xmax,
                                ref double ymin,
                                ref double ymax,
                                ref double zmin,
                                ref double zmax,
                                ChMatrix33<double> Rot)
            {

            }

            /// Returns the radius of the sphere which can enclose the geometry
            public virtual double Size()
            {
                return 0;
            }

            /// Compute center of mass
            /// It should be overridden by inherited classes
            public virtual ChVector Baricenter() { return ChVector.VNULL; }

            /// Compute the 3x3 covariance matrix (only the diagonal and upper part)
            /// It should be overridden by inherited classes
            // TODO: obsolete (unused)
            public virtual void CovarianceMatrix(ChMatrix33<double> C) { C.nm.matrix.Reset(); }

            /// Tells the dimension of the geometry
            /// (0=point, 1=line, 2=surface, 3=solid)
            public virtual int GetManifoldDimension() { return 0; }

            /// Generic update of internal data. Default, does nothing.
            /// Most often it is not needed to implement this method, but
            /// some inherited classes may implement it (ex. to update references to
            /// external data. etc.).
            public virtual void Update() { }
        }
    }
}
