using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
using System.Runtime.InteropServices;

namespace chrono
{
    namespace geometry
    {

        /// Base class for triangle meshes.
        public abstract class ChTriangleMesh : ChGeometry {

            public ChTriangleMesh() { }

            /// Add a triangle to this triangle mesh, by specifying the three coordinates
            public abstract void addTriangle(ChVector vertex0, ChVector vertex1, ChVector vertex2);

            /// Add a triangle to this triangle mesh, by specifying a ChTriangle
            public abstract void addTriangle(ChTriangle atriangle);

            /// Get the number of triangles already added to this mesh
            public abstract int getNumTriangles();

            /// Get the n-th triangle in mesh
            public abstract ChTriangle getTriangle(int index);

            /// Clear all data
            public abstract void Clear();

            /// Transform all vertexes, by displacing and rotating (rotation  via matrix, so also scaling if needed)
            public abstract void Transform(ChVector displ, ChMatrix33<double> rotscale);

            /// Transform all vertexes, by displacing and rotating (rotation  via matrix, so also scaling if needed)
            public virtual void Transform(ChVector displ, ChQuaternion mquat)
            {
                this.Transform(displ, new ChMatrix33<double>(mquat));
            }

            public override GeometryType GetClassType() { return GeometryType.TRIANGLEMESH; }

            public override void GetBoundingBox(ref double xmin,
                                        ref double xmax,
                                        ref double ymin,
                                        ref double ymax,
                                        ref double zmin,
                                        ref double zmax,
                                        ChMatrix33<double> Rot) {
                //// TODO
            }

            //// TODO
            //// virtual ChVector<> Baricenter() const override;

            public virtual void CovarianceMatrix(ChMatrix33<double> C) {
                //// TODO
            }

            /// This is a surface
            public virtual int GetManifoldDimension() { return 2; }

        }
    }
}

