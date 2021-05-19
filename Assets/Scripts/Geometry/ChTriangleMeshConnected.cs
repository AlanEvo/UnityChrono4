using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;


namespace chrono
{
    namespace geometry
    {

        /// A triangle mesh with connectivity info: vertices can be
        /// shared between faces.

        public class ChTriangleMeshConnected : ChTriangleMesh
        {

            public List<ChVector> m_vertices = new List<ChVector>();
            public List<ChVector> m_normals = new List<ChVector>();
            public List<ChVector> m_UV = new List<ChVector>();
            public List<ChVector> m_colors = new List<ChVector>();

            public List<ChVector> m_face_v_indices = new List<ChVector>();
            public List<ChVector> m_face_n_indices = new List<ChVector>();
            public List<ChVector> m_face_uv_indices = new List<ChVector>();
            public List<ChVector> m_face_col_indices = new List<ChVector>();


            public ChTriangleMeshConnected() { }
            public ChTriangleMeshConnected(ChTriangleMeshConnected source)
            {

            }

            /// "Virtual" copy constructor (covariant return type).
            public override ChGeometry Clone()
            {
                return new ChTriangleMeshConnected(this);
            }

            public ref List<ChVector> getCoordsVertices() { return ref m_vertices; }
            public ref List<ChVector> getCoordsNormals() { return ref m_normals; }
            public ref List<ChVector> getCoordsUV() { return ref m_UV; }
            public ref List<ChVector> getCoordsColors() { return ref m_colors; }

            public ref List<ChVector> getIndicesVertexes() { return ref m_face_v_indices; }
            public ref List<ChVector> getIndicesNormals() { return ref m_face_n_indices; }
            public ref List<ChVector> getIndicesUV() { return ref m_face_uv_indices; }
            public ref List<ChVector> getIndicesColors() { return ref m_face_col_indices; }

            /// Load a triangle mesh saved as a Wavefront .obj file
            public void LoadWavefrontMesh(Mesh mesh, bool load_normals = true, bool load_uv = false)
            {
                this.m_vertices.Clear();
                this.m_normals.Clear();
                this.m_UV.Clear();
                this.m_face_v_indices.Clear();
                this.m_face_n_indices.Clear();
                this.m_face_uv_indices.Clear();

                /* GeometryInterface emptybm;  // BuildMesh bm;

                 m_filename = filename;

                 OBJ obj;*/

                // obj.LoadMesh(filename.c_str(), &emptybm, true);

                int[] triangles;
                List<int> indices = new List<int>();    //// not an array anymore

                triangles = mesh.triangles;

                for (int i = 0; i < triangles.Length; i += 3)
                {
                    indices.Add(triangles[i + 0]);
                    indices.Add(triangles[i + 1]);
                    indices.Add(triangles[i + 2]);
                }

                for (int iv = 0; iv < mesh.vertices.Length; iv += 3)
                {
                    this.m_vertices.Add(new ChVector(mesh.vertices[iv].x, mesh.vertices[iv + 1].y, mesh.vertices[iv + 2].z));
                }
                for (int i = 0; i < mesh.normals.Length; i += 3) {
                    this.m_normals.Add(new ChVector(mesh.normals[i].x, mesh.normals[i +1].y, mesh.normals[i +2].z));
                }
               /* for (unsigned int it = 0; it < obj.mTexels.size(); it += 2)  // +2 because only u,v each texel
                {
                    this.m_UV.push_back(ChVector<double>(obj.mTexels[it], obj.mTexels[it + 1], 0));
                }*/
                for (int iiv = 0; iiv < indices.Count; iiv += 3)
                {
                    this.m_face_v_indices.Add(
                        new ChVector(indices[iiv], indices[iiv + 1], indices[iiv + 2]));
                }
               /* for (unsigned int iin = 0; iin < obj.mIndexesNormals.size(); iin += 3)
                {
                    this.m_face_n_indices.push_back(
                        ChVector<int>(obj.mIndexesNormals[iin], obj.mIndexesNormals[iin + 1], obj.mIndexesNormals[iin + 2]));
                }*/
               /* for (unsigned int iit = 0; iit < obj.mIndexesTexels.size(); iit += 3)
                {
                    this.m_face_uv_indices.push_back(
                        ChVector<int>(obj.mIndexesTexels[iit], obj.mIndexesTexels[iit + 1], obj.mIndexesTexels[iit + 2]));
                }*/

                if (!load_normals)
                {
                    this.m_normals.Clear();
                    this.m_face_n_indices.Clear();
                }
                if (!load_uv)
                {
                    this.m_UV.Clear();
                    this.m_face_uv_indices.Clear();
                }
            }

            /// Transform all vertexes, by displacing and rotating (rotation  via matrix, so also scaling if needed)
            public override void Transform(ChVector displ, ChMatrix33<double> rotscale) { 
            
            }

            /// Add a triangle to this triangle mesh, by specifying the three coordinates.
            /// This is disconnected - no vertex sharing is used even if it could be..
            public override void addTriangle(ChVector vertex0, ChVector vertex1, ChVector vertex2)
            {
                int base_v = (int)m_vertices.Count;
                m_vertices.Add(vertex0);
                m_vertices.Add(vertex1);
                m_vertices.Add(vertex2);
                m_face_v_indices.Add(new ChVector(base_v, base_v + 1, base_v + 2));
            }

            /// Add a triangle to this triangle mesh, by specifying a ChTriangle
            public override void addTriangle(ChTriangle atriangle)
            {
                int base_v = (int)m_vertices.Count;
                m_vertices.Add(atriangle.p1);
                m_vertices.Add(atriangle.p2);
                m_vertices.Add(atriangle.p3);
                m_face_v_indices.Add(new ChVector(base_v, base_v + 1, base_v + 2));
            }

            /// Get the number of triangles already added to this mesh
            public override int getNumTriangles()
            {
                return (int)m_face_v_indices.Count;
            }

            /// Access the n-th triangle in mesh
            public override ChTriangle getTriangle(int index)
            {
                return new ChTriangle(m_vertices[(int)m_face_v_indices[index].x], m_vertices[(int)m_face_v_indices[index].y],
                                  m_vertices[(int)m_face_v_indices[index].z]);
            }

            public override void Clear()
            {
                this.getCoordsVertices().Clear();
                this.getCoordsNormals().Clear();
                this.getCoordsUV().Clear();
                this.getCoordsColors().Clear();
                this.getIndicesVertexes().Clear();
                this.getIndicesNormals().Clear();
                this.getIndicesUV().Clear();
                this.getIndicesColors().Clear();
            }

    }
    }
}