using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
using System.Runtime.InteropServices;

namespace chrono
{
    namespace geometry
    {

        public class ChTriangleMeshConnected : ChTriangleMesh
        {
            [DllImport("ChronoEngine", EntryPoint = "ChTriangleMeshConnected_Create")]
            protected static extern IntPtr ChTriangleMeshConnected_Create(out IntPtr tmesh);
            [DllImport("ChronoEngine", EntryPoint = "ChTriangleMeshConnected_LoadWavefrontMesh")]
            public static extern void ChTriangleMeshConnected_LoadWavefrontMesh(IntPtr tmesh, string filename, bool load_normals, bool load_uv);
            [DllImport("ChronoEngine", EntryPoint = "ChTriangleMeshConnected_ComputeMassProperties")]
            public static extern void ChTriangleMeshConnected_ComputeMassProperties(IntPtr tmesh, bool coords, double mass, ref IntPtr center, ref IntPtr inertia);



            public ChTriangleMeshConnected() {

                m_ChTriangleMesh = ChTriangleMeshConnected_Create(out m_ChTriangleMesh);
                m_system = GameObject.FindObjectOfType<ChSystem>();
            }

            /// Load a triangle mesh saved as a Wavefront .obj file
            public void LoadWavefrontMesh(string filename, bool load_normals = true, bool load_uv = false)
            {
                ChTriangleMeshConnected_LoadWavefrontMesh(m_ChTriangleMesh, filename, load_normals, load_uv);
            }

            /// Compute barycenter, mass, inertia tensor
          /*  public void ComputeMassProperties(bool bodyCoords, double mass, ref ChVector center, ref ChMatrix33 inertia) {
                ChTriangleMeshConnected_ComputeMassProperties(m_ChTriangleMesh, bodyCoords, mass, ref center.m_ChVector, ref inertia.m_ChMatrix);
                center.GetX();
                center.GetY();
                center.GetZ();
            }*/

            /* // Use this for initialization
             void Start()
             {

             }

             // Update is called once per frame
             void Update()
             {

             }*/
        }
    }
}