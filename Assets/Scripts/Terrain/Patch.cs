using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono
{

    public class Patch : MonoBehaviour
    {
        public enum Type { BOX, MESH, HEIGHT_MAP, UNITY_TERRAIN };
        public Texture2D texture;// = new Texture2D(250, 250);       

        public void Start()
        {
            switch (m_type)
            {
                case Type.UNITY_TERRAIN:
                    Terrain terrain = GetComponent<Terrain>();
                    Texture2D texture2D = new Texture2D(512, 512, TextureFormat.RGB24, false);

                    RenderTexture myRenderTexture = terrain.terrainData.heightmapTexture;  

                    RenderTexture.active = myRenderTexture;
                    texture2D.ReadPixels(new Rect(0, 0, myRenderTexture.width, myRenderTexture.height), 0, 0);
                    texture2D.Apply();
                    texture = texture2D;
                    break;
               
            }
        }

        /// Set coefficient of friction.
        /// The default value is 0.7
        public void SetContactFrictionCoefficient(float friction_coefficient) { }

        /// Set coefficient of restitution.
        /// The default value is 0.1
        public void SetContactRestitutionCoefficient(float restitution_coefficient) { }

        /// Set contact material properties.
        /// These values are used to calculate contact material coefficients (if the containing
        /// system is so configured and if the SMC contact method is being used).
        /// The default values are: Y = 2e5 and nu = 0.3
        public void SetContactMaterialProperties(float young_modulus,  ///< [in] Young's modulus of elasticity
                                          float poisson_ratio   ///< [in] Poisson ratio
        )
        {

        }

        /// Set contact material coefficients.
        /// These values are used directly to compute contact forces (if the containing system
        /// is so configured and if the SMC contact method is being used).
        /// The default values are: kn=2e5, gn=40, kt=2e5, gt=20
        public void SetContactMaterialCoefficients(float kn,  ///< [in] normal contact stiffness
                                            float gn,  ///< [in] normal contact damping
                                            float kt,  ///< [in] tangential contact stiffness
                                            float gt   ///< [in] tangential contact damping
        )
        {

        }

        /// Return a handle to the ground body.
        public ChBody GetGroundBody()
        {
            return m_body;
        }

        public Type m_type;
        private ChBody m_body;
        //std::string m_mesh_name;
        private float m_friction;
    };

}
