using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;
using chrono;


public class BallGenerator : MonoBehaviour
{
    public Material ballMaterial;
    public bool drawBallFrames = false;
    public int ballCount;
    float density = 1000.0f;

    void CreateBall(Object prefab,
                    Vector3 pos
                    )
    {
        // Create a new clone of the BodyConvexHull prefab
        GameObject clone = Instantiate(prefab, Vector3.zero, UnityEngine.Quaternion.identity) as GameObject;

        // Get the UChBodyConvexHull component and set its properties...
        var body = clone.GetComponent<ChBody>();
        //body.radius = radius;
        body.density = density;
       // body.showFrameGizmo = drawBallFrames;

        // Set the material for the underlying MeshRenderer
        if (ballMaterial != null)
        {
            clone.GetComponent<MeshRenderer>().material = ballMaterial;
        }

        // Set the position and scale of the new clone...
        clone.transform.position = pos;
       // clone.transform.localScale = new Vector3(2 * radius, 2 * radius, 2 * radius);

        // Recreate the body...
        // Note: the creation of the clone (Instantiate above) invoked UChBody.Awake which
        // created a ChBody with default parameters. So we must delete the ChBody and invoke
        // UChBody.Awake again to create a body with the new settings.
       // body.Destroy();
        body.Awake();
    }

    void Start()
    {
      //  Object prefab = AssetDatabase.LoadAssetAtPath("Assets/Prefabs/Sphere.prefab", typeof(GameObject));

        //float radius = 0.8f;
        

       /* for (int i = 0; i < ballCount; i++)
        {
            Vector3 pos = new Vector3(-5 + Random.Range(0, 10), 4 + i * 0.05f, -5 + Random.Range(0, 10));
            CreateBall(prefab, pos);
        }*/
    }
}