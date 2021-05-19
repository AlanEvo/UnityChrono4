using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using chrono;


public class TestScene : MonoBehaviour
{
    // Draws a line from "startVertex" var to the curent mouse position.
    public Material mat;
    Vector3 startVertex;
    Vector3 mousePos;
    Camera cam;

    void Start()
    {
        startVertex = Vector3.zero;
        cam = Camera.main;

        DrawLines(cam);
    }


    void Update()
    {
        mousePos = Input.mousePosition;
        // Press space to update startVertex
        if (Input.GetKeyDown(KeyCode.Space))
        {
            startVertex = new Vector3(mousePos.x / Screen.width, mousePos.y / Screen.height, 0);
        }
    }

    public void DrawLines(Camera cam)
    {

        GL.PushMatrix();
        Matrix4x4 projection = GL.GetGPUProjectionMatrix(cam.projectionMatrix, false); //passing the camera's projection matrix through this function doesn't seem to do anything.
        GL.LoadProjectionMatrix(projection);
        GL.LoadIdentity();
        GL.MultMatrix(transform.localToWorldMatrix);
        GL.MultMatrix(cam.worldToCameraMatrix);
        Debug.Log("drawing lines");

        mat.SetPass(0);
        GL.Begin(GL.LINES);
        GL.Color(Color.red);
        GL.Vertex(startVertex);
        GL.Vertex(new Vector3(mousePos.x / Screen.width, mousePos.y / Screen.height, 0));
        GL.End();

        GL.PopMatrix();
        
    }

      
}
