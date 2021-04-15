using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DrawEllipse : MonoBehaviour
{


    // To show the lines in the game window whne it is running
    void OnPostRender()
    {
       // DrawConnectingLines();
    }

    // To show the lines in the editor
    void OnDrawGizmos()
    {
       // DrawConnectingLines();
    }
}
