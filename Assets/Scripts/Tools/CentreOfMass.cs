using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CentreOfMass : MonoBehaviour
{
    public Vector3 com = new Vector3(0.5f, 0.0f, 0.0f);
    public Rigidbody rb;

    // Start is called before the first frame update
    void Start()
    {
        rb = GetComponent<Rigidbody>();
        rb.centerOfMass = com;
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
