using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TestSpeed : MonoBehaviour {
    public Vector3 position = new Vector3();
    private float forwardVelocity;
    private float forwardAcceleration;
    private Vector3 velocity;
    private Vector3 prevVelocity;
    private Vector3 acceleration;

    public float speedMPH;
    public float speedKPH;

    private Rigidbody rb;

    /// <summary>
    /// Velocity in forward direction in local coordinates (z-forward).
    /// </summary>
    public float ForwardVelocity
    {
        get
        {
            return forwardVelocity;
        }
    }

    /// <summary>
    /// Speed in kilometers per hour.
    /// </summary>
    public float SpeedKPH
    {
        get
        {
            return Speed * 3.6f;
        }
    }

    /// <summary>
    /// Speed is (US) miles per hour.
    /// </summary>
    public float SpeedMPH
    {
        get
        {
            return Speed * 2.237f;
        }
    }

    /// <summary>
    /// Speed in forward direction in local coordinates (z-forward). Always positive.
    /// For positive/negative version use ForwardVelocity.
    /// </summary>
    public float Speed
    {
        get
        {
            return Mathf.Abs(ForwardVelocity);
        }
    }

    // Use this for initialization
    void Start () {
        rb = GetComponent<Rigidbody>();
	}
	
	// Update is called once per frame
	void LateUpdate () {
        forwardVelocity = ((transform.position - prevVelocity).magnitude) / Time.deltaTime;
        prevVelocity = transform.position;

        speedMPH = SpeedMPH;
        speedKPH = SpeedKPH;
    }
}
