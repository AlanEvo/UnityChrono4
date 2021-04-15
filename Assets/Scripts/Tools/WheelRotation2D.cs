using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class WheelRotation2D : MonoBehaviour {

	// Use this for initialization
	void Start () {
		
	}
	
	// Update is called once per frame
	void Update () {
		float lockPos = 0;
		transform.rotation = Quaternion.Euler(lockPos, lockPos, transform.eulerAngles.z);
	}
}
