using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ParticlePlaying : MonoBehaviour {
    private ParticleSystem ps;
    public bool playing = false;

	// Use this for initialization
	void Start () {
        ps = GetComponent<ParticleSystem>();
	}
	
	// Update is called once per frame
	void Update () {
		if(ps.isPlaying)
        {
            playing = true;
            //Debug.Log("Particle Playing");

        }
        else
        {
            playing = false;
        }
	}
}
