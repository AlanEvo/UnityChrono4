using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using chrono;


public class BrakeShafts : MonoBehaviour
{
    public ChShaftsClutch m_clutch;             //< clutch between the brake shaft and axle shaft
    public double m_modulation;                       //< current braking input
    public bool m_locked;                             //< is brake locked?

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        m_modulation = Input.GetAxis("Brake");
        m_clutch.SetModulation(m_modulation);
    }
}
