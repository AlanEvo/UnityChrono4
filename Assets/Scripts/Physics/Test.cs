using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
//using UnityEditor;
using System.Runtime.InteropServices;

namespace chrono
{

    public class Test : MonoBehaviour
    {

        [SerializeField]
        public ChSystemNSC system = new ChSystemNSC();

        public double chronoTime = 0;

        // Use this for initialization
        void Start()
        {
            //system = new ChSystemNSC();

            //ChBody body = new ChBody();
            //system.AddFlappaBody(body.m_ChBody);
            //system.AddFlappaBody(body.m_ChBody);

        }

        // Update is called once per frame
        void Update()
        {
            while (chronoTime < 2.5)
            {
                chronoTime += 0.01;
                //system.DoFrameDynamics(chronoTime);
            }
        }
    }
}
