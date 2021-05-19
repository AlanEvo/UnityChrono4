using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono
{

    public class TestClass : ChPhysicsItem
    {
        // Start is called before the first frame update
        void Start()
        {
            ChSystem msystem = FindObjectOfType<ChSystem>();
            msystem.Add(this);
        }

        // Update is called once per frame
        void Update()
        {

        }
    }

}
