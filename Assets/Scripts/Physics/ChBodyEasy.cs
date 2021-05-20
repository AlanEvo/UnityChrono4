using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;

namespace chrono
{
    public class ChBodyEasy : ChBody
    {
       // public enum CollisionType { Cube, Sphere, Cylinder }
        //public CollisionType collisionType = CollisionType.Cube;

        //public Vector3 SetRotationForce;
        public Vector3 SetLinearForce;
        

        public ChBodyEasy()
            : base()
        {
           
        }

        // Use this for initialization
     /*   public new void Start()
        {
            base.Start();

            switch (collisionType)
            {
                case CollisionType.Cube:

                    double Xsize;
                    double Ysize;
                    double Zsize;
                    Xsize = transform.localScale.x;
                    Ysize = transform.localScale.y;
                    Zsize = transform.localScale.z;
                    double mmass = mdensity * (Xsize * Ysize * Zsize);
                    this.SetDensity((double)mdensity);
                    this.SetMass(mmass);
                    this.SetInertiaXX(new ChVector((1.0f / 12.0f) * (double)mmass * (Math.Pow(Ysize, 2) + Math.Pow(Zsize, 2)),
                                                  (1.0f / 12.0f) * (double)mmass * (Math.Pow(Xsize, 2) + Math.Pow(Zsize, 2)),
                                                  (1.0f / 12.0f) * (double)mmass * (Math.Pow(Xsize, 2) + Math.Pow(Ysize, 2))));

                    if (collide)
                    {
                        GetCollisionModel().ClearModel();
                        GetCollisionModel().AddBox(Xsize * 0.5f, Ysize * 0.5f, Zsize * 0.5f);  // radius x, radius z, height on y
                        GetCollisionModel().BuildModel();
                        SetCollide(true);
                    }
                    break;
                case CollisionType.Sphere:

                    //double radius = gameObject.GetComponent<Renderer>().bounds.extents.magnitude;
                    double radius = transform.localScale.x / 2.2;
                    double mmass2 = mdensity * ((4.0f / 3.0f) * (double)ChMaths.CH_C_PI * Math.Pow(radius, 3));
                    double inertia = (2.0f / 5.0f) * mmass2 * Math.Pow(radius, 2);

                    this.SetDensity((double)mdensity);
                    this.SetMass(mmass2);
                    this.SetInertiaXX(new ChVector(inertia, inertia, inertia));

                    if (collide)
                    {
                        GetCollisionModel().ClearModel();
                        GetCollisionModel().AddSphere(radius);  // radius, radius, height on y
                        GetCollisionModel().BuildModel();
                        SetCollide(true);
                    }
                    break;
                case CollisionType.Cylinder:

                    double height = transform.localScale.y;
                    double radius2 = transform.localScale.x / 2;
                    double mmass3 = mdensity * ((double)ChMaths.CH_C_PI * Math.Pow(radius2, 2) * height);

                    this.SetDensity((double)mdensity);
                    this.SetMass(mmass3);
                    this.SetInertiaXX(new ChVector((1.0f / 12.0f) * mmass3 * (3 * Math.Pow(radius2, 2) + Math.Pow(height, 2)),
                                                  0.5f * mmass3 * Math.Pow(radius2, 2),
                                                  (1.0f / 12.0f) * mmass3 * (3 * Math.Pow(radius2, 2) + Math.Pow(height, 2))));
                    if (collide)
                    {
                        GetCollisionModel().ClearModel();
                        GetCollisionModel().AddCylinder(radius2, radius2, height * 1.0f);  // radius x, radius z, height on y
                        GetCollisionModel().BuildModel();
                        SetCollide(true);
                    }
                    break;
            }

            if (mass > 0)
            {
                //this.SetMass(mass);
            }       

            

           // SetWvel_par(new ChVector(SetRotationForce.x, SetRotationForce.y, SetRotationForce.z));
            SetPos_dt(new ChVector(SetLinearForce.x, SetLinearForce.y, SetLinearForce.z));
        }

        // Update is called once per frame
      /*  public override void FixedUpdate()
        {
            base.FixedUpdate();
        }*/
    }
}
