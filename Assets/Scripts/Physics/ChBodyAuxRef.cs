using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
using System.Security.Permissions;


namespace chrono
{
    /// Class for rigid bodies with an auxiliary reference frame.
    /// Unlike ChBody, where the COG frame is used as the reference frame, the
    /// auxiliary reference frame of a ChBodyAuxRef can be different from its
    /// COG frame.  This specialization is provided for situations where it is more
    /// convenient to specify collision shapes, visualization assets, and marker
    /// positions with respect to a reference frame other than the COG frame.
    /// Note that, because of the auxiliary reference, this type of rigid bodies
    /// can be slightly less efficient than the base ChBody object.
    ///
    /// Additional information can be found in the @ref rigid_bodies manual page.
    public class ChBodyAuxRef : ChBody
    {
        private ChFrame<double> auxref_to_cog; //< auxiliary REF location, relative to COG
        private ChFrame<double> auxref_to_abs; //< auxiliary REF location, relative to abs coords (needs Update() )
        

        public Vector3 COM;

        public ChBodyAuxRef() :base()
        {
        }

        public ChBodyAuxRef(ChMaterialSurface.ContactMethod contact_method = ChMaterialSurface.ContactMethod.NSC)
        : base(contact_method) {
        }
        public ChBodyAuxRef(collision.ChCollisionModel new_coll_model,
                     ChMaterialSurface.ContactMethod contact_method = ChMaterialSurface.ContactMethod.NSC)
            : base(new_coll_model, contact_method) { }
        public ChBodyAuxRef(ChBodyAuxRef other) : base(other) {

        }

        /// "Virtual" copy constructor (covariant return type).
        public override ChObj Clone()
        {
            return new ChBodyAuxRef(this);
        }

        public override void Start()
        {
            auxref_to_cog = new ChFrameMoving<double>();
            auxref_to_abs = new ChFrameMoving<double>();

            switch (materialType)
            {
                case MaterialType.NSC:
                    matsurface = gameObject.AddComponent<ChMaterialSurfaceNSC>();
                    matsurface.GetComponent<ChMaterialSurfaceNSC>().static_friction = friction;
                    matsurface.GetComponent<ChMaterialSurfaceNSC>().sliding_friction = friction;
                    matsurface.GetComponent<ChMaterialSurfaceNSC>().rolling_friction = rolling_friction;
                    matsurface.GetComponent<ChMaterialSurfaceNSC>().spinning_friction = spinning_friction;
                    matsurface.GetComponent<ChMaterialSurfaceNSC>().restitution = restitution;
                    matsurface.GetComponent<ChMaterialSurfaceNSC>().cohesion = cohesion;
                    matsurface.GetComponent<ChMaterialSurfaceNSC>().dampingf = dampingf;
                    matsurface.GetComponent<ChMaterialSurfaceNSC>().compliance = compliance;
                    matsurface.GetComponent<ChMaterialSurfaceNSC>().complianceT = complianceT;
                    matsurface.GetComponent<ChMaterialSurfaceNSC>().complianceRoll = complianceRoll;
                    matsurface.GetComponent<ChMaterialSurfaceNSC>().complianceSpin = complianceSpin;
                    break;
                case MaterialType.SMC:
                    matsurface = gameObject.AddComponent<ChMaterialSurfaceSMC>();
                    matsurface.GetComponent<ChMaterialSurfaceSMC>().static_friction = friction;
                    matsurface.GetComponent<ChMaterialSurfaceSMC>().sliding_friction = friction;
                    matsurface.GetComponent<ChMaterialSurfaceSMC>().restitution = restitution;
                    // matsurface.GetComponent<ChMaterialSurfaceSMC>().rolling_friction = rolling_friction;
                    // matsurface.GetComponent<ChMaterialSurfaceSMC>().spinning_friction = spinning_friction;
                    matsurface.GetComponent<ChMaterialSurfaceSMC>().young_modulus = young_modulus;
                    matsurface.GetComponent<ChMaterialSurfaceSMC>().poisson_ratio = poisson_ratio;
                    matsurface.GetComponent<ChMaterialSurfaceSMC>().constant_adhesion = constant_adhesion;
                    matsurface.GetComponent<ChMaterialSurfaceSMC>().adhesionMultDMT = adhesionMultDMT;
                    matsurface.GetComponent<ChMaterialSurfaceSMC>().kn = kn;
                    matsurface.GetComponent<ChMaterialSurfaceSMC>().kt = kt;
                    matsurface.GetComponent<ChMaterialSurfaceSMC>().gn = gn;
                    matsurface.GetComponent<ChMaterialSurfaceSMC>().gt = gt;
                    break;
            }

            // Switch
            switch (type)
            {
                case CollisionType.Cube:

                    var size = transform.localScale * 1f;

                    if (automaticMass)
                    {                        
                        mass = density * (size.x * size.y * size.z);
                        this.SetDensity((float)density);
                        this.SetMass(mass);
                        inertiaMoments.x = (float)((1.0 / 12.0) * mass * (Math.Pow(size.y, 2) + Math.Pow(size.z, 2)));
                        inertiaMoments.y = (float)((1.0 / 12.0) * mass * (Math.Pow(size.x, 2) + Math.Pow(size.z, 2)));
                        inertiaMoments.z = (float)((1.0 / 12.0) * mass * (Math.Pow(size.x, 2) + Math.Pow(size.y, 2)));
                    }

                    if (collide)
                    {
                        GetCollisionModel().ClearModel();
                        GetCollisionModel().AddBox(size.x * 0.473f, size.y * 0.473f, size.z * 0.473f, new ChVector(0, 0, 0), new ChMatrix33<double>(1));  // radius x, radius z, height on y
                        GetCollisionModel().BuildModel();
                        SetCollide(true);
                    }

                    this.SetDensity((float)density);
                    this.SetMass(mass);

                    SetFrame_COG_to_REF(new ChFrame<double>(Utils.ToChrono(COM), new ChQuaternion(1, 0, 0, 0)));
                    SetInertiaXX(ToChrono(inertiaMoments));
                    SetInertiaXY(ToChrono(inertiaProducts));

                    SetBodyFixed(bodyfixed);

                    SetFrame_REF_to_abs(new ChFrame<double>(Utils.ToChrono(transform.position), Utils.ToChrono(transform.rotation)));

                    BodyFrame.SetPos_dt(ToChrono(linearVelocity));
                    BodyFrame.SetWvel_loc(ToChrono(angularVelocity));

                    ChSystem.system.AddBody(this);

                    break;
                case CollisionType.Sphere:

                    var size2 = transform.localScale.y / 1.9;

                    if (automaticMass)
                    {
                        mass = density * ((4.0 / 3.0) * ChMaths.CH_C_PI * Math.Pow(size2, 3));
                        double inertia = (2.0 / 5.0) * mass * Math.Pow(size2, 2);
                        this.SetDensity((float)density);
                        this.SetMass(mass);
                        inertiaMoments.x = (float)inertia;
                        inertiaMoments.y = (float)inertia;
                        inertiaMoments.z = (float)inertia;
                    }

                    if (collide)
                    {
                        GetCollisionModel().ClearModel();
                        GetCollisionModel().AddSphere(size2, new ChVector(0, 0, 0));  // radius, radius, height on y
                        GetCollisionModel().BuildModel();
                        SetCollide(true);
                    }

                    this.SetDensity((float)density);
                    this.SetMass(mass);

                    SetFrame_COG_to_REF(new ChFrame<double>(Utils.ToChrono(COM), new ChQuaternion(1, 0, 0, 0)));
                    SetInertiaXX(ToChrono(inertiaMoments));
                    SetInertiaXY(ToChrono(inertiaProducts));

                    SetBodyFixed(bodyfixed);

                    SetFrame_REF_to_abs(new ChFrame<double>(Utils.ToChrono(transform.position), Utils.ToChrono(transform.rotation)));

                    BodyFrame.SetPos_dt(ToChrono(linearVelocity));
                    BodyFrame.SetWvel_loc(ToChrono(angularVelocity));

                    ChSystem.system.AddBody(this);

                    break;
                case CollisionType.Cylinder:

                    var height = 2 * transform.localScale.y;
                    var radiusX = transform.localScale.x / 2.0;
                    var radiusZ = transform.localScale.z / 2.0;
                    radius = transform.localScale.x / 2.0;

                    if (automaticMass)
                    {
                        mass = density * (ChMaths.CH_C_PI * Math.Pow(radiusX, 2) * height);
                        this.SetDensity((float)density);
                        this.SetMass(mass);
                        inertiaMoments.x = (float)((1.0 / 12.0) * mass * (3 * Math.Pow(radiusX, 2) + Math.Pow(height, 2)));
                        inertiaMoments.y = (float)(0.5 * mass * Math.Pow(radiusX, 2));
                        inertiaMoments.z = (float)((1.0 / 12.0) * mass * (3 * Math.Pow(radiusX, 2) + Math.Pow(height, 2)));
                    }

                    if (collide)
                    {
                        GetCollisionModel().ClearModel();
                        GetCollisionModel().AddCylinder(radiusX, radiusZ, height * 0.5f, new ChVector(0, 0, 0), new ChMatrix33<double>(1));  // radius, radius, height on y
                        GetCollisionModel().BuildModel();
                        SetCollide(true);
                    }

                    this.SetDensity((float)density);
                    this.SetMass(mass);

                    SetFrame_COG_to_REF(new ChFrame<double>(Utils.ToChrono(COM), new ChQuaternion(1, 0, 0, 0)));
                    SetInertiaXX(ToChrono(inertiaMoments));
                    SetInertiaXY(ToChrono(inertiaProducts));

                    SetBodyFixed(bodyfixed);

                    SetFrame_REF_to_abs(new ChFrame<double>(Utils.ToChrono(transform.position), Utils.ToChrono(transform.rotation)));

                    BodyFrame.SetPos_dt(ToChrono(linearVelocity));
                    BodyFrame.SetWvel_loc(ToChrono(angularVelocity));

                    ChSystem.system.AddBody(this);
                    break;
                case CollisionType.Mesh:

                    Mesh mesh = GetComponent<MeshFilter>().sharedMesh;
                    double sweep_sphere_radius = 0.1;

                    //  geometry.ChTriangleMeshConnected trimesh = new geometry.ChTriangleMeshConnected();
                    //  trimesh.LoadWavefrontMesh(mesh);

                    // Create the collision model

                    if (collide)
                    {
                        GetCollisionModel().ClearModel();
                        GetCollisionModel().AddTriangleMesh(mesh, true, false, ChVector.VNULL, new ChMatrix33<double>(1));
                        GetCollisionModel().BuildModel();
                        SetCollide(true);
                    }

                    this.SetDensity((float)density);
                    this.SetMass(mass);

                    SetInertiaXX(ToChrono(inertiaMoments));
                    SetInertiaXY(ToChrono(inertiaProducts));

                    SetBodyFixed(bodyfixed);

                    BodyFrame.SetPos(new ChVector(transform.position.x, transform.position.y, transform.position.z));
                    BodyFrame.SetRot(new ChQuaternion(transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z));

                    // BodyFrame.SetPos_dt(ToChrono(linearVelocity));
                    // BodyFrame.SetWvel_loc(ToChrono(angularVelocity));

                    // ChSystem msystem3 = FindObjectOfType<ChSystem>();
                    // msystem3.AddBody(this);
                    ChSystem.system.AddBody(this);
                    break;
                case CollisionType.Terrain:

                    //Texture2D hMap = Resources.Load("Heightmap") as Texture2D;

                    Terrain terrain = GetComponent<Terrain>();
                    Texture2D texture2D = new Texture2D(512, 512, TextureFormat.RGB24, false);

                    myRenderTexture = terrain.terrainData.heightmapTexture;

                    RenderTexture.active = myRenderTexture;
                    texture2D.ReadPixels(new Rect(0, 0, myRenderTexture.width, myRenderTexture.height), 0, 0);
                    texture2D.Apply();
                    texture = texture2D;

                    // List<Vector3> verts = new List<Vector3>();
                    //List<int> tris = new List<int>();

                    int dimensions = 65;

                    //Bottom left section of the map, other sections are similar
                    for (int i = 0; i < dimensions; i++)
                    {
                        for (int j = 0; j < dimensions; j++)
                        {
                            //Add each new vertex in the plane
                            verts.Add(new Vector3(i, texture.GetPixel(i, j).grayscale * 50, j));
                            //Skip if a new square on the plane hasn't been formed
                            if (i == 0 || j == 0) continue;
                            //Adds the index of the three vertices in order to make up each of the two tris
                            tris.Add(dimensions * i + j); //Top right
                            tris.Add(dimensions * i + j - 1); //Bottom right
                            tris.Add(dimensions * (i - 1) + j - 1); //Bottom left - First triangle
                            tris.Add(dimensions * (i - 1) + j - 1); //Bottom left 
                            tris.Add(dimensions * (i - 1) + j); //Top left
                            tris.Add(dimensions * i + j); //Top right - Second triangle
                        }
                    }

                    Mesh procMesh = new Mesh();
                    procMesh.vertices = verts.ToArray(); //Assign verts, uvs, and tris to the mesh
                                                         // procMesh.uv = uvs;
                    procMesh.triangles = tris.ToArray();
                    procMesh.RecalculateNormals(); //Determines which way the triangles are facing

                    if (collide)
                    {
                        GetCollisionModel().ClearModel();
                        GetCollisionModel().AddTriangleMesh(procMesh, true, false, new ChVector(0, 0, 0), new ChMatrix33<double>(1));
                        GetCollisionModel().BuildModel();
                        SetCollide(true);
                    }

                    this.SetDensity((float)density);
                    this.SetMass(mass);

                    SetInertiaXX(ToChrono(inertiaMoments));
                    SetInertiaXY(ToChrono(inertiaProducts));

                    SetBodyFixed(bodyfixed);

                    BodyFrame.SetPos(new ChVector(transform.position.x, transform.position.y, transform.position.z));
                    BodyFrame.SetRot(new ChQuaternion(transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z));

                    ChSystem.system.AddBody(this);
                    break;
            }

        }

        public override collision.ChCollisionModel GetCollisionModel() { return collision_model; }


        public override void FixedUpdate()
        {
            base.FixedUpdate();
        }

        public void OnDrawGizmos()
        {
            Gizmos.matrix = Matrix4x4.TRS(transform.position, transform.rotation, new Vector3(1, 1, 1));

            if (showFrameGizmo)
            {
                Gizmos.color = Color.yellow;
                Gizmos.DrawSphere(COM, 0.1f);
            }

        }

        /// Set the auxiliary reference frame with respect to the absolute frame.
        /// This moves the entire body; the body COG is rigidly moved as well.
        public void SetFrame_REF_to_abs(ChFrame<double> mfra)
        {
            // PROBLEM  need to find out why the GetInverse isn't working, I think it has something to do
            // with it not being const like the c++ version?  So the coord is being changed along the way?
            mfra.TransformLocalToParent(this.auxref_to_cog.GetInverse(), this.BodyFrame);
            // or, also, using overloaded operators for frames:
            //   *this = this->auxref_to_cog.GetInverse() >> mfra;

            auxref_to_abs = mfra;
        }

        /// Get the auxiliary reference frame with respect to the absolute frame.
        /// Note that, in general, this is different from GetFrame_COG_to_abs().
        public override ChFrame<double> GetFrame_REF_to_abs() { return auxref_to_abs; }

        /// Set the COG frame with respect to the auxiliary reference frame.
        /// Note that this also moves the body absolute COG (the REF is fixed).
        /// The position of contained ChMarker objects, if any, is not changed with respect
        /// to the reference.
        public void SetFrame_COG_to_REF(ChFrame<double> mloc) {
            // Problem here, old_cog_to_abs should get changed by this.
            ChFrameMoving<double> old_cog_to_abs = ChFrameMoving<double>.FMNULL;// new ChFrameMoving<double>();//= this.BodyFrame;  

            ChFrameMoving<double> tmpref_to_abs = ChFrameMoving<double>.FMNULL;//new ChFrameMoving<double>();
            this.BodyFrame.TransformLocalToParent(this.auxref_to_cog, tmpref_to_abs);
            tmpref_to_abs.TransformLocalToParent(new ChFrameMoving<double>(mloc), this.BodyFrame); // Gets changed here.

            // or, also, using overloaded operators for frames:
            //   tmpref_to_abs = auxref_to_cog >> *this;
            //   *this         = ChFrameMoving<>(mloc) >> tmpref_to_abs;

            ChFrameMoving<double> new_cog_to_abs = this.BodyFrame;

            auxref_to_cog = mloc.GetInverse();
            this.BodyFrame.TransformLocalToParent(this.auxref_to_cog, this.auxref_to_abs);
            // or, also, using overloaded operators for frames:
            //   *this->auxref_to_abs = this->auxref_to_cog.GetInverse() >> this;

            // Restore marker/forces positions, keeping unchanged respect to aux ref.
            ChFrameMoving<double> cog_oldnew = old_cog_to_abs.BitShiftRight( new_cog_to_abs.GetInverse()); // This is not return correct due to old_cog_to_abs changing by this.BodyFrame!

            for (int i = 0; i < marklist.Count; i++)
            {
                marklist[i].FrameMoving.ConcatenatePreTransformation(cog_oldnew);
                marklist[i].update(ChTime);
            }

            // Forces: ?? to do...
            /*
            HIER_FORCE_INIT
            while (HIER_FORCE_NOSTOP)
            {
                FORCEpointer->
                FORCEpointer->Update (mytime);

                HIER_FORCE_NEXT
            }
            */
        }

        /// Get the COG frame with respect to the auxiliary reference frame.
        public ChFrame<double> GetFrame_COG_to_REF() { return auxref_to_cog.GetInverse(); }

        /// Set the auxiliary reference frame with respect to the COG frame.
        /// Note that this does not move the body absolute COG (the COG is fixed).
        public void SetFrame_REF_to_COG(ChFrameMoving<double> mloc) { auxref_to_cog = mloc; }

        /// Get the auxiliary reference frame with respect to the COG frame.
        public ChFrame<double> GetFrame_REF_to_COG() { return auxref_to_cog; }

        /// Update all auxiliary data of the rigid body and of
        /// its children (markers, forces..)
        public override void update(bool update_assets = true) {
            // update parent class
            base.update(update_assets);

            // update own data
            this.BodyFrame.TransformLocalToParent(this.auxref_to_cog, this.auxref_to_abs);
        }

    }

 
}
