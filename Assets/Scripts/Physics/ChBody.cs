using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
using System.Runtime.InteropServices;
using System.Security.Permissions;

namespace chrono
{
    /// Class for rigid bodies. A rigid body is an entity which
    /// can move in 3D space, and can be constrained to other rigid
    /// bodies using ChLink objects. Rigid bodies can contain auxiliary
    /// references (the ChMarker objects) and forces (the ChForce objects).
    /// These objects have mass and inertia properties. A shape can also
    /// be associated to the body, for collision detection.
    ///
    /// Further info at the @ref rigid_bodies  manual page.
    /// 
   // [System.Serializable]
    public class ChBody : ChPhysicsItem, ChContactable_1vars<IntInterface.Six>, IChLoadableUVW
    {

        // Alan // TO DO make both BodyFrame and collision_model protected and create a GetBody() and GetCollisionModel()
        public ChBodyFrame BodyFrame;

        public collision.ChCollisionModel collision_model;  //< pointer to the collision model

        protected int body_id;   //< body-specific identifier, used for indexing (internal use only)
        protected int body_gid;  //< body-specific identifier, used for global indexing (internal use only)

        protected List<ChMarker> marklist = new List<ChMarker>();  //< list of markers
        protected List<ChForce> forcelist = new List<ChForce>();  //< list of forces

        protected ChVector gyro;// = ChVector.VNULL;  //< gyroscopic torque, i.e. Qm = Wvel x (XInertia*Wvel)

        protected ChVector Xforce;// = ChVector.VNULL;   //< force  acting on body, applied to COG (in absolute coords)
        protected ChVector Xtorque;// = ChVector.VNULL;  //< torque acting on body  (in body relative coords)

        protected ChVector Force_acc;// = ChVector.VNULL;   //< force accumulator, applied to COG (in absolute coords)
        protected ChVector Torque_acc;// = ChVector.VNULL;  //< torque accumulator (in abs space)

        protected ChVector Scr_force;   //< script force accumulator, applied to COG (in absolute coords)
        protected ChVector Scr_torque;  //< script torque accumulator (in absolute coords)

        protected ChMaterialSurface matsurface;  //< data for surface contact and impact

        // Auxiliary, stores position/rotation once a while when collision detection
        // routines require to know the last time that coll. detect. was satisfied
        protected ChCoordsys last_coll_pos;  //< cached position at last collision

        public float density;  //< used when doing the 'recompute mass' operation.

        //protected ChVariablesBodyOwnMass variables = new ChVariablesBodyOwnMass();  //< interface to solver (store inertia and coordinates)

        protected float max_speed;  //< limit on linear speed
        protected float max_wvel;   //< limit on angular velocity

        protected float sleep_time;
        protected float sleep_minspeed;
        protected float sleep_minwvel;
        protected float sleep_starttime;

        // Unity Inspector variables
        // public ChSystem m_system;
        public bool automaticMass;
        public bool bodyfixed;
        public bool collide = true;
        public double mass = 1;
        //  public Vector3 COM;
        public Vector3 inertiaMoments;
        public Vector3 inertiaProducts;
        private double chronotime;
        private int contacts;
        private double acceleration;

        public Vector3 linearVelocity;
        public Vector3 angularVelocity;

        public double radius;

        public double rot;

       // [System.Serializable]
        public enum MaterialType
        {
            NSC,
            SMC
        }

        public MaterialType materialType = MaterialType.NSC;

        public enum CollisionType
        {
            Cube,
            Sphere,
            Cylinder
        }

        public CollisionType type = CollisionType.Cube;

        // NSC material properties  
        public float friction;

        public float rolling_friction;
        public float spinning_friction;
        public float restitution;
        public float cohesion;
        public float dampingf;
        public float compliance;
        public float complianceT;
        public float complianceRoll;
        public float complianceSpin;

        // // SMC material properties
        public float young_modulus;      /// Young's modulus (elastic modulus)
        public float poisson_ratio;      /// Poisson ratio
        public float static_friction;    /// Static coefficient of friction
        public float sliding_friction;   /// Kinetic coefficient of friction        
        public float constant_adhesion;  /// Constant adhesion force, when constant adhesion model is used
        public float adhesionMultDMT;    /// Adhesion multiplier used in DMT model.

        public int Masta { get { return 0; } }

        public virtual void Awake()
        {

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
                    matsurface.GetComponent<ChMaterialSurfaceSMC>().young_modulus = young_modulus;
                    matsurface.GetComponent<ChMaterialSurfaceSMC>().poisson_ratio = poisson_ratio;
                    matsurface.GetComponent<ChMaterialSurfaceSMC>().static_friction = static_friction;
                    matsurface.GetComponent<ChMaterialSurfaceSMC>().sliding_friction = sliding_friction;
                    matsurface.GetComponent<ChMaterialSurfaceSMC>().constant_adhesion = constant_adhesion;
                    matsurface.GetComponent<ChMaterialSurfaceSMC>().adhesionMultDMT = adhesionMultDMT;
                    break;
            }

            // Switch
            switch (type)
            {
                case CollisionType.Cube:

                    var size = transform.localScale * 1.05f;

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

                    SetInertiaXX(ToChrono(inertiaMoments));
                    SetInertiaXY(ToChrono(inertiaProducts));

                    SetBodyFixed(bodyfixed);

                    BodyFrame.SetPos(new ChVector(transform.position.x, transform.position.y, transform.position.z));
                    BodyFrame.SetRot(new ChQuaternion(transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z));

                    BodyFrame.SetPos_dt(ToChrono(linearVelocity));
                    BodyFrame.SetWvel_loc(ToChrono(angularVelocity));

                    //ChSystem msystem = FindObjectOfType<ChSystem>();
                    //msystem.AddBody(this);
                    ChSystem.system.AddBody(this);

                    break;
                case CollisionType.Sphere:

                    var size2 = transform.localScale.y / 2.1;

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

                    SetInertiaXX(ToChrono(inertiaMoments));
                    SetInertiaXY(ToChrono(inertiaProducts));

                    SetBodyFixed(bodyfixed);

                    BodyFrame.SetPos(new ChVector(transform.position.x, transform.position.y, transform.position.z));
                    BodyFrame.SetRot(new ChQuaternion(transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z));

                    BodyFrame.SetPos_dt(ToChrono(linearVelocity));
                   // BodyFrame.SetWvel_par(ToChrono(angularVelocity));
                    BodyFrame.SetWvel_loc(ToChrono(angularVelocity));

                    // ChSystem msystem2 = FindObjectOfType<ChSystem>();
                    // msystem2.AddBody(this);
                    ChSystem.system.AddBody(this);

                    break;
                case CollisionType.Cylinder:

                    var height = 2 * transform.localScale.y;
                    var radiusX = transform.localScale.x / 2;
                    var radiusZ = transform.localScale.z / 2;
                    radius = transform.localScale.x / 2;

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

                    SetInertiaXX(ToChrono(inertiaMoments));
                    SetInertiaXY(ToChrono(inertiaProducts));

                    SetBodyFixed(bodyfixed);

                    BodyFrame.SetPos(new ChVector(transform.position.x, transform.position.y, transform.position.z));
                    BodyFrame.SetRot(new ChQuaternion(transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z));

                    BodyFrame.SetPos_dt(ToChrono(linearVelocity));
                    BodyFrame.SetWvel_loc(ToChrono(angularVelocity));

                    // ChSystem msystem3 = FindObjectOfType<ChSystem>();
                    // msystem3.AddBody(this);
                    ChSystem.system.AddBody(this);

                    break;
            }

        }


        public ChBody()
        {
            BodyFrame = new ChBodyFrame();

            marklist.Clear();
            forcelist.Clear();

            BFlagsSetAllOFF();  // no flags

            Xforce = ChVector.VNULL;
            Xtorque = ChVector.VNULL;

            Force_acc = ChVector.VNULL;
            Torque_acc = ChVector.VNULL;

            collision_model = InstanceCollisionModel();

            density = 1000.0f;

            max_speed = 0.5f;
            max_wvel = 2.0f * (float)(ChMaths.CH_C_PI);

            sleep_time = 0.6f;
            sleep_starttime = 0;
            sleep_minspeed = 0.1f;
            sleep_minwvel = 0.04f;
            SetUseSleeping(true);

            automaticMass = false;

            GCHandle handle1 = GCHandle.Alloc(this);
            IntPtr parameter = (IntPtr)handle1;
            BodyFrame.variables.SetUserData(parameter);

            body_id = 0;
        }

        /// Build a rigid body.
        public ChBody(ChMaterialSurface.ContactMethod contact_method = ChMaterialSurface.ContactMethod.NSC)
        {
            BodyFrame = new ChBodyFrame();

            marklist.Clear();
            forcelist.Clear();

            BFlagsSetAllOFF();  // no flags

            Xforce = ChVector.VNULL;
            Xtorque = ChVector.VNULL;

            Force_acc = ChVector.VNULL;
            Torque_acc = ChVector.VNULL;

            collision_model = InstanceCollisionModel();

            density = 1000.0f;

            max_speed = 0.5f;
            max_wvel = 2.0f * (float)(ChMaths.CH_C_PI);

            sleep_time = 0.6f;
            sleep_starttime = 0;
            sleep_minspeed = 0.1f;
            sleep_minwvel = 0.04f;
            SetUseSleeping(true);

            GCHandle handle1 = GCHandle.Alloc(this);
            IntPtr parameter = (IntPtr)handle1;
            BodyFrame.variables.SetUserData(parameter);

            body_id = 0;
        }

        /// Build a rigid body with a different collision model.
        public ChBody(collision.ChCollisionModel new_collision_model,
               ChMaterialSurface.ContactMethod contact_method = ChMaterialSurface.ContactMethod.NSC)
        {
            BodyFrame = new ChBodyFrame();

            marklist.Clear();
            forcelist.Clear();

            BFlagsSetAllOFF();  // no flags

            Xforce = ChVector.VNULL;
            Xtorque = ChVector.VNULL;

            Force_acc = ChVector.VNULL;
            Torque_acc = ChVector.VNULL;
            Scr_force = ChVector.VNULL;
            Scr_torque = ChVector.VNULL;

            collision_model = new_collision_model;
            collision_model.SetContactable(this);

            /* switch (contact_method)
             {
                 case ChMaterialSurface.NSC:
                     matsurface = new ChMaterialSurfaceNSC();
                     break;
                 case ChMaterialSurface.SMC:
                     matsurface = new ChMaterialSurfaceSMC();
                     break;
             }*/

            density = 1000.0f;

            last_coll_pos = ChCoordsys.CSYSNULL;

            max_speed = 0.5f;
            max_wvel = 2.0f * (float)ChMaths.CH_C_PI;

            sleep_time = 0.6f;
            sleep_starttime = 0;
            sleep_minspeed = 0.1f;
            sleep_minwvel = 0.04f;
            SetUseSleeping(true);

            GCHandle handle1 = GCHandle.Alloc(this);
            IntPtr parameter = (IntPtr)handle1;
            BodyFrame.variables.SetUserData(parameter);

            body_id = 0;
        }

        public ChBody(ChBody other) : base(other)
        {
            BodyFrame = new ChBodyFrame(other.BodyFrame);

            bflags = other.bflags;

            BodyFrame.variables = other.BodyFrame.variables;
            GCHandle handle1 = GCHandle.Alloc(this);
            IntPtr parameter = (IntPtr)handle1;
            BodyFrame.variables.SetUserData(parameter);

            gyro = other.Get_gyro();

            RemoveAllForces();   // also copy-duplicate the forces? Let the user handle this..
            RemoveAllMarkers();  // also copy-duplicate the markers? Let the user handle this..

            SetChTime(other.GetChTime());

            // also copy-duplicate the collision model? Let the user handle this..
            collision_model = InstanceCollisionModel();

            matsurface = other.matsurface;  // also copy-duplicate the material? Let the user handle this..

            // density = other.density;

            Scr_force = other.Scr_force;
            Scr_torque = other.Scr_torque;

            last_coll_pos = other.last_coll_pos;

            max_speed = other.max_speed;
            max_wvel = other.max_wvel;

            sleep_time = other.sleep_time;
            sleep_starttime = other.sleep_starttime;
            sleep_minspeed = other.sleep_minspeed;
            sleep_minwvel = other.sleep_minwvel;
        }

        public virtual void FixedUpdate()
        {

            // Test = ChQuaternion.QNULL;
            //var frame = this.GetFrame_REF_to_abs();
            //transform.position = FromChrono(frame.GetPos());
            // transform.rotation = FromChrono(frame.GetRot());
            //transform.position = ExtractTranslationFromMatrix(uMat);
            //transform.rotation = ExtractRotationFromMatrix(uMat);

            Matrix4x4 uMat = new Matrix4x4();

            ChMatrix33<double> chMat = GetFrame_REF_to_abs().GetA();

            /* chMat.address = chMat.GetAddress();
             chMat.rows = chMat.GetRows();
             chMat.columns = chMat.GetColumns();*/

            uMat[0] = (float)chMat.GetElementN(0);
            uMat[1] = (float)chMat.GetElementN(3);
            uMat[2] = (float)chMat.GetElementN(6);

            uMat[4] = (float)chMat.GetElementN(1);
            uMat[5] = (float)chMat.GetElementN(4);
            uMat[6] = (float)chMat.GetElementN(7);

            uMat[8] = (float)chMat.GetElementN(2);
            uMat[9] = (float)chMat.GetElementN(5);
            uMat[10] = (float)chMat.GetElementN(8);


            uMat[12] = (float)GetFrame_REF_to_abs().GetPos().x;
            uMat[13] = (float)GetFrame_REF_to_abs().GetPos().y;
            uMat[14] = (float)GetFrame_REF_to_abs().GetPos().z;


            // Clear the last column to 0 and set low-right corner to 1 as in
            // Denavitt-Hartemberg matrices, transposed.
            uMat[3] = uMat[7] = uMat[11] = 0.0f;
            uMat[15] = 1.0f;

           // transform.position = ExtractTranslationFromMatrix(uMat);
            //rot = ExtractRotationFromMatrix(uMat).eulerAngles.y;
            // transform.rotation = ExtractRotationFromMatrix(uMat);

             var frame = GetFrame_REF_to_abs();
             transform.position = Utils.FromChrono(frame.GetPos());
             transform.rotation = Utils.FromChrono(frame.GetRot());
        }

        public static Vector3 FromChrono(ChVector v)
        {
            return new Vector3((float)v.x, (float)v.y, (float)v.z);
        }
        public static UnityEngine.Quaternion FromChrono(ChQuaternion q)
        {
            return new UnityEngine.Quaternion((float)q.e1, (float)q.e2, (float)q.e3, (float)q.e0);
        }
        public static ChVector ToChrono(Vector3 v)
        {
            return new ChVector(v.x, v.y, v.z);
        }

        public static ChQuaternion ToChrono(UnityEngine.Quaternion q)
        {
            return new ChQuaternion(q.w, q.x, q.y, q.z);
        }

        public collision.ChCollisionModel InstanceCollisionModel()
        {
            var collision_model_t = new collision.ChModelBullet();
            collision_model_t.SetContactable(this);
            return collision_model_t;
        }

        /// <summary>
        /// Extract translation from transform matrix.
        /// </summary>
        /// <param name="matrix">Transform matrix. This parameter is passed by reference
        /// to improve performance; no changes will be made to it.</param>
        /// <returns>
        /// Translation offset.
        /// </returns>
        public static Vector3 ExtractTranslationFromMatrix(Matrix4x4 matrix)
        {
            Vector3 translate;
            translate.x = matrix.m03;
            translate.y = matrix.m13;
            translate.z = matrix.m23;
            return translate;
        }
        /// <summary>
        /// Extract rotation quaternion from transform matrix.
        /// </summary>
        /// <param name="matrix">Transform matrix. This parameter is passed by reference
        /// to improve performance; no changes will be made to it.</param>
        /// <returns>
        /// Quaternion representation of rotation transform.
        /// </returns>
        public static UnityEngine.Quaternion ExtractRotationFromMatrix(Matrix4x4 matrix)
        {
            Vector3 forward;
            forward.x = matrix.m02;
            forward.y = matrix.m12;
            forward.z = matrix.m22;

            Vector3 upwards;
            upwards.x = matrix.m01;
            upwards.y = matrix.m11;
            upwards.z = matrix.m21;

            return UnityEngine.Quaternion.LookRotation(forward, upwards);
        }

        public bool IsTetrahedronIntegrationNeeded() { return true; }

        //
        // FLAGS
        //

        /// Sets the 'fixed' state of the body. If true, it does not move
        /// respect to the absolute world, despite constraints, forces, etc.
        public void SetBodyFixed(bool state) {
            BodyFrame.variables.SetDisabled(state);
            if (state == BFlagGet(BodyFlag.FIXED))

                return;
            BFlagSet(BodyFlag.FIXED, state);
            // RecomputeCollisionModel(); // because one may use different model types for static or dynamic coll.shapes
        }

        /// Return true if this body is fixed to ground.
        public bool GetBodyFixed() { return BFlagGet(BodyFlag.FIXED); }

        /// If true, the normal restitution coefficient is evaluated from painted material channel.
        public void SetEvalContactCn(bool state) { }
        public bool GetEvalContactCn() {
            return BFlagGet(BodyFlag.EVAL_CONTACT_CN);
        }

        /// If true, the tangential restitution coefficient is evaluated from painted material channel.
        public void SetEvalContactCt(bool state) {
            BFlagSet(BodyFlag.EVAL_CONTACT_CT, state);
        }
        public bool GetEvalContactCt() {
            return BFlagGet(BodyFlag.EVAL_CONTACT_CT);
        }

        /// If true, the kinetic friction coefficient is evaluated from painted material channel.
        public void SetEvalContactKf(bool state) {
            BFlagSet(BodyFlag.EVAL_CONTACT_KF, state);
        }
        public bool GetEvalContactKf() {
            return BFlagGet(BodyFlag.EVAL_CONTACT_KF);
        }

        /// If true, the static friction coefficient is evaluated
        /// from painted material channel.
        public void SetEvalContactSf(bool state) {
            BFlagSet(BodyFlag.EVAL_CONTACT_SF, state);
        }
        public bool GetEvalContactSf() {
            return BFlagGet(BodyFlag.EVAL_CONTACT_SF);
        }

        /// Enable/disable the collision for this rigid body.
        /// (After setting ON, you may need RecomputeCollisionModel()
        /// before anim starts, if you added an external object
        /// that implements onAddCollisionGeometries(), ex. in a plug-in for a CAD)
        public void SetCollide(bool state) {
            if (state == BFlagGet(BodyFlag.COLLIDE))
                return;

            if (state)
            {
                SyncCollisionModels();
                BFlagSetON(BodyFlag.COLLIDE);
                if (GetSystem())
                    GetSystem().GetCollisionSystem().Add(collision_model);
            }
            else
            {
                BFlagSetOFF(BodyFlag.COLLIDE);
                if (GetSystem())
                    GetSystem().GetCollisionSystem().Remove(collision_model);
            }
        }

        /// Return true if collision is enabled for this body.
        public override bool GetCollide() {
            return BFlagGet(BodyFlag.COLLIDE);
        }

        /// Show collision mesh in 3D views.
        public void SetShowCollisionMesh(bool state) {
            BFlagSet(BodyFlag.SHOW_COLLMESH, state);
        }

        /// Return true if collision mesh is shown in 3D views.
        public bool GetShowCollisionMesh() {
            return BFlagGet(BodyFlag.SHOW_COLLMESH);
        }

        /// Enable the maximum linear speed limit (beyond this limit it will be clamped).
        /// This is useful in virtual reality and real-time simulations, because
        /// it reduces the risk of bad collision detection.
        /// The realism is limited, but the simulation is more stable.
        public void SetLimitSpeed(bool state) {
            BFlagSet(BodyFlag.LIMITSPEED, state);
        }

        /// Return true if maximum linear speed is limited.
        public bool GetLimitSpeed() {
            return BFlagGet(BodyFlag.LIMITSPEED);
        }

        /// Deactivate the gyroscopic torque (quadratic term).
        /// This is useful in virtual reality and real-time
        /// simulations, where objects that spin too fast with non-uniform inertia
        /// tensors (ex thin cylinders) might cause the integration to diverge quickly.
        /// The realism is limited, but the simulation is more stable.
        public void SetNoGyroTorque(bool state) {
            BFlagSet(BodyFlag.NOGYROTORQUE, state);
        }

        /// Return true if gyroscopic torque is deactivated.
        public bool GetNoGyroTorque() {
            return BFlagGet(BodyFlag.NOGYROTORQUE);
        }

        /// Enable/disable option for setting bodies to "sleep".
        /// If use sleeping = true, bodies which stay in same place
        /// for long enough time will be deactivated, for optimization.
        /// The realism is limited, but the simulation is faster.
        public void SetUseSleeping(bool state) {
            BFlagSet(BodyFlag.USESLEEPING, state);
        }

        /// Return true if 'sleep' mode is activated.
        public bool GetUseSleeping() {
            return BFlagGet(BodyFlag.USESLEEPING);
        }

        /// Force the body in sleeping mode or not (usually this state change is not
        /// handled by users, anyway, because it is mostly automatic).
        public void SetSleeping(bool state) {
            BFlagSet(BodyFlag.SLEEPING, state);
        }

        /// Return true if this body is currently in 'sleep' mode.
        public bool GetSleeping() {
            return BFlagGet(BodyFlag.SLEEPING);
        }

        /// Test if a body could go in sleeping state if requirements are satisfied.
        /// Return true if state could be changed from no sleep to sleep.
        public bool TrySleeping() {
            BFlagSet(BodyFlag.COULDSLEEP, false);

            if (this.GetUseSleeping())
            {
                if (!this.IsActive())
                    return false;

                // if not yet sleeping:
                if ((this.BodyFrame.coord_dt.pos.LengthInf() < this.sleep_minspeed) &&
                    (2.0 * this.BodyFrame.coord_dt.rot.LengthInf() < this.sleep_minwvel))
                {
                    if ((this.GetChTime() - this.sleep_starttime) > this.sleep_time)
                    {
                        BFlagSet(BodyFlag.COULDSLEEP, true);  // mark as sleep candidate
                        return true;                    // could go to sleep!
                    }
                }
                else
                {
                    this.sleep_starttime = (float)(this.GetChTime());
                }
            }
            return false;
        }

        /// Return true if the body is active; i.e. it is neither fixed to ground
        /// nor is it in "sleep" mode. Return false otherwise.
        public bool IsActive() {
            return !BFlagGet(BodyFlag.SLEEPING) && !BFlagGet(BodyFlag.FIXED);
        }

        /// Set body id for indexing (internal use only)
        public void SetId(int id) { body_id = id; }

        /// Set body id for indexing (internal use only)
        public int GetId() { return body_id; }

        /// Set global body index (internal use only)
        public void SetGid(int id) { body_gid = id; }

        /// Get the global body index (internal use only)
        public int GetGid() { return body_gid; }

        //
        // FUNCTIONS
        //

        /// Number of coordinates of body: 7 because uses quaternions for rotation
        public override int GetDOF() { return 7; }
        /// Number of coordinates of body: 6 because derivatives use angular velocity
        public override int GetDOF_w() { return 6; }

        /// Returns reference to the encapsulated ChVariablesBody, representing
        /// body variables (pos, speed, or accel.) and forces.
        /// The ChVariablesBodyOwnMass is the interface to the system solver.
        public virtual ChVariablesBodyOwnMass VariablesBody() {
            // BodyFrame.VariablesBody();
            return BodyFrame.variables;
        }
        public virtual ChVariables Variables() {
            //BodyFrame.SetVariables(variables);
            return BodyFrame.variables;
        }

        //
        // STATE FUNCTIONS
        //

        // (override/implement interfaces for global state vectors, see ChPhysicsItem for comments.)

        public override void IntStateGather(int off_x,
                                    ref ChState x,
                                int off_v,
                                ref ChStateDelta v,
                                ref double T) {

            x.PasteCoordsys(this.BodyFrame.coord, off_x, 0);
            v.PasteVector(this.BodyFrame.coord_dt.pos, off_v, 0);
            v.PasteVector(this.BodyFrame.GetWvel_loc(), off_v + 3, 0);
            T = this.GetChTime();
        }

        public override void IntStateScatter(int off_x,
                                 ChState x,
                                 int off_v,
                                 ChStateDelta v,
                                 double T)
        {
            this.BodyFrame.SetCoord(x.ClipCoordsys(off_x, 0));
            this.BodyFrame.SetPos_dt(v.ClipVector(off_v, 0));
            this.BodyFrame.SetWvel_loc(v.ClipVector(off_v + 3, 0));
            this.SetChTime(T);
            this.update();
        }
        public override void IntStateGatherAcceleration(int off_a, ref ChStateDelta a) {
            a.PasteVector(this.BodyFrame.coord_dtdt.pos, off_a, 0);
            a.PasteVector(this.BodyFrame.GetWacc_loc(), off_a + 3, 0);
        }
        public override void IntStateScatterAcceleration(int off_a, ChStateDelta a)
        {
            this.BodyFrame.SetPos_dtdt(a.ClipVector(off_a, 0));
            this.BodyFrame.SetWacc_loc(a.ClipVector(off_a + 3, 0));
        }

        public override void IntStateIncrement(int off_x,
                                       ref ChState x_new,
                                       ChState x,
                                       int off_v,
                                       ChStateDelta Dv)
        {
            // ADVANCE POSITION:
            x_new[off_x] = x[off_x] + Dv[off_v];
            x_new[off_x + 1] = x[off_x + 1] + Dv[off_v + 1];
            x_new[off_x + 2] = x[off_x + 2] + Dv[off_v + 2];

            // ADVANCE ROTATION: rot' = delta*rot  (use quaternion for delta rotation)
            ChQuaternion mdeltarot = new ChQuaternion(1, 0, 0, 0);
            ChQuaternion moldrot = x.ClipQuaternion((int)off_x + 3, 0);
            ChVector newwel_abs = BodyFrame.Amatrix * Dv.ClipVector((int)off_v + 3, 0);
            double mangle = newwel_abs.Length();
            newwel_abs.Normalize();
            mdeltarot.Q_from_AngAxis(mangle, newwel_abs);
            ChQuaternion mnewrot = mdeltarot * moldrot;  // quaternion product
            x_new.PasteQuaternion(mnewrot, (int)off_x + 3, 0);
            // Debug.Log("ang " + x_new[2]);
        }

        public override void IntLoadResidual_F(int off, ref ChVectorDynamic<double> R, double c) {
            // add applied forces to 'fb' vector
            R.PasteSumVector(Xforce * c, (int)off, 0);

            // add applied torques to 'fb' vector, including gyroscopic torque
            if (this.GetNoGyroTorque())
                R.PasteSumVector((Xtorque) * c, (int)off + 3, 0);
            else
                R.PasteSumVector((Xtorque - gyro) * c, (int)off + 3, 0);
        }

        public override void IntLoadResidual_Mv(int off,
                                    ref ChVectorDynamic<double> R,
                                    ChVectorDynamic<double> w,
                                    double c) {
            R[off + 0] += c * GetMass() * w[off + 0];  // w not working // Alan
            R[off + 1] += c * GetMass() * w[off + 1];
            R[off + 2] += c * GetMass() * w[off + 2];

            ChVector Iw = GetInertia() * w.ClipVector(off + 3, 0);
            Iw *= c;
            R.PasteSumVector(Iw, off + 3, 0);
        }

        public override void IntToDescriptor(int off_v,
                                 ChStateDelta v,
                                 ChVectorDynamic<double> R,
                                 int off_L,
                                 ChVectorDynamic<double> L,
                                 ChVectorDynamic<double> Qc)
        {

            this.BodyFrame.variables.Get_qb().PasteClippedMatrix(v, off_v, 0, 6, 1, 0, 0);  // for solver warm starting only
            this.BodyFrame.variables.Get_fb().PasteClippedMatrix(R, off_v, 0, 6, 1, 0, 0);  // solver known term
        }

        public override void IntFromDescriptor(int off_v,  // offset in v
                               ref ChStateDelta v,
                               int off_L,  // offset in L
                               ref ChVectorDynamic<double> L)
        {
            v.PasteMatrix(this.BodyFrame.variables.Get_qb(), off_v, 0);
            // Debug.Log("clappa " + v[1]);
        }

        //
        // SOLVER FUNCTIONS
        //

        // Override/implement solver system functions of ChPhysicsItem
        // (to assemble/manage data for system solver)

        /// Sets the 'fb' part of the encapsulated ChVariablesBodyOwnMass to zero.
        public override void VariablesFbReset() {
            this.BodyFrame.variables.Get_fb().FillElem(0.0);
        }

        /// Adds the current forces applied to body (including gyroscopic torque) in
        /// encapsulated ChVariablesBody, in the 'fb' part: qf+=forces*factor
        public override void VariablesFbLoadForces(double factor = 1) {
            // add applied forces to 'fb' vector
            this.BodyFrame.variables.Get_fb().PasteSumVector(Xforce * factor, 0, 0);

            // add applied torques to 'fb' vector, including gyroscopic torque
            if (this.GetNoGyroTorque())
                this.BodyFrame.variables.Get_fb().PasteSumVector((Xtorque) * factor, 3, 0);
            else
                this.BodyFrame.variables.Get_fb().PasteSumVector((Xtorque - gyro) * factor, 3, 0);
        }

        /// Initialize the 'qb' part of the ChVariablesBody with the
        /// current value of body speeds. Note: since 'qb' is the unknown, this
        /// function seems unnecessary, unless used before VariablesFbIncrementMq()
        public override void VariablesQbLoadSpeed() {
            // set current speed in 'qb', it can be used by the solver when working in incremental mode
            this.BodyFrame.variables.Get_qb().PasteVector(BodyFrame.GetCoord_dt().pos, 0, 0);
            this.BodyFrame.variables.Get_qb().PasteVector(BodyFrame.GetWvel_loc(), 3, 0);
        }

        /// Adds M*q (masses multiplied current 'qb') to Fb, ex. if qb is initialized
        /// with v_old using VariablesQbLoadSpeed, this method can be used in
        /// timestepping schemes that do: M*v_new = M*v_old + forces*dt
        public override void VariablesFbIncrementMq() {
            this.BodyFrame.variables.Compute_inc_Mb_v(ref this.BodyFrame.variables.Get_fb(), this.BodyFrame.variables.Get_qb());
        }

        /// Fetches the body speed (both linear and angular) from the
        /// 'qb' part of the ChVariablesBody (does not updates the full body&markers state)
        /// and sets it as the current body speed.
        /// If 'step' is not 0, also computes the approximate acceleration of
        /// the body using backward differences, that is  accel=(new_speed-old_speed)/step.
        /// Mostly used after the solver provided the solution in ChVariablesBody .
        public override void VariablesQbSetSpeed(double step = 0) {
            ChCoordsys old_coord_dt = this.BodyFrame.GetCoord_dt();

            // from 'qb' vector, sets body speed, and updates auxiliary data
            this.BodyFrame.SetPos_dt(this.BodyFrame.variables.Get_qb().ClipVector(0, 0));
            this.BodyFrame.SetWvel_loc(this.BodyFrame.variables.Get_qb().ClipVector(3, 0));

            // apply limits (if in speed clamping mode) to speeds.
            ClampSpeed();

            // compute auxiliary gyroscopic forces
            ComputeGyro();

            // Compute accel. by BDF (approximate by differentiation);
            if (step != 0)
            {
                this.BodyFrame.SetPos_dtdt((this.BodyFrame.GetCoord_dt().pos - old_coord_dt.pos) / step);
                this.BodyFrame.SetRot_dtdt((this.BodyFrame.GetCoord_dt().rot - old_coord_dt.rot) / step);
            }
        }

        /// Increment body position by the 'qb' part of the ChVariablesBody,
        /// multiplied by a 'step' factor.
        ///     pos+=qb*step
        /// If qb is a speed, this behaves like a single step of 1-st order
        /// numerical integration (Eulero integration).
        /// Does not automatically update markers & forces.
        public override void VariablesQbIncrementPosition(double dt_step)
        {
            if (!this.IsActive())
                return;

            // Updates position with incremental action of speed contained in the
            // 'qb' vector:  pos' = pos + dt * speed   , like in an Eulero step.

            ChVector newspeed = BodyFrame.variables.Get_qb().ClipVector(0, 0);
            ChVector newwel = BodyFrame.variables.Get_qb().ClipVector(3, 0);

            // ADVANCE POSITION: pos' = pos + dt * vel
            this.BodyFrame.SetPos(this.BodyFrame.GetPos() + newspeed * dt_step);

            // ADVANCE ROTATION: rot' = [dt*wwel]%rot  (use quaternion for delta rotation)
            ChQuaternion mdeltarot = new ChQuaternion(1, 0, 0, 0);
            ChQuaternion moldrot = this.BodyFrame.GetRot();
            ChVector newwel_abs = BodyFrame.Amatrix * newwel;
            double mangle = newwel_abs.Length() * dt_step;
            newwel_abs.Normalize();
            mdeltarot.Q_from_AngAxis(mangle, newwel_abs);
            ChQuaternion mnewrot = mdeltarot % moldrot;
            this.BodyFrame.SetRot(mnewrot);
        }

        /// Tell to a system descriptor that there are variables of type
        /// ChVariables in this object (for further passing it to a solver)
        public override void InjectVariables(ref ChSystemDescriptor mdescriptor) {
            this.BodyFrame.variables.SetDisabled(!this.IsActive());

            mdescriptor.InsertVariables(this.BodyFrame.variables);
        }

        // Other functions

        /// Set no speed and no accelerations (but does not change the position)
        public override void SetNoSpeedNoAcceleration() {
            this.BodyFrame.SetPos_dt(ChVector.VNULL);
            this.BodyFrame.SetWvel_loc(ChVector.VNULL);
            this.BodyFrame.SetPos_dtdt(ChVector.VNULL);
            this.BodyFrame.SetRot_dtdt(ChQuaternion.QNULL);
        }

        /// Change the collision model.
        public void SetCollisionModel(collision.ChCollisionModel new_collision_model) {
            if (collision_model != null)
            {
                if (GetSystem())
                    GetSystem().GetCollisionSystem().Remove(collision_model);
            }

            collision_model = new_collision_model;
            collision_model.SetContactable(this);

        }

        /// Access the collision model for the collision engine.
        /// To get a non-null pointer, remember to SetCollide(true), before.
        public virtual collision.ChCollisionModel GetCollisionModel() { return collision_model; }

        /// Synchronize coll.model coordinate and bounding box to the position of the body.
        public override void SyncCollisionModels() {
            if (this.GetCollide())
                this.GetCollisionModel().SyncPosition();
        }
        public override void AddCollisionModelsToSystem() {
            // Debug.Assert(this.GetSystem());
            SyncCollisionModels();
            if (this.GetCollide())
                this.GetSystem().GetCollisionSystem().Add(collision_model);
        }
        public override void RemoveCollisionModelsFromSystem() {
            // Debug.Assert(this.GetSystem());
            if (this.GetCollide())
                this.GetSystem().GetCollisionSystem().Remove(collision_model);
        }

        /// Update the optimization structures (OOBB, ABB, etc.)
        /// of the collision model, from the associated geometry in some external object (es.CAD).
        public bool RecomputeCollisionModel() {
            if (!GetCollide())
                return false;  // do nothing unless collision enabled

            collision_model.ClearModel();  // ++++ start geometry definition

            // ... external geometry fetch shapes?

            collision_model.BuildModel();  // ++++ complete geometry definition

            return true;
        }

        /// Gets the last position when the collision detection was
        /// performed last time (i.e. last time SynchronizeLastCollPos() was used)
        public ChCoordsys GetLastCollPos() { return last_coll_pos; }
        /// Stores the current position in the last-collision-position buffer.
        public void SynchronizeLastCollPos() { last_coll_pos = this.BodyFrame.coord; }

        /// Get the rigid body coordinate system that represents
        /// the GOG (Center of Gravity). The mass and inertia tensor
        /// are defined respect to this coordinate system, that is also
        /// assumed the default main coordinates of the body.
        /// By default, doing mybody.GetPos() etc. is like mybody.GetFrame_COG_abs().GetPos() etc.
        public virtual ChFrameMoving<double> GetFrame_COG_to_abs() { return BodyFrame; }

        /// Get the rigid body coordinate system that is used for
        /// defining the collision shapes and the ChMarker objects.
        /// For the base ChBody, this is always the same reference of the COG.
        public virtual ChFrameMoving<double> GetFrame_REF_to_abs() { return BodyFrame; }

        /// Get the master coordinate system for the assets (this will return the
        /// main coordinate system of the rigid body)
        public virtual ChFrame<double> GetAssetsFrame(int nclone = 0) { return (GetFrame_REF_to_abs()); }

        /// Get the entire AABB axis-aligned bounding box of the object,
        /// as defined by the collision model (if any).
        public virtual void GetTotalAABB(ref ChVector bbmin, ref ChVector bbmax) {
            if (this.GetCollisionModel() != null)
                this.GetCollisionModel().GetAABB(ref bbmin, ref bbmax);
            else
                GetTotalAABB(ref bbmin, ref bbmax);  // default: infinite aabb
        }



        /// Infer the contact method from the underlying material properties object.
        public ChMaterialSurface.ContactMethod GetContactMethod() { return matsurface.GetContactMethod(); }

        /// Access the NSC material surface properties associated with this body.
        /// This function performs a dynamic cast (and returns an empty pointer
        /// if matsurface is in fact of SMC type).  As such, it must return a copy
        /// of the shared pointer and is therefore NOT thread safe.
        public ChMaterialSurfaceNSC GetMaterialSurfaceNSC()
        {
            return (ChMaterialSurfaceNSC)(matsurface);
        }

        /// Access the SMC material surface properties associated with this body.
        /// This function performs a dynamic cast (and returns an empty pointer
        /// if matsurface is in fact of NSC type).  As such, it must return a copy
        /// of the shared pointer and is therefore NOT thread safe.
        public ChMaterialSurfaceSMC GetMaterialSurfaceSMC()
        {
            return (ChMaterialSurfaceSMC)(matsurface);
        }

        /// Set the material surface properties by passing a ChMaterialSurfaceNSC or
        /// ChMaterialSurfaceSMC object.
        public void SetMaterialSurface(ChMaterialSurface mnewsurf) { matsurface = mnewsurf; }

        /// The density of the rigid body, as [mass]/[unit volume]. Used just if
        /// the inertia tensor and mass are automatically recomputed from the
        /// geometry (in case a CAD plugin for example provides the surfaces.)
        //float GetDensity() const { return density; } //  obsolete, use the base ChLoadable::GetDensity()
        public void SetDensity(float mdensity) { density = mdensity; }

        //
        // DATABASE HANDLING.
        //
        // To attach/remove items (rigid bodies, links, etc.) you must use
        // shared pointer, so that you don't need to care about item deletion,
        // which will be automatic when needed.
        // Please don't add the same item multiple times; also, don't remove
        // items which haven't ever been added.
        // NOTE! After adding/removing items to the system, you should call Update() !

        /// Attach a marker to this body.
        public void AddMarker(ChMarker amarker) {
            // don't allow double insertion of same object
            //  Debug.Assert(List<ChMarker>.Enumerator (marklist.begin(), marklist.end(), amarker) ==
            //       marklist.end());

            amarker.SetBody(this);
            marklist.Add(amarker);
        }
        /// Attach a force to this body.
        public void AddForce(ChForce aforce) {
            // don't allow double insertion of same object
            //  assert(std::find<std::vector<std::shared_ptr<ChForce>>::iterator>(forcelist.begin(), forcelist.end(), aforce) ==
            //       forcelist.end());

            aforce.SetBody(this);
            forcelist.Add(aforce);
        }

        /// Remove a specific marker from this body. Warning: linear time search.
        public void RemoveMarker(ChMarker mmarker) {
            // trying to remove objects not previously added?
            // assert(std::find<std::vector<std::shared_ptr<ChMarker>>::iterator>(marklist.begin(), marklist.end(), mmarker) !=
            //      marklist.end());

            // warning! linear time search
            // marklist.erase(
            //   std::find<std::vector<std::shared_ptr<ChMarker>>::iterator>(marklist.begin(), marklist.end(), mmarker));

            mmarker.SetBody(null);
        }
        /// Remove a specific force from this body. Warning: linear time search.
        public void RemoveForce(ChForce aforce) {
            // trying to remove objects not previously added?
            //  assert(std::find<std::vector<std::shared_ptr<ChForce>>::iterator>(forcelist.begin(), forcelist.end(), mforce) !=
            //       forcelist.end());

            // warning! linear time search
            // forcelist.erase(
            //     std::find<std::vector<std::shared_ptr<ChForce>>::iterator>(forcelist.begin(), forcelist.end(), mforce));

            aforce.SetBody(null);
        }

        /// Remove all markers at once. Faster than doing multiple RemoveForce()
        /// Don't care about deletion: it is automatic, only when needed.
        public void RemoveAllForces() {
            foreach (ChForce force in forcelist)
            {
                force.SetBody(null);
            }
            forcelist.Clear();
        }
        /// Remove all markers at once. Faster than doing multiple RemoveForce()
        /// Don't care about deletion: it is automatic, only when needed.
        public void RemoveAllMarkers() {
            foreach (ChMarker marker in marklist)
            {
                marker.SetBody(null);
            }
            marklist.Clear();
        }

        /// Finds a marker from its ChObject name
       /* public ChMarker SearchMarker(char m_name) {
            return new ChMarker();
            // return new ChContainerSearchFromName<ChMarker>, List<ChMarker>.Enumerator>(
            //            m_name, marklist.begin(), marklist.end());
        }*/
        /// Finds a force from its ChObject name
        public ChForce SearchForce(char m_name) {
            return new ChForce();
            // return ChContainerSearchFromName<std::shared_ptr<ChForce>, std::vector<std::shared_ptr<ChForce>>::iterator>(
            // m_name, forcelist.begin(), forcelist.end());
        }

        /// Gets the list of children markers.
        /// NOTE: to modify this list, use the appropriate Remove..
        /// and Add.. functions.
        public List<ChMarker> GetMarkerList() { return marklist; }

        /// Gets the list of children forces.
        /// NOTE: to modify this list, use the appropriate Remove..
        /// and Add.. functions.
        public List<ChForce> GetForceList() { return forcelist; }

        //
        // Point/vector transf.(NOTE! you may also use operators of ChMovingFrame)
        //

        public ChVector Point_World2Body(ChVector mpoint) {
            //return new ChFrame<double>().TransformParentToLocal(mpoint);
            return BodyFrame.TransformParentToLocal(mpoint);
        }
        public ChVector Point_Body2World(ChVector mpoint) {
            return new ChFrame<double>().TransformLocalToParent(mpoint);
        }
        public ChVector Dir_World2Body(ChVector mpoint) {
            return BodyFrame.Amatrix.MatrT_x_Vect(mpoint);
        }
        public ChVector Dir_Body2World(ChVector mpoint) {
            return BodyFrame.Amatrix.Matr_x_Vect(mpoint);
        }
        public ChVector RelPoint_AbsSpeed(ChVector mrelpoint) {
            return BodyFrame.PointSpeedLocalToParent(mrelpoint);
        }
        public ChVector RelPoint_AbsAcc(ChVector mrelpoint) {
            return BodyFrame.PointAccelerationLocalToParent(mrelpoint);
        }



        /// Mass of the rigid body. Must be positive.
        /// Try not to mix bodies with too high/too low values of mass, for numerical stability.
        public void SetMass(double newmass)
        {
            if (newmass > 0.0)
                BodyFrame.variables.SetBodyMass(newmass);
        }

        /// Set the inertia tensor of the body.
        /// The provided 3x3 matrix should be symmetric and contain the inertia
        /// tensor, expressed in the local coordinate system:
        /// <pre>
        ///               [ int{x^2+z^2}dm    -int{xy}dm    -int{xz}dm    ]
        /// newXInertia = [                  int{x^2+z^2}   -int{yz}dm    ]
        ///               [                                int{x^2+y^2}dm ]
        /// </pre>
        public void SetInertia(ChMatrix33<double> newXInertia) {
            BodyFrame.variables.SetBodyInertia(newXInertia);
        }

        /// G
        /// et a reference to the inertia tensor, expressed in local coordinate system.
        /// The return 3xe3 symmetric matrix contains the following values:
        /// <pre>
        ///  [ int{x^2+z^2}dm    -int{xy}dm    -int{xz}dm    ]
        ///  [                  int{x^2+z^2}   -int{yz}dm    ]
        ///  [                                int{x^2+y^2}dm ]
        /// </pre>
        public ChMatrix33<double> GetInertia() { return BodyFrame.variables.GetBodyInertia(); }
        /// Set the diagonal part of the inertia tensor (Ixx, Iyy, Izz values).
        /// The provided 3x1 vector should contain the moments of inertia,
        /// expressed in the local coordinate frame:
        /// <pre>
        /// iner = [  int{x^2+z^2}dm   int{x^2+z^2}   int{x^2+y^2}dm ]
        /// </pre>
        public void SetInertiaXX(ChVector iner) {
            BodyFrame.variables.GetBodyInertia().SetElement(0, 0, iner.x);
            BodyFrame.variables.GetBodyInertia().SetElement(1, 1, iner.y);
            BodyFrame.variables.GetBodyInertia().SetElement(2, 2, iner.z);
            BodyFrame.variables.GetBodyInertia().FastInvert(BodyFrame.variables.GetBodyInvInertia());
        }
        /// Get the diagonal part of the inertia tensor (Ixx, Iyy, Izz values).
        /// The return 3x1 vector contains the following values:
        /// <pre>
        /// [  int{x^2+z^2}dm   int{x^2+z^2}   int{x^2+y^2}dm ]
        /// </pre>   
        public ChVector GetInertiaXX() {
            ChVector iner = new ChVector(0, 0, 0);
            iner.x = BodyFrame.variables.GetBodyInertia().GetElement(0, 0);
            iner.y = BodyFrame.variables.GetBodyInertia().GetElement(1, 1);
            iner.z = BodyFrame.variables.GetBodyInertia().GetElement(2, 2);
            return iner;
        }
        /// Set the off-diagonal part of the inertia tensor (Ixy, Ixz, Iyz values).
        /// Warning about sign: in some books they write the inertia tensor as
        /// I=[Ixx, -Ixy, -Ixz; etc.] but here is I=[Ixx, Ixy, Ixz; ...].
        /// The provided 3x1 vector should contain the products of inertia,
        /// expressed in the local coordinate frame:
        /// <pre>
        /// iner = [ -int{xy}dm   -int{xz}dm   -int{yz}dm ]
        /// </pre>
        public void SetInertiaXY(ChVector iner) {
            BodyFrame.variables.GetBodyInertia().SetElement(0, 1, iner.x);
            BodyFrame.variables.GetBodyInertia().SetElement(0, 2, iner.y);
            BodyFrame.variables.GetBodyInertia().SetElement(1, 2, iner.z);
            BodyFrame.variables.GetBodyInertia().SetElement(1, 0, iner.x);
            BodyFrame.variables.GetBodyInertia().SetElement(2, 0, iner.y);
            BodyFrame.variables.GetBodyInertia().SetElement(2, 1, iner.z);
            BodyFrame.variables.GetBodyInertia().FastInvert(BodyFrame.variables.GetBodyInvInertia());
        }
        /// Get the extra-diagonal part of the inertia tensor (Ixy, Ixz, Iyz values)
        /// Warning about sign: in some books they write the inertia tensor as
        /// I=[Ixx, -Ixy, -Ixz; etc.] but here is I=[Ixx, Ixy, Ixz; ...].
        /// The return 3x1 vector contains the following values:
        /// <pre>
        /// [ -int{xy}dm   -int{xz}dm   -int{yz}dm ]
        /// </pre>  
        public ChVector GetInertiaXY() {
            ChVector iner = new ChVector(0, 0, 0);
            iner.x = BodyFrame.variables.GetBodyInertia().GetElement(0, 1);
            iner.y = BodyFrame.variables.GetBodyInertia().GetElement(0, 2);
            iner.z = BodyFrame.variables.GetBodyInertia().GetElement(1, 2);
            return iner;
        }

        /// Set the maximum linear speed (beyond this limit it will be clamped).
        /// This is useful in virtual reality and real-time simulations, because
        /// it reduces the risk of bad collision detection.
        /// This speed limit is active only if you set  SetLimitSpeed(true);
        public void SetMaxSpeed(float m_max_speed) { max_speed = m_max_speed; }
        public float GetMaxSpeed() { return max_speed; }

        /// Set the maximum angular speed (beyond this limit it will be clamped).
        /// This is useful in virtual reality and real-time simulations, because
        /// it reduces the risk of bad collision detection.
        /// This speed limit is active only if you set  SetLimitSpeed(true);
        public void SetMaxWvel(float m_max_wvel) { max_wvel = m_max_wvel; }
        public float GetMaxWvel() { return max_wvel; }

        /// Clamp the body speed to the provided limits.
        /// When this function is called, the speed of the body is clamped
        /// to the range specified by max_speed and max_wvel. Remember to
        /// put the body in the SetLimitSpeed(true) mode.
        public void ClampSpeed() {
            if (this.GetLimitSpeed())
            {
                double w = 2.0 * this.BodyFrame.coord_dt.rot.Length();
                if (w > max_wvel)
                    BodyFrame.coord_dt.rot *= max_wvel / w;

                double v = this.BodyFrame.coord_dt.pos.Length();
                if (v > max_speed)
                    BodyFrame.coord_dt.pos *= max_speed / v;
            }
        }

        /// Set the amount of time which must pass before going automatically in
        /// sleep mode when the body has very small movements.
        public void SetSleepTime(float m_t) { sleep_time = m_t; }
        public float GetSleepTime() { return sleep_time; }

        /// Set the max linear speed to be kept for 'sleep_time' before freezing.
        public void SetSleepMinSpeed(float m_t) { sleep_minspeed = m_t; }
        public float GetSleepMinSpeed() { return sleep_minspeed; }

        /// Set the max linear speed to be kept for 'sleep_time' before freezing.
        public void SetSleepMinWvel(float m_t) { sleep_minwvel = m_t; }
        public float GetSleepMinWvel() { return sleep_minwvel; }

        /// Computes the 4x4 inertia tensor in quaternion space, if needed.
        public void ComputeQInertia(ChMatrixNM<IntInterface.Four, IntInterface.Four> mQInertia) {
            ChMatrixNM<IntInterface.Three, IntInterface.Four> res = new ChMatrixNM<IntInterface.Three, IntInterface.Four>();
            ChMatrixNM<IntInterface.Three, IntInterface.Four> Gl = new ChMatrixNM<IntInterface.Three, IntInterface.Four>();
            ChMatrixNM<IntInterface.Four, IntInterface.Three> GlT = new ChMatrixNM<IntInterface.Four, IntInterface.Three>();

            //ChFrame<double> gl = new ChFrame<double>();
            ChFrame<double>.SetMatrix_Gl(ref Gl, BodyFrame.coord.rot);
            GlT.CopyFromMatrixT(Gl);

            res.MatrMultiply(this.GetInertia(), Gl);
            mQInertia.MatrMultiply(GlT, res);  // [Iq]=[G'][Ix][G]
        }

        /// Computes the gyroscopic torque. In fact, in sake of highest
        /// speed, the gyroscopic torque isn't automatically updated each time a
        /// SetCoord() or SetCoord_dt() etc. is called, but only if necessary,
        /// for each UpdateState().
        public void ComputeGyro() {
            ChVector Wvel = this.BodyFrame.GetWvel_loc();
            gyro = ChVector.Vcross(Wvel, (BodyFrame.variables.GetBodyInertia().Matr_x_Vect(Wvel)));
        }

        /// Transform and adds a Cartesian force to a generic 7x1 vector of body lagrangian forces mQf .
        /// The Cartesian force must be passed as vector and application point, and vcan be either in local
        /// (local = true) or absolute reference (local = false)
        public void Add_as_lagrangian_force(ChVector force,
                                 ChVector appl_point,
                             bool local,
                             ChMatrixNM<IntInterface.Seven, IntInterface.One> mQf)
        {
            ChVector mabsforce = ChVector.VNULL;
            ChVector mabstorque = ChVector.VNULL;
            BodyFrame.To_abs_forcetorque(force, appl_point, local, ref mabsforce, ref mabstorque);
            mQf.PasteSumVector(mabsforce, 0, 0);
            mQf.PasteSumQuaternion(new ChFrame<double>().GlT_x_Vect(BodyFrame.coord.rot, Dir_World2Body(mabstorque)), 3, 0);
        }

        public void Add_as_lagrangian_torque(ChVector torque, bool local, ChMatrixNM<IntInterface.Seven, IntInterface.One> mQf) {
            ChVector mabstorque = new ChVector(0, 0, 0);
            BodyFrame.To_abs_torque(torque, local, ref mabstorque);
            mQf.PasteSumQuaternion(new ChFrame<double>().GlT_x_Vect(BodyFrame.coord.rot, Dir_World2Body(mabstorque)), 3, 0);
        }

        //
        // UTILITIES FOR FORCES/TORQUES:
        //

        /// Add forces and torques into the "accumulators", as increment.
        /// Forces and torques currently in accumulators will affect the body.
        /// It's up to the user to remember to empty them and/or set again at each
        /// integration step. Useful to apply forces to bodies without needing to
        /// add ChForce() objects. If local=true, force,appl.point or torque are considered
        /// expressed in body coordinates, otherwise are considered in absolute coordinates.
        public void Accumulate_force(ChVector force, ChVector appl_point, bool local) {
            ChVector mabsforce = ChVector.VNULL;
            ChVector mabstorque = ChVector.VNULL;
            BodyFrame.To_abs_forcetorque(force, appl_point, local, ref mabsforce, ref mabstorque);

            Force_acc += mabsforce;
            Torque_acc += mabstorque;
        }
        public void Accumulate_torque(ChVector torque, bool local) {
            ChVector mabstorque = ChVector.VNULL;
            BodyFrame.To_abs_torque(torque, local, ref mabstorque);
            Torque_acc += mabstorque;
        }
        public void Empty_forces_accumulators()
        {
            Force_acc = ChVector.VNULL;
            Torque_acc = ChVector.VNULL;
        }
        public ChVector Get_accumulated_force() { return Force_acc; }
        public ChVector Get_accumulated_torque() { return Torque_acc; }

        /// To get & set the 'script' force buffers(only accessed by external scripts, so
        /// It's up to the script to remember to set& reset them -link class just add them to
        /// all other forces. Script forces&torques are considered applied to COG, in abs csys.
        public ChVector Get_Scr_force() { return Scr_force; }
        public ChVector Get_Scr_torque() { return Scr_torque; }
        public void Set_Scr_force(ChVector mf) { Scr_force = mf; }
        public void Set_Scr_torque(ChVector mf) { Scr_torque = mf; }
        public void Accumulate_script_force(ChVector force, ChVector appl_point, bool local) {
            ChVector mabsforce = ChVector.VNULL;
            ChVector mabstorque = ChVector.VNULL;
            BodyFrame.To_abs_forcetorque(force, appl_point, local, ref mabsforce, ref mabstorque);

            Scr_force += mabsforce;
            Scr_torque += mabstorque;
        }
        public void Accumulate_script_torque(ChVector torque, bool local) {
            ChVector mabstorque = ChVector.VNULL;
            BodyFrame.To_abs_torque(torque, local, ref mabstorque);

            Scr_torque += mabstorque;
        }

        /// Return the gyroscopic torque.
        public ChVector Get_gyro() { return gyro; }

        /// Get the total force applied to the rigid body (applied at center of mass.
        /// expressed in absolute coordinates).
        public ChVector Get_Xforce() { return Xforce; }
        /// Get the total torque applied to the rigid body (expressed in body coordinates).
        /// This does not include the gyroscopic torque.
        public ChVector Get_Xtorque() { return Xtorque; }

        //
        // UPDATE FUNCTIONS
        //

        /// Update all children markers of the rigid body, at current body state
        public void UpdateMarkers(double mytime) {
            for (int i = 0; i < marklist.Count; i++)
            {
                marklist[i].update(mytime);
            }
        }
        /// Update all children forces of the rigid body, at current body state.
        public void UpdateForces(double mytime) {
            // COMPUTE LAGRANGIAN FORCES APPLIED TO BODY

            // 1a- force caused by accumulation of forces in body's accumulator Force_acc
            Xforce = Force_acc;

            // 1b- force caused by accumulation of torques in body's accumulator Force_acc
            if (ChVector.Vnotnull(Torque_acc)) {
                Xtorque = Dir_World2Body(Torque_acc);
            } else {
                Xtorque = ChVector.VNULL;
            }

            // Debug.Log("torque " + Xtorque.y);

            // 2 - accumulation of other applied forces
            ChVector mforce = new ChVector(0, 0, 0);
            ChVector mtorque = new ChVector(0, 0, 0);

            foreach (ChForce force in forcelist) {
                // update positions, f=f(t,q)
                force.update(mytime);

                force.GetBodyForceTorque(ref mforce, ref mtorque);
                Xforce += mforce;
                Xtorque += mtorque;
            }

            // 3 - accumulation of script forces
            Xforce += Scr_force;

            if (ChVector.Vnotnull(Scr_torque)) {
                Xtorque += Dir_World2Body(Scr_torque);
            }

            if (GetSystem()) {
                Xforce += GetSystem().Get_G_acc() * this.GetMass();
            }
        }

        /// Update local time of rigid body, and time-dependent data
        public void UpdateTime(double mytime) {
            SetChTime(mytime);
        }

        /// Update all auxiliary data of the rigid body and of
        /// its children (markers, forces..)
        public override void update(bool update_assets = true)
        {
            // TrySleeping();         // See if the body can fall asleep; if so, put it to sleeping
            ClampSpeed();   // Apply limits (if in speed clamping mode) to speeds.
            ComputeGyro();  // Set the gyroscopic momentum.

            // Also update the children "markers" and
            // "forces" depending on the body current state.
            UpdateMarkers(GetChTime());

            UpdateForces(GetChTime());
            chronotime = GetChTime();

            // This will update assets
            base.update(GetChTime(), update_assets);
        }

        /// Update all auxiliary data of the rigid body and of
        /// its children (markers, forces..), at given time
        public override void update(double mytime, bool update_assets = true) {
            // For the body:
            UpdateTime(mytime);

            update(update_assets);
        }

        //
        // INTERFACE TO ChContactable
        //

        public ChVariables GetVariables1() { return this.BodyFrame.variables; }

        public double GetMass() { return BodyFrame.variables.GetBodyMass(); }

        /// Tell if the object must be considered in collision detection
        public bool IsContactActive() {
            //this.m_variableTupleCarrier_1vars.Is
            return this.IsActive();
        }

        /// Get the number of DOFs affected by this object (position part)
        public int ContactableGet_ndof_x() { return 7; }

        /// Get the number of DOFs affected by this object (speed part)
        public int ContactableGet_ndof_w() { return 6; }

        /// Get all the DOFs packed in a single vector (position part)
        public void ContactableGetStateBlock_x(ref ChState x) { x.PasteCoordsys(this.BodyFrame.GetCoord(), 0, 0); }

        /// Get all the DOFs packed in a single vector (speed part)
        public void ContactableGetStateBlock_w(ref ChStateDelta w) {
            w.PasteVector(this.BodyFrame.GetPos_dt(), 0, 0);
            w.PasteVector(this.BodyFrame.GetWvel_loc(), 3, 0);
        }

        /// Increment the provided state of this object by the given state-delta increment.
        /// Compute: x_new = x + dw.
        public void ContactableIncrementState(ChState x, ChStateDelta dw, ref ChState x_new) {
            IntStateIncrement(0, ref x_new, x, 0, dw);
        }

        /// Return the pointer to the surface material.
        /// Use dynamic cast to understand if this is a ChMaterialSurfaceSMC, ChMaterialSurfaceNSC or others.
        /// This function returns a reference to the shared pointer member variable and is therefore THREAD SAFE.
        public ChMaterialSurface GetMaterialSurfaceBase() { return matsurface; }

        /// Get the resultant contact force acting on this body.
        public ChVector GetContactForce() {
            return GetSystem().GetContactContainer().GetContactableForce(this);
        }

        /// Get the resultant contact torque acting on this body.
        public ChVector GetContactTorque() {
            return GetSystem().GetContactContainer().GetContactableTorque(this);
        }

        /// Express the local point in absolute frame, for the given state position.
        ChVector ChContactable.GetContactPoint(ChVector loc_point, ChState state_x) {
            ChCoordsys csys = state_x.ClipCoordsys(0, 0);
            return csys.TransformPointLocalToParent(loc_point);
        }

        /// Express the local point in absolute frame, for the given state position.
        public ChVector GetContactPoint(ChVector loc_point, ChState state_x)
        {
            ChCoordsys csys = state_x.ClipCoordsys(0, 0);
            return csys.TransformPointLocalToParent(loc_point);
        }

        /// Get the absolute speed of a local point attached to the contactable.
        /// The given point is assumed to be expressed in the local frame of this object.
        /// This function must use the provided states.
        public ChVector GetContactPointSpeed(ChVector loc_point,
                                                    ChState state_x,
                                                    ChStateDelta state_w) {
            ChCoordsys csys = state_x.ClipCoordsys(0, 0);
            ChVector abs_vel = state_w.ClipVector(0, 0);
            ChVector loc_omg = state_w.ClipVector(3, 0);
            ChVector abs_omg = csys.TransformDirectionLocalToParent(loc_omg);

            return abs_vel + ChVector.Vcross(abs_omg, loc_point);
        }

        /// Get the absolute speed of point abs_point if attached to the surface.
        public ChVector GetContactPointSpeed(ChVector abs_point) {
            ChVector m_p1_loc = this.Point_World2Body(abs_point);
            return this.BodyFrame.PointSpeedLocalToParent(m_p1_loc);
        }

        /// Return the coordinate system for the associated collision model.
        /// ChCollisionModel might call this to get the position of the
        /// contact model (when rigid) and sync it.
        public ChCoordsys GetCsysForCollisionModel() { return new ChCoordsys(this.GetFrame_REF_to_abs().coord); }

        /// Apply the force, expressed in absolute reference, applied in pos, to the
        /// coordinates of the variables. Force for example could come from a penalty model.
        /// 
        public void ContactForceLoadResidual_F(ChVector F,
                                                    ChVector abs_point,
                                                ref ChVectorDynamic<double> R) {
            ChVector m_p1_loc = this.Point_World2Body(abs_point);
            ChVector force1_loc = this.Dir_World2Body(F);
            ChVector torque1_loc = ChVector.Vcross(m_p1_loc, force1_loc);
            R.PasteSumVector(F, (int)this.GetOffset_w() + 0, 0);
            R.PasteSumVector(torque1_loc, (int)this.GetOffset_w() + 3, 0);
        }

        /// Apply the given force at the given point and load the generalized force array.
        /// The force and its application point are specified in the global frame.
        /// Each object must set the entries in Q corresponding to its variables, starting at the specified offset.
        /// If needed, the object states must be extracted from the provided state position.
        public void ContactForceLoadQ(ChVector F,
                                       ChVector point,
                                       ChState state_x,
                                       ref ChVectorDynamic<double> Q,
                                       int offset)
        {
            ChCoordsys csys = state_x.ClipCoordsys(0, 0);
            ChVector point_loc = csys.TransformPointParentToLocal(point);
            ChVector force_loc = csys.TransformDirectionParentToLocal(F);
            ChVector torque_loc = ChVector.Vcross(point_loc, force_loc);
            Q.PasteVector(F, offset + 0, 0);
            Q.PasteVector(torque_loc, offset + 3, 0);
        }


        /// Compute the jacobian(s) part(s) for this contactable item.
        /// For a ChBody, this updates the corresponding 1x6 jacobian.
        /*public void ComputeJacobianForContactPart<Ta, Tb>(ChVector abs_point,
                                               ref ChMatrix33<double> contact_plane,
                                               ref ChConstraintTuple_1vars<ChVariableTupleCarrier_1vars<ChContactNSC<Ta, Tb>.typecarr_a>> jacobian_tuple_N,
                                               ref ChConstraintTuple_1vars<ChVariableTupleCarrier_1vars<ChContactNSC<Ta, Tb>.typecarr_a>> jacobian_tuple_U,
                                               ref ChConstraintTuple_1vars<ChVariableTupleCarrier_1vars<ChContactNSC<Ta, Tb>.typecarr_a>> jacobian_tuple_V,
                                               bool second)*/
        public void ComputeJacobianForContactPart(ChVector abs_point,
                                               ref ChMatrix33<double> contact_plane,
                                               ref IntInterface.ChVariableTupleCarrier_1vars.type_constraint_tuple jacobian_tuple_N,
                                               ref IntInterface.ChVariableTupleCarrier_1vars.type_constraint_tuple jacobian_tuple_U,
                                               ref IntInterface.ChVariableTupleCarrier_1vars.type_constraint_tuple jacobian_tuple_V,
                                               bool second)
        {
            ChVector m_p1_loc = this.Point_World2Body(abs_point);
            ChMatrix33<double> Jx1 = new ChMatrix33<double>();
            ChMatrix33<double> Jr1 = new ChMatrix33<double>();
            ChMatrix33<double> Ps1 = new ChMatrix33<double>();
            ChMatrix33<double> Jtemp = new ChMatrix33<double>();
            Ps1.Set_X_matrix(m_p1_loc);

            Jx1.CopyFromMatrixT(contact_plane);
            if (!second)
                Jx1.MatrNeg();

            Jtemp.MatrMultiply(this.BodyFrame.GetA(), Ps1);
            Jr1.MatrTMultiply(contact_plane, Jtemp);
            if (second)
                Jr1.MatrNeg();

            jacobian_tuple_N.Get_Cq().PasteClippedMatrix(Jx1, 0, 0, 1, 3, 0, 0);
            jacobian_tuple_U.Get_Cq().PasteClippedMatrix(Jx1, 1, 0, 1, 3, 0, 0);
            jacobian_tuple_V.Get_Cq().PasteClippedMatrix(Jx1, 2, 0, 1, 3, 0, 0);
            jacobian_tuple_N.Get_Cq().PasteClippedMatrix(Jr1, 0, 0, 1, 3, 0, 3);
            jacobian_tuple_U.Get_Cq().PasteClippedMatrix(Jr1, 1, 0, 1, 3, 0, 3);
            jacobian_tuple_V.Get_Cq().PasteClippedMatrix(Jr1, 2, 0, 1, 3, 0, 3);
        }

        /// Compute the jacobian(s) part(s) for this contactable item, for rolling about N,u,v.
        /// Used only for rolling friction NSC contacts.
        public void ComputeJacobianForRollingContactPart(ChVector abs_point,
                                                        ref ChMatrix33<double> contact_plane,
                                                        ref IntInterface.ChVariableTupleCarrier_1vars.type_constraint_tuple jacobian_tuple_N,
                                                        ref IntInterface.ChVariableTupleCarrier_1vars.type_constraint_tuple jacobian_tuple_U,
                                                        ref IntInterface.ChVariableTupleCarrier_1vars.type_constraint_tuple jacobian_tuple_V,
                                                        bool second) {
            ChMatrix33<double> Jx1 = new ChMatrix33<double>();
            ChMatrix33<double> Jr1 = new ChMatrix33<double>();

            Jr1.MatrTMultiply(contact_plane, this.BodyFrame.GetA());
            if (!second)
                Jr1.MatrNeg();

            jacobian_tuple_N.Get_Cq().PasteClippedMatrix(Jx1, 0, 0, 1, 3, 0, 0);
            jacobian_tuple_U.Get_Cq().PasteClippedMatrix(Jx1, 1, 0, 1, 3, 0, 0);
            jacobian_tuple_V.Get_Cq().PasteClippedMatrix(Jx1, 2, 0, 1, 3, 0, 0);
            jacobian_tuple_N.Get_Cq().PasteClippedMatrix(Jr1, 0, 0, 1, 3, 0, 3);
            jacobian_tuple_U.Get_Cq().PasteClippedMatrix(Jr1, 1, 0, 1, 3, 0, 3);
            jacobian_tuple_V.Get_Cq().PasteClippedMatrix(Jr1, 2, 0, 1, 3, 0, 3);
        }

        /// Used by some SMC code
        public double GetContactableMass() { return this.GetMass(); }

        /// This is only for backward compatibility
        ChPhysicsItem ChContactable.GetPhysicsItem() { return this; }

        //
        // INTERFACE to ChLoadable
        //

        /// Gets the number of DOFs affected by this element (position part)
        public int LoadableGet_ndof_x() { return 7; }

        /// Gets the number of DOFs affected by this element (speed part)
        public int LoadableGet_ndof_w() { return 6; }

        /// Gets all the DOFs packed in a single vector (position part)
        public void LoadableGetStateBlock_x(int block_offset, ChState mD) {
            mD.PasteCoordsys(this.BodyFrame.GetCoord(), block_offset, 0);
        }

        /// Gets all the DOFs packed in a single vector (speed part)
        public void LoadableGetStateBlock_w(int block_offset, ChStateDelta mD) {
            mD.PasteVector(this.BodyFrame.GetPos_dt(), block_offset, 0);
            mD.PasteVector(this.BodyFrame.GetWvel_loc(), block_offset + 3, 0);
        }

        /// Increment all DOFs using a delta.
        public void LoadableStateIncrement(int off_x, ChState x_new, ChState x, int off_v, ChStateDelta Dv) {
            IntStateIncrement(off_x, ref x_new, x, off_v, Dv);
        }


        /// Number of coordinates in the interpolated field, ex=3 for a
        /// tetrahedron finite element or a cable, etc. Here is 6: xyz displ + xyz rots
        public int Get_field_ncoords() { return 6; }

        /// Tell the number of DOFs blocks (ex. =1 for a body, =4 for a tetrahedron, etc.)
        public int GetSubBlocks() { return 1; }

        /// Get the offset of the i-th sub-block of DOFs in global vector
        public int GetSubBlockOffset(int nblock) { return this.GetOffset_w(); }

        /// Get the size of the i-th sub-block of DOFs in global vector
        public int GetSubBlockSize(int nblock) { return 6; }

        /// Get the pointers to the contained ChVariables, appending to the mvars vector.
        public void LoadableGetVariables(List<ChVariables> mvars) {
            mvars.Add(this.Variables());
        }

        /// Evaluate Q=N'*F , for Q generalized lagrangian load, where N is some type of matrix
        /// evaluated at point P(U,V,W) assumed in absolute coordinates, and
        /// F is a load assumed in absolute coordinates.
        /// The det[J] is unused.
        public void ComputeNF(
                double U,              //< x coordinate of application point in absolute space
                double V,              //< y coordinate of application point in absolute space
                double W,              //< z coordinate of application point in absolute space
                ChVectorDynamic<double> Qi,       //< Return result of N'*F  here, maybe with offset block_offset
                double detJ,                //< Return det[J] here
                ChVectorDynamic<double> F,  //< Input F vector, size is 6, it is {Force,Torque} in absolute coords.
                ChVectorDynamic<double> state_x,  //< if != 0, update state (pos. part) to this, then evaluate Q
                ChVectorDynamic<double> state_w   /// if != 0, update state (speed part) to this, then evaluate Q
        ) {
            ChVector abs_pos = new ChVector(U, V, W);
            ChVector absF = F.ClipVector(0, 0);
            ChVector absT = F.ClipVector(3, 0);
            ChVector body_absF = ChVector.VNULL;
            ChVector body_locT = ChVector.VNULL;
            ChCoordsys bodycoord = ChCoordsys.CSYSNULL;
            if (state_x != null)
                bodycoord = state_x.ClipCoordsys(0, 0);  // the numerical jacobian algo might change state_x
            else
                bodycoord = this.BodyFrame.coord;
            // compute Q components F,T, given current state of body 'bodycoord'. Note T in Q is in local csys, F is an abs
            // csys
            body_absF = absF;
            body_locT = bodycoord.rot.RotateBack(absT + ((abs_pos - bodycoord.pos) % absF));
            Qi.PasteVector(body_absF, 0, 0);
            Qi.PasteVector(body_locT, 3, 0);
            detJ = 1;  // not needed because not used in quadrature.
        }

        /// This is not needed because not used in quadrature.
        public double GetDensity() { return density; }



        /// Bit flags
        public enum BodyFlag
        {
            COLLIDE = ((int)1L << 0),          // detects collisions
            CDINVISIBLE = ((int)1L << 1),      // collision detection invisible
            EVAL_CONTACT_CN = ((int)1L << 2),  // evaluate CONTACT_CN channel (normal restitution)
            EVAL_CONTACT_CT = ((int)1L << 3),  // evaluate CONTACT_CT channel (tangential rest.)
            EVAL_CONTACT_KF = ((int)1L << 4),  // evaluate CONTACT_KF channel (kinetic friction coeff)
            EVAL_CONTACT_SF = ((int)1L << 5),  // evaluate CONTACT_SF channel (static friction coeff)
            SHOW_COLLMESH = ((int)1L << 6),    // show collision mesh - obsolete
            FIXED = ((int)1L << 7),            // body is fixed to ground
            LIMITSPEED = ((int)1L << 8),       // body angular and linear speed is limited (clamped)
            SLEEPING = ((int)1L << 9),         // body is sleeping [internal]
            USESLEEPING = ((int)1L << 10),     // if body remains in same place for too long time, it will be frozen
            NOGYROTORQUE = ((int)1L << 11),    // do not get the gyroscopic (quadratic) term, for low-fi but stable simulation
            COULDSLEEP = ((int)1L << 12)       // if body remains in same place for too long time, it will be frozen
        };

        int bflags = 0;  //< encoding for all body flags

        // Flags handling functions
        protected void BFlagsSetAllOFF()
        {
            bflags = 0;
        }
        void BFlagsSetAllON()
        {
            bflags = 0;
            bflags = ~bflags;
        }
        void BFlagSetON(BodyFlag mask) {
            bflags |= (int)mask;
        }
        void BFlagSetOFF(BodyFlag mask)
        {
            bflags &= (int)~mask;
        }
        public void BFlagSet(BodyFlag mask, bool state)
        {
            if (state)
                bflags |= (int)mask;
            else
                bflags &= (int)~mask;
        }
        public bool BFlagGet(BodyFlag mask)
        {
            return (bflags & (int)mask) != 0;
        }

        public const int BODY_DOF = 6;   //< degrees of freedom of body in 3d space
        public const int BODY_QDOF = 7;  //< degrees of freedom with quaternion rotation state
        public const int BODY_ROT = 3;   //< rotational dof in Newton dynamics
    }

    
  
}



