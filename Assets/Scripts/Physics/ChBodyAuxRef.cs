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
        private ChFrameMoving<double> auxref_to_cog; //< auxiliary REF location, relative to COG
        private ChFrameMoving<double> auxref_to_abs; //< auxiliary REF location, relative to abs coords (needs Update() )

        public bool showFrameGizmo;

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

        public override void Awake()
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
                    matsurface.GetComponent<ChMaterialSurfaceSMC>().young_modulus = young_modulus;
                    matsurface.GetComponent<ChMaterialSurfaceSMC>().poisson_ratio = poisson_ratio;
                    matsurface.GetComponent<ChMaterialSurfaceSMC>().sliding_friction = static_friction;
                    matsurface.GetComponent<ChMaterialSurfaceSMC>().sliding_friction = sliding_friction;
                    matsurface.GetComponent<ChMaterialSurfaceSMC>().constant_adhesion = constant_adhesion;
                    matsurface.GetComponent<ChMaterialSurfaceSMC>().adhesionMultDMT = adhesionMultDMT;
                    break;
            }

            // Switch
            switch (type)
            {
                case CollisionType.Cube:

                    var size = transform.localScale;

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
                        GetCollisionModel().AddBox(size.x * 0.473f, size.y * 0.473f, size.z * 0.473f, new ChVector(), new ChMatrix33<double>(1));  // radius x, radius z, height on y
                        GetCollisionModel().BuildModel();
                        SetCollide(true);
                    }

                    SetFrame_COG_to_REF(new ChFrame<double>(Utils.ToChrono(COM), new ChQuaternion(1, 0, 0, 0)));
                    SetInertiaXX(ToChrono(inertiaMoments));
                    SetInertiaXY(ToChrono(inertiaProducts));

                    BodyFrame.SetPos(new ChVector(transform.position.x, transform.position.y, transform.position.z));
                    BodyFrame.SetRot(new ChQuaternion(transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z));

                    BodyFrame.SetPos_dt(ToChrono(linearVelocity));
                    BodyFrame.SetWvel_loc(ToChrono(angularVelocity));

                    ChSystem msystem = FindObjectOfType<ChSystem>();
                    msystem.AddBody(this);

                    break;
                case CollisionType.Sphere:

                    var size2 = transform.localScale.y / 2;

                    if (automaticMass)
                    {
                        mass = density * ((4.0 / 3.0) * Math.PI * Math.Pow(size2, 3));
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
                        GetCollisionModel().AddSphere(size2, new ChVector());  // radius, radius, height on y
                        GetCollisionModel().BuildModel();
                        SetCollide(true);
                    }

                    if (collide)
                    {
                        GetCollisionModel().ClearModel();
                        GetCollisionModel().AddSphere(size2, new ChVector());  // radius, radius, height on y
                        GetCollisionModel().BuildModel();
                        SetCollide(true);
                    }

                    SetFrame_COG_to_REF(new ChFrame<double>(Utils.ToChrono(COM), new ChQuaternion(1, 0, 0, 0)));
                    SetInertiaXX(ToChrono(inertiaMoments));
                    SetInertiaXY(ToChrono(inertiaProducts));

                    SetBodyFixed(bodyfixed);

                    ChFrame<double> frame = new ChFrameMoving<double>(Utils.ToChrono(transform.position), Utils.ToChrono(transform.rotation));
                    SetFrame_REF_to_abs(frame);

                    //BodyFrame.SetPos(new ChVector(transform.position.x, transform.position.y, transform.position.z));
                   // BodyFrame.SetRot(new ChQuaternion(transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z));

                    ChSystem msystem2 = FindObjectOfType<ChSystem>();
                    msystem2.AddBody(this);

                    break;
            }

        }

        public override collision.ChCollisionModel GetCollisionModel() { return collision_model; }


        public override void FixedUpdate()
        {
            base.FixedUpdate();
           /* var frame = GetFrame_REF_to_abs();
            transform.position = Utils.FromChrono(frame.GetPos());
            transform.rotation = Utils.FromChrono(frame.GetRot());*/
        }
        void OnDrawGizmos()
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
            mfra.TransformLocalToParent(this.auxref_to_cog/*.GetInverse()*/, this.BodyFrame);
            // or, also, using overloaded operators for frames:
            //   *this = this->auxref_to_cog.GetInverse() >> mfra;

            auxref_to_abs = (ChFrameMoving<double>)mfra;
        }

        /// Get the auxiliary reference frame with respect to the absolute frame.
        /// Note that, in general, this is different from GetFrame_COG_to_abs().
        public override ChFrameMoving<double> GetFrame_REF_to_abs() { return auxref_to_abs; }

        /// Set the COG frame with respect to the auxiliary reference frame.
        /// Note that this also moves the body absolute COG (the REF is fixed).
        /// The position of contained ChMarker objects, if any, is not changed with respect
        /// to the reference.
        public void SetFrame_COG_to_REF(ChFrame<double> mloc) {
            ChFrameMoving<double> old_cog_to_abs = this.BodyFrame;

            ChFrameMoving<double> tmpref_to_abs = new ChFrameMoving<double>();
            this.BodyFrame.TransformLocalToParent(this.auxref_to_cog, tmpref_to_abs);
            tmpref_to_abs.TransformLocalToParent(new ChFrameMoving<double>(mloc), this.BodyFrame);
            // or, also, using overloaded operators for frames:
            //   tmpref_to_abs = auxref_to_cog >> *this;
            //   *this         = ChFrameMoving<>(mloc) >> tmpref_to_abs;

            ChFrameMoving<double> new_cog_to_abs = this.BodyFrame;

            auxref_to_cog = mloc.GetInverse();

            this.BodyFrame.TransformLocalToParent(this.auxref_to_cog, this.auxref_to_abs);
            // or, also, using overloaded operators for frames:
            //   *this->auxref_to_abs = this->auxref_to_cog.GetInverse() >> this;

            // Restore marker/forces positions, keeping unchanged respect to aux ref.
            ChFrameMoving<double> cog_oldnew = ChFrameMoving<double>.BitShiftRight(old_cog_to_abs, new_cog_to_abs.GetInverse()); // Might not work?

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
            this.BodyFrame.TransformLocalToParent(this.auxref_to_cog, ref this.auxref_to_abs);
        }

    }

 
}
