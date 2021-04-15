using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;


namespace chrono
{
    /// <summary>
    /// Class for enforcing a fixed polar distance
    /// between two points on two ChBodyFrame objects.
    /// The two points which are used to define the end points
    /// of the distance are assumed not to move respect to the
    /// two owner ChBody, as well as the amount of the distance
    /// is assumed not to change during the simulation. If you
    /// need to have a time-varying distance, or distance between
    /// two points which move respect to the bodies, please use
    /// the more advanced ChLinkLinActuator.
    /// </summary>
    public class ChLinkDistance : ChLink
    {
        protected double distance;           ////< the imposed distance
        protected ChVector pos1;           ////< first endpoint, in body rel.coords
        protected ChVector pos2;           ////< second endpoint, in body rel.coords
        protected ChConstraintTwoBodies Cx = new ChConstraintTwoBodies();  ////< the constraint object
        protected double curr_dist;          ////< used for internal optimizations

        public ChBody body1;
        public ChBody body2;

        //public Vector3 position1;
        // public Vector3 position2;
        public Transform position2;

        public bool useRelativePos = false;

        private ChVector position1Abs;
        private ChVector position2Abs;

        private Vector3 EndPoint1Abs = new Vector3();
        private Vector3 EndPoint2Abs = new Vector3();


        public ChLinkDistance() {
            pos1 = new ChVector(0, 0, 0);
            pos2 = new ChVector(0, 0, 0);
            distance = 0;
            curr_dist = 0;
        }
        public ChLinkDistance(ChLinkDistance other) {
            Body1 = other.Body1;
            Body2 = other.Body2;
           // system = other.system;
            Cx.SetVariables(other.Body1.Variables(), other.Body2.Variables());
            pos1 = other.pos1;
            pos2 = other.pos2;
            distance = other.distance;
            curr_dist = other.curr_dist;
        }

        /// "Virtual" copy constructor (covariant return type).
        public override ChObj Clone()  { return new ChLinkDistance(this); }

        public void Start()
        {
            pos1.x = transform.position.x;
            pos1.y = transform.position.y;
            pos1.z = transform.position.z;
            pos2.x = position2.position.x;
            pos2.y = position2.position.y;
            pos2.z = position2.position.z;
            Initialize(body1.BodyFrame, body2.BodyFrame, useRelativePos, pos1, pos2);

            ChSystem msystem = FindObjectOfType<ChSystem>();
            msystem.AddLink(this);
        }

        /// Initialize this constraint, given the two bodies to be connected, the
        /// positions of the two anchor endpoints of the distance (each expressed
        /// in body or abs. coordinates) and the imposed distance.
        public virtual bool Initialize(
                ChBodyFrame mbody1,  //< first frame to link
                ChBodyFrame mbody2,  //< second frame to link
                bool pos_are_relative,                //< true: following pos. are relative to bodies
                ChVector mpos1,                     //< pos. of distance endpoint, for 1st body (rel. or abs., see flag above)
                ChVector mpos2,                     //< pos. of distance endpoint, for 2nd body (rel. or abs., see flag above)
                bool auto_distance = true,            //< if true, initializes the imposed distance as the distance between mpos1 and mpos2
                double mdistance = 0                  //< imposed distance (no need to define, if auto_distance=true.)
        )
        {
            Body1 = mbody1;
            Body2 = mbody2;
            Cx.SetVariables(Body1.Variables(), Body2.Variables());

            if (pos_are_relative)
            {
                pos1 = mpos1;
                pos2 = mpos2;
            }
            else
            {
                pos1 = Body1.TransformPointParentToLocal(mpos1);
                pos2 = Body2.TransformPointParentToLocal(mpos2);
            }

            ChVector AbsDist = Body1.TransformPointLocalToParent(pos1) - Body2.TransformPointLocalToParent(pos2);
            curr_dist = AbsDist.Length();

            if (auto_distance)
            {
                distance = curr_dist;
            }
            else
            {
                distance = mdistance;
            }

            return true;
        }

        /// Get the number of (bilateral) constraints introduced by this link.
        public override int GetDOC_c() { return 1; }

        /// Get the link coordinate system, expressed relative to Body2 (the 'master'
        /// body). This represents the 'main' reference of the link: reaction forces
        /// are expressed in this coordinate system.
        /// (It is the coordinate system of the contact plane relative to Body2)
        public override ChCoordsys GetLinkRelativeCoords() {
            //ChVector D2local;
            ChVector D2temp = (ChVector.Vnorm(Body1.TransformPointLocalToParent(pos1) - Body2.TransformPointLocalToParent(pos2)));
            ChVector D2rel = Body2.TransformDirectionParentToLocal(D2temp);
            ChVector Vx = new ChVector(0, 0, 0), Vy = new ChVector(0, 0, 0), Vz = new ChVector(0, 0, 0);
            ChMatrix33<double> rel_matrix = new ChMatrix33<double>();
            ChVector.XdirToDxDyDz(D2rel, ChVector.VECT_Y, ref Vx, ref Vy, ref Vz);
            rel_matrix.Set_A_axis(Vx, Vy, Vz);

            ChQuaternion Ql2 = rel_matrix.Get_A_quaternion();
            return new ChCoordsys(pos2, Ql2);
        }

        /// Get the 1st anchor endpoint for the distance (expressed in Body1 coordinate system)
        public ChVector GetEndPoint1Rel() { return pos1; }
        /// Set the 1st anchor endpoint for the distance (expressed in Body1 coordinate system)
        public void SetEndPoint1Rel(ChVector mset) { pos1 = mset; }
        /// Get the 1st anchor endpoint for the distance (expressed in absolute coordinate system)
        public ChVector GetEndPoint1Abs() { return ((ChFrame<double>)Body1).TransformLocalToParent(pos1); }
        /// Set the 1st anchor endpoint for the distance (expressed in absolute coordinate system)
        public void SetEndPoint1Abs(ChVector mset) { pos1 = ((ChFrame<double>)Body1).TransformParentToLocal(mset); }

        /// Get the 2nd anchor endpoint for the distance (expressed in Body2 coordinate system)
        public ChVector GetEndPoint2Rel() { return pos2; }
        /// Set the 2nd anchor endpoint for the distance (expressed in Body2 coordinate system)
        public void SetEndPoint2Rel(ChVector mset) { pos2 = mset; }
        /// Get the 1st anchor endpoint for the distance (expressed in absolute coordinate system)
        public ChVector GetEndPoint2Abs() { return ((ChFrame<double>) Body2).TransformLocalToParent(pos2); }
        /// Set the 1st anchor endpoint for the distance (expressed in absolute coordinate system)
        public void SetEndPoint2Abs(ref ChVector mset) { pos2 = ((ChFrame<double>)Body2).TransformParentToLocal(mset); }

        /// Get the imposed distance
        public double GetImposedDistance() { return distance; }
        /// Set the imposed distance
        public void SetImposedDistance(double mset) { distance = mset; }
        /// Get the distance currently existing between the two endpoints
        public double GetCurrentDistance() {
            return (((ChFrame<double>)Body1).TransformLocalToParent(pos1) -
            ((ChFrame<double>)Body2).TransformLocalToParent(pos2))
        .Length();
        }

        /// Get the constraint violation
        public double GetC() { return GetCurrentDistance() - distance; }

        /// Override _all_ time, jacobian etc. updating.
        /// In detail, it computes jacobians, violations, etc. and stores
        /// results in inner structures.
        public override void update(double mytime, bool update_assets = true) {
            // Inherit time changes of parent class (ChLink), basically doing nothing :)
            base.update(mytime, update_assets);

            // compute jacobians
            ChVector AbsDist = Body1.TransformPointLocalToParent(pos1) - Body2.TransformPointLocalToParent(pos2);
            curr_dist = AbsDist.Length();
            ChVector D2abs = ChVector.Vnorm(AbsDist);
            ChVector D2relB = Body2.TransformDirectionParentToLocal(D2abs);
            ChVector D2relA = Body1.TransformDirectionParentToLocal(D2abs);

            ChVector CqAx = D2abs;
            ChVector CqBx = -D2abs;

            ChVector CqAr = -ChVector.Vcross(D2relA, pos1);
            ChVector CqBr = ChVector.Vcross(D2relB, pos2);

            Cx.Get_Cq_a().ElementN(0) = CqAx.x;
            Cx.Get_Cq_a().ElementN(1) = CqAx.y;
            Cx.Get_Cq_a().ElementN(2) = CqAx.z;
            Cx.Get_Cq_a().ElementN(3) = CqAr.x;
            Cx.Get_Cq_a().ElementN(4) = CqAr.y;
            Cx.Get_Cq_a().ElementN(5) = CqAr.z;

            Cx.Get_Cq_b().ElementN(0) = CqBx.x;
            Cx.Get_Cq_b().ElementN(1) = CqBx.y;
            Cx.Get_Cq_b().ElementN(2) = CqBx.z;
            Cx.Get_Cq_b().ElementN(3) = CqBr.x;
            Cx.Get_Cq_b().ElementN(4) = CqBr.y;
            Cx.Get_Cq_b().ElementN(5) = CqBr.z;

            //***TO DO***  C_dt? C_dtdt? (may be never used..)
        }

        //
        // STATE FUNCTIONS
        //

        public override void IntStateGatherReactions(int off_L, ref ChVectorDynamic<double> L) {
            L[off_L] = -react_force.x;
        }
        public override void IntStateScatterReactions(int off_L, ChVectorDynamic<double> L) {
            react_force.x = -L[off_L];
            react_force.y = 0;
            react_force.z = 0;

            react_torque = ChVector.VNULL;
        }
        public override void IntLoadResidual_CqL(int off_L,
                                         ref ChVectorDynamic<double> R,
                                         ChVectorDynamic<double> L,
                                         double c) {
            if (!IsActive())
                return;

            Cx.MultiplyTandAdd(R, L[off_L] * c);
        }
        public override void IntLoadConstraint_C(int off_L,
                                         ref ChVectorDynamic<double> Qc,
                                         double c,
                                         bool do_clamp,
                                         double recovery_clamp) {
            if (!IsActive())
                return;

            if (do_clamp)
                Qc[off_L] += ChMaths.ChMin(ChMaths.ChMax(c * (curr_dist - distance), -recovery_clamp), recovery_clamp);
            else
                Qc[off_L] += c * (curr_dist - distance);
        }
        public override void IntToDescriptor(int off_v,
                                     ChStateDelta v,
                                     ChVectorDynamic<double> R,
                                     int off_L,
                                     ChVectorDynamic<double> L,
                                     ChVectorDynamic<double> Qc) {
            if (!IsActive())
                return;

            Cx.Set_l_i(L[off_L]);

            Cx.Set_b_i(Qc[off_L]);
        }
        public override void IntFromDescriptor(int off_v,
                                       ref ChStateDelta v,
                                       int off_L,
                                       ref ChVectorDynamic<double> L) {
            if (!IsActive())
                return;

            L[off_L] = Cx.Get_l_i();
        }

        //
        // SOLVER INTERFACE
        //

        public override void InjectConstraints(ref ChSystemDescriptor mdescriptor) {
            if (!IsActive())
                return;

            mdescriptor.InsertConstraint(Cx);
        }
        public override void ConstraintsBiReset() {
            Cx.Set_b_i(0.0);
        }
        public override void ConstraintsBiLoad_C(double factor = 1, double recovery_clamp = 0.1, bool do_clamp = false) {
            if (!IsActive())
                return;

            if (do_clamp)
                Cx.Set_b_i(Cx.Get_b_i() + ChMaths.ChMin(ChMaths.ChMax(factor * (curr_dist - distance), -recovery_clamp), recovery_clamp));
            else
                Cx.Set_b_i(Cx.Get_b_i() + factor * (curr_dist - distance));
        }
        public override void ConstraintsLoadJacobians() {
            // already loaded when doing Update (which used the matrices of the scalar constraint objects)
        }
        public override void ConstraintsFetch_react(double factor = 1) {
            // From constraints to react vector:
            react_force.x = -Cx.Get_l_i() * factor;
            react_force.y = 0;
            react_force.z = 0;

            react_torque = new ChVector(0, 0, 0);
        }

        private void FixedUpdate()
        {
            position1Abs = GetEndPoint1Abs();
            position2Abs = GetEndPoint2Abs();

            EndPoint1Abs.x = (float)position1Abs.x;
            EndPoint1Abs.y = (float)position1Abs.y;
            EndPoint1Abs.z = (float)position1Abs.z;
            EndPoint2Abs.x = (float)position2Abs.x;
            EndPoint2Abs.y = (float)position2Abs.y;
            EndPoint2Abs.z = (float)position2Abs.z;

            //p1 = (float)GetPoint2Abs().GetX(), (float)GetPoint2Abs().GetY(), (float)GetPoint2Abs().GetZ());
        }

        void OnDrawGizmos()
        {            
            if (Application.isPlaying)
            {
                Gizmos.color = new Color(0, 255, 0);
                Gizmos.DrawSphere(EndPoint1Abs, 0.01f);

               // Gizmos.color = new Color(0, 0, 255);
               // Gizmos.DrawSphere(EndPoint2Abs, 0.01f);

                Gizmos.color = new Color(255, 0, 20);
                Gizmos.DrawLine(EndPoint1Abs, EndPoint2Abs);
            }
            else
            {
                Gizmos.color = new Color(0, 255, 0);
                Gizmos.DrawSphere(transform.position, 0.01f);

               // Gizmos.color = new Color(0, 0, 255);
               // Gizmos.DrawSphere(position2.position, 0.01f);

                Gizmos.color = new Color(255, 0, 20);
                Gizmos.DrawLine(transform.position, position2.position);
            }
        }

    };
}
