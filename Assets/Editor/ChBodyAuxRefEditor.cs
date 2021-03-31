using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

namespace chrono
{

    // ChBodyAuxReference Editor ////
    [CustomEditor(typeof(ChBodyAuxRef))]
    public class ChBodyAuxRefEditor : Editor
    {
        SerializedProperty density;

        // NSC material properties
        SerializedProperty friction;

        SerializedProperty rolling_friction;
        SerializedProperty spinning_friction;
        SerializedProperty restitution;
        SerializedProperty cohesion;
        SerializedProperty dampingf;
        SerializedProperty compliance;
        SerializedProperty complianceT;
        SerializedProperty complianceRoll;
        SerializedProperty complianceSpin;

        // SMC material properties
        SerializedProperty young_modulus;
        SerializedProperty poisson_ratio;      /// Poisson ratio
        SerializedProperty static_friction;    /// Static coefficient of friction
        SerializedProperty sliding_friction;   /// Kinetic coefficient of friction        
        SerializedProperty constant_adhesion;  /// Constant adhesion force, when constant adhesion model is used
        SerializedProperty adhesionMultDMT;    /// Adhesion multiplier used in DMT model.

        public void OnEnable()
        {

            friction = serializedObject.FindProperty("friction");
            rolling_friction = serializedObject.FindProperty("rolling_friction");
            spinning_friction = serializedObject.FindProperty("spinning_friction");
            restitution = serializedObject.FindProperty("restitution");
            cohesion = serializedObject.FindProperty("cohesion");
            dampingf = serializedObject.FindProperty("dampingf");
            compliance = serializedObject.FindProperty("compliance");
            complianceT = serializedObject.FindProperty("complianceT");
            complianceRoll = serializedObject.FindProperty("complianceRoll");
            complianceSpin = serializedObject.FindProperty("complianceSpin");
            density = serializedObject.FindProperty("density");

            young_modulus = serializedObject.FindProperty("young_modulus");
            poisson_ratio = serializedObject.FindProperty("poisson_ratio");
            static_friction = serializedObject.FindProperty("static_friction");
            sliding_friction = serializedObject.FindProperty("sliding_friction");
            constant_adhesion = serializedObject.FindProperty("constant_adhesion");
            adhesionMultDMT = serializedObject.FindProperty("adhesionMultDMT");
        }

        public void OnInspectorGUI()
        {
            ChBodyAuxRef body = (ChBodyAuxRef)target;

            EditorGUILayout.HelpBox("Class for rigid bodies with an auxiliary reference frame. Unlike ChBody, where the COG frame is used as the reference frame, the auxiliary reference frame of a ChBodyAuxRef can be different from its COG frame.  This specialization is provided for situations where it is more convenient to specify collision shapes, visualization assets, and marker positions with respect to a reference frame other than the COG frame. Note that, because of the auxiliary reference, this type of rigid bodies can be slightly less efficient than the base ChBody object.", MessageType.Info);

            GUIContent type = new GUIContent("Collision Type", "Change what type of collision shape.");
            body.type = (ChBodyAuxRef.CollisionType)EditorGUILayout.EnumPopup(type, body.type);
            GUIContent bodyfixed = new GUIContent("Fixed", "Toggle whether the object is a static/fixed rigid body or dynamic.");
            body.bodyfixed = EditorGUILayout.Toggle(bodyfixed, body.bodyfixed);
            GUIContent collide = new GUIContent("Collide", "Toggle whether the rigid body can collide and interact with other collidable rigid body.");
            body.collide = EditorGUILayout.Toggle(collide, body.collide);

            GUIContent autoInertia = new GUIContent("Automatic Mass/Inertia/COM", "Toggle whether the rigid body's mass, inertia and COM is calculated automatically based on dimensions and custom density, or if you would like to set the mass, inertia and COM manually.");
            body.automaticMass = EditorGUILayout.Toggle(autoInertia, body.automaticMass);

            if (!body.automaticMass)
            {
                GUIContent mass = new GUIContent("Mass", "Mass of the rigid body. Must be positive. Try not to mix bodies with too high/too low values of mass, for numerical stability.");
                body.mass = EditorGUILayout.DoubleField(mass, body.mass);
                GUIContent com = new GUIContent("COM", "Set the COG frame with respect to the auxiliary reference frame. Note that this also moves the body absolute COG (the REF is fixed). The position of contained ChMarker objects, if any, is not changed with respect to the reference.");
                body.COM = EditorGUILayout.Vector3Field(com, body.COM);
                GUIContent inertXX = new GUIContent("Moments of Inertia", "Advanced, Set the diagonal part of the inertia tensor, The provided 3x1 vector should contain the moments of inertia, expressed in the local coordinate frame");
                body.inertiaMoments = EditorGUILayout.Vector3Field(inertXX, body.inertiaMoments);
                GUIContent inertXY = new GUIContent("Products of Inertia", "Advanced, Set the off-diagonal part of the inertia tensor (Ixy, Ixz, Iyz values). Warning about sign: in some books they write the inertia tensor as I=[Ixx, -Ixy, -Ixz; etc.] but here is I=[Ixx, Ixy, Ixz; ...]. The provided 3x1 vector should contain the products of inertia, expressed in the local coordinate frame:");
                body.inertiaProducts = EditorGUILayout.Vector3Field(inertXY, body.inertiaProducts);
            }
            else
            {
                GUIContent density = new GUIContent("MDensity", "Set the rigid body density. Used when doing the 'recompute mass' operation.");
                body.density = EditorGUILayout.FloatField(density, body.density);
            }

            EditorGUILayout.LabelField("", GUI.skin.horizontalSlider);

            GUIContent linVel = new GUIContent("Linear Velocity", "Set the linear speed.  This is executed at startup once.");
            body.linearVelocity = EditorGUILayout.Vector3Field(linVel, body.linearVelocity);
            GUIContent angVel = new GUIContent("Angular Velocity", "Set the rotation speed from given angular speed.  This is executed at startup once.");
            body.angularVelocity = EditorGUILayout.Vector3Field(angVel, body.angularVelocity);

            EditorGUILayout.LabelField("", GUI.skin.horizontalSlider);

            GUIContent matType = new GUIContent("Material Type", "Change the material physics type. NSC surface for use with non-smooth (complementarity) contact method.  SMC Material data for a surface for use with smooth (penalty) contact method. This data is used to define surface properties owned by ChBody rigid bodies and similar objects; it carries information that is used to make contacts.");
            body.materialType = (ChBody.MaterialType)EditorGUILayout.EnumPopup("Material Type", body.materialType);

            EditorGUI.indentLevel++;
            switch (body.materialType)
            {
                case ChBody.MaterialType.NSC:
                    EditorGUILayout.PropertyField(friction, new GUIContent("Friction"), GUILayout.Height(20));
                    GUIContent rolling = new GUIContent("Rolling Friction", "The rolling friction (rolling parameter, it has the dimension of a length). Rolling resistant torque is Tr <= (normal force) * (this parameter) Usually a very low value. Measuring unit: m  Default =0. Note! a non-zero value will make the simulation 2x slower! Also, the GPU solver currently does not support rolling friction. Default: 0.");
                    EditorGUILayout.PropertyField(rolling_friction, new GUIContent(rolling), GUILayout.Height(20));
                    GUIContent spinning = new GUIContent("Spinning Friction", "The spinning friction (it has the dimension of a length). Spinning resistant torque is Ts <= (normal force) * (this parameter) Usually a very low value.  Measuring unit: m Default =0. Note! a non-zero value will make the simulation 2x slower! Also, the GPU solver currently does not support spinning friction. Default: 0.");
                    EditorGUILayout.PropertyField(spinning_friction, new GUIContent(spinning), GUILayout.Height(20));
                    GUIContent rest = new GUIContent("Restitution", "The normal restitution coefficient, for collisions. Should be in 0..1 range. Default =0.");
                    EditorGUILayout.PropertyField(restitution, new GUIContent(rest), GUILayout.Height(20));
                    GUIContent coh = new GUIContent("Cohesion", "The cohesion max. force for normal pulling traction in contacts. Measuring unit: N Default =0.");
                    EditorGUILayout.PropertyField(cohesion, new GUIContent(coh), GUILayout.Height(20));
                    GUIContent dampf = new GUIContent("Dampingf", "The damping in contact, as a factor 'f': damping is a multiple of stiffness [K], that is: [R]=f*[K] Measuring unit: time, s. Default =0.");
                    EditorGUILayout.PropertyField(dampingf, new GUIContent(dampf), GUILayout.Height(20));
                    GUIContent comp = new GUIContent("Compliance", "Compliance of the contact, in normal direction. It is the inverse of the stiffness [K] , so for zero value one has a perfectly rigid contact. Measuring unit: m/N Default =0.");
                    EditorGUILayout.PropertyField(compliance, new GUIContent(comp), GUILayout.Height(20));
                    GUIContent compt = new GUIContent("ComplianceT", "Compliance of the contact, in normal direction. It is the inverse of the stiffness [K] , so for zero value one has a perfectly rigid contact. Measuring unit: m/N Default =0.");
                    EditorGUILayout.PropertyField(complianceT, new GUIContent(compt), GUILayout.Height(20));
                    GUIContent comproll = new GUIContent("Compliance Rolling", "Rolling compliance of the contact, if using a nonzero rolling friction. (If there is no rolling friction, this has no effect.) Measuring unit: rad/Nm Default =0.");
                    EditorGUILayout.PropertyField(complianceRoll, new GUIContent(comproll), GUILayout.Height(20));
                    GUIContent compspin = new GUIContent("Compliance Spinning", " Spinning compliance of the contact, if using a nonzero rolling friction. (If there is no spinning friction, this has no effect.) Measuring unit: rad/Nm Default =0.");
                    EditorGUILayout.PropertyField(complianceSpin, new GUIContent(compspin), GUILayout.Height(20));

                    serializedObject.ApplyModifiedProperties();

                    //body.matsurface = body.gameObject.AddComponent<ChMaterialSurfaceNSC>() as ChMaterialSurfaceNSC;
                    break;
                case ChBody.MaterialType.SMC:
                    GUIContent young = new GUIContent("Young Modulus", "Young's modulus, modulus of elasticity");
                    EditorGUILayout.PropertyField(young_modulus, new GUIContent(young), GUILayout.Height(20));
                    GUIContent poison = new GUIContent("Poisson Ratio", "Poisson's ratio is a measure of the Poisson effect, the phenomenon in which a material tends to expand in directions perpendicular to the direction of compression. Conversely, if the material is stretched rather than compressed, it usually tends to contract in the directions transverse to the direction of stretching.");
                    EditorGUILayout.PropertyField(poisson_ratio, new GUIContent(poison), GUILayout.Height(20));
                    EditorGUILayout.PropertyField(static_friction, new GUIContent("Static Friction"), GUILayout.Height(20));
                    EditorGUILayout.PropertyField(sliding_friction, new GUIContent("Sliding Friction"), GUILayout.Height(20));
                    EditorGUILayout.PropertyField(constant_adhesion, new GUIContent("Constant Adhesion"), GUILayout.Height(20));
                    EditorGUILayout.PropertyField(adhesionMultDMT, new GUIContent("AdhesionMultDMT"), GUILayout.Height(20));

                    serializedObject.ApplyModifiedProperties();
                    //matsurface = gameObject.AddComponent<ChMaterialSurfaceSMC>() as ChMaterialSurfaceSMC;
                    break;

            }
        }
    }
}
