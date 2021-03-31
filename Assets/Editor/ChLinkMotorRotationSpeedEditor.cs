using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

namespace chrono
{

    [CustomEditor(typeof(ChLinkMotorRotationSpeed))]
    public class ChLinkMotorRotationSpeedEditor : Editor
    {
        override public void OnInspectorGUI()
        {
            var motor = target as ChLinkMotorRotationSpeed;

            EditorGUILayout.HelpBox(" A motor that enforces the angular speed w(t) between two frames on two bodies, using a rheonomic constraint. Note: no compliance is allowed, so if the actuator hits an undeformable obstacle it hits a pathological situation and the solver result can be unstable/unpredictable. Think at it as a servo drive with ' infinitely stiff ' control. This type of motor is very easy to use, stable and efficient, and should be used if the 'infinitely stiff' control assumption is a good approximation of what you simulate (e.g., very good and reactive controllers). By default it is initialized with constant angular speed: df/dt = 1. Use SetSpeedFunction() to change to other speed functions.", MessageType.Info);

            GUIContent body1 = new GUIContent("Body 1", "First connected body.");
            motor.body1 = (ChBody)EditorGUILayout.ObjectField(body1, motor.body1, typeof(ChBody), true);
            GUIContent body2 = new GUIContent("Body 2", "Second connected body.");
            motor.body2 = (ChBody)EditorGUILayout.ObjectField(body2, motor.body2, typeof(ChBody), true);

            if (!motor.body1)
            {
                EditorGUILayout.HelpBox("Add a first connecting body.", MessageType.Warning);
            }
            if (!motor.body2)
            {
                EditorGUILayout.HelpBox("Add a second connecting body.", MessageType.Warning);
            }

            if (GUI.changed)
            {
                EditorUtility.SetDirty(motor);
            }
        }

    }

}
