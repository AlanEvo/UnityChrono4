using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

namespace chrono
{

    [CustomEditor(typeof(ChLinkUniversal))]
    public class ChLinkUniversalEditor : Editor
    {
        override public void OnInspectorGUI()
        {
            var universal = target as ChLinkUniversal;

            EditorGUILayout.HelpBox("Class for modeling a universal joint between two two ChBodyFrame objects. This joint is defined through 4 constraint equations between two marker frames, one on each body.  Kinematically, these constraints impose the condition that the two marker origins coincide (3 constraints) and that two directions (one on each body) are always perpendicular. Together, these constraints model the cross in a physical universal joint.", MessageType.Info);

            GUIContent body1 = new GUIContent("Body 1", "First connected body.");
            universal.body1 = (ChBody)EditorGUILayout.ObjectField(body1, universal.body1, typeof(ChBody), true);
            GUIContent body2 = new GUIContent("Body 2", "Second connected body.");
            universal.body2 = (ChBody)EditorGUILayout.ObjectField(body2, universal.body2, typeof(ChBody), true);

            if (!universal.body1)
            {
                EditorGUILayout.HelpBox("Add a first connecting body.", MessageType.Warning);
            }
            if (!universal.body2)
            {
                EditorGUILayout.HelpBox("Add a second connecting body.", MessageType.Warning);
            }
        }

    }

}
