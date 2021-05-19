using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

namespace chrono
{

    [CustomEditor(typeof(ChFunction_Const))]
    public class ChFunction_ConstEditor : Editor
    {
        override public void OnInspectorGUI()
        {
            var function = target as ChFunction_Const;

            EditorGUILayout.HelpBox("Constant function:  y = C", MessageType.Info);

            GUIContent constant_val = new GUIContent("Constant Value", "Set the constant C for the function, y=C.");
            function.constant_val = EditorGUILayout.DoubleField(constant_val, function.constant_val);

        }

    }

}
