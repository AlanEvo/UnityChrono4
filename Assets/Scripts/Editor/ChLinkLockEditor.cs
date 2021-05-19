using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

namespace chrono
{

    [CustomEditor(typeof(ChLinkLock))]
    public class ChLinkLockEditor : Editor
    {
        override public void OnInspectorGUI()
        {
            var joint = target as ChLinkLock;

            EditorGUILayout.HelpBox("ChLinkLock class. This class implements lot of sub types like the revolute joint, the linear guide, the spherical joint, etc. using the 'lock formulation'. Also, it optionally allows the adoption of 'limits' over upper-lower motions on all the 6 degrees of freedom, thank to the ChLinkLimit objects.", MessageType.Info);

            GUIContent type = new GUIContent("Link Type", "Change what type of joint you want.");
            joint.type = (ChLinkLock.LinkType)EditorGUILayout.EnumPopup(type, joint.type);
            GUIContent body1 = new GUIContent("Body 1", "First connected body.");
            joint.body1 = (ChBody)EditorGUILayout.ObjectField(body1, joint.body1, typeof(ChBody), true);
            GUIContent body2 = new GUIContent("Body 2", "Second connected body.");
            joint.body2 = (ChBody)EditorGUILayout.ObjectField(body2, joint.body2, typeof(ChBody), true);

            GUIContent limit = new GUIContent("Joint Limits", "Toggle if the joint is to have upper and lower limits.");
            joint.enableLimits = EditorGUILayout.Toggle("Joint Limits", joint.enableLimits);
            GUIContent showlimit = new GUIContent("Joint Limits", "Toggle if the joint is to have upper and lower limits.");
            joint.showLimits = EditorGUILayout.Toggle("Show Limits", joint.showLimits);

            if (!joint.body1)
            {
                EditorGUILayout.HelpBox("Add a first connecting body.", MessageType.Warning);
            }
            if (!joint.body2)
            {
                EditorGUILayout.HelpBox("Add a second connecting body.", MessageType.Warning);
            }

            if (joint.enableLimits)
            {
                switch (joint.type)
                {
                    case ChLinkLock.LinkType.FREE:

                        break;
                    case ChLinkLock.LinkType.LOCK:

                        break;
                    case ChLinkLock.LinkType.SPHERICAL:

                        break;
                    case ChLinkLock.LinkType.POINTPLANE:

                        break;
                    case ChLinkLock.LinkType.POINTLINE:

                        break;
                    case ChLinkLock.LinkType.REVOLUTE:
                        EditorGUI.indentLevel++;
                        joint.minAngle = EditorGUILayout.DoubleField("Min Angle", joint.minAngle);
                        joint.maxAngle = EditorGUILayout.DoubleField("Max Angle", joint.maxAngle);
                        break;
                    case ChLinkLock.LinkType.CYLINDRICAL:

                        break;
                    case ChLinkLock.LinkType.PRISMATIC:
                        EditorGUI.indentLevel++;
                        joint.minDisplacement = EditorGUILayout.DoubleField("Min Displacement", joint.minDisplacement);
                        joint.maxDisplacement = EditorGUILayout.DoubleField("Max Displacement", joint.maxDisplacement);
                        break;
                    case ChLinkLock.LinkType.PLANEPLANE:

                        break;
                    case ChLinkLock.LinkType.OLDHAM:

                        break;
                    case ChLinkLock.LinkType.ALIGN:

                        break;
                    case ChLinkLock.LinkType.PARALLEL:

                        break;
                    case ChLinkLock.LinkType.PERPEND:

                        break;
                    case ChLinkLock.LinkType.REVOLUTEPRISMATIC:

                        break;
                    default:

                        break;
                }
                EditorGUI.indentLevel--;
            }

            EditorGUILayout.LabelField("Angle", joint.angle.ToString());

            if (GUI.changed)
            {
                EditorUtility.SetDirty(joint);
            }
        }
    }

}
