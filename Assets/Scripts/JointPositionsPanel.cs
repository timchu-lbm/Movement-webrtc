using System;
using System.Text;
using Oculus.Interaction.Body.Input;
using UnityEngine;
using TMPro;

public class JointPositionsPanel : MonoBehaviour
{
    [Tooltip("Assign the in-scene OVRBody that provides body tracking.")]
    public OVRBody bodyProvider;

    [Tooltip("TextMeshPro output to display joint positions. If not set, it will auto-bind from this object or its children.")]
    public TMP_Text outputText;

    private void LateUpdate()
    {
        if (outputText == null) return;

        if (bodyProvider == null || !bodyProvider.BodyState.HasValue)
        {
            outputText.text = "Waiting for body tracking...";
            return;
        }

        var state = bodyProvider.BodyState.Value;
        var joints = state.JointLocations;
        // BodyJointId comes from the Oculus.Interaction.Body.Input namespace
        int total = Mathf.Min(joints.Length, (int)BodyJointId.Body_End);

        var sb = new StringBuilder(4096);
        for (int j = 0; j < total; j++)
        {
            var joint = joints[j];
            var p = joint.Pose.Position; // OVRPlugin.Vector3f
            var pos = new Vector3(p.x, p.y, -p.z); // Flip Z to Unity
            // BodyJointId comes from the Oculus.Interaction.Body.Input namespace
            string name = Enum.GetName(typeof(BodyJointId), (BodyJointId)j);
            sb.Append(name);
            sb.Append(": ");
            sb.AppendFormat("x={0:F3}, y={1:F3}, z={2:F3}\n", pos.x, pos.y, pos.z);
        }

        outputText.text = sb.ToString();
    }
}


