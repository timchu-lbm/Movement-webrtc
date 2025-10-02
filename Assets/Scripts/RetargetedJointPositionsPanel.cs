using System.Text;
using Meta.XR.Movement.Retargeting;
using TMPro;
using UnityEngine;

public class RetargetedJointPositionsPanel : MonoBehaviour
{
    [Tooltip("Assign the CharacterRetargeter that drives the avatar. If not set, it will auto-find in the scene.")]
    public CharacterRetargeter retargeter;

    [Tooltip("TextMeshPro output to display retargeted joint positions. If not set, it will auto-bind from this object or its children.")]
    public TMP_Text outputText;

    [Tooltip("If true, prints local positions relative to each joint's parent. Otherwise, prints world positions.")]
    public bool useLocalSpace = false;

    private void Awake()
    {
        if (outputText == null)
        {
            outputText = GetComponent<TMP_Text>();
            if (outputText == null)
            {
                outputText = GetComponentInChildren<TMP_Text>();
            }
        }
    }

    private void LateUpdate()
    {
        if (outputText == null) return;

        if (retargeter == null)
        {
            retargeter = FindAnyObjectByType<CharacterRetargeter>();
            if (retargeter == null)
            {
                outputText.text = "Waiting for CharacterRetargeter...";
                return;
            }
        }

        var jointPairs = retargeter.JointPairs;
        if (jointPairs == null || jointPairs.Length == 0)
        {
            outputText.text = "Retargeter has no joints configured.";
            return;
        }

        var sb = new StringBuilder(4096);
        for (int i = 0; i < jointPairs.Length; i++)
        {
            var jointTransform = jointPairs[i].Joint;
            if (jointTransform == null) continue;

            Vector3 pos = useLocalSpace ? jointTransform.localPosition : jointTransform.position;
            sb.Append(jointTransform.name);
            sb.Append(": ");
            sb.AppendFormat("x={0:F3}, y={1:F3}, z={2:F3}\n", pos.x, pos.y, pos.z);
        }

        outputText.text = sb.ToString();
    }
}


