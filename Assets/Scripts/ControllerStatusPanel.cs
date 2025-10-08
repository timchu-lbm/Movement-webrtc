using System.Text;
using TMPro;
using UnityEngine;
using UnityEngine.XR;

public class ControllerStatusPanel : MonoBehaviour
{
    [Tooltip("TextMeshPro output to display controller status. If not set, it will auto-bind from this object or its children.")]
    public TMP_Text outputText;

    [Tooltip("Show extra fields like touches and menu button if available.")]
    public bool showExtended = true;

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

        var sb = new StringBuilder(1024);
        AppendCameraRigStatus(sb);
        sb.Append('\n');
        AppendControllerStatus(sb, "Left Controller", XRNode.LeftHand);
        sb.Append('\n');
        AppendControllerStatus(sb, "Right Controller", XRNode.RightHand);

        outputText.text = sb.ToString();
    }

    private void AppendCameraRigStatus(StringBuilder sb)
    {
        var head = InputDevices.GetDeviceAtXRNode(XRNode.Head);
        sb.Append("Camera Rig");
        sb.Append(':');
        sb.Append('\n');

        if (!head.isValid)
        {
            sb.Append("  status: not found");
            sb.Append('\n');
            return;
        }

        if (head.TryGetFeatureValue(CommonUsages.isTracked, out bool isTracked))
        {
            sb.Append("  tracked: ");
            sb.Append(isTracked ? "true" : "false");
            sb.Append('\n');
        }

        if (head.TryGetFeatureValue(CommonUsages.devicePosition, out Vector3 pos))
        {
            sb.AppendFormat("  pos: x={0:F3}, y={1:F3}, z={2:F3}\n", pos.x, pos.y, pos.z);
        }

        if (head.TryGetFeatureValue(CommonUsages.deviceRotation, out Quaternion rot))
        {
            sb.AppendFormat("  rot(quat): x={0:F3}, y={1:F3}, z={2:F3}, w={3:F3}\n", rot.x, rot.y, rot.z, rot.w);
        }
    }

    private void AppendControllerStatus(StringBuilder sb, string label, XRNode node)
    {
        var device = InputDevices.GetDeviceAtXRNode(node);
        sb.Append(label);
        sb.Append(':');
        sb.Append('\n');

        if (!device.isValid)
        {
            sb.Append("  status: not found");
            sb.Append('\n');
            return;
        }

        // Tracking and pose
        if (device.TryGetFeatureValue(CommonUsages.isTracked, out bool isTracked))
        {
            sb.Append("  tracked: ");
            sb.Append(isTracked ? "true" : "false");
            sb.Append('\n');
        }

        if (device.TryGetFeatureValue(CommonUsages.devicePosition, out Vector3 pos))
        {
            sb.AppendFormat("  pos: x={0:F3}, y={1:F3}, z={2:F3}\n", pos.x, pos.y, pos.z);
        }

        if (device.TryGetFeatureValue(CommonUsages.deviceRotation, out Quaternion rot))
        {
            sb.AppendFormat("  rot(quat): x={0:F3}, y={1:F3}, z={2:F3}, w={3:F3}\n", rot.x, rot.y, rot.z, rot.w);
        }

        // Thumbstick / primary2DAxis
        if (device.TryGetFeatureValue(CommonUsages.primary2DAxis, out Vector2 stick))
        {
            sb.AppendFormat("  stick: x={0:F3}, y={1:F3}", stick.x, stick.y);
            if (device.TryGetFeatureValue(CommonUsages.primary2DAxisClick, out bool stickClick))
            {
                sb.Append(", click=");
                sb.Append(stickClick ? "true" : "false");
            }
            if (showExtended && device.TryGetFeatureValue(CommonUsages.primary2DAxisTouch, out bool stickTouch))
            {
                sb.Append(", touch=");
                sb.Append(stickTouch ? "true" : "false");
            }
            sb.Append('\n');
        }

        // Analog triggers
        if (device.TryGetFeatureValue(CommonUsages.trigger, out float trigger))
        {
            bool triggerPressed = trigger >= 0.75f;
            sb.AppendFormat("  trigger: val={0:F3}, pressed={1}\n", trigger, triggerPressed ? "true" : "false");
        }

        if (device.TryGetFeatureValue(CommonUsages.grip, out float grip))
        {
            bool gripPressed = grip >= 0.75f;
            sb.AppendFormat("  grip: val={0:F3}, pressed={1}\n", grip, gripPressed ? "true" : "false");
        }

        // Digital buttons
        if (device.TryGetFeatureValue(CommonUsages.primaryButton, out bool primaryBtn))
        {
            sb.Append("  primaryButton: ");
            sb.Append(primaryBtn ? "true" : "false");
            sb.Append('\n');
        }

        if (device.TryGetFeatureValue(CommonUsages.secondaryButton, out bool secondaryBtn))
        {
            sb.Append("  secondaryButton: ");
            sb.Append(secondaryBtn ? "true" : "false");
            sb.Append('\n');
        }

        if (device.TryGetFeatureValue(CommonUsages.triggerButton, out bool triggerBtn))
        {
            sb.Append("  triggerButton: ");
            sb.Append(triggerBtn ? "true" : "false");
            sb.Append('\n');
        }

        if (device.TryGetFeatureValue(CommonUsages.gripButton, out bool gripBtn))
        {
            sb.Append("  gripButton: ");
            sb.Append(gripBtn ? "true" : "false");
            sb.Append('\n');
        }

        if (showExtended && device.TryGetFeatureValue(CommonUsages.menuButton, out bool menuBtn))
        {
            sb.Append("  menuButton: ");
            sb.Append(menuBtn ? "true" : "false");
            sb.Append('\n');
        }

        if (showExtended)
        {
            if (device.TryGetFeatureValue(CommonUsages.primaryTouch, out bool primaryTouch))
            {
                sb.Append("  primaryTouch: ");
                sb.Append(primaryTouch ? "true" : "false");
                sb.Append('\n');
            }

            if (device.TryGetFeatureValue(CommonUsages.secondaryTouch, out bool secondaryTouch))
            {
                sb.Append("  secondaryTouch: ");
                sb.Append(secondaryTouch ? "true" : "false");
                sb.Append('\n');
            }
        }
    }
}


