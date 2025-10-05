using UnityEngine;
using Oculus.Interaction.Body.Input;

public class ArmIKController : MonoBehaviour
{
    [Header("IK Components")]
    [SerializeField] private EasyIK leftArmIK;
    [SerializeField] private EasyIK rightArmIK;

    [Header("Controller Transforms")]
    [SerializeField] private Transform leftController;
    [SerializeField] private Transform rightController;

    [Header("Body tracking (optional)")]
    [Tooltip("Assign the in-scene OVRBody that provides body tracking.")]
    public OVRBody bodyProvider;
    [Tooltip("Select the LEFT elbow joint id from your SDK version (e.g., LeftElbow / LeftLowerArm)")]
    [SerializeField] private BodyJointId leftElbowJointId;
    [Tooltip("Select the RIGHT elbow joint id from your SDK version (e.g., RightElbow / RightLowerArm)")]
    [SerializeField] private BodyJointId rightElbowJointId;

    [Header("Options")]
    [SerializeField] private bool setArmJointCountTo3 = true;

    private void Awake()
    {
        if (setArmJointCountTo3)
        {
            if (leftArmIK != null) leftArmIK.numberOfJoints = 3;
            if (rightArmIK != null) rightArmIK.numberOfJoints = 3;
        }
    }

    private void Update()
    {
        if (leftArmIK != null && leftArmIK.ikTarget != null && leftController != null)
        {
            leftArmIK.ikTarget.position = leftController.position;
            leftArmIK.ikTarget.rotation = leftController.rotation;
        }

        if (rightArmIK != null && rightArmIK.ikTarget != null && rightController != null)
        {
            rightArmIK.ikTarget.position = rightController.position;
            rightArmIK.ikTarget.rotation = rightController.rotation;
        }

        // Drive elbow pole targets from OVRBody if available
        if (bodyProvider != null && bodyProvider.BodyState.HasValue)
        {
            var state = bodyProvider.BodyState.Value;
            var joints = state.JointLocations;

            if (leftArmIK != null && leftArmIK.poleTarget != null)
            {
                int iLeft = (int)leftElbowJointId;
                if (iLeft >= 0 && iLeft < joints.Length)
                {
                    var p = joints[iLeft].Pose.Position;
                    Vector3 posLocal = new Vector3(p.x, p.y, -p.z);
                    Vector3 posWorld = bodyProvider.transform.TransformPoint(posLocal);
                    leftArmIK.poleTarget.position = posWorld;
                }
            }

            if (rightArmIK != null && rightArmIK.poleTarget != null)
            {
                int iRight = (int)rightElbowJointId;
                if (iRight >= 0 && iRight < joints.Length)
                {
                    var p = joints[iRight].Pose.Position;
                    Vector3 posLocal = new Vector3(p.x, p.y, -p.z);
                    Vector3 posWorld = bodyProvider.transform.TransformPoint(posLocal);
                    rightArmIK.poleTarget.position = posWorld;
                }
            }
        }
    }
}


