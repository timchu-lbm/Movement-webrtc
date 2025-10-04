using UnityEngine;

public class ArmIKController : MonoBehaviour
{
    [Header("IK Components")]
    [SerializeField] private EasyIK leftArmIK;
    [SerializeField] private EasyIK rightArmIK;

    [Header("Controller Transforms")]
    [SerializeField] private Transform leftController;
    [SerializeField] private Transform rightController;

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

    private void LateUpdate()
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
    }
}


