using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using System.Linq;

public class EasyIK : MonoBehaviour
{
    [Header("IK properties")]
    public int numberOfJoints = 2;
    public Transform ikTarget;
    public int iterations = 10;
    public float tolerance = 0.05f;

    private Transform[] jointTransforms;
    private Vector3[] jointPositions;
    private float[] boneLength;
    private float jointChainLength;
    private float distanceToTarget;

    private Quaternion[] startRotationLocal;
    private Vector3[] jointStartDirectionLocal;
    private Quaternion ikTargetStartRotLocal;
    private Quaternion lastJointStartRotLocal;

    [Header("Pole target (3 joint chain)")]
    public Transform poleTarget;

    [Header("Debug")]
    public bool debugJoints = true;
    public bool localRotationAxis = false;

    [Range(0.0f, 1.0f)]
    public float gizmoSize = 0.05f;
    public bool poleDirection = false;
    public bool poleRotationAxis = false;

    void Awake()
    {
        jointChainLength = 0;
        jointTransforms = new Transform[numberOfJoints];
        jointPositions = new Vector3[numberOfJoints];
        boneLength = new float[numberOfJoints];
        jointStartDirectionLocal = new Vector3[numberOfJoints];
        startRotationLocal = new Quaternion[numberOfJoints];

        // store target initial rotation in local space
        ikTargetStartRotLocal = Quaternion.Inverse(transform.rotation) * ikTarget.rotation;

        var current = transform;
        for (int i = 0; i < jointTransforms.Length; i++)
        {
            jointTransforms[i] = current;

            if (i == jointTransforms.Length - 1)
            {
                lastJointStartRotLocal = Quaternion.Inverse(transform.rotation) * current.rotation;
                return;
            }
            else
            {
                boneLength[i] = Vector3.Distance(current.position, current.GetChild(0).position);
                jointChainLength += boneLength[i];

                // store start direction relative to holder
                jointStartDirectionLocal[i] = transform.InverseTransformDirection(
                    current.GetChild(0).position - current.position
                );

                // store start rotation relative to holder
                startRotationLocal[i] = Quaternion.Inverse(transform.rotation) * current.rotation;
            }

            current = current.GetChild(0);
        }
    }

    void PoleConstraint()
    {
        if (poleTarget != null && numberOfJoints < 4)
        {
            var limbAxis = (jointPositions[2] - jointPositions[0]).normalized;
            Vector3 poleDirectionVec = (poleTarget.position - jointPositions[0]).normalized;
            Vector3 boneDirection = (jointPositions[1] - jointPositions[0]).normalized;

            Vector3.OrthoNormalize(ref limbAxis, ref poleDirectionVec);
            Vector3.OrthoNormalize(ref limbAxis, ref boneDirection);

            Quaternion angle = Quaternion.FromToRotation(boneDirection, poleDirectionVec);
            jointPositions[1] = angle * (jointPositions[1] - jointPositions[0]) + jointPositions[0];
        }
    }

    void Backward()
    {
        for (int i = jointPositions.Length - 1; i >= 0; i--)
        {
            if (i == jointPositions.Length - 1)
            {
                jointPositions[i] = ikTarget.position;
            }
            else
            {
                jointPositions[i] = jointPositions[i + 1] +
                    (jointPositions[i] - jointPositions[i + 1]).normalized * boneLength[i];
            }
        }
    }

    void Forward()
    {
        for (int i = 0; i < jointPositions.Length; i++)
        {
            if (i == 0)
            {
                jointPositions[i] = jointTransforms[0].position;
            }
            else
            {
                jointPositions[i] = jointPositions[i - 1] +
                    (jointPositions[i] - jointPositions[i - 1]).normalized * boneLength[i - 1];
            }
        }
    }

    private void SolveIK()
    {
        for (int i = 0; i < jointTransforms.Length; i++)
        {
            jointPositions[i] = jointTransforms[i].position;
        }

        distanceToTarget = Vector3.Distance(jointPositions[0], ikTarget.position);

        if (distanceToTarget > jointChainLength)
        {
            var direction = ikTarget.position - jointPositions[0];
            for (int i = 1; i < jointPositions.Length; i++)
            {
                jointPositions[i] = jointPositions[i - 1] + direction.normalized * boneLength[i - 1];
            }
        }
        else
        {
            float distToTarget = Vector3.Distance(
                jointPositions[jointPositions.Length - 1],
                ikTarget.position
            );
            int counter = 0;
            while (distToTarget > tolerance)
            {
                Backward();
                Forward();
                distToTarget = Vector3.Distance(
                    jointPositions[jointPositions.Length - 1],
                    ikTarget.position
                );
                counter++;
                if (counter > iterations) break;
            }
        }

        PoleConstraint();

        // apply rotations relative to script holder
        for (int i = 0; i < jointPositions.Length - 1; i++)
        {
            jointTransforms[i].position = jointPositions[i];

            Vector3 localStartDir = jointStartDirectionLocal[i];
            Vector3 localTargetDir = transform.InverseTransformDirection(
                jointPositions[i + 1] - jointPositions[i]
            );

            Quaternion localRotDelta = Quaternion.FromToRotation(localStartDir, localTargetDir);
            Quaternion localResult = localRotDelta * startRotationLocal[i];

            jointTransforms[i].rotation = transform.rotation * localResult;
        }

        // last joint constrained to IK target
        Quaternion offset = lastJointStartRotLocal * Quaternion.Inverse(ikTargetStartRotLocal);
        jointTransforms.Last().rotation = ikTarget.rotation * offset;
    }

    void Update()
    {
        SolveIK();
    }

#if UNITY_EDITOR
    void OnDrawGizmos()
    {
        if (debugJoints && transform.childCount > 0)
        {
            var current = transform;
            var child = transform.GetChild(0);

            for (int i = 0; i < numberOfJoints; i++)
            {
                float length = Vector3.Distance(current.position, child.position);
                DrawWireCapsule(
                    current.position + (child.position - current.position).normalized * length / 2,
                    Quaternion.FromToRotation(Vector3.up, (child.position - current.position).normalized),
                    gizmoSize,
                    length,
                    Color.cyan
                );

                if (i == numberOfJoints - 2) break;

                current = current.GetChild(0);
                child = current.GetChild(0);
            }
        }

        if (localRotationAxis)
        {
            var current = transform;
            for (int i = 0; i < numberOfJoints; i++)
            {
                drawHandle(current);
                if (i < numberOfJoints - 1) current = current.GetChild(0);
            }
        }

        if (poleRotationAxis && poleTarget != null && numberOfJoints < 4)
        {
            Handles.color = Color.white;
            Handles.DrawLine(transform.position, transform.GetChild(0).GetChild(0).position);
        }

        if (poleDirection && poleTarget != null && numberOfJoints < 4)
        {
            var start = transform;
            var mid = start.GetChild(0);
            var end = mid.GetChild(0);

            Handles.color = Color.grey;
            Handles.DrawLine(start.position, poleTarget.position);
            Handles.DrawLine(end.position, poleTarget.position);
        }
    }

    void drawHandle(Transform debugJoint)
    {
        Handles.color = Handles.xAxisColor;
        Handles.ArrowHandleCap(
            0,
            debugJoint.position,
            debugJoint.rotation * Quaternion.LookRotation(Vector3.right),
            gizmoSize,
            EventType.Repaint
        );

        Handles.color = Handles.yAxisColor;
        Handles.ArrowHandleCap(
            0,
            debugJoint.position,
            debugJoint.rotation * Quaternion.LookRotation(Vector3.up),
            gizmoSize,
            EventType.Repaint
        );

        Handles.color = Handles.zAxisColor;
        Handles.ArrowHandleCap(
            0,
            debugJoint.position,
            debugJoint.rotation * Quaternion.LookRotation(Vector3.forward),
            gizmoSize,
            EventType.Repaint
        );
    }

    public static void DrawWireCapsule(
        Vector3 _pos,
        Quaternion _rot,
        float _radius,
        float _height,
        Color _color = default(Color))
    {
        Handles.color = _color;
        Matrix4x4 angleMatrix = Matrix4x4.TRS(_pos, _rot, Handles.matrix.lossyScale);
        using (new Handles.DrawingScope(angleMatrix))
        {
            var pointOffset = (_height - (_radius * 2)) / 2;

            Handles.DrawWireArc(Vector3.up * pointOffset, Vector3.left, Vector3.back, -180, _radius);
            Handles.DrawLine(
                new Vector3(0, pointOffset, -_radius),
                new Vector3(0, -pointOffset, -_radius)
            );
            Handles.DrawLine(
                new Vector3(0, pointOffset, _radius),
                new Vector3(0, -pointOffset, _radius)
            );
            Handles.DrawWireArc(Vector3.down * pointOffset, Vector3.left, Vector3.back, 180, _radius);

            Handles.DrawWireArc(Vector3.up * pointOffset, Vector3.back, Vector3.left, 180, _radius);
            Handles.DrawLine(
                new Vector3(-_radius, pointOffset, 0),
                new Vector3(-_radius, -pointOffset, 0)
            );
            Handles.DrawLine(
                new Vector3(_radius, pointOffset, 0),
                new Vector3(_radius, -pointOffset, 0)
            );
            Handles.DrawWireArc(
                Vector3.down * pointOffset,
                Vector3.back,
                Vector3.left,
                -180,
                _radius
            );

            Handles.DrawWireDisc(Vector3.up * pointOffset, Vector3.up, _radius);
            Handles.DrawWireDisc(Vector3.down * pointOffset, Vector3.up, _radius);
        }
    }
#endif
}
