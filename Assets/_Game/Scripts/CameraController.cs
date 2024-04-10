using Cinemachine;
using UnityEngine;

namespace Game
{
    public class CameraController : MonoBehaviour
    {
        public Transform Target;
        public float RotationSpeed = 2;
        public Vector2 xRotationRange = new(-70, 70);
        [Range(0.5f, 2.5f)] public float Sensitivity = 1;

        public Vector2 targetLook;

        public GameObject DefaultCamera;

        public Quaternion LookRotation => Target.rotation;

        private void Awake()
        {
            var rot = Target.transform.rotation.eulerAngles;
            targetLook.x = rot.x;
            targetLook.y = rot.y;
        }

        private void LateUpdate()
        {
            if (Application.isPlaying)
            {
                Target.transform.rotation = Quaternion.Euler(targetLook.x, targetLook.y, 0);
            }
        }

        public void IncrementLookRotation(Vector2 lookDelta)
        {
            const float threshold = 0.01f;
            if (lookDelta.sqrMagnitude >= threshold)
            {
                targetLook += lookDelta * RotationSpeed * Sensitivity;
                targetLook.x = Mathf.Clamp(targetLook.x, xRotationRange.x, xRotationRange.y);
            }
        }

        public void SetLookRotation(Quaternion rotation)
        {
            Target.transform.rotation = rotation;
            var euler = rotation.eulerAngles;
            targetLook.x = euler.x;
            targetLook.y = euler.y;
        }

        public float GetCurrentCameraDistance()
        {
            if (DefaultCamera.TryGetComponent<CinemachineVirtualCamera>(out var vcam))
            {
                var tpFollow = vcam.GetCinemachineComponent<Cinemachine3rdPersonFollow>();
                if (tpFollow != null)
                {
                    return tpFollow.CameraDistance;
                }
            }

            return 0;
        }

        public Vector3 Damping
        {
            get
            {
                if (DefaultCamera.TryGetComponent<CinemachineVirtualCamera>(out var vcam))
                {
                    var tpFollow = vcam.GetCinemachineComponent<Cinemachine3rdPersonFollow>();
                    if (tpFollow != null)
                    {
                        return tpFollow.Damping;
                    }
                }

                return default;
            }
            set
            {
                if (DefaultCamera.TryGetComponent<CinemachineVirtualCamera>(out var vcam))
                {
                    var tpFollow = vcam.GetCinemachineComponent<Cinemachine3rdPersonFollow>();
                    if (tpFollow != null)
                    {
                        tpFollow.Damping = value;
                    }
                }
            }
        }


        public void SetCameraDistance(float distance)
        {
            if (DefaultCamera.TryGetComponent<CinemachineVirtualCamera>(out var vcam))
            {
                var tpFollow = vcam.GetCinemachineComponent<Cinemachine3rdPersonFollow>();
                if (tpFollow != null)
                {
                    tpFollow.CameraDistance = distance;
                }
            }
        }

        public void ForcePosition(Vector3 pos, Quaternion quaternion)
        {
            if (DefaultCamera.TryGetComponent<CinemachineVirtualCamera>(out var vcam))
            {
                var tpFollow = vcam.GetCinemachineComponent<Cinemachine3rdPersonFollow>();
                if (tpFollow != null)
                {
                    tpFollow.ForceCameraPosition(pos, quaternion);
                }
            }
        }
    }
}
