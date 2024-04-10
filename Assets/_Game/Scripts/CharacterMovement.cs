using System;
using KinematicCharacterController;
using UnityEngine;

namespace Game
{
    public struct CharacterMovementInputs
    {
        public Vector2 MoveInput;
        public Quaternion LookRotation;
        public bool WantsToJump;
    }

    [Serializable]
    public class MovementParameters
    {
        public float Speed = 5;
        public float Acceleration = 50;
        public float RotationSpeed = 20;
    }

    [Serializable]
    public class JumpParameters
    {
        public float JumpHeight = 2;
        public float TimeToApex = 0.5f;

        [Min(0.03f)] public float JumpRequestExpire;

        public float CalculateJumpSpeed(float gravity)
        {
            return (float)(JumpHeight - 0.5 * -gravity * TimeToApex * TimeToApex) / TimeToApex;
        }
    }

    [Serializable]
    public class AirborneParameters
    {
        public float Gravity = 10;
        public float MaxFallSpeed = 1;
        public float MaxSpeedXZ = 1;
        public float Acceleration = 10;
        [Range(0, 1)] public float Drag = 0.1f;
    }

    public enum CharacterBodyRotationMode
    {
        None,
        Movement,
        LookDirection
    }

    [RequireComponent(typeof(KinematicCharacterMotor))]
    public class CharacterMovement : MonoBehaviour, ICharacterController
    {
        public CharacterBodyRotationMode RotationMode = CharacterBodyRotationMode.Movement;
        public bool IsSprinting;

        [Header("Ground Movement")] public MovementParameters WalkParameters = new();
        public MovementParameters SprintParameters = new();
        public JumpParameters JumpParameters = new();

        [Header("Airborne")] public AirborneParameters DefaultAirborneParameters;

        private Vector3 moveInput;
        private Quaternion lookRotation;
        private float wantsToJumpExpireTime;
        private float jumpEndTime;

        public Vector3? targetPosition;

        public bool DisableMovementFromInput;

        public Vector3 Position => Motor.TransientPosition;
        public Quaternion Rotation => Motor.TransientRotation;
        public virtual float CurrentGroundMaxSpeed => CurrentMovementParameters.Speed;

        public AirborneParameters CurrentAirborneParameters
        {
            get { return DefaultAirborneParameters; }
        }

        public bool WantsToJump
        {
            get => Time.time < wantsToJumpExpireTime;
            set
            {
                if (value)
                {
                    wantsToJumpExpireTime = Time.time + JumpParameters.JumpRequestExpire;
                }
                else
                {
                    wantsToJumpExpireTime = -1;
                }
            }
        }

        public bool IsJumping => !IsGrounded && Time.time < jumpEndTime;
        public bool IsGrounded => Motor.GroundingStatus.IsStableOnGround && !Motor.MustUnground();

        private KinematicCharacterMotor motor;

        public KinematicCharacterMotor Motor
        {
            get
            {
                if (motor == null)
                {
                    motor = GetComponent<KinematicCharacterMotor>();
                }

                return motor;
            }
        }

        public Vector3 MoveInput => moveInput;
        
        public Vector3 Velocity => Motor.Velocity;

        public MovementParameters CurrentMovementParameters
        {
            get
            {
                if (IsSprinting)
                {
                    return SprintParameters;
                }

                return WalkParameters;
            }
        }

        private void Awake()
        {
            Motor.CharacterController = this;
        }

        public void SetMovementInput(in CharacterMovementInputs inputs)
        {
            if (DisableMovementFromInput)
            {
                return;
            }

            moveInput = Vector3.zero;

            if (inputs.MoveInput != Vector2.zero)
            {
                moveInput = new Vector3(inputs.MoveInput.x, 0, inputs.MoveInput.y).normalized;
                moveInput = Motor.GetDirectionTangentToSurface(inputs.LookRotation * moveInput, Vector3.up);
            }

            lookRotation = inputs.LookRotation;

            if (inputs.WantsToJump)
            {
                WantsToJump = true;
            }
        }

        public void UpdateRotation(ref Quaternion currentRotation, float deltaTime)
        {
            switch (RotationMode)
            {
                case CharacterBodyRotationMode.Movement:
                {
                    if (moveInput != Vector3.zero)
                    {
                        var movementParameters = CurrentMovementParameters;
                        var targetRot =
                            Quaternion.LookRotation(new Vector3(moveInput.x, 0, moveInput.z), Vector3.up);
                        currentRotation = Quaternion.Slerp(
                            currentRotation,
                            targetRot,
                            1 - Mathf.Exp(-movementParameters.RotationSpeed * deltaTime));
                    }

                    break;
                }
                case CharacterBodyRotationMode.LookDirection:
                {
                    if (lookRotation != default)
                    {
                        var movementParameters = CurrentMovementParameters;
                        var lookDir = lookRotation * Vector3.forward;
                        lookDir.y = 0;
                        var targetRot = Quaternion.LookRotation(lookDir, Vector3.up);
                        currentRotation = Quaternion.Slerp(
                            currentRotation,
                            targetRot,
                            1 - Mathf.Exp(-movementParameters.RotationSpeed * deltaTime));
                    }

                    break;
                }
                case CharacterBodyRotationMode.None:
                default:
                    break;
            }
        }

        private Vector3 OrientVectorToGroundTangent(Vector3 vector)
        {
            return Motor.GetDirectionTangentToSurface(vector, Motor.GroundingStatus.GroundNormal) *
                   vector.magnitude;
        }

        public void UpdateVelocity(ref Vector3 currentVelocity, float deltaTime)
        {
            var movementParameters = CurrentMovementParameters;
            var airborneParameters = CurrentAirborneParameters;

            if (Motor.GroundingStatus.IsStableOnGround)
            {
                Vector3 targetMovementVelocity;

                if (targetPosition.HasValue)
                {
                    targetMovementVelocity =
                        Motor.GetVelocityForMovePosition(Motor.TransientPosition, targetPosition.Value, deltaTime);
                }
                else
                {
                    targetMovementVelocity = moveInput * movementParameters.Speed;
                }

                // targetMovementVelocity = OrientVectorToGroundTangent(targetMovementVelocity);

                currentVelocity = Vector3.Lerp(
                    currentVelocity,
                    targetMovementVelocity,
                    1 - Mathf.Exp(-movementParameters.Acceleration * deltaTime));

                //instant change if jumping
                if (WantsToJump)
                {
                    currentVelocity.y = JumpParameters.CalculateJumpSpeed(airborneParameters.Gravity);
                    jumpEndTime = Time.time + JumpParameters.TimeToApex;
                    WantsToJump = false;
                    //required so KinematicMotor doesn't snap us to the ground
                    Motor.ForceUnground();
                }
            }
            else
            {
                var targetVelocityXZ = new Vector2(moveInput.x, moveInput.z) * airborneParameters.MaxSpeedXZ;
                var currentVelocityXZ = new Vector2(currentVelocity.x, currentVelocity.z);

                currentVelocityXZ = Vector2.Lerp(
                    currentVelocityXZ,
                    targetVelocityXZ,
                    1 - Mathf.Exp(-airborneParameters.Acceleration * deltaTime));

                var vy = Mathf.MoveTowards(currentVelocity.y, -airborneParameters.MaxFallSpeed,
                    airborneParameters.Gravity * deltaTime);

                currentVelocity.x = currentVelocityXZ.x;
                currentVelocity.y = vy;
                currentVelocity.z = currentVelocityXZ.y;

                ApplyDrag(ref currentVelocity.x, airborneParameters.Drag, deltaTime);
                ApplyDrag(ref currentVelocity.y, airborneParameters.Drag, deltaTime);
                ApplyDrag(ref currentVelocity.z, airborneParameters.Drag, deltaTime);
            }
        }

        private void ApplyDrag(ref float n, float drag, float deltaTime)
        {
            n *= 1f / (1f + drag * deltaTime);
        }

        public void ForceStop()
        {
            moveInput = Vector3.zero;
            Motor.BaseVelocity.x = 0;
            Motor.BaseVelocity.z = 0;
        }

        public bool CheckGround(float checkDistance)
        {
            var pos = Motor.TransientPosition;
            var groundingReport = default(CharacterGroundingReport);
            Motor.ProbeGround(ref pos, Motor.TransientRotation, checkDistance, ref groundingReport);
            return groundingReport.FoundAnyGround;
        }

        public void SetPosition(Vector3 position)
        {
            //Make sure we stick to the slope if we're grounded, and don't want to move up
            if (Motor.GroundingStatus.IsStableOnGround)
            {
                position.y = Mathf.Max(position.y, Position.y);
            }

            targetPosition = position;
        }

        public void SetRotation(Quaternion rotation)
        {
            var r = rotation.eulerAngles;
            rotation = Quaternion.Euler(Vector3.up * r.y);
            Motor.SetRotation(rotation);
        }

        public void TeleportTo(Vector3 position, Quaternion? rotation = null)
        {
            if (rotation != null)
            {
                Motor.SetPositionAndRotation(position, rotation.Value);
            }
            else
            {
                Motor.SetPosition(position);
            }
        }

        public void FaceLookDirection(Vector3 lookDir, float rotationSpeed, float deltaTime)
        {
            var smoothedLookInputDirection = Vector3.Slerp(Motor.CharacterForward, lookDir,
                1 - Mathf.Exp(-rotationSpeed * deltaTime)).normalized;
            var rot = Quaternion.LookRotation(smoothedLookInputDirection, Motor.CharacterUp);
            SetRotation(rot);
        }

        #region not implemented

        public void AfterCharacterUpdate(float deltaTime)
        {
            targetPosition = null;
        }

        public void BeforeCharacterUpdate(float deltaTime)
        {
        }

        public bool IsColliderValidForCollisions(Collider coll)
        {
            return true;
        }

        public void OnDiscreteCollisionDetected(Collider hitCollider)
        {
        }

        public void OnGroundHit(Collider hitCollider, Vector3 hitNormal, Vector3 hitPoint,
            ref HitStabilityReport hitStabilityReport)
        {
        }

        public void OnMovementHit(Collider hitCollider, Vector3 hitNormal, Vector3 hitPoint,
            ref HitStabilityReport hitStabilityReport)
        {
        }

        public void PostGroundingUpdate(float deltaTime)
        {
        }

        public void ProcessHitStabilityReport(Collider hitCollider, Vector3 hitNormal, Vector3 hitPoint,
            Vector3 atCharacterPosition, Quaternion atCharacterRotation, ref HitStabilityReport hitStabilityReport)
        {
        }

        #endregion
    }
}
