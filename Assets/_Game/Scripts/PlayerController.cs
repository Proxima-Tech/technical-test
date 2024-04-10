using System;
using UnityEngine;

namespace Game
{
    public class PlayerController : MonoBehaviour
    {
        [NonSerialized] public InputActions InputActions;
        public CharacterMovement CharacterMovement;
        public CameraController CameraController;

        private void Awake()
        {
            InputActions = new InputActions();
            InputActions.Enable();
        }

        private void OnDisable()
        {
            if (CharacterMovement != null)
            {
                CharacterMovement.SetMovementInput(new CharacterMovementInputs { MoveInput = Vector2.zero });
            }
        }

        void Update()
        {
            //movement
            CharacterMovement.SetMovementInput(new CharacterMovementInputs
            {
                MoveInput = InputActions.Game.Move.ReadValue<Vector2>(),
                LookRotation = CameraController.LookRotation,
                WantsToJump = InputActions.Game.Jump.WasPressedThisFrame()
            });
            CharacterMovement.IsSprinting = InputActions.Game.Sprint.IsPressed();

            //camera
            var look = InputActions.Game.Look.ReadValue<Vector2>();
            CameraController.IncrementLookRotation(new Vector2(look.y, look.x));
        }
    }
}