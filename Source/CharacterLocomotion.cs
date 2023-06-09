using UnityEngine;

public class CharacterLocomotion : MonoBehaviour
{
	[SerializeField] private float _moveSpeed;
	[SerializeField] private float _jumpHeight;
	[SerializeField] private float _timeToPeakReached;
	[SerializeField] private bool _stickToGround;
	[SerializeField] private Transform _planet;

	private ExampleCharacterController2D _controller;

	private ExampleCharacterController2D.GroundInfo _previousGround;

	private Vector2 _velocity;
	private FrameInput _input;

	private float _gravity;
	private float _jumpVelocity;

	struct FrameInput
	{
		public float x;
		public bool jump;
	}

	private void Awake()
	{
		_controller = GetComponent<ExampleCharacterController2D>();
		_controller.OnControllerCollision += (type) =>
		{
			if (type == CollisionType.Vertical && _velocity.y > 0)
				_velocity.y = 0;
		};
	}

	private void Update()
	{
		if (_planet != null) {
			Vector2 vec = (transform.position - _planet.position);
			_controller.SetUpVector(vec.normalized);
		}

		_input = new FrameInput()
		{
			x = Input.GetAxisRaw("Horizontal"),
			jump = Input.GetButton("Jump")
		};

		_gravity = 2 * _jumpHeight / Mathf.Pow(_timeToPeakReached, 2);
		_jumpVelocity = _gravity * _timeToPeakReached;
	}

	void FixedUpdate()
    {
		var dt = Time.deltaTime;
		var groundInfo = _controller.ProbeGround();

		_velocity.x = _input.x * _moveSpeed;

		if (!groundInfo.isGrounded) {
			_velocity.y -= _gravity * dt;
		} else {
			_velocity.y = 0;
			_controller.EnableGroundClamping(true);
		}

		if (groundInfo.isGrounded && _input.jump) {
			_velocity.y = _jumpVelocity;
			_controller.EnableGroundClamping(false);
		}

		if (!groundInfo.isGrounded && _previousGround.isGrounded && !_stickToGround)
			_controller.EnableGroundClamping(false);

		_controller.BeginMove(groundInfo, dt);
		var state = _controller.Update(_velocity);

		if (groundInfo.isGrounded && _controller.HorizontalTouch()) {
			state.finalPosition = _controller.HandleStep();
		}

		_controller.EndMove(state.finalPosition, state.finalRotation);
		_previousGround = groundInfo;
    }
}