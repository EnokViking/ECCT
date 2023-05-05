using System;
using System.Collections.Generic;

using UnityEngine;

[RequireComponent(typeof(Rigidbody2D), typeof(BoxCollider2D))]
public class ExampleCharacterController2D : MonoBehaviour
{
	//[SerializeField] private float _moveSpeed;
	[SerializeField] int _maxContactsToQuery;
	[SerializeField] private LayerMask _obstacles;
	[SerializeField] private int _solverIterations;
	[SerializeField] private float _maxStepHeight;
	[SerializeField] private float _maxGroundAngle;

	public bool _drawDebugInfo;

	private float _pose;
	private Vector2 _localDir;

	private Rigidbody2D _rBody;
	private BoxCollider2D _collider;

	private RaycastHit2D[] _hitResults;

	private Collider2D[] _overlapResults;
	private ContactPoint2D[] _contactPoints;
	private ContactFilter2D _contactFilter;

	private Vector2 _velocity;

	private Vector2 _upVector;
	private float _contactOffset;
	private List<GroundInfo> _groundPoints;

	public struct GroundInfo
	{
		public bool isGrounded;
		public int contactCount;
		public float angle;

		public Vector2 normalA;
		public Vector2 normalB;

		public Vector2 pointA;
		public Vector2 pointB;
	}

	// Start is called before the first frame update
	private void Awake()
	{
		//Having a rigidbody component in Unity is a good idea if you intend to have physics shapes that move around
		_rBody = GetComponent<Rigidbody2D>();
		_collider = GetComponent<BoxCollider2D>();

		_rBody.isKinematic = true;
		_rBody.interpolation = RigidbodyInterpolation2D.Interpolate;
		_rBody.useFullKinematicContacts = true;

		_hitResults = new RaycastHit2D[2];
		_overlapResults = new Collider2D[8];

		_contactPoints = new ContactPoint2D[_maxContactsToQuery];
		
		_upVector = Vector2.up;

		_contactFilter = new ContactFilter2D() { layerMask = _obstacles };
		_contactOffset = Physics2D.defaultContactOffset;
		_groundPoints = new List<GroundInfo>();
	}

	private void Update()
	{
		if (_drawDebugInfo) {
			var headPos = transform.position + transform.up * _collider.size.y * 0.5f;
			var vel = GetVelocity();

			var up = transform.up;
			var right = transform.right;
			var left = -transform.right;

			Debug.DrawLine(headPos + up * 1.0f, headPos + up * 2.0f, Color.green);

			Debug.DrawLine(headPos + up * 1.5f + left * 0.5f, headPos + up * 1.5f + right * 0.5f, Color.green);
			Debug.DrawRay(headPos + up * 1.5f, _localDir.normalized * 0.5f, Color.red);
		}
	}
	public void SetUpVector(Vector2 up) => _upVector = up;
	public Vector2 GetUpVector() => _upVector;
	public Vector2 GetVelocity() => _velocity;

	//We collect input in Update which runs every frame and then we drive the attached rigidbody with its MovePosition method for first order control.
	//Interpolation is enabled, which lets the renderer display our player graphics between physics steps to smooth things out.
	//This effectively works the same way as described in this excellent post by Glenn Fiedler: https://gafferongames.com/post/fix_your_timestep/
	public void Move(Vector2 velocity, GroundInfo groundInfo, float dt)
	{
		_velocity = velocity;

		var startRot = _rBody.rotation;
		var startPos = _rBody.position;

		_rBody.SetRotation(Quaternion.LookRotation(Vector3.forward, _upVector));
		_pose = _rBody.rotation;

		if (groundInfo.isGrounded && velocity.y <= 0) {
			_localDir = Vector2.Perpendicular(-SelectRelevantNormal(groundInfo) * Math.Sign(_velocity.x));
		} else {
			_localDir = transform.right * _velocity.normalized.x + transform.up * _velocity.normalized.y;
		}

		bool horizontalCollision = false;
		bool verticalCollision = false;

		if (HandleSweep(startPos, _localDir, Mathf.Max(2 * _contactOffset, _velocity.magnitude * dt), out Vector2 normal)) {
			float h = Vector2.Dot(normal, new Vector2(_localDir.x, 0f));
			float v = Vector2.Dot(normal, new Vector2(0f, _localDir.y));

			horizontalCollision = Math.Sign(h) < 0 && !IsGroundNormal(normal);
			verticalCollision = Math.Sign(v) < 0;

			if (horizontalCollision) {
				_velocity.x = 0;
			}

			if (verticalCollision) {
				_velocity.y = 0;
			}
		}

		Vector2 finalPosition = Vector2.MoveTowards(_rBody.position, _rBody.position + _localDir, _velocity.magnitude * dt);
		_rBody.position = finalPosition;

		if (horizontalCollision) {
			HandleStep(groundInfo);
		}

		OverlapRecovery(ref finalPosition);

		//Move the rigidbody back to the start position of this physics frame and move it properly with MovePosition to take interpolation into consideration. (this rhymes too!)
		_rBody.position = startPos;
		_rBody.rotation = startRot;

		//This solves an edge case introduced by the contact offset when the character is wedged into 2 opposing slopes and is moving too slow to overcome the combined offset
		//of both slope geometries.
		//Just add the contact offset to the final move direction to overcome the "magnetic force".
		//this is only seems relevant when the character is in contact with multiple points on a slope.
		if (groundInfo.contactCount >= 2 && groundInfo.angle > 0 && Vector2.Dot(groundInfo.normalA, groundInfo.normalB) != 1) {
			_rBody.MovePosition(finalPosition + _localDir * (_contactOffset * 2));
			return;
		}
			
		_rBody.MovePosition(finalPosition);
		_rBody.MoveRotation(_pose);
	}

	private void OnDrawGizmos()
	{
		if (_drawDebugInfo && _collider != null) {
			Gizmos.color = Color.red;
			Gizmos.matrix = transform.localToWorldMatrix;
			Gizmos.DrawWireCube(transform.InverseTransformDirection(_localDir), new Vector3(_collider.size.x, _collider.size.y));
		}
	}

	private void OverlapRecovery(ref Vector2 finalPosition) 
	{
		for (int it = 0; it < _solverIterations; it++) {
			int count = Physics2D.OverlapBoxNonAlloc(_rBody.position, _collider.size, _pose, _overlapResults, _obstacles);

			for (int i = 0; i < count; i++) {
				var colliderDist = Physics2D.Distance(_collider, _overlapResults[i]);

				if (colliderDist.isOverlapped) {
					var dist = colliderDist.distance + _contactOffset * 0.5f;
					var dir = (colliderDist.pointB - colliderDist.pointA).normalized;

					//Corrective displacement
					_rBody.position -= dir * dist;
				}
			}
		}

		finalPosition = _rBody.position;
	}

	private void HandleStep(GroundInfo info) 
	{
		if (!info.isGrounded)
			return;

		var width = _collider.size.x + 2 * _contactOffset;
		var height = _contactOffset;

		if (Physics2D.BoxCastNonAlloc(_rBody.position + _upVector * _collider.size.y * 0.5f, new Vector2(width, height), _pose, 
			-_upVector, _hitResults, _collider.size.y, _obstacles) == 0)
			return;

		var diff = _collider.size.y - _hitResults[0].distance;

		if (diff <= _maxStepHeight) {
			_rBody.position += _upVector * diff + _localDir * (_contactOffset * 0.5f);
			return;
		}
	}

	private bool HandleSweep(Vector2 startPos, Vector2 direction, float distance, out Vector2 normal) 
	{
		int sweepCount = Physics2D.BoxCastNonAlloc(_rBody.position, _collider.size, _pose, direction, _hitResults, distance, _obstacles);
		normal = Vector2.zero;

		if (sweepCount > 0) {
			_rBody.position = startPos;

			float hitDistance = _hitResults[0].distance - _contactOffset * 0.5f;
			_rBody.position += direction * hitDistance;
			normal = _hitResults[0].normal;
		}

		return sweepCount > 0;
	}

	public GroundInfo ProbeGround() 
	{
		var groundInfo = new GroundInfo();

		int count = Physics2D.GetContacts(_collider, _contactFilter, _contactPoints);
		Vector2 feetPos = _rBody.position - _upVector * _collider.size.y * 0.5f;

		for (int i = 0; i < count; i++) {
			var contact = _contactPoints[i];
			var angle = Vector2.Angle(_upVector, contact.normal);

			bool grounded = angle >= 0 && angle <= _maxGroundAngle 
				&& Vector2.Distance(feetPos, contact.point) <= (_collider.size.x)
				&& _velocity.y <= 0;

			if (grounded) {
				groundInfo.normalA = contact.normal;
				groundInfo.pointA = contact.point;
				groundInfo.angle = angle;
				_groundPoints.Add(groundInfo);
			}
		}

		count = _groundPoints.Count;
		groundInfo.isGrounded = count > 0;
		groundInfo.contactCount = count;

		if (count > 0) {
			groundInfo.normalA = _groundPoints[0].normalA;
			groundInfo.pointA = _groundPoints[0].pointA;

			groundInfo.normalB = _groundPoints[0].normalA;
			groundInfo.pointB = _groundPoints[0].pointA;

			if (count >= 2) {
				groundInfo.normalB = _groundPoints[1].normalA;
				groundInfo.pointB = _groundPoints[1].pointA;
			}
		}
			
		_groundPoints.Clear();
		return groundInfo;
	}

	Vector2 SelectRelevantNormal(GroundInfo info) 
	{
		var pos = _rBody.position - _upVector * (_collider.size.y * 0.5f) + (_velocity.normalized * _collider.size.x * 0.5f);

		if (info.normalA != Vector2.zero && info.normalB != Vector2.zero) {
			if (Vector2.Distance(info.pointA, pos) < Vector2.Distance(info.pointB, pos)) {
				return info.normalA;
			}
		}
	
		return info.normalA == Vector2.zero ? info.normalB : info.normalA;
	}

	bool IsGroundNormal(Vector2 normal) 
	{
		var angle = Vector2.Angle(normal, _upVector);
		return angle >= 0 && angle <= _maxGroundAngle;
	}
}
