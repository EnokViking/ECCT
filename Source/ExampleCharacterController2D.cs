using System;
using System.Collections.Generic;

using UnityEngine;

public enum CollisionType
{
	Horizontal,
	Vertical
}

public delegate void OnCollision(CollisionType type);

[RequireComponent(typeof(Rigidbody2D), typeof(BoxCollider2D))]
public class ExampleCharacterController2D : MonoBehaviour
{
	[Header("Contacts")]
	[SerializeField] int _maxContactsToQuery;
	[SerializeField] private LayerMask _layerMask;

	[Header("Collision")]
	[SerializeField] private int _solverIterations;
	[SerializeField] private int _subSteps;

	[Header("Ground")]
	[SerializeField] private float _maxStepHeight;
	[SerializeField] private float _maxGroundAngle;

	[Header("Debug")]
	[SerializeField] private bool _drawDebugInfo;
	[SerializeField] private bool _enableGroundClamping;

	public event OnCollision OnControllerCollision;

	private Vector2 _direction;

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

	private float _dt;
	private GroundInfo _groundInfo;

	private Vector2 _startPos;

	private float _startRot;
	private float _finalRot;

	private bool _horizontalCollision;
	private bool _verticalCollision;

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

	public struct ControllerState 
	{
		public Vector2 finalPosition;
		public float finalRotation;
	}

	private void Awake()
	{
		_rBody = GetComponent<Rigidbody2D>();
		_collider = GetComponent<BoxCollider2D>();

		_rBody.isKinematic = true;
		_rBody.interpolation = RigidbodyInterpolation2D.Interpolate;
		_rBody.useFullKinematicContacts = true;

		_hitResults = new RaycastHit2D[2];
		_overlapResults = new Collider2D[8];

		_contactPoints = new ContactPoint2D[_maxContactsToQuery];
		
		_upVector = Vector2.up;

		_contactFilter = new ContactFilter2D() { layerMask = _layerMask };
		_contactOffset = Physics2D.defaultContactOffset;
		_groundPoints = new List<GroundInfo>();
	}

	private void Update()
	{
		if (_drawDebugInfo) {
			var headPos = transform.position + transform.up * _collider.size.y * 1f;

			Debug.DrawLine(headPos - Vector3.up * 0.5f, headPos + Vector3.up * 0.5f, Color.green);
			Debug.DrawLine(headPos - Vector3.right * 0.5f, headPos + Vector3.right * 0.5f, Color.green);
			Debug.DrawRay(headPos, _velocity.normalized * 0.5f, Color.red);
		}
	}

	public void SetUpVector(Vector2 up) => _upVector = up;
	public Vector2 GetUpVector() => _upVector;
	public Vector2 GetVelocity() => _velocity;
	public void EnableGroundClamping(bool flag) => _enableGroundClamping = flag;

	public bool HorizontalTouch() => _horizontalCollision;
	public bool VerticalTouch() => _verticalCollision;

	public void BeginMove(GroundInfo groundInfo, float dt)
	{
		_startRot = _rBody.rotation;
		_startPos = _rBody.position;

		_groundInfo = groundInfo;

		_rBody.SetRotation(Quaternion.LookRotation(Vector3.forward, _upVector));
		_finalRot = _rBody.rotation;

		_dt = dt;

		_verticalCollision = false;
		_horizontalCollision = false;
	}

	public ControllerState Update(Vector2 velocity)
	{
		_velocity = velocity;
		_direction = transform.TransformDirection(_velocity.normalized);

		if (_groundInfo.isGrounded && _velocity.y <= 0) {
			Vector2 relevantNormal = -SelectRelevantNormal(_groundInfo) * Math.Sign(_velocity.x);
			_direction = Vector2.Perpendicular(relevantNormal);
		}
			
		if (HandleSweep(_startPos, _direction, Mathf.Max(2 * _contactOffset, _velocity.magnitude * _dt), out Vector2 normal)) {
			float h = Vector2.Dot(normal, new Vector2(_direction.x, 0f));
			float v = Vector2.Dot(normal, new Vector2(0f, _direction.y));

			_horizontalCollision = h < 0 && !IsGroundNormal(normal) && !IsGroundNormal(-normal);
			_verticalCollision = v < 0 && (IsGroundNormal(normal) || IsGroundNormal(-normal));

			if (_horizontalCollision) {
				_velocity.x = 0;
				OnControllerCollision?.Invoke(CollisionType.Horizontal);
			}

			if (_verticalCollision) {
				_velocity.y = 0;
				OnControllerCollision?.Invoke(CollisionType.Vertical);
			}
		}

		float f = _dt / Mathf.Clamp(_subSteps, 1, 120);
		for (float step = 0; step < _dt; step += f) {
			_rBody.position = Vector2.MoveTowards(_rBody.position, _rBody.position + _direction, _velocity.magnitude * f);
			if (ClampToGround(out Vector2 newNormal)) {
				_direction = Vector2.Perpendicular(-newNormal) * Math.Sign(_velocity.x);
			}
		}

		var finalPos = _rBody.position;

		return new ControllerState { finalPosition = finalPos, finalRotation = _finalRot };
	}

	public void EndMove(Vector2 finalPosition, float finalRotation) 
	{
		OverlapRecovery(ref finalPosition, _solverIterations);

		_rBody.position = _startPos;
		_rBody.rotation = _startRot;

		_rBody.MovePosition(finalPosition);
		_rBody.MoveRotation(finalRotation);
	}

	private void OnDrawGizmos()
	{
		if (_drawDebugInfo && _collider != null) {
			Gizmos.color = Color.red;
			Gizmos.matrix = transform.localToWorldMatrix;
			Gizmos.DrawWireCube(transform.InverseTransformDirection(_direction), new Vector3(_collider.size.x, _collider.size.y));
		}
	}

	private bool ClampToGround(out Vector2 normal) 
	{
		normal = Vector2.zero;

		Vector2 dir = _groundInfo.isGrounded ? -SelectRelevantNormal(_groundInfo) : -_upVector;
		int count = _rBody.Cast(dir, _hitResults, Mathf.Infinity);

		if (!_enableGroundClamping || count == 0 && !IsGroundNormal(_hitResults[0].normal))
			return false;

		if (_groundInfo.isGrounded && _hitResults[0].distance > 2 * _contactOffset)
			return false;

		if (_hitResults[0].distance <= _contactOffset)
			return false;

		_rBody.position += dir * (_hitResults[0].distance - _contactOffset);
		normal = _hitResults[0].normal;
		return true;
	}

	private void OverlapRecovery(ref Vector2 finalPosition, int solverIterations) 
	{
		for (int it = 0; it < solverIterations; it++) {
			int count = _rBody.OverlapCollider(_contactFilter, _overlapResults);

			for (int i = 0; i < count; i++) {
				var colliderDist = Physics2D.Distance(_collider, _overlapResults[i]);

				if (colliderDist.isOverlapped) {
					var dist = colliderDist.distance + (_contactOffset * 0.5f);
					var dir = (colliderDist.pointB - colliderDist.pointA).normalized;

					_rBody.position -= dir * dist;
				}
			}
		}

		finalPosition = _rBody.position;
	}

	public Vector2 HandleStep() 
	{
		var width = _collider.size.x + 0.1f;
		var height = _contactOffset;

		if (Physics2D.BoxCastNonAlloc(_rBody.position + _upVector * _collider.size.y * 0.5f, new Vector2(width, height), _finalRot, 
			-_upVector, _hitResults, _collider.size.y, _layerMask) == 0)
			return _rBody.position;

		var diff = _collider.size.y - _hitResults[0].distance;

		if (diff <= _maxStepHeight && IsGroundNormal(_hitResults[0].normal)) {

			_rBody.position += _upVector * (diff + _contactOffset);
			_rBody.position += _direction * (2 * _contactOffset);
		}

		return _rBody.position;
	}

	private bool HandleSweep(Vector2 startPos, Vector2 direction, float distance, out Vector2 normal) 
	{
		int sweepCount = Physics2D.BoxCastNonAlloc(_rBody.position, _collider.size, _finalRot, direction, _hitResults, distance, _layerMask);
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
		var right = new Vector2(transform.right.x, transform.right.y);
		var pos = _rBody.position - _upVector * (_collider.size.y * 0.5f) + right * (_velocity.x * 0.5f);

		if (info.normalA != Vector2.zero && info.normalB != Vector2.zero) {
			if (Vector2.Distance(info.pointA, pos) < Vector2.Distance(info.pointB, pos)) {
				return info.normalA;
			}

			return info.normalB;
		}
	
		return Vector2.zero;
	}

	bool IsGroundNormal(Vector2 normal) 
	{
		var angle = Vector2.Angle(_upVector, normal);
		return angle >= 0 && angle <= _maxGroundAngle;
	}
}