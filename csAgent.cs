using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MEC;

/// <summary>
/// 경로를 받아 실제로 이동을 수행하는 클래스
/// </summary>
public class csAgent : MonoBehaviour
{
	// 경로를 따라 이동하는 코루틴
	private IEnumerator<float> m_CoFollowPath = null;

	// 이동 완료 콜백
	private Action m_Callback_FollowComplete = null;

	private Transform m_Transform = null;

	private CharacterController m_CharacterController = null;

	// 타일맵을 생성하는 클래스로 현재 자신의 위치에 해당하는 타일을 받아오기 위한 용도
	protected csGridCreator m_GridCreator = null;

	// 경로찾기 기능
	private csPathFinding m_PathFinding = null;

	// 경로를 따라 회전
	protected csRotationToTarget m_RotationToTarget = null;

	public Vector3[] m_Path = null;
	public Vector3 m_Target = Vector3.zero;
	public Vector3 m_CurrentWayPoint = Vector3.zero;
	public Vector3 m_LastWayPoint = Vector3.zero;

	public float m_fFollowingSpeed = 4.0f;
	public float m_fDelayCheckCycle = 0.0f;

	public int m_iWayPointIndex;

	public bool m_bIsFollowing = false;

	protected virtual void Awake()
	{
		m_Transform = GetComponent<Transform>();
		m_CharacterController.GetComponent<CharacterController>();
		m_GridCreator = csPathRequestManager.Instance.GetComponent<csGridCreator>();
		m_PathFinding = GetComponent<csPathFinding>();
		m_RotationToTarget = GetComponent<csRotationToTarget>();
	}

	public void StartFollowPath(Vector3 _target, Action _callback_Complete = null)
	{
		m_Target = _target;

		// 길찾기 요청
		csPathRequestManager.RequestPath(m_PathFinding, m_Transform.position, m_Target, null, OnPathFound);

		if (_callback_Complete != null)
		{
			m_Callback_FollowComplete = _callback_Complete;
		}		
	}

	/// <summary>
	/// 경로 이동이 완료됐을 때, 혹은 AI 상태에 따라서 이동을 멈춰야 할 때 호출
	/// </summary>
	public void StopFollowPath()
	{
		if (m_CoFollowPath != null)
		{
			StopCoroutine(m_CoFollowPath);
			m_CoFollowPath = null;
		}

		if (m_Callback_FollowComplete != null)
		{
			m_Callback_FollowComplete = null;
		}

		m_PathFinding.Initialize();

		m_Path = default;

		m_bIsFollowing = false;
	}

	/// <summary>
	/// 경로 찾기 결과 콜백 함수
	/// </summary>
	/// <param name="_newPath"></param>
	/// <param name="_success"></param>
	private void OnPathFound(Vector3[] _newPath, bool _success)
	{
		if (_success)
		{
			m_bIsFollowing = true;

			m_iWayPointIndex = 0;
			m_Path = _newPath;

			m_LastWayPoint = m_Path[m_Path.Length - 1];

			ChangeCurrentWayPoint(0);

			// 현재 이동 중이 아닐 때만 코루틴 호출
			if (m_CoFollowPath == null)
			{
				m_CoFollowPath = CoFollowPath();
				StartCoroutine(m_CoFollowPath);
			}

			return;
		}
	}

	private void ChangeCurrentWayPoint(int _index)
	{
		m_CurrentWayPoint = m_Path[_index];

		Vector3 RelativePositionFromMeToWayPoint = Utility.RelativePosition(new Vector3(m_Transform.position.x, 0.0f, m_Transform.position.z), m_CurrentWayPoint);

		// 이동할 웨이포인트 쪽을 바라봄
		m_RotationToTarget.Rotation(m_Transform, RelativePositionFromMeToWayPoint);
	}

	#region Coroutine

	private IEnumerator<float> CoFollowPath()
	{
		Vector3 PrevPosition = Vector3.zero;

		float Timer = 0.0f;
		float DistanceFromMeToWayPoint = 0.0f;

		while (true)
		{
			yield return Timing.WaitForOneFrame;

			Timer += Time.deltaTime;

			// 일정 시간마다 이동한 거리를 체크하여 원래 이동했어야 할 거리보다 작다면
			// 예기치 못한 충돌이 발생했다는 의미이므로 경로를 다시 요청
			if (Timer >= m_fDelayCheckCycle)
			{
				float DistanceFromPrev = Utility.Distance(PrevPosition, m_Transform.position);
				PrevPosition = m_Transform.position;

				if (DistanceFromPrev < (m_fFollowingSpeed * m_fDelayCheckCycle) - 0.2f)
				{
					csPathRequestManager.RequestPath(m_PathFinding, m_Transform.position, m_Target, m_GridCreator.NodeFromWorldPoint_Unit(m_CurrentWayPoint), OnPathFound);
				}

				Timer = 0.0f;
			}

			DistanceFromMeToWayPoint = Utility.Distance(m_Transform.position, m_CurrentWayPoint);

			// 목적지 도착
			if (m_iWayPointIndex == m_Path.Length)
			{
				m_Callback_FollowComplete?.Invoke();

				StopFollowPath();

				break;
			}
			else
			{
				// 현재 웨이포인트에 도달했다면 다음 포인트로 바꿔줌
				if (DistanceFromMeToWayPoint == 0.0f)
				{
					m_iWayPointIndex++;

					if (m_iWayPointIndex < m_Path.Length)
					{
						ChangeCurrentWayPoint(m_iWayPointIndex);
					}
				}

				Vector3 RelativePosition = Utility.RelativePosition(m_Transform.position, m_CurrentWayPoint);
				Vector3 Direction = RelativePosition.normalized * m_fFollowingSpeed * Time.deltaTime;

				m_CharacterController.Move(Direction);
			}
		}
	}

	#endregion
}
