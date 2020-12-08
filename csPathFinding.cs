using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System;
using UnityEngine;

/// <summary>
/// 경로를 찾아서 전달해주는 클래스
/// </summary>
public class csPathFinding : MonoBehaviour
{
	// 경로 탐색을 수행하는 코루틴
	private IEnumerator m_CoFindPath = null;

	// 예기치 못한 이동 방해 요소를 제외시키기 위함
	private List<csNode> m_ListExceptionNode = new List<csNode>();

	// 타일맵을 생성하는 클래스
	private csGridCreator m_GridCreator = null;

	private csNode m_StartNode = null;
	public csNode m_EndNode = null;

	protected virtual void Start()
	{
		m_GridCreator = csPathRequestManager.Instance.GetComponent<csGridCreator>();
	}

	public void Initialize()
	{
		for (int i = 0; i < m_ListExceptionNode.Count; i++)
		{
			m_ListExceptionNode[i].ResetType(gameObject.GetInstanceID());
		}

		m_ListExceptionNode.Clear();
	}

	public void StartFindPath(Vector3 _startPosition, Vector3 _endPosition, csNode _exceptionNode)
	{
		if (m_CoFindPath != null)
		{
			StopCoroutine(m_CoFindPath);
		}

		m_CoFindPath = CoFindPath(_startPosition, _endPosition, _exceptionNode);
		StartCoroutine(m_CoFindPath);
	}

	private IEnumerator CoFindPath(Vector3 _startPosition, Vector3 _endPosition, csNode _exceptionNode)
	{
		// 요청을 받을 때 만약 탐색에 제외할 노드를 전달 받으면 저장했다가 탐색에 제외시킴
		if (_exceptionNode != null)
		{
			if (!m_ListExceptionNode.Contains(_exceptionNode))
			{
				_exceptionNode.SetType(gameObject.GetInstanceID(), eNodeType.Obstacle);
				m_ListExceptionNode.Add(_exceptionNode);
			}
		}

		bool IsPathSuccess = false;

		if (m_StartNode != null)
		{
			m_StartNode.ResetType(gameObject.GetInstanceID());
		}

		if (m_EndNode != null)
		{
			m_EndNode.ResetType(gameObject.GetInstanceID());
		}

		m_StartNode = m_GridCreator.NodeFromWorldPoint_Unit(_startPosition);
		m_EndNode = m_GridCreator.NodeFromWorldPoint_Unit(_endPosition);

		m_EndNode.SetType(gameObject.GetInstanceID(), eNodeType.End);

		if (m_EndNode.NodeType != eNodeType.Obstacle)
		{ 
			Heap<csNode> OpenList = new Heap<csNode>(m_GridCreator.m_Grid_Unit.MaxSize);
			HashSet<csNode> CloseSet = new HashSet<csNode>();

			OpenList.Add(m_StartNode);

			while (OpenList.Count > 0)
			{
				csNode Node = OpenList.RemoveFirst();

				CloseSet.Add(Node);

				// 경로 탐색 완료
				if (Node == m_EndNode)
				{
					IsPathSuccess = true;

					break;
				}		

				for (int i = 0; i < Node.m_ListNeighbours.Count; i++)
				{
					csNode Neighbour = Node.m_ListNeighbours[i];
				
					if (CloseSet.Contains(Neighbour))
					{
						continue;
					}

					if (CheckNeighbours(Node, Neighbour, i))
					{
						CloseSet.Add(Neighbour);

						continue;
					}

					int NewCostNeighbour = Node.G + GetDistance(Node, Neighbour);

					if (NewCostNeighbour < Neighbour.G || !OpenList.Contains(Neighbour))
					{
						Neighbour.G = NewCostNeighbour;
						Neighbour.H = GetDistance(Neighbour, m_EndNode);
						Neighbour.m_Parent = Node;

						if (!OpenList.Contains(Neighbour))
						{
							OpenList.Add(Neighbour);
						}
					}
				}
			}
		}

		yield return null;

		Vector3[] WayPoints;

		if (IsPathSuccess)
		{
			List<csNode> ListPathNodes = RetracePath(m_StartNode, m_EndNode);

			List<Vector3> ListWayPoint = new List<Vector3>();

			// 경로에 포함된 노드들의 위치만 추출하여 저장
			for (int i = 0; i < ListPathNodes.Count; i++)
			{
				ListWayPoint.Add(ListPathNodes[i].m_WorldPosition);
			}

			WayPoints = ListWayPoint.ToArray();

			// 경로가 현재 역순이므로 
			Array.Reverse(WayPoints);
		}

		// 경로 탐색 결과 전달
		csPathRequestManager.Instance.FinishedProcessingPath(WayPoints, IsPathSuccess);

		m_CoFindPath = null;
	}

	/// <summary>
	/// 오픈된 노드의 주변 노드가 오픈할 수 있는지 확인
	/// </summary>
	/// <param name="_open"></param>
	/// <param name="_neighbour"></param>
	/// <param name="_index"></param>
	/// <returns></returns>
	private bool CheckNeighbours(csNode _open, csNode _neighbour, int _index)
    {
		// 제외 대상이라면
		if (m_ListExceptionNode.Contains(_neighbour))
		{
			return false;
		}

		// 장애물이라면
		if (_neighbour.NodeType == NodeType.Obstacle)
		{
			return false;
		}

		// 대각선일 때
		if (_index % 2 != 0)
		{
			/* 1 0
			 * 0 1
			 * 왼쪽 상단의 1이 Open Node 오른쪽 하단의 1이 현재 확인하려는 인접노드, 즉 대각선일 때
			 * 0에 해당하는 노드를 얻음
			 */
			csNode NodeA = m_GridCreator.m_Grid_Unit.Grid[_neighbour.m_Grid.x, _open.m_Grid.y];
			csNode NodeB = m_GridCreator.m_Grid_Unit.Grid[_open.m_Grid.x, _neighbour.m_Grid.y];

			/* 해당하는 노드들이 이동할 수 있는 없는 노드인지 확인
			 * 이것은 노드상으로는 이동이 가능하다고 판단되지만 경로는 이동시키는 개체의 크기를 고려하지 않으므로
			 * 추가로 확인하는 작업
			 */
			if (NodeA.NodeType != eNodeType.Walkable || NodeB.NodeType != eNodeType.Walkable)
			{
				Vector3 BetweenNodePosition = (_open.m_WorldPosition + _neighbour.m_WorldPosition) / 2.0f;

				// 그럼에도 불구하고도 실제로 이동이 불가능한지 확인
				if (IsRealImpossible(BetweenNodePosition))
				{
					return false;
				}
			}
		}

		return true;
	}

	/// <summary>
	/// 전달받은 위치가 실제로 이동이 불가능한지 확인
	/// </summary>
	/// <param name="_position"></param>
	/// <returns></returns>
	private bool IsRealImpossible(Vector3 _position)
	{
		if (Physics.CheckBox(_position, Vector3.one * m_GridCreator.m_Grid_Unit.NodeRadius, Quaternion.identity, csBattleManager.Instance.m_iLayerMask_Map))
		{
			return true;
		}

		return false;
	}

	/// <summary>
	/// 목적지부터 출발지까의 노드들을 추적하여 저장 후 반환
	/// </summary>
	/// <param name="_startNode"></param>
	/// <param name="_endNode"></param>
	/// <returns></returns>
	private List<csNode> RetracePath(csNode _startNode, csNode _endNode)
	{
		List<csNode> ListPath = new List<csNode>();

		csNode CurrentNode = _endNode;

		while (CurrentNode != _startNode)
		{
			ListPath.Add(CurrentNode);
			CurrentNode = CurrentNode.m_Parent;
		}

		return ListPath;
	}

	private int GetDistance(csNode _nodeA, csNode _nodeB)
	{
		int DistanceX = Mathf.Abs(_nodeA.m_Grid.x - _nodeB.m_Grid.x);
		int DistanceY = Mathf.Abs(_nodeA.m_Grid.y - _nodeB.m_Grid.y);

		if (DistanceX > DistanceY)
		{
			return 14 * DistanceY + 10 * (DistanceX - DistanceY);
		}

		return 14 * DistanceX + 10 * (DistanceY - DistanceX);
	}
}