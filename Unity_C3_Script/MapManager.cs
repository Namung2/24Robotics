using UnityEngine;
using System.Collections.Generic;

public class MapManager : MonoBehaviour
{
    [Header("Map Settings")]
    public float cellSize = 1f;

    [Header("Prefabs")]
    public GameObject obstaclePrefab;
    public GameObject landmarkPrefab;

    // 장애물과 랜드마크를 저장할 Dictionary
    private Dictionary<Vector2Int, GameObject> obstacles = new Dictionary<Vector2Int, GameObject>();
    private Dictionary<Vector2Int, GameObject> landmarks = new Dictionary<Vector2Int, GameObject>();
  void Update()
{
    // O키를 누르면 장애물 숨기기
    if (Input.GetKeyDown(KeyCode.O))
    {
        ShowObstacles(false);
    }
    // P키를 누르면 장애물 보이기
    if (Input.GetKeyDown(KeyCode.P))
    {
        ShowObstacles(true);
    }
}
    void Start()
    {
        InitializeMap();
    }

    public void InitializeMap(bool createObstacles = true)
    {
        ClearMap(); // 기존 객체들 제거

        if (createObstacles)
        {
            // Map A의 장애물 위치
            Vector2Int[] obstaclePositions = new Vector2Int[]
            {
                new Vector2Int(3, 12), new Vector2Int(4, 12), 
                new Vector2Int(3, 11), new Vector2Int(4, 11), 
                new Vector2Int(3, 10), new Vector2Int(4, 10),
                new Vector2Int(3, 9), new Vector2Int(4, 9),   // 왼쪽 첫 번째 그룹

                new Vector2Int(8, 12), new Vector2Int(9, 12), 
                new Vector2Int(8, 11), new Vector2Int(9, 11),
                new Vector2Int(8, 10), new Vector2Int(9, 10),
                new Vector2Int(8, 9), new Vector2Int(9, 9), // 왼쪽에서 두 번째 그룹
                
                new Vector2Int(14, 12), new Vector2Int(15, 12), 
                new Vector2Int(14, 11), new Vector2Int(15, 11),
                new Vector2Int(14, 10), new Vector2Int(15, 10),
                new Vector2Int(14, 9), new Vector2Int(15, 9), // 오른쪽에서 두 번째 그룹
                
                new Vector2Int(18, 12), new Vector2Int(18, 11), 
                new Vector2Int(18, 10), new Vector2Int(18, 9),
                new Vector2Int(18, 8), new Vector2Int(18, 7),
                new Vector2Int(18, 6), new Vector2Int(17, 6),
                new Vector2Int(16, 6), new Vector2Int(15, 6),
                new Vector2Int(14, 6), new Vector2Int(13, 6),
                new Vector2Int(12, 6), new Vector2Int(11, 6),
                new Vector2Int(10, 6), new Vector2Int(9, 6), // 니은자 장애물

                new Vector2Int(21, 9), new Vector2Int(22, 9),
                new Vector2Int(23, 9), new Vector2Int(24, 9),
                new Vector2Int(21, 8), new Vector2Int(22, 8),
                new Vector2Int(23, 8), new Vector2Int(24, 8), // 맨 오른쪽 장애물

                // 아래쪽 장애물들
                new Vector2Int(3, 6), new Vector2Int(4, 6),
                new Vector2Int(5, 6), new Vector2Int(6, 6),
                new Vector2Int(3, 5), new Vector2Int(4, 5),
                new Vector2Int(5, 5), new Vector2Int(6, 5), // 맨 왼쪽

                new Vector2Int(12, 3), new Vector2Int(13, 3),
                new Vector2Int(12, 2), new Vector2Int(13, 2),

                new Vector2Int(16, 3), new Vector2Int(17, 3),
                new Vector2Int(16, 2), new Vector2Int(17, 2),

                new Vector2Int(20, 3), new Vector2Int(21, 3),
                new Vector2Int(20, 2), new Vector2Int(21, 2),
            };

            // 랜드마크 위치
            Vector2Int[] landmarkPositions = new Vector2Int[]
            {
                new Vector2Int(5, 11),   // 왼쪽 위
                new Vector2Int(16, 9),  // 오른쪽 위
                new Vector2Int(5, 4),    // 왼쪽 아래
                new Vector2Int(15, 2)    // 오른쪽 아래
            };

            CreateObjects(obstaclePositions, obstaclePrefab, obstacles, "Obstacle");
            CreateObjects(landmarkPositions, landmarkPrefab, landmarks, "Landmark");
        }
    }

    private void CreateObjects(Vector2Int[] positions, GameObject prefab, 
        Dictionary<Vector2Int, GameObject> dictionary, string namePrefix)
    {
        foreach (Vector2Int pos in positions)
        {
            Vector3 worldPos = GridToWorld(pos);
            GameObject obj = Instantiate(prefab, worldPos, Quaternion.identity, transform);
            obj.name = $"{namePrefix}_{pos.x}_{pos.y}";
            if (namePrefix == "Landmark")
        {
            obj.tag = "Landmark";
        }
        else if (namePrefix == "Obstacle")// 문제 5번 감지범위및 장애물 추가ㄴ
        {
        obj.tag = "Landmark";//이거 obstcles로 하면 1,2,3용 4,5번은 Landmark 
        }
            dictionary[pos] = obj;
        }
    }

    public void ClearMap()
    {
        foreach (var obj in obstacles.Values)
        {
            if (obj != null) Destroy(obj);
        }
        obstacles.Clear();

        foreach (var obj in landmarks.Values)
        {
            if (obj != null) Destroy(obj);
        }
        landmarks.Clear();
    }

    public Vector3 GridToWorld(Vector2Int gridPos)
    {
        return new Vector3(gridPos.x * cellSize + cellSize / 2f, 0.5f, 
            gridPos.y * cellSize + cellSize / 2f);
    }

    public Vector2Int WorldToGrid(Vector3 worldPos)
    {
        return new Vector2Int(
            Mathf.FloorToInt(worldPos.x / cellSize),
            Mathf.FloorToInt(worldPos.z / cellSize)
        );
    }

    public bool IsObstacle(Vector2Int gridPos)
    {
        return obstacles.ContainsKey(gridPos);
    }

    public bool IsLandmark(Vector2Int gridPos)
    {
        return landmarks.ContainsKey(gridPos);
    }

    // 장애물을 보여주거나 숨기는 메서드 추가
    public void ShowObstacles(bool show)
    {
        foreach (var obstacle in obstacles.Values)
        {
            if (obstacle != null)
            {
                obstacle.SetActive(show);
            }
        }
    }
}
