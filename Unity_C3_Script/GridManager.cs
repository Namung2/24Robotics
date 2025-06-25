using UnityEngine;

public class GridManager : MonoBehaviour
{
    public int width = 25;   // 맵 가로 크기로 수정
    public int height = 15;  // 맵 세로 크기로 수정
    public float cellSize = 1f;
    public Material gridMaterial;
    public bool showGrid = true;

    private void Start()
    {
        CreateGrid();
    }

    void CreateGrid()
    {
        if (showGrid)
        {
            // 가로선 (Z축 방향)
            for (int i = 0; i <= height; i++)
            {
                CreateLine(new Vector3(0, 0.01f, i), new Vector3(width, 0.01f, i));
            }

            // 세로선 (X축 방향)
            for (int i = 0; i <= width; i++)
            {
                CreateLine(new Vector3(i, 0.01f, 0), new Vector3(i, 0.01f, height));
            }
        }
    }

    void CreateLine(Vector3 start, Vector3 end)
    {
        GameObject line = new GameObject("GridLine");
        line.transform.parent = transform;
        
        LineRenderer lineRenderer = line.AddComponent<LineRenderer>();
        lineRenderer.material = gridMaterial;
        lineRenderer.startWidth = 0.01f;
        lineRenderer.endWidth = 0.01f;
        lineRenderer.SetPosition(0, start);
        lineRenderer.SetPosition(1, end);
        
        // 라인이 잘 보이도록 추가 설정
        lineRenderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        lineRenderer.receiveShadows = false;
        lineRenderer.positionCount = 2;
    }

    public Vector2Int WorldToGrid(Vector3 worldPosition)
    {
        return new Vector2Int(
            Mathf.FloorToInt(worldPosition.x / cellSize),
            Mathf.FloorToInt(worldPosition.z / cellSize)
        );
    }

    public Vector3 GridToWorld(Vector2Int gridPosition)
    {
        return new Vector3(
            gridPosition.x * cellSize + cellSize/2f,
            0f,
            gridPosition.y * cellSize + cellSize/2f
        );
    }
}