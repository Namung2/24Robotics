using UnityEngine;
using System.Collections.Generic;
public class SensorSystem : MonoBehaviour
{
    public enum OperationMode
    {
        SLAM,           // 문제 1용 SLAM 모드
        Landmark,       // 문제 2,3용 랜드마크 기반 모드
        RangeFinder     // 문제 4,5용 거리센서 기반 모드
    }
    [Header("Operation Settings")]
    public OperationMode currentMode = OperationMode.SLAM;

    [Header("Noise Settings")]// 노이즈 setting
    public bool useNoise = true;
    public float noiseSigma = 0.4f;

    [Header("Range Finder Settings")]// 문제 4,5용 setting
    public float rangeFinderMaxRange = 8.0f;
    public float rangeFinderNoiseStd = 0.2f;
    public float angleResolution = 10.0f;  // 10도 간격
    private const int RANGE_MEASUREMENTS = 36; // 360도/10도 = 36개 측정점
    
    [Header("Landmark Mode Settings")]
    public float landmarkMaxRange = 5.0f;
    public float baseNoiseStd = 0.4f;         // 기본 노이즈 크기
    public float distanceNoiseFactor = 0.1f;  // 거리에 따른 노이즈 증가 계수
    public float maxNoiseStd = 1.0f;          // 최대 노이즈 크기
    
    [Header("Visualization")] // 센서 Sence화면 시 시각화
    public bool showDebugLines = true;
    public Color landmarkDetectionColor = Color.yellow;
    public Color obstacleDetectionColor = Color.red;    // 장애물용 빨간색
public struct LandmarkMeasurement
{
    public int id;
    public Vector2 relativePosition;
    public float distance;      // 전체 거리
    public float bearing;       // 방위각
    public Vector3 globalPosition;  // 실제 위치 (검증용)
}    
// Localization용 랜드마크 측정 구조체 문제 2,3번용
    public struct LocalizationLandmarkMeasurement
    {
        public int id;
        public float distance;        // 직선 거리만 사용
        public Vector3 globalPosition;
    }

// 거리 센서 측정 구조체 문제 4,5번용
    public struct RangeFinderMeasurement
    {
        public float angle;           // 측정 각도
        public float distance;        // 측정된 거리
        public bool isLandmark;       // 랜드마크 여부
        public Vector3 hitPosition;   // 충돌 위치
    }

void Update()
    {
         switch (currentMode)
        {
            case OperationMode.SLAM://1번 SLAM용
                DetectLandmarksForSLAM();
                break;
            case OperationMode.Landmark:// 문제 2,3을 위한 랜드마크 감지
                DetectLandmarksForLocalization();
                break;
            case OperationMode.RangeFinder://4,5번용 모든 장애물 감지
                DetectObjectsWithRangeFinder();
                break;
        }
    }
 void DetectLandmarksForSLAM()
    {
        foreach (GameObject landmark in GameObject.FindGameObjectsWithTag("Landmark"))
        {
            Vector3 relativePos = landmark.transform.position - transform.position;
            float xDistance = Vector3.Dot(relativePos, transform.right);
            float yDistance = Vector3.Dot(relativePos, transform.forward);
            
            if (useNoise)
            {
                xDistance += GenerateGaussianNoise(noiseSigma);
                yDistance += GenerateGaussianNoise(noiseSigma);
            }
            
            if (showDebugLines)
            {
                Debug.DrawLine(transform.position, landmark.transform.position, landmarkDetectionColor, 0.1f);
            }
        }
    }


void DetectLandmarksForLocalization() //문제 2,3용 센서메서드
{
    // 랜드마크 감지 (진한 파란색)
    foreach (GameObject landmark in GameObject.FindGameObjectsWithTag("Landmark"))
    {
        Vector3 relativePos = landmark.transform.position - transform.position;
            float distance = relativePos.magnitude;

            if (distance > landmarkMaxRange) continue;

            if (useNoise)
            {
                // 거리에 따른 노이즈 크기 계산
                float currentNoiseStd = CalculateNoiseForDistance(distance);
                
                // 계산된 노이즈 크기로 가우시안 노이즈 생성
                distance += GenerateGaussianNoise(currentNoiseStd);
            }

            if (showDebugLines)
            {
                Debug.DrawLine(transform.position, landmark.transform.position, landmarkDetectionColor, 0.1f);
            }
    }
}
 // 문제 4,5용 거리 센서 감지 메서드
    void DetectObjectsWithRangeFinder()
    {
        for (int i = 0; i < RANGE_MEASUREMENTS; i++)
        {
            float angle = i * angleResolution * Mathf.Deg2Rad;
            Vector3 direction = new Vector3(
                Mathf.Cos(angle + transform.eulerAngles.y * Mathf.Deg2Rad),
                0,
                Mathf.Sin(angle + transform.eulerAngles.y * Mathf.Deg2Rad)
            );

            RaycastHit hit;
            if (Physics.Raycast(transform.position, direction, out hit, rangeFinderMaxRange))
            {
                Color debugColor = hit.collider.CompareTag("Landmark") ? //장애물인지 Landmark인지판단
                    landmarkDetectionColor : obstacleDetectionColor;

                float distance = hit.distance;
                if (useNoise)
                {
                    distance += GenerateGaussianNoise(rangeFinderNoiseStd);
                }
                
                if (showDebugLines)
                {
                    Debug.DrawLine(transform.position, hit.point, debugColor, 0.1f);
                }
            }
        }
    }

public Dictionary<int, LandmarkMeasurement> GetSLAMMeasurements() //SLAM용
{
     if (currentMode != OperationMode.SLAM) 
            return new Dictionary<int, LandmarkMeasurement>();

    Dictionary<int, LandmarkMeasurement> measurements = new Dictionary<int, LandmarkMeasurement>();
    int landmarkId = 0;

    foreach (GameObject landmark in GameObject.FindGameObjectsWithTag("Landmark"))
    {
        Vector3 relativePos = landmark.transform.position - transform.position;
        float relativeX = Vector3.Dot(relativePos, transform.right);
        float relativeY = Vector3.Dot(relativePos, transform.forward);

        if (useNoise)
        {
            relativeX += GenerateGaussianNoise(noiseSigma);
            relativeY += GenerateGaussianNoise(noiseSigma);
        }

        // 거리와 방위각 계산
        float distance = Mathf.Sqrt(relativeX * relativeX + relativeY * relativeY);
        float bearing = Mathf.Atan2(relativeY, relativeX);

        measurements[landmarkId] = new LandmarkMeasurement
        {
            id = landmarkId,
            relativePosition = new Vector2(relativeX, relativeY),
            distance = distance,
            bearing = bearing,
            globalPosition = landmark.transform.position
        };

        landmarkId++;
    }
    //Debug.Log($"Landmark measurements count: {measurements.Count}");;

    return measurements;
}
 // 랜드마크 기반 Localization용 측정값 반환 (문제 2,3번)
public Dictionary<int, LocalizationLandmarkMeasurement> GetLocalizationMeasurements()
    {
        if (currentMode != OperationMode.Landmark)
            return new Dictionary<int, LocalizationLandmarkMeasurement>();

        Dictionary<int, LocalizationLandmarkMeasurement> measurements = 
            new Dictionary<int, LocalizationLandmarkMeasurement>();
        int landmarkId = 0;

        foreach (GameObject landmark in GameObject.FindGameObjectsWithTag("Landmark"))
        {
            Vector3 relativePos = landmark.transform.position - transform.position;
            float distance = relativePos.magnitude;

            if (distance > landmarkMaxRange) continue;

            if (useNoise)
            {
                distance += GenerateGaussianNoise(baseNoiseStd);
            }

            measurements[landmarkId] = new LocalizationLandmarkMeasurement
            {
                id = landmarkId,
                distance = distance,
                globalPosition = landmark.transform.position
            };

            landmarkId++;
        }

        return measurements;
    }
// 거리 센서 측정값 반환(문제 4,5)
public RangeFinderMeasurement[] GetRangeFinderMeasurements()
{
    if (currentMode != OperationMode.RangeFinder)
        return new RangeFinderMeasurement[0];

        RangeFinderMeasurement[] measurements = new RangeFinderMeasurement[RANGE_MEASUREMENTS];

        for (int i = 0; i < RANGE_MEASUREMENTS; i++)
        {
            float angle = i * angleResolution * Mathf.Deg2Rad;
            Vector3 direction = new Vector3(
                Mathf.Cos(angle + transform.eulerAngles.y * Mathf.Deg2Rad),
                0,
                Mathf.Sin(angle + transform.eulerAngles.y * Mathf.Deg2Rad)
            );

            RaycastHit hit;
            if (Physics.Raycast(transform.position, direction, out hit, rangeFinderMaxRange))
            {
                float distance = hit.distance;
                if (useNoise)
                {
                    distance += GenerateGaussianNoise(rangeFinderNoiseStd);
                }

                measurements[i] = new RangeFinderMeasurement
                {
                    angle = angle,
                    distance = distance,
                    isLandmark = hit.collider.CompareTag("Landmark"),
                    hitPosition = hit.point
                };
            }
            else
            {
                measurements[i] = new RangeFinderMeasurement
                {
                    angle = angle,
                    distance = rangeFinderMaxRange,
                    isLandmark = false,
                    hitPosition = transform.position + direction * rangeFinderMaxRange
                };
            }
        }

        return measurements;
    }


  private float CalculateNoiseForDistance(float distance)
    {
        // 거리에 따라 선형적으로 증가하는 노이즈 계산
        // distance가 0일 때는 baseNoiseStd
        // distance가 증가할수록 노이즈도 비례하여 증가
        float noiseStd = baseNoiseStd + (distance * distanceNoiseFactor);
        
        // 노이즈가 너무 커지는 것을 방지
        return Mathf.Min(noiseStd, maxNoiseStd);
    }
    private float GenerateGaussianNoise(float sigma)
    {
        //u1,u2는 독립적인 균일분포 난수 
        //Box-Muller 변환을 사용하여 균일분포 난수(u1, u2)를 표준 정규분포(Standard normal distribution) 난수로 변환
        float u1 = Random.value;
        float u2 = Random.value;
        float randomStandardNormal = Mathf.Sqrt(-2.0f * Mathf.Log(u1)) * Mathf.Sin(2.0f * Mathf.PI * u2);
        return randomStandardNormal * sigma;
    }
}