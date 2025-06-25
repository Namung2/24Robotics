using UnityEngine;
using System.Collections.Generic;
using System.Collections;
//Auto모드밖에 없음 수동조작모드 삭제 12/09
public class RobotController : MonoBehaviour
{
    [Header("Debug Settings")]
    public bool showDebugLines = false;
    [Header("Components")]
    public Transform robotBody;
    public Transform frontWheel;
    public Transform rearWheel;
    public enum OperationMode
    {
        SLAM,           // 문제 1용 SLAM 모드
        Landmark,       // 문제 2,3용 랜드마크 기반 모드
        RangeFinder    // 문제 4,5용 거리센서 기반 모드
    }

    [Header("Operation Mode")]//모드 선택
    public OperationMode currentMode = OperationMode.SLAM;

    [Header("Robot Parameters")] // 로봇 자체 스펙 설정
    public float wheelBase = 0.5f;
    public float maxSteeringAngle = 30f;
    public float stepDistance = 0.5f;
    public float targetReachedThreshold = 0.5f;
    public float stepDuration = 1.0f; // 한 스텝당 소요 시간 1초 고정
    
    [Header("Noise Settings")]
    public bool useNoise = false;
    public float noiseSigma = 0.1f;

    [Header("Localization & Path Following Parameters")]
    public float explorationSpeed = 45f;  // 탐색 시 회전 속도

    [Header("PID Parameters")]//PID Gain값
    public float Kp = 0.5f;
    public float Ki = 0.1f;
    public float Kd = 0.2f;

    // 현재 상태 변수들 SLAM용
    private float currentSteeringAngle = 0f;
    private int currentStep = 0;
    private float stepProgress = 0f;           // 현재 스텝의 진행도
    private Vector3 stepStartPosition;         // 현재 스텝 시작 위치
    private bool isStepInProgress = false;     // 스텝 진행 중 여부

    //이벤트
    public delegate void StepCompletedHandler(Vector3 position, float orientation);
    public event StepCompletedHandler OnStepCompleted;

    public MapManager mapManager;
    
    void Start()
    {
        InitializeRobot();
    }

public void InitializeRobot()
   {
 
        // 초기 위치와 방향 설정
        switch (currentMode)
        {
            case OperationMode.SLAM:
                transform.position = new Vector3(22.5f, 0.5f, 11.5f);
                transform.rotation = Quaternion.Euler(0f, 270f, 0f);  // 1.5π = 270도
                stepStartPosition = transform.position;
                currentSteeringAngle = 0f; // 초기 핸들 각도
                break;
            case OperationMode.Landmark://문제 2,3번
                transform.position = new Vector3(1.5f, 0.5f, 13.5f); // 시작점 A
                break;
            case OperationMode.RangeFinder://문제 4,5번
                transform.position = new Vector3(1.5f, 0.5f, 13.5f); // 시작점 A
                transform.rotation = Quaternion.Euler(0f, 0f, 0f);
                break;
        }
    }
[Header("Timing Settings")]
    private float messageInterval = 1.0f;  // 1초마다 메시지 전송
    private float lastMessageTime = 0f;
    void Update()
    {
        if (currentMode==OperationMode.SLAM)
        {
            UpdateSLAMMode();
        }
        else{
            if (currentMode==OperationMode.Landmark){
            float currentTime = Time.time;
            // 메시지 전송 타이밍 체크
            if (currentTime - lastMessageTime >= messageInterval)
            {
                // 현재 위치와 측정값을 포함한 메시지 전송
                OnStepCompleted?.Invoke(transform.position, transform.eulerAngles.y);
                lastMessageTime = currentTime;
            }
        }
        }
        
    }
//==SLAM 메서드모음====================================================SLAM 메서드모음

    void UpdateSLAMMode()//SLAM용 action모델
{   

    if (currentStep >= 50)
    {
        Debug.Log("All steps completed");  // 선택적인 디버그 로그
        return;
    }

    if (!isStepInProgress)
    {
        StartNewStep();
    }

    stepProgress += Time.deltaTime;
    float stepCompletion = stepProgress / stepDuration;

    if (stepProgress >= stepDuration)
    {
        //Debug.Log($"Step {currentStep} duration completed: {stepProgress}"); // 디버깅 로그 추가
        CompleteCurrentStep();
    }
    else
    {
        float distancePerSecond = stepDistance / stepDuration;
        float currentDistance = distancePerSecond * Time.deltaTime;
        float steeringAngle = 2f * Mathf.PI / 50f;  // 한 스텝당 회전각
        Move(steeringAngle, currentDistance);
    }
}
    void StartNewStep()
{
    stepStartPosition = transform.position;
    stepProgress = 0f;
    isStepInProgress = true;
    Debug.Log($"Starting step {currentStep },Position {transform.position}");
}
    void CompleteCurrentStep()
{
    //Debug.Log("Step " + currentStep + " completed");  // 디버깅 로그 추가
    currentStep++;
    stepProgress = 0f;
    isStepInProgress = false;
    // 스텝 완료 이벤트 트리거
    OnStepCompleted?.Invoke(transform.position, transform.eulerAngles.y);
}
//===SLAM 메서드모음========================================SLAM 메서드모음

void PerformRangeFinderBasedExploration()//문제 4,5번
    {
        // 거리 센서를 활용한 효율적인 탐색
        // 더 넓은 영역을 커버하는 탐색 패턴
        float explorationAngle = Time.time * explorationSpeed * 1.5f;
        float steering = maxSteeringAngle * Mathf.Sin(explorationAngle * Mathf.Deg2Rad);
        Move(steering, stepDistance * 1.2f);
    }

void Move(float steeringAngle, float distance)//조향각(방향)과 거리 입력받음 기본적인 로봇움직임
{
    // 로봇의 움직임에 노이즈를 추가합니다
    if (useNoise)
    {
        // 이동 거리에 노이즈를 추가합니다
        // 실제 로봇은 명령받은 거리만큼 정확히 움직이지 않을 수 있습니다
        distance += GenerateGaussianNoise(noiseSigma);

        // 조향각에도 노이즈를 추가합니다
        // 실제 로봇의 바퀴는 정확한 각도로 회전하지 않을 수 있습니다
        // 조향각 노이즈는 이동 거리 노이즈보다 작게 설정합니다 (0.1을 곱함)
        steeringAngle += GenerateGaussianNoise(noiseSigma * 0.1f);
    }

    //조향각 제한
    currentSteeringAngle = Mathf.Clamp(steeringAngle, -maxSteeringAngle * Mathf.Deg2Rad, maxSteeringAngle * Mathf.Deg2Rad);

    // 프레임 단위 이동 거리 계산
    float deltaX = distance * Mathf.Cos(transform.eulerAngles.y * Mathf.Deg2Rad);
    float deltaZ = distance * Mathf.Sin(transform.eulerAngles.y * Mathf.Deg2Rad);

    transform.position += new Vector3(deltaX, 0, deltaZ);

    // 조향각에 따른 회전
    if (Mathf.Abs(currentSteeringAngle) > 0.01f)
    {
        float turnRadius = wheelBase / Mathf.Tan(currentSteeringAngle);
        float angularDisplacement = distance / turnRadius;
        transform.Rotate(Vector3.up, angularDisplacement * Mathf.Rad2Deg);
    }
     // 바퀴 회전 애니메이션
        if (Mathf.Abs(distance) > 0.01f)
        {
            frontWheel.localRotation = Quaternion.Euler(90f, steeringAngle * Mathf.Rad2Deg, 0f);
        }

        OnStepCompleted?.Invoke(transform.position, transform.eulerAngles.y);
}

//조향각 & 입력받은 거리 노이즈 시그마 할당 (MOVE용)
private float GenerateGaussianNoise(float sigma)//action가우시안 노이즈 시그마 연산메서드
{
    float u1 = 1.0f - Random.value; // Uniform(0,1] 난수
    float u2 = 1.0f - Random.value;
    float randStdNormal = Mathf.Sqrt(-2.0f * Mathf.Log(u1)) * Mathf.Sin(2.0f * Mathf.PI * u2); // 표준 정규 분포
    return sigma * randStdNormal; // sigma를 곱해 원하는 분산의 노이즈 생성
}

    private void PerformLandmarkBasedExploration()//문제 2번용 임의로 움직여서 랜드마크 센싱
{
    if (!isStepInProgress)
    {
        StartNewStep();
        if(currentStep<5)
        {Debug.Log("Moving forward");
        // 1m 이동
        Move(0, 1.0f);}
        else if (currentStep <7)
        {
            Debug.Log("Moving left.");
            transform.position += new Vector3(1.0f,0,0); // x축 방향으로 이동
        }

    }
    
    stepProgress += Time.deltaTime;
    if (stepProgress >= 1.0f)  // 1초마다 한 스텝
    {
        CompleteCurrentStep();//이번트 트리거
    }
}


//RobotataTransmitter에서 부르는거
private bool isMoving=false;
public bool IsMoving()
{
    return isMoving;
}
public void ExcuteMovement(float dx, float dy)
{    

    if (isMoving)
        {
            Debug.LogWarning("ExcuteMovement called while already moving.");
            return;
        }
    isMoving=true;
    Vector3 targetPosition = transform.position + new Vector3(dx, 0, dy);

    // Execute movement
    // 실제 이동 실행
        transform.position = Vector3.MoveTowards(
            transform.position,
            targetPosition,
            5  // stepDistance는 이동 속도 조절용
        );
        // 바퀴 회전 애니메이션 적용 (있는 경우)
        OnStepCompleted?.Invoke(transform.position, transform.eulerAngles.y);
    isMoving = false;
}


}