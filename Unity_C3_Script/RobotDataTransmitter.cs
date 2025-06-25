using UnityEngine;
using System.Collections.Generic;
using System.Net.Sockets;
using System.Text;
using Newtonsoft.Json;
using System.Linq;
using System.Linq.Expressions;
using System;
using UnityEngine.Rendering.Universal;
using Newtonsoft.Json.Linq;


public class RobotDataTransmitter : MonoBehaviour
{
    public enum OperationMode
    {
        SLAM,           // 문제 1용 SLAM 모드
        Landmark,       // 문제 2,3용 랜드마크 기반 모드
        RangeFinder    // 문제 4,5용 거리센서 기반 모드
    }
    [Header("Operation Mode")]//모드 선택
    public OperationMode currentMode = OperationMode.SLAM;

    [Header("Connection Settings")]
    public int remotePort = 5000;
    public float transmissionInterval = 1f; // 1000ms 주기로 데이터 전송 

    [Header("Transmission Settings")]
    public bool isTransmittingData = false;

    private RobotController robotController;
    private SensorSystem sensorSystem;
    private TcpClient tcpClient;

    private bool isConnected = false; //연결상태 구분

    void Start()
    {
        robotController = GetComponent<RobotController>();
        if (robotController == null)
    {
        Debug.LogError("RobotController component is missing! Please add it to the GameObject.");
    }
    else
    {
        Debug.Log("RobotController initialized successfully.");
    }
        sensorSystem = GetComponent<SensorSystem>();
        ConnectToServer();
        robotController.OnStepCompleted += HandleStepCompleted;

    
    }

void ConnectToServer()
{
    try{
        tcpClient = new TcpClient("127.0.0.1",remotePort);
        tcpClient.ReceiveTimeout = 1000;  // 1초 타임아웃
        Debug.Log($"Attempting to connect to {"127.0.0.1"}:{remotePort}");

        if(currentMode!=OperationMode.SLAM){
            isConnected = true;
            StartReceiving();//SLAM제외 데이터 수신 필요함
        }
        else{
            Debug.Log($"This Mode is SLAM!");
            // 데이터 전송 루틴 시작
            isTransmittingData = true;
             isConnected = true;
            // RobotController의 스텝 완료 이벤트 구독

        }
    }catch (System.Exception e)
    {
        // 연결 오류 발생
        Debug.LogError($"원격 호스트 연결 오류: {e.Message}");
        isConnected =false;
        return;
    }
}
async void StartReceiving()//파이썬에서온 데이터 수신
    {
        while (isConnected)
        {
            try
            {
                byte[] buffer = new byte[4096];
                NetworkStream stream = tcpClient.GetStream();
                int bytesRead = await stream.ReadAsync(buffer, 0, buffer.Length);
                
                if (bytesRead > 0)
                {
                    string jsonData = Encoding.UTF8.GetString(buffer, 0, bytesRead);
                    ProcessReceivedData(jsonData);
                }
            }
            catch (System.Exception e)
            {
                Debug.LogError($"Error receiving data: {e.Message}");
                break;
            }
        }
    
    }
public class ReceivedData
{
    public float? confidence { get; set; } // Nullable float
    public List<Vector3> planned_path { get; set; }
    public Vector3 target_position { get; set; }
}
private void ProcessReceivedData(string jsonData)
{
    try
        {
            JObject data = JObject.Parse(jsonData);
            
            // Debug log
            Debug.Log($"Received data: {jsonData}");
            
            string commandType = data["command_type"].ToString();

            switch(commandType)
            {
                case "movement":
                    ProcessP3Movement(data);
                    break;
                default:
                    Debug.LogWarning($"Unknown command type: {commandType}");
                    break;
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"Error processing received data: {e.Message}\nData: {jsonData}");
        }
    
}

private void ProcessP3Movement(JObject data)
{
    try
        {
             if (data["dx"] == null || data["dy"] == null)
        {
            Debug.LogError("dx or dy is missing in the movement command.");
            return;
        }
            float dx = data["dx"].Value<float>();
            float dy = data["dy"].Value<float>();
            
            // Debug log
            Debug.Log($"Received movement command: dx={dx}, dy={dy}");
            if(!robotController.IsMoving())
            {robotController.ExcuteMovement(dx, dy);}
        }
        catch (Exception e)
        {
            Debug.LogError($"Error in ProcessP3Movement: {e.Message}\nData: {data}");
        }
}

    void HandleStepCompleted(Vector3 position, float heading)
{
    string jsonData="";
    // action모델에 따른 경우로 나누기
    switch(robotController.currentMode){
        case RobotController.OperationMode.SLAM:
            jsonData=PackSLAMData(position,heading);
            break;
        case RobotController.OperationMode.Landmark:
            jsonData=PackLandmarkData(position,heading);
            break;
        case RobotController.OperationMode.RangeFinder:
            jsonData=PackRangeFinderData(position,heading);
            break;
    }

    SendData(jsonData);
    
}
    string PackSLAMData(Vector3 position, float heading)
{
    //SLAM용 센싱데이터 추출
    var slamMeasurements = sensorSystem.GetSLAMMeasurements();

    // 데이터를 JSON 형식으로 패킹
    var data = new
    {
        timestamp = Time.time,
        robot_pose = new
        {
            x = position.x,
            y = position.z,  // Unity의 z축은 Python의 y축으로 매핑
            orientation = heading
        },
        landmarks = slamMeasurements.Values.Select(m => new
        {
            id = m.id,
            relative_x = m.relativePosition.x,  // 키 이름을 Python 코드와 일치시킴
            relative_y = m.relativePosition.y,  // 키 이름을 Python 코드와 일치시킴
            distance = m.distance
        }).ToArray()
    };
    return JsonConvert.SerializeObject(data);
}
string PackLandmarkData(Vector3 position, float heading)//2번,3번의 Localization 단계에 필요
    {
        // 랜드마크 모드용 데이터 패킹
        var measurements = sensorSystem.GetLocalizationMeasurements();
        
         // 랜드마크 모드에서는 직선 거리만 필요 landmark센싱 데이터 정리
        var landmarks = measurements.Values.Select(m => new
        {
            id = m.id,
            distance = m.distance  // 직선 거리만 포함
        }).ToArray();
        if (landmarks.Length == 0)
    {
        Debug.LogWarning("No landmarks detected. Sending empty measurements.");
    }
        //데이터 패킹
        var data = new
        {
            timestamp = Time.time,
            robot_pose = new//python에서 액션데이터 처리를 위해 현재 위치보냄
            {
                x = position.x,
                y = position.z,  // Unity의 Z축은 Python의 Y축으로 매핑
                orientation = heading
            },
            measurements = landmarks
        };
         // JSON 생성 및 검증
    string jsonData = JsonConvert.SerializeObject(data);
    //Debug.Log($"Packed JSON Data: {jsonData}");

        return JsonConvert.SerializeObject(data);
    }
string PackRangeFinderData(Vector3 position, float heading)
    {
        // 거리 센서 모드용 데이터 패킹
        var measurements = sensorSystem.GetRangeFinderMeasurements();
        
         // 거리센서 측정값 변환
        var rangeMeasurements = measurements.Select(m => new
        {
            angle = m.angle,           // 측정 각도
            distance = m.distance,      // 측정된 거리
            is_obstacle = !m.isLandmark // 장애물 여부
        }).ToArray();

        var data = new
        {
            timestamp = Time.time,
            robot_pose = new
            {
                x = position.x,
                y = position.z,
                orientation = heading
            },
            range_measurements = rangeMeasurements
        };

        return JsonConvert.SerializeObject(data);
    }

    void SendData(string jsonData)
    {
        if (!isConnected || tcpClient == null || !tcpClient.Connected) {
        Debug.Log("Reconnecting to server...");
        ConnectToServer();
    }

        try
        {
            if (isConnected && tcpClient != null && tcpClient.Connected) {
            // TCP 클라이언트를 통해 데이터 전송
            byte[] dataBytes = Encoding.UTF8.GetBytes(jsonData);
            NetworkStream stream = tcpClient.GetStream();
            stream.Write(dataBytes, 0, dataBytes.Length);
            }
        }
        catch (System.Exception e)
        {
            Debug.LogError($"Error transmitting data: {e.Message}");
            // 오류 처리 로직 구현 (재전송, 연결 재설정 등)
            isConnected=false;
        }
    }

    void OnApplicationQuit()
    {
        // 애플리케이션 종료 시 TCP 연결 해제
        isTransmittingData = false;
        if (tcpClient != null)
        {
            tcpClient.Close();
        }
    }
}