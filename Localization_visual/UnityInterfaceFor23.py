import socket
import json
import numpy as np
import threading
from time import sleep
import time

class UnityInterfaceFor23:
    def __init__(self, host='localhost', port=5000):
        # 소켓 통신 초기화
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.bind((host, port))
        self.socket.listen(1)
        print("Waiting for Unity connection...")

        # 연결 상태 및 데이터 관리를 위한 변수들
        self.connection = None
        self.is_connected = False
        self.current_data = None
        self.lock = threading.Lock()

        # 연결 시도 (20초 타임아웃)
        try:
            self.socket.settimeout(20)
            self.connection, _ = self.socket.accept()
            self.connection.settimeout(None)
            self.is_connected = True
            print("Connected to Unity!")
        except socket.timeout:
            print("Connection timeout. Please check if Unity is running.")
            raise

        # 데이터 수신 스레드 시작
        self.receive_thread = threading.Thread(target=self._receive_data)
        self.receive_thread.daemon = True
        self.receive_thread.start()

    def _receive_data(self):
        """Unity로부터 지속적으로 데이터를 수신하는 메서드"""
        buffer = ""
        last_update_time = 0
        min_update_interval = 1.0  # 1초 간격으로 제한
        while self.is_connected:
            try:
                data = self.connection.recv(4096).decode('utf-8')
                if not data:
                    break

                buffer += data
                current_time = time.time()
                
                # 완전한 JSON 데이터를 받았는지 확인
                try:
                    parsed_data = json.loads(buffer)
                    if current_time - last_update_time >=min_update_interval:
                        with self.lock:
                            self.current_data = self._process_unity_data(parsed_data)
                        last_update_time = current_time
                    buffer = ""
                except json.JSONDecodeError:
                    continue  # 데이터가 완전하지 않음, 더 받기

            except Exception as e:
                print(f"Error receiving data: {e}")
                self.is_connected = False
                break

        print("Connection closed")
        self.connection.close()

    def _process_unity_data(self, raw_data):
        """Unity에서 받은 데이터를 파티클 필터에 적합한 형태로 변환"""
        try:
            print("Raw data received:", raw_data)  # 수신된 원본 데이터 출력
            processed_data = {
                "timestamp": raw_data["timestamp"],
                "robot_pose": raw_data["robot_pose"]
            }

            # measurements 키 확인
            if "measurements" in raw_data and isinstance(raw_data["measurements"], list):
                processed_data["measurements"] = [
                    {
                        "id": measurement["id"],
                        "distance": measurement["distance"]
                    }
                    for measurement in raw_data["measurements"]
                ]
            else:
                print("Invalid measurements data or not a list:", raw_data.get("measurements", None))
                processed_data["measurements"] = []
            return processed_data

        except KeyError as e:
            print(f"Missing key in Unity data: {e}")
            return None
        except Exception as e:
            print(f"Error processing Unity data: {e}")
            return None

    def get_latest_data(self):
        """가장 최근에 받은 처리된 데이터를 반환"""
        with self.lock:
            return self.current_data

    def send_response(self, response_data):
        """Unity로 제어 명령이나 상태 정보를 전송"""
        if not self.is_connected:
            return

        try:
            # 데이터 직렬화
            json_data = json.dumps(response_data)
            
            # 데이터 전송
            self.connection.sendall(json_data.encode('utf-8'))
        except Exception as e:
            print(f"Error sending response: {e}")
            self.is_connected = False

    def close(self):
        """연결 종료 및 리소스 정리"""
        self.is_connected = False
        if self.connection:
            self.connection.close()
        self.socket.close()

    def reconnect(self):
        """연결이 끊어졌을 때 재연결 시도"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.bind(('localhost', 5000))
            self.socket.listen(1)
            self.connection, _ = self.socket.accept()
            self.is_connected = True
            print("Reconnected to Unity!")
            
            # 수신 스레드 재시작
            self.receive_thread = threading.Thread(target=self._receive_data)
            self.receive_thread.daemon = True
            self.receive_thread.start()
            
        except Exception as e:
            print(f"Reconnection failed: {e}")