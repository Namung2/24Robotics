import socket
import json
import threading
from typing import Dict, List, Optional, Tuple
import numpy as np
from BaseLocalization import BaseLocalization

class UnityInterface3:
    def __init__(self, host='localhost', port=5002):
        self.localizer=BaseLocalization()
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.bind((host, port))
        self.socket.listen(1)
        print("Waiting for Unity connection...")

        self.connection = None
        self.is_connected = False
        self.current_data = None
        self.lock = threading.Lock()
        self.target_position = (18.5, 1.5)  # P2 coordinates

        try:
            self.socket.settimeout(20)
            self.connection, _ = self.socket.accept()
            self.connection.settimeout(None)
            self.is_connected = True
            print("Connected to Unity!")
        except socket.timeout:
            print("Connection timeout. Please check if Unity is running.")
            raise

        self.receive_thread = threading.Thread(target=self._receive_data)
        self.receive_thread.daemon = True
        self.receive_thread.start()

    def _receive_data(self):
        buffer = ""
        while self.is_connected:
            try:
                data = self.connection.recv(4096).decode('utf-8')
                if not data:
                    break

                buffer += data
                try:
                    parsed_data = json.loads(buffer)
                    with self.lock:
                        self.current_data = self._process_unity_data(parsed_data)
                    buffer = ""
                except json.JSONDecodeError:
                    continue

            except Exception as e:
                print(f"Error receiving data: {e}")
                self.is_connected = False
                break

        print("Connection closed")
        self.connection.close()

    def _process_unity_data(self, raw_data):
        """Unity에서 받은 데이터를 파티클 필터에 적합한 형태로 변환"""
        try:
            #print("Raw data received:", raw_data)  # 수신된 원본 데이터 출력
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

    def get_latest_data(self):
        """가장 최근에 받은 처리된 데이터를 반환"""
        with self.lock:
            return self.current_data

    def send_movement_command(self, dx: float, dy: float):
        
        """Send grid movement command to Unity"""
        if not self.is_connected:
            return

        try:
            # Convert floating point values to appropriate format
            command = {
                "command_type": "movement",
                "dx": float(dx),  # Ensure floating point
                "dy": float(dy),   # Ensure floating point
            }
            #print(f"Sending movement command: dx={dx}, dy={dy}")
            self._send_data(command)
        except Exception as e:
            print(f"Error sending movement command: {e}")

    def _send_data(self, data: Dict):
        try:
            json_data = json.dumps(data)
            #print(f"Sending data: {json_data}")
            self.connection.sendall(json_data.encode('utf-8'))
        except Exception as e:
            print(f"Error in send_data: {e}")
            self.is_connected = False

    def get_target_position(self) -> Tuple[float, float]:
        return self.target_position

    def get_landmark_measurements(self) -> List[Dict]:
        """Get landmark measurements for localization"""
        data = self.get_latest_data()
        if data and "landmarks" in data:
            return data["landmarks"]
        return []
    
    def close(self):
        self.is_connected = False
        if self.connection:
            self.connection.close()
        self.socket.close()

    def reconnect(self):
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.bind(('localhost', 5002))
            self.socket.listen(1)
            self.connection, _ = self.socket.accept()
            self.is_connected = True
            print("Reconnected to Unity!")

            self.receive_thread = threading.Thread(target=self._receive_data)
            self.receive_thread.daemon = True
            self.receive_thread.start()

        except Exception as e:
            print(f"Reconnection failed: {e}")
