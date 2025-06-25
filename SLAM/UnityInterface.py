import socket
import json
import threading
import numpy as np

class UnityInterface:
    def __init__(self, host='localhost', port=5000):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.bind((host, port))
        self.socket.listen(1)
        print("Waiting for Unity connection...")

        try:
            self.socket.settimeout(20)
            self.connection, _ = self.socket.accept()
            print("Connected to Unity!")
            self.connection.settimeout(None)
        except TimeoutError:
            print("Connection timeout. Please check if Unity is running.")
            raise

        self.lock = threading.Lock()
        self.current_data = None
        
        self.receive_thread = threading.Thread(target=self.receive_data)
        self.receive_thread.daemon = True
        self.receive_thread.start()

    def receive_data(self):
        buffer = ""
        while True:
            try:
                data = self.connection.recv(4096).decode()
                if not data:
                    break

                buffer += data
                
                try:
                    parsed_data = json.loads(buffer)
                    with self.lock:
                        self.current_data = parsed_data
                    buffer = ""
                except json.JSONDecodeError:
                    continue
                    
            except Exception as e:
                print(f"Error receiving data: {e}")
                break
                
        print("Connection closed")
        self.connection.close()

    def process_unity_data(self, raw_data):
        if not raw_data:
            return None

        try:
            # Unity의 Y축 회전을 라디안으로 변환
            orientation = np.radians(raw_data["robot_pose"]["orientation"])
            
            processed_data = {
                "timestamp": raw_data["timestamp"],
                "robot_pose": {
                    "x": raw_data["robot_pose"]["x"],
                    "y": raw_data["robot_pose"]["y"],  # Unity에서 이미 z를 y로 변환해서 보냄
                    "orientation": orientation
                },
                "landmarks": []
            }

            # 랜드마크 데이터 처리
            if "landmarks" in raw_data:
                for landmark in raw_data["landmarks"]:
                    processed_landmark = {
                        "id": landmark["id"],
                        "relative_x": landmark["relative_x"],
                        "relative_y": landmark["relative_y"]  # Unity에서 이미 변환된 값
                    }
                    processed_data["landmarks"].append(processed_landmark)

            return processed_data

        except KeyError as e:
            print(f"Missing key in Unity data: {e}")
            print(f"Raw data structure: {raw_data}")
            return None
        except Exception as e:
            print(f"Error processing Unity data: {e}")
            return None

    def get_latest_data(self):
        with self.lock:
            raw_data = self.current_data
            return self.process_unity_data(raw_data) if raw_data else None