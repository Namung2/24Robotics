import socket

HOST = '127.0.0.1'  # Unity와 같은 IP
PORT = 5000         # Unity와 동일한 포트

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((HOST, PORT))
server_socket.listen(1)
print("Waiting for Unity connection...")

server_socket.settimeout(10)
connection, address = server_socket.accept()
print(f"Connected to Unity: {address}")

while True:
    data = connection.recv(1024).decode()
    if not data:
        break
    print(f"Received: {data}")
    connection.sendall(b'ACK')  # Unity로 응답 보내기
