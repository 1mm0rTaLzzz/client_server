import socket
import threading
import numpy as np
import math
from robot import Robot
import time


# Get data from user
def handle_client(client_socket):
    robot = Robot("COM5")
    while True:
        # try:
        #     data = client_socket.recv(1024)
    # except:
        #     pass
        #
        # if not data:
        #     break
        # print(data.decode())
        # response = "Response from server"
        #received = data.decode().split(' ')
        received = ['0.1', '0.1']

        # print(received)
        # robot.tvec[0][0][0]
        received = np.array(list(map(float, received)))
        target = received - robot.pos
        robot.setSpeed(60)#min(70, int(np.linalg.norm(target)*100)))
        robot.holdAng(math.pi - math.atan2(*target))
        time.sleep(0.2)
        print(robot.pos, robot.targetAng, robot.currAng)
        time.sleep(0.1)
        while 500 > np.linalg.norm(received - robot.pos) > 10:
            print(robot.pos, robot.targetAng, robot.currAng)
            time.sleep(0.1)
        #print("reached")
        #client_socket.send(response.encode())

    #client_socket.close()
handle_client(1)

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(('192.168.88.94', 8882))
server_socket.listen(10)
server_socket.setblocking(0)
server_socket.settimeout(0.1)


pending_connections = []


# Accept connection and adding into the array
def accept_connections():
    while True:
        try:
            client_socket, address = server_socket.accept()
            pending_connections.append(client_socket)
            print("Pending connections:")
            for i, conn in enumerate(pending_connections):
                print(f"{i + 1}. {conn.getpeername()}")
        except:
            pass


# Managing connections
def process_connections():
    while True:
        choice = input("Enter the index of the connection to process (or 'q' to quit): ")
        if choice.lower() == 'q':
            break

        try:
            index = int(choice) - 1
            if index < 0 or index >= len(pending_connections):
                print("Invalid choice. Try again.")
            else:
                client_socket = pending_connections.pop(index)
                client_thread = threading.Thread(target=handle_client, args=(client_socket,))
                client_thread.start()
        except ValueError:
            print("Invalid choice. Try again.")
    server_socket.close()


accept_thread = threading.Thread(target=accept_connections)
accept_thread.start()

process_thread = threading.Thread(target=process_connections)
process_thread.start()

accept_thread.join()
process_thread.join()