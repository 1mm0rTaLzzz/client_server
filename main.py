# test-client.py
import socket
import sys
import time

# СоздаемTCP/IP сокет
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Подключаем сокет к порту, через который прослушивается сервер
server_address = ('192.168.88.94', 8882)
print('Подключено к {} порт {}'.format(*server_address))
sock.connect(server_address)

try:
    # Отправка данных
    mess = input()
    print(f'Отправка: {mess}')
    message = mess.encode()
    sock.sendall(message)

    # Смотрим ответ
    amount_received = 0
    amount_expected = len(message)
    while amount_received < amount_expected:
        data = sock.recv(16)
        amount_received += len(data)
        mess = data.decode()
        print(f'Получено: {data.decode()}')

finally:
    time.sleep(50)
    print('Закрываем сокет')
    sock.close()