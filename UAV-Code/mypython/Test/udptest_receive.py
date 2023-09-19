import socket
import time
import pickle
send_port = 2887
recv_port = 3333
server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1) 
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1) 
recv_address = ("0.0.0.0", send_port)
send_address = ("255.255.255.255",recv_port)
server_socket.bind(recv_address)
server_socket.settimeout(1.0)
mslist=[170,558,-464,255]
received_data = []
state=0
print("接收启动")
while True:
    #print("接收到的数据是",data)
    bydata=pickle.dumps(mslist)
    client_socket.sendto(bydata, send_address) 
    try:
        data, client_address = server_socket.recvfrom(1024)
        realdata=pickle.loads(data)
        if realdata[0]==170 and realdata[3]==255:
            received_data=[realdata[1],realdata[2]]
            print("realdata x y",received_data)
    except:
        pass
    time.sleep(0.02)
