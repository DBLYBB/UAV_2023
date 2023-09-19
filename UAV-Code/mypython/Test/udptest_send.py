import socket
import time
import os
import pickle
#os.system("netsh wlan connect name= LESP32WF")
#client 发送端
client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1) 
PORT = 2887
mslist=[170,228,-264,False,255]
server_address = ("255.255.255.255", PORT)  # 接收方 服务器的ip地址和端口号
while True:
      bydata=pickle.dumps(mslist)
      client_socket.sendto(bydata, server_address)
      print("bydata=%b",bydata)
      time.sleep(0.05)