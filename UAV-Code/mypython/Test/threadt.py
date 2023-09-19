import threading
import time
import serial
def show(arg):
    time.sleep(1)
    print('thread '+str(arg)+" running....")

if __name__ == '__main__':
    for i in range(10):
        t = threading.Thread(target=show, args=(i,))
        t.start()