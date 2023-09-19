import queue
from random import choice
from threading import Thread

q = queue.Queue(maxsize=5)
dealList = ["红烧猪蹄", "卤鸡爪", "酸菜鱼", "糖醋里脊", "九转大肠", "阳春面", "烤鸭", "烧鸡", "剁椒鱼头", "酸汤肥牛", "炖羊肉"]


def cooking(chefname: str):
    for i in range(4):
        deal = choice(dealList)
        q.put(deal, block=True)
        print("厨师{}给大家带来一道：{}  ".format(chefname, deal))


def eating(custname: str):
    for i in range(3):
        deal = q.get(block=True)
        print("顾客{}吃掉了：{}  ".format(custname, deal))
        q.task_done()


if __name__ == "__main__":
    # 创建并启动厨师ABC线程，创建并启动顾客1234线程
    threadlist_chef = [Thread(target=cooking, args=chefname).start() for chefname in ["A", "B", "C"]]
    threadlist_cust = [Thread(target=eating, args=str(custname)).start() for custname in range(4)]
    # 队列阻塞，直到所有线程对每个元素都调用了task_done
    q.join()
