from multiprocessing import Queue, Process
import sys, tty, termios

def getch():
    sys.stdin = open('/dev/tty')
    ch = raw_input("Introduce next command:\n")
    return ch

def f(q):
    while True:
        r = getch()
        q.put(r)



if __name__ == '__main__':
    q = Queue()
    p = Process(target=f, args=(q,))
    p.start()
    a = 0
    while True:
        if not q.empty():
            q.get()
            print("Front value: {}".format(a))
        else:
            a += 1
