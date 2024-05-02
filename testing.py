import time

start = time.time()
while (time.time() - start )< 10:
    time.sleep(0.5)
    print(time.time() - start) 