#!/c/Users/User/AppData/Local/Programs/Python/Python39/python
# -*- coding: utf-8 -*-

# import sys
import asyncio
import ws_server as wsServer
import threading
import queue
import tcp_client as tcpClient
import tcp_server as tcpServer

# tcpRecvQueue = queue.Queue()
tcpSendQueue = queue.Queue(1) # size 1
tcpClientStopQueue = queue.Queue()
tcpServerStopQueue = queue.Queue()

# Setting an async function as a thread target.
# Reference:
# https://stackoverflow.com/a/59645689
def awaitedTcpClientThread(stopQueue):
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(tcpClient.clientThread(stopQueue))
    loop.close()

tcpClientThread = None
tcpServerThread = None
inputThread = None
async def main():
    print("[WS] Start server...")
    myWsServer = await wsServer.createServer(tcpSendQueue)

    print("[TCP] Start client thread...")
    global tcpClientThread
    tcpClientThread = threading.Thread(
        target = awaitedTcpClientThread, args=(tcpClientStopQueue,))
    tcpClientThread.start()

    print("[TCP] Start server thread...")
    global tcpServerThread
    tcpServerThread = threading.Thread(
        target = tcpServer.serverThread, args=(tcpServerStopQueue, tcpSendQueue))
    tcpServerThread.start()
    
    await myWsServer.wait_closed()
    
try:
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())
    loop.close()
except KeyboardInterrupt:
    tcpClientStopQueue.put("")
    tcpServerStopQueue.put("")

    print("Waiting for tcpClientThread to join...")
    tcpClientThread.join()
    print("tcpClientThread joined")
    print("Waiting for tcpServerThread to join...")
    tcpServerThread.join()
    print("tcpServerThread joined")
