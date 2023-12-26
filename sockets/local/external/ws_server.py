import websockets.server as server
import time
import queue
import functools


# There will be only one single client:
# the browser tab running ganja.js.
_client = None
async def _clientHandler(newClient, tcpSendQueue):
    global _client
    if _client is not None:
        print("[WS] Already connected to a client!")
    print("[WS] Connected to client")
    _client = newClient
    # Wait for messages from browser.
    async for message in newClient:
        print("[WS] recv: %s"%message)
        try:
            tcpSendQueue.put_nowait(message)
            print("[WS] put to TCP send queue")
            time.sleep(0.1)
        except queue.Full:
            print("[WS] TCP send queue full!")
    print("[WS] Client closed")
    _client = None

# Returns `False` if there is no client present and therefore cannot send.
async def sendToClient(message):
    if _client is None:
        return False
    await _client.send(message)
    return True

# For how to bind extra parameters to a function, see:
# https://stackoverflow.com/a/69765225
HOST = "localhost"
PORT = 9999
async def createServer(tcpSendQueue):
    queuedClientHandler = functools.partial(_clientHandler, tcpSendQueue=tcpSendQueue)
    return await server.serve(queuedClientHandler, HOST, PORT)
