import socket
import queue


# Server config.
HOST = "localhost"
PORT = 7004
MAX_CONNECTION_COUNT = 1
LISTEN_TIMEOUT_SEC = 3
QUEUE_TIMEOUT_SEC = 3

def _startServerSocket():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        # Bind to IP.
        s.bind((HOST, PORT))
    except OSError:
        print("[TCP] Host address %s is not correct!"%HOST)
        s.close()
        return None
    
    # Start listening for clients.
    s.listen(MAX_CONNECTION_COUNT)
    s.settimeout(LISTEN_TIMEOUT_SEC) # timeout for listening
    print('[TCP] server start at: %s:%s' % (HOST, PORT))
    return s

# Attempt to connect to a client.
# Returns (None, None) if no connection is made.
def _connectToClient(s):
    conn, addr = None, None
    print('[TCP] wait for connection...')
    try:
        # Yield regularly to check stop signal.
        conn, addr = s.accept()
        print('[TCP] connected by ' + str(addr))
    except socket.timeout:
        print("[TCP] no connections in %d seconds"%LISTEN_TIMEOUT_SEC)
    finally:
        return (conn, addr)

# Attempt to pop an item from sendQueue
# and send to client.
# Returns False on connection close.
def _sendData(conn, addr, sendQueue):
    connectionAlive = True
    try:
        # Yield regularly to check stop signal.
        print('[TCP send] wait for item...')
        item = sendQueue.get(timeout=QUEUE_TIMEOUT_SEC)
        # print('[TCP send] item is "%s"'%item)
        message = item + "@"
        print('[TCP send] message is "%s"'%message)
        conn.send(message.encode())
    except queue.Empty:
        print('[TCP send] no items in %d seconds'%QUEUE_TIMEOUT_SEC)
    except ConnectionAbortedError:
        print('[TCP] connection from %s closed'%str(addr))
        connectionAlive = False
    finally:
        return connectionAlive

# Send anything to stopQueue to stop this thread.
def serverThread(stopQueue, sendQueue):
    s = _startServerSocket()
    if s is None:
        print('[TCP] server start failed')
        return

    conn, addr = None, None
    # Run while nothing has been sent to stopQueue.
    while stopQueue.empty():
        conn, addr = _connectToClient(s)
        if conn is None or addr is None:
            continue

        # Loop to send data to client.
        while stopQueue.empty():
            connectionAlive = _sendData(conn, addr, sendQueue)
            if not connectionAlive:
                conn.close()
                conn = None
                addr = None
                break
    print('[TCP] server close...')
    s.close()
