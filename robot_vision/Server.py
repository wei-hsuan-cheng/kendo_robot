import socket
import threading

class Server:
    def __init__(self, host, port, session_func):
        self.active = False

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.bind((host, port))
        self.socket.settimeout(20 / 1000)
        self.socket.listen(1024)

        self.serve_thread = threading.Thread(target=self.Serve)

        self.conns = dict() # {conn_id: (conn, addr, thread), ...}

        self.lock = threading.Lock()

        self.session_func = session_func

    def __del__(self):
        self.Stop()

    def is_active(self):
        return self.active

    def Start(self):
        if self.active:
            return

        print(f"starting server")

        self.active = True

        self.serve_thread.start()

        print(f"started")

    def Stop(self):
        if not self.active:
            return

        print(f"stopping server")

        self.active = False

        self.serve_thread.join()

        print(f"stopped")

    def Serve(self):
        while self.active:
            try:
                conn, addr = self.socket.accept()
            except socket.timeout:
                if self.active:
                    continue
                break

            if not self.active:
                conn.close()
                break

            print(f"connection from {addr}")

            conn_id = addr

            self.lock.acquire()

            thread = threading.Thread(
                target=self.session_func,
                args=(self, conn, addr,
                      lambda : self.EndCallback(conn_id)))

            self.conns[conn_id] = (conn, addr, thread)

            self.lock.release()

            thread.start()

    def EndCallback(self, conn_id):
        self.lock.acquire()

        conn, addr, thread = self.conns[conn_id]
        conn.close()
        del self.conns[conn_id]

        self.lock.release()

        print(f"diconnect from {addr}")
