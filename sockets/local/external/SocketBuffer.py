import socket
import cv2 as cv
import time
from collections import deque
import numpy as np

class DisconnectionException(Exception):
    def __init__(self):
        super().__init__("DisconnectionException")

class TimeoutException(Exception):
    def __init__(self):
        super().__init__("TimeoutException")

class SocketBufferByte:
    def __init__(self, s):
        self.s = s
        self.buf = deque()

    def Recv(self, size, deadline):
        assert 0 <= size

        if size == 0:
            return bytearray(0)

        if deadline is not None and deadline <= time.time():
            raise TimeoutException()

        old_timeout = self.s.gettimeout()

        while len(self.buf) < size:
            remaining_time = None

            if deadline is not None:
                remaining_time = deadline - time.time()

                if remaining_time <= 0:
                    break

                remaining_time = max(remaining_time, 1 / 1000)

            self.s.settimeout(remaining_time)

            recv_size = max(1024, size - len(self.buf))

            try:
                b = self.s.recv(recv_size)
            except socket.timeout:
                continue

            if len(b) == 0:
                raise DisconnectionException()

            self.buf.extend(b)

        self.s.settimeout(old_timeout)

        if len(self.buf) < size:
            raise TimeoutException()

        ret = bytearray()

        for _ in range(size):
            ret.append(self.buf.popleft())

        return ret

    def Send(self, data):
        self.s.send(data)

dtype_idx_to_name = [
    np.dtype(np.int8).name,
    np.dtype(np.int32).name,
    np.dtype(np.int64).name,
    np.dtype(np.float32).name,
    np.dtype(np.float64).name,
]

dtype_name_to_idx = {s:i for i, s in enumerate(dtype_idx_to_name)}

class SocketBufferArr:
    def __init__(self, sb_byte):
        self.sb_byte = sb_byte

        self.dtype_name = None
        self.shape_len = None
        self.shape = []
        self.data_len = None
        self.data = None

        '''
            dtype_idx len=8

            shape_len len=8
            shape[0] len=8
            shape[1] len=8
            shape[2] len=8

            data_len len=8
            data len=data_len
        '''

    def Recv(self, deadline):
        if self.dtype_name is None:
            dtype_idx = int.from_bytes(self.sb_byte.Recv(8, deadline), "little")
            assert 0 <= dtype_idx and dtype_idx < len(dtype_idx_to_name)
            self.dtype_name = dtype_idx_to_name[dtype_idx]

        if self.shape_len is None:
            self.shape_len = int.from_bytes(self.sb_byte.Recv(8, deadline),
                                            "little")
            assert 0 <= self.shape_len

        while len(self.shape) < self.shape_len:
            val = int.from_bytes(self.sb_byte.Recv(8, deadline), "little")
            assert 0 <= val
            self.shape.append(val)

        if self.data_len == None:
            self.data_len = int.from_bytes(self.sb_byte.Recv(8, deadline),
                                          "little")
            assert 0 <= self.data_len

        if self.data == None:
            self.data = self.sb_byte.Recv(self.data_len, deadline)

        ret = np.frombuffer(self.data, dtype=np.dtype(self.dtype_name))
        ret = ret.reshape(self.shape)

        self.dtype_name = None
        self.shape_len = None
        self.shape = []
        self.data_len = None
        self.data = None

        return ret

    def Send(self, a):
        dtype_idx = dtype_name_to_idx[a.dtype.name].to_bytes(8, "little")
        shape = list(a.shape)
        shape_len = len(shape).to_bytes(8, "little")
        data = a.tobytes()
        data_len = len(data).to_bytes(8, "little")

        self.sb_byte.Send(dtype_idx)

        self.sb_byte.Send(shape_len)

        for val in shape:
            self.sb_byte.Send(val.to_bytes(8, "little"))

        self.sb_byte.Send(data_len)
        self.sb_byte.Send(data)

class SocketBufferImage:
    def __init__(self, sb_byte):
        self.sb_byte = sb_byte

        self.h = None
        self.w = None
        self.c = None
        self.data_len = None
        self.data = None

        '''
            h len=8
            w len=8
            c len=8
            data_len
            data
        '''

    def Recv(self, deadline):
        if self.h == None:
            self.h = int.from_bytes(self.sb_byte.Recv(8, deadline), "little")
            assert 0 <= self.h

        if self.w == None:
            self.w = int.from_bytes(self.sb_byte.Recv(8, deadline), "little")
            assert 0 <= self.w

        if self.c == None:
            self.c = int.from_bytes(self.sb_byte.Recv(8, deadline), "little")
            assert 0 <= self.c

        if self.data_len == None:
            self.data_len = int.from_bytes(self.sb_byte.Recv(8, deadline),
                                          "little")
            assert 0 <= self.data_len

        if self.data == None:
            self.data = np.frombuffer(
                self.sb_byte.Recv(self.data_len, deadline), dtype=np.int8)

        img = cv.imdecode(self.data, cv.IMREAD_COLOR)
        # img = np.zeros((self.h, self.w, self.c))

        self.h = None
        self.w = None
        self.c = None
        self.data_len = None
        self.data = None

        return img

    def Send(self, img):
        encode_params = (cv.IMWRITE_JPEG_QUALITY, 80)
        success, data = cv.imencode(".jpeg", img, encode_params)

        h = img.shape[0].to_bytes(8, "little")
        w = img.shape[1].to_bytes(8, "little")
        c = img.shape[2].to_bytes(8, "little")
        data = data.tobytes()
        data_len = len(data)

        data_len = data_len.to_bytes(8, "little")

        self.sb_byte.Send(h)
        self.sb_byte.Send(w)
        self.sb_byte.Send(c)
        self.sb_byte.Send(data_len)
        self.sb_byte.Send(data)
