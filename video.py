import socket
import struct
import zlib

HOST = '127.0.0.1'
PORT = 9000

def recv_exact(sock, n):
    data = bytearray()
    while len(data) < n:
        chunk = sock.recv(n - len(data))
        if not chunk:
            raise IOError('Connection closed while reading')
        data.extend(chunk)
    return bytes(data)

def handle_client(conn):
    while True:
        # read 4-byte prefix: message length
        raw = conn.recv(4)
        if not raw:
            break
        (msg_len,) = struct.unpack('!I', raw)
        msg = recv_exact(conn, msg_len)

        # parse header
        fmt_len = struct.unpack('!B', msg[0:1])[0]
        idx = 1
        fmt_bytes = msg[idx:idx+fmt_len]; idx += fmt_len
        image_format = fmt_bytes.decode('ascii')
        width, height, comp_flag, payload_len = struct.unpack('!II B I', msg[idx:idx+13]); idx += 13
        payload = msg[idx:idx+payload_len]

        if comp_flag:
            payload = zlib.decompress(payload)

        # payload is raw pixel bytes; process or convert to image here
        print('Received frame:', image_format, width, height, len(payload))

def main():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((HOST, PORT))
    s.listen(1)
    print('Listening on', (HOST, PORT))
    conn, addr = s.accept()
    print('Accepted connection from', addr)
    try:
        handle_client(conn)
    finally:
        conn.close()
        s.close()

if __name__ == '__main__':
    main()
