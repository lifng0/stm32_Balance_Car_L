import json
import socket


def request_backend(host: str, port: int, payload: dict, timeout: float = 0.5) -> dict:
    message = json.dumps(payload, ensure_ascii=False).encode("utf-8") + b"\n"
    with socket.create_connection((host, port), timeout=timeout) as sock:
        sock.sendall(message)
        sock.shutdown(socket.SHUT_WR)
        chunks = []
        while True:
            data = sock.recv(4096)
            if not data:
                break
            chunks.append(data)

    raw = b"".join(chunks).decode("utf-8").strip()
    if not raw:
        raise RuntimeError("empty backend response")
    return json.loads(raw)
