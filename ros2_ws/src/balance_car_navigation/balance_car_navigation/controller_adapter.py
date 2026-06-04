from .backend_client import request_backend


class BackendControllerAdapter:
    def __init__(self, host: str, port: int, timeout: float = 0.6) -> None:
        self.host = host
        self.port = port
        self.timeout = timeout

    def _request(self, payload: dict) -> dict:
        response = request_backend(self.host, self.port, payload, timeout=self.timeout)
        if not response.get("ok"):
            raise RuntimeError(f"backend rejected command: {response}")
        return response

    def get_state(self) -> dict:
        return self._request({"cmd": "get_state"})

    def set_mode(self, mode_id: int) -> dict:
        return self._request({"cmd": "set_mode", "value": int(mode_id)})

    def set_move(self, move_x: float, move_z: float) -> dict:
        return self._request({"cmd": "set_move", "move_x": float(move_x), "move_z": float(move_z)})
