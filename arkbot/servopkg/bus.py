from typing import Optional
from servopkg import PortHandler, sts

class ServoBus:
    """
    Thin wrapper around PortHandler + sts so higher layers don't touch raw SDK.
    """
    def __init__(self, port: str, baudrate: int):
        self._port = PortHandler(port)
        if not self._port.openPort():
            raise RuntimeError(f"Failed to open port {port}")
        if not self._port.setBaudRate(baudrate):
            raise RuntimeError(f"Failed to set baudrate {baudrate}")
        self._pkt = sts(self._port)

    @property
    def sdk(self) -> sts:
        return self._pkt

    def close(self):
        try:
            self._port.closePort()
        except Exception:
            pass
