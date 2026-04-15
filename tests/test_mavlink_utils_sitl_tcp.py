"""Tests for SITL TCP connection helper (no MAVProxy)."""

from core.mavlink.utils import sitl_tcp_connection_string


def test_sitl_tcp_connection_string_ports() -> None:
    assert sitl_tcp_connection_string(0) == "tcp:127.0.0.1:5760"
    assert sitl_tcp_connection_string(1) == "tcp:127.0.0.1:5770"
    assert sitl_tcp_connection_string(2) == "tcp:127.0.0.1:5780"
