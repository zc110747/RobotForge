"""传输抽象层：串口 / CAN / USB / TCP 的接口预留（v0.1 不实现）。

⚠️ 本模块**没有任何调用方** —— 它是 spec §48 要求的设计预留，
不参与 v0.1 的任何数据流。`RobotRuntime` 走 WebSocket，不经此处。
"""

from backend.transport.transport import Transport

__all__ = ["Transport"]

