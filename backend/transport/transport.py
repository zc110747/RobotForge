"""Transport 抽象（spec §48）—— **v0.1 只定接口，不实现任何具体传输**。

## 这个模块存在的意义

v0.1 的机器人在 Web 里跑（`RobotRuntime` ← WebSocket ← 浏览器）。
但真实机器人的下一站一定是**硬件链路**：

```text
SerialTransport   UART / USB-CDC
CANTransport      CAN / CAN-FD
USBTransport      USB bulk
TCPTransport      裸 TCP（机器人本体上的 socket 服务）
```

spec §48 要求**现在就把接口形状定下来**，理由是：

1. **写得出接口 ⇒ 证明分层是对的**。如果定不出一个与传输介质无关的
   `Transport`，说明 `RobotRuntime` 里已经混进了介质细节。
2. **接口是给未来看的设计文档**。比"将来重构"便宜得多。

## 与 `backend/simulation/` 的对称性

```text
backend/simulation/   引擎无关：SimulationBackend（协议）
                      引擎特定：MuJoCoBackend
backend/transport/    介质无关：Transport（协议）   ← 本文件
                      介质特定：SerialTransport / ...（未来）
```

两层是同一个模式：**上层只依赖协议，实际实现由调用方注入**。

## ⚠️ Runtime **不得**直接依赖硬件 Transport

spec §48 末句与 §46 是同一个约束的两面：

```text
RobotRuntime  →  Transport（协议）      ✅
RobotRuntime  →  SerialTransport        ❌（绑死介质）
RobotRuntime  →  pyserial / socketcan   ❌（绑死库）
```

`tools/accept_phase4.py` 用正则扫描 `backend/runtime/` 下的**代码**
（已剥离注释与字符串），断言其中不出现 `serial` / `can` / `usb`
字面量，并要求 Runtime 不 import `fastapi` / `mujoco` / `starlette`。
本文件不在扫描范围内（它在 `backend/transport/`，且那些字面量
只出现在 docstring 里）—— 这正是"§48 是预留"应有的样子。

## `send` / `receive` 的**类型**：故意用 `bytes`

spec 给的签名是 `send(self, data)` / `receive(self)`，没有标注类型。
这里**刻意**固定为 `bytes`，因为：

- 各介质的**帧结构不同**（CAN 有 ID + 8 字节、Serial 是字节流、
  TCP 是字节流），只有"一帧载荷"这个概念是共同的
- 变成 `str` 会引入编码假设；变成 `dict` 会把 JSON 序列化层
  漏进传输层 —— 那属于 `Runtime` 与传输之间的**适配层**，不是传输本身

⇒ 编解码（`RobotCommand` ⇄ `bytes`）是**适配层**的职责，
   不在 `Transport` 上。

## 一个真实的坑：`receive()` 返回 `None` 还是抛异常？

这里定的是 **返回 `None` 表示"当前没有数据"**，而不是抛异常。理由：

- 抛异常会让正常的"暂时没数据"与真正的"链路断了"共用一条控制流，
  调用方只能靠 `except` 的类型去区分 —— 而**自定义异常类型无法跨进程**
  （TCP / CAN 场景下错误码才是真值源）
- 返回 `None` 与"链路已断开"（由 `connect` / `disconnect` 的
  状态机表达）是**正交**的两件事，调用方可以分别处理

> ⚠️ 这是**设计约定**，不是实测结论 —— v0.1 没有实现任何传输，
> 所以这条约定**尚未被任何代码验证**。真做第一个
> `SerialTransport` 时，请回头核对这条是否站得住。
"""

from __future__ import annotations

from typing import Protocol, runtime_checkable


@runtime_checkable
class Transport(Protocol):
    """与介质无关的双向字节传输接口（spec §48）。

    用 `Protocol` 而不是 ABC：实现方**不需要**继承 `Transport`，
    只要形状对得上即可（结构化子类型）。这对未来的实现方更友好 ——
    比如包装一个第三方库已有的 `connect/send/receive` 接口时，
    不必为了满足继承关系再造一层适配类。

    `runtime_checkable` 让 `isinstance(obj, Transport)` 可用，
    便于装配处做**早期**校验（只检查方法存在性，不检查签名）。
    """

    async def connect(self) -> None:
        """建立链路。

        约定：
          - 幂等（已连接时重复调用不报错）
          - 失败时抛异常，**不**返回一个"表示失败的值" ——
            连接失败是需要调用方立刻知道的，而不是靠检查返回值
        """
        ...

    async def disconnect(self) -> None:
        """断开链路。

        约定：
          - 幂等（未连接时调用不报错）
          - 最好的努力关闭：即使底层报错也应释放本地资源
        """
        ...

    async def send(self, data: bytes) -> None:
        """发送**一帧**完整载荷。

        Args:
            data: 已编码好的字节。编解码是适配层的职责（见模块 docstring）。

        约定：
          - 未连接时抛异常（这是**编程错误**，不是运行时状况）
          - "发送完成"的语义按介质定义：Serial/TCP 是写入缓冲返回，
            对端是否收到不由本层保证
        """
        ...

    async def receive(self) -> bytes | None:
        """接收**一帧**完整载荷。

        Returns:
            收到的字节；**当前没有数据时返回 `None`**（见模块 docstring
            关于"为什么不用异常"的说明）。

        约定：
          - 非阻塞：不应在这里做长轮询，调度交给调用方
          - 未连接时抛异常（同 `send`）
        """
        ...


__all__ = ["Transport"]
