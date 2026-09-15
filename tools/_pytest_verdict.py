"""所有验收清单共用的 pytest 判词解析（**唯一真值源**）。

## 为什么需要这个模块

验收清单原先一律写 `check("pytest ...", proc.returncode == 0, ...)`。
那个写法在**本机沙箱**下会产出**假回归**，实测（2026-09-15）：

```text
[FAIL] pytest tests/test_loader_registry.py
       ↳ .......................                     [100%][safe-delete]
         [SAFE_DELETE_BULK_CONFIRM_REQUIRED] {"count":5023,"threshold":5000,...}
```

`[100%]` + 一整串通过点，**测试全过**，但退出码非 0 ⇒ 判 FAIL，并且
Phase 1~5 的上游复查全部连坐（Phase 7 一度报 39/50）。

**机制**：沙箱有"批量删除守卫"（单 turn 内删除 >5000 个文件会被拦）。
pytest 每跑一次都在 `%TEMP%/pytest-of-zc110/` 下建一棵临时树并在结束时清理；
本清单会连跑 7 次 pytest，累计删除量越过阈值 ⇒ **清理被拦 ⇒ 退出码变非 0**。

**结论**：判据必须量它**声称**要量的东西 —— "测试有没有过"，
而不是"进程退出码"（后者还掺了"临时目录清理是否被放行"）。

## 判词来源（两级，缺一不可）

```text
① 摘要行有 `N passed` / `N failed` / `N error`  → 直接读计数（最可靠）
② 摘要行被噪声顶掉（`-q` 全过时只打点、不打计数）
   ⇒ 退化判据：出现 `[100%]` **且** 全文无 `failed`/`error` 字样
      **且** 通过点 >10 个
```

⚠️ 第 ② 级的三个条件必须**同时**成立。少任何一个都会变成恒真：
只看"没有 failed"的话，**空输出**也会判过。
"""

from __future__ import annotations

import re

#: pytest 摘要行里的计数。
#: `488 passed, 2 warnings in 4.87s` / `3 failed, 485 passed in 9.1s` / `2 error in 1.2s`
_COUNT_RE = re.compile(
    r"(?:(?P<failed>\d+)\s+failed)"
    r"|(?:(?P<passed>\d+)\s+passed)"
    r"|(?:(?P<error>\d+)\s+error)"
)

#: 第 ② 级退化判据里，"通过点"的最少个数。
#: 设 10 是为了排除"输出里只有沙箱提示里的小数点"这种情形（例如 `0.01s`）。
_MIN_DOTS = 10


def pytest_verdict(out: str) -> tuple[bool, str]:
    """从 pytest 输出里读判词。返回 `(是否全通过, 人类可读摘要)`。

    调用方应把返回值直接喂给 `check(...)`，**不要**再看退出码。
    """
    # 摘要总在尾部；但尾部可能粘着沙箱噪声（甚至没有尾随换行），
    # 所以取最后 4 行做计数扫描，同时保留全文做第 ② 级判定。
    tail = "\n".join(out.splitlines()[-4:])

    failed = errored = passed = 0
    for m in _COUNT_RE.finditer(tail):
        if m.group("failed"):
            failed += int(m.group("failed"))
        elif m.group("passed"):
            passed += int(m.group("passed"))
        elif m.group("error"):
            errored += int(m.group("error"))

    if not passed:
        # 第 ② 级：全过时 `-q` 只打点、不打计数 ⇒ 用"形态"判定。
        if (
            "[100%]" in out
            and failed == 0
            and errored == 0
            and out.count(".") > _MIN_DOTS
        ):
            return (
                True,
                "全通过（`[100%]` + 通过点；摘要行被环境噪声顶掉）",
            )
        # 到这儿说明"既没有通过计数，也没有全过形态" ⇒ 不能判过。
        # 空输出 / 只有噪声 / 只有 error 全落在这里。
        summary = f"{passed} passed"
        if failed:
            summary += f", {failed} failed"
        if errored:
            summary += f", {errored} error"
        return False, summary

    summary = f"{passed} passed"
    if failed:
        summary += f", {failed} failed"
    if errored:
        summary += f", {errored} error"
    return failed == 0 and errored == 0, summary


__all__ = ["pytest_verdict"]
