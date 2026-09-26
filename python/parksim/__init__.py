"""ParkSim 包入口：标记 package，并装一个运行期兼容层。

━━ 兼容层解决的问题（实测，非推测）━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
rclpy 的 ``RcutilsLogger.log`` 以

    CallerId = (function_name, abspath(filename), frame.f_lineno, frame.f_lasti)

为键，缓存「这个调用点用的是哪个 severity」。同一个键第二次以**不同 severity**
调用就抛::

    ValueError: Logger severity cannot be changed between calls.

而 Cython 编译后的帧在**一整个函数内** ``f_lineno`` 与 ``f_lasti`` 恒定。微实验
（同一份源码，只改编译参数）实测：

    纯 Python   同一函数内两个调用点 A=(26,16)  B=(29,42)   可区分 -> INFO→WARN 正常
    Cython 编译 同一函数内两个调用点 A=( 9,92)  B=( 9,92)   不可区分 -> INFO→WARN 抛错

后果：任何「同一函数里先记 INFO、后记 WARN」的编译模块，一走到 WARN 就崩。
实测现场是 vehicle_node 的
「逐泊位机动表不存在，将回退 legacy 16 组合离线表」这条 warn ——
它一抛错，车辆节点随即退出，仿真跟着 sim_dead；
``SimulatorNode._release_guard`` 的 warn 也是同一机制。

已经实测证伪的两条**编译期**修法（都试过，都不行）：
    -X profile=True                     -> 仍不可区分
    -X linetrace=True -DCYTHON_TRACE=1  -> 仍不可区分
（Cython 只在「模块导入时已经装了跟踪器」的情况下才更新行号，
 不做覆盖率/性能分析时 == 没有行号。）

━━ 做法 ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
把 ``log()`` 包一层，**只在**撞上这条特定错误时，把该 CallerId 缓存的 severity
改成本次的 severity 再重试一次。效果等价于「按 (调用点, severity) 建缓存」。

代价：该调用点的 once/throttle 过滤状态会在 severity 之间共享。在编译帧下
本来也分不开调用点，所以这不算额外损失。

━━ 为什么放在这里而不是做成编译期选项 ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Cython 不提供「让每个调用点有独立 CallerId」的能力，所以只能在 rclpy 这一侧让步。
放在包的 ``__init__`` 里，作用域恰好是「凡导入 parksim 的进程」，
不必给全镜像所有 python 进程加负担（sitecustomize 会让 ``python3 -c 'print(1)'``
也去 import rclpy）。

━━ 生效条件 ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
只在本仓库**无源码产物**里启用，判据是构建阶段写下的台账文件
``/opt/parksim/SOURCELESS.txt``（见 docker/sourceless_build.sh S7）。
未编译环境（v1/v2）里该文件不存在，整个补丁**完全不安装**，
因此对 CPython 版本零行为差异、零开销。
可用环境变量 ``PARKSIM_RCLPY_LOGFIX=1|0`` 覆盖，便于实验。
"""

import os as _os

_SENTINEL = "/opt/parksim/SOURCELESS.txt"
_MSG = "Logger severity cannot be changed between calls."
_ATTR = "_parksim_cython_logfix"


def _install_rclpy_cython_logfix():
    """安装兼容层。返回 True 表示已安装（或此前已安装）。

    全过程包在 try/except 里：兼容层自身绝不允许影响 parksim 的导入。
    """
    force = _os.environ.get("PARKSIM_RCLPY_LOGFIX", "").strip()
    if force == "0":
        return False
    if force != "1" and not _os.path.exists(_SENTINEL):
        return False

    try:
        import rclpy.impl.rcutils_logger as _rl
        from rclpy.logging import LoggingSeverity
    except Exception:
        return False

    if getattr(_rl.RcutilsLogger, _ATTR, False):
        return True

    _orig = _rl.RcutilsLogger.log

    # rclpy 明确留了这个口子：第三方可以把自己的文件登记为「内部调用者」，
    # 这样 _find_caller 才会跳过本包装层、把调用点还原成用户代码，
    # 使包装前后算出的 CallerId 完全一致（否则包装层自身会成为调用点）。
    try:
        _rl._internal_callers.append(_os.path.realpath(__file__))
    except Exception:
        pass

    def log(self, message, severity, **kwargs):
        try:
            return _orig(self, message, severity, **kwargs)
        except ValueError as exc:
            if _MSG not in str(exc):
                raise
            ctx = None
            try:
                ctx = self.contexts.get(_rl.CallerId())
            except Exception:
                ctx = None
            if not isinstance(ctx, dict) or "severity" not in ctx:
                raise
            # 编译帧区分不出调用点：把缓存里的 severity 改成本次的再重试。
            ctx["severity"] = LoggingSeverity(severity)
            return _orig(self, message, severity, **kwargs)

    log.__name__ = "log"
    log.__doc__ = _orig.__doc__
    log.__wrapped__ = _orig
    _rl.RcutilsLogger.log = log
    setattr(_rl.RcutilsLogger, _ATTR, True)
    return True


try:
    _install_rclpy_cython_logfix()
except Exception:
    # 兼容层失败绝不阻断业务：宁可回到「编译产物下 warn 会抛错」的原状，
    # 也不要让 import parksim 直接失败。
    pass
