"""控制逻辑占位。

根据实际项目需求实现控制、规划或推理逻辑。
"""

from dataclasses import dataclass


@dataclass
class ControllerConfig:
    rate_hz: int = 20
    enabled: bool = True


class Controller:
    def __init__(self, config: ControllerConfig) -> None:
        self.config = config

    def step(self, observation: dict) -> dict:
        """单步控制接口。

        Args:
            observation: 传感器/视觉观测数据。

        Returns:
            dict: 控制输出（占位）。
        """
        if not self.config.enabled:
            return {"enabled": False}

        return {"enabled": True, "command": None}
