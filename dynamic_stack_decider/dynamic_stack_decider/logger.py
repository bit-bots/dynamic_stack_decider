from typing import cast

from rclpy import logging
from rclpy.impl.rcutils_logger import RcutilsLogger


def get_logger() -> RcutilsLogger:
    return cast(RcutilsLogger, logging.get_logger("dynamic_stack_decider"))
