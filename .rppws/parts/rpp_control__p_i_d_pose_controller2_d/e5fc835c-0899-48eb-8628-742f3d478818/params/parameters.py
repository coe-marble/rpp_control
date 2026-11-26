from __future__ import annotations

from rpp_interfaces.parameter_helpers import get_pid_parameters


class ComponentParameters:
    pid_x = get_pid_parameters()
    pid_y = get_pid_parameters()
    pid_n = get_pid_parameters()
