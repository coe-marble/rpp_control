from __future__ import annotations

from rpp_interfaces.parameter_helpers import get_pid_parameters


class ComponentParameters:
    pid_x = get_pid_parameters(kp=1.0, ki=2.0, kd=3.0)
    pid_y = get_pid_parameters(kp=1.0, ki=2.0, kd=3.0)
    pid_n = get_pid_parameters(kp=1.0, ki=2.0, kd=3.0)
