from __future__ import annotations

from rpp_interfaces.parameter_helpers import get_navigation_state


class ComponentParameters:
    state = get_navigation_state(resource="base")
