from __future__ import annotations

from rpp_interfaces.parameter_helpers import get_navigation_reference, get_navigation_state


class ComponentParameters:
    state = get_navigation_state(resource='base')
    references = get_navigation_reference(resource='base', inner_controller='ctl0')
    targetMask = [True] * len(references['fields'])
