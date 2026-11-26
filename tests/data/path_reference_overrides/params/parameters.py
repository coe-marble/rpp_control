from rpp_interfaces.parameter_helpers import get_velocity_reference


class ComponentParameters:
    resource = "base"
    path_capacity = 4
    max_points = 16
    additional_targets = get_velocity_reference(resource="base", inner_controller="ctl0")
