#include <rpp_control/ros/control_script_node.hpp>

namespace rpp_control
{
void ControlScriptNode::request_reference(
  const RequestReference::Request & request, RequestReference::Response & response)
{
  std::lock_guard<std::mutex> tick_lock(control_tick_mutex_);
  {
    std::lock_guard<std::mutex> lock(mailbox_mutex_);
    if (!routes_ready_ || slot_operation_pending_ || stop_requested_ ||
      !layout_->goal_routes.count(request.interface_name)) {
      response.message = "Reference interface is unknown or unavailable. See control layout.";
      return;
    }
  }
  const auto token = reference_claims_.claim(
    request.interface_name, script_->goal_instance_id(request.interface_name), request.identity);
  if (!token) {
    response.message = "Reference interface is already claimed; its owner must release it.";
    return;
  }
  // A new owner starts without the previous unclaimed navigation target.
  if (!cancel_reference(request.interface_name)) {
    reference_claims_.release(request.interface_name);
    response.message = "Could not clear the previous goal; see control status.";
    return;
  }
  response.success = true;
  response.identity_token = *token;
  response.message = "Reference interface claimed.";
}

void ControlScriptNode::release_reference(
  const ReleaseReference::Request & request, ReleaseReference::Response & response)
{
  std::lock_guard<std::mutex> tick_lock(control_tick_mutex_);
  const auto interface = reference_claims_.interface_for(request.identity_token);
  if (!interface) {
    response.message = "Unknown reference token.";
    return;
  }
  if (!cancel_reference(*interface)) {
    response.message = "Could not cancel the owned goal; see control status.";
    return;
  }
  reference_claims_.release(*interface);
  response.success = true;
  response.message = "Reference interface released and its goals canceled.";
}

// Caller holds the tick mutex. Cancel this generator without resetting other branches.
bool ControlScriptNode::cancel_reference(const std::string & interface)
{
  {
    std::lock_guard<std::mutex> lock(mailbox_mutex_);
    // A latched plugin exception prohibits further plugin calls.
    if (stop_requested_) {return true;}
  }
  try {
    const auto status = script_->cancel(interface);
    if (!control_success(status)) {return false;}
    goal_stamps_.erase(script_->goal_instance_id(interface));
    std::lock_guard<std::mutex> lock(mailbox_mutex_);
    output_pending_ = false;
    have_output_ = false;
    return true;
  } catch (const std::exception & error) {
    mark_worker_failed();
    RCLCPP_ERROR(get_logger(), "Reference cancellation threw; control worker stopped: %s", error.what());
  } catch (...) {
    mark_worker_failed();
    RCLCPP_ERROR(get_logger(), "Reference cancellation threw; control worker stopped.");
  }
  return false;
}
}  // namespace rpp_control
