#include "gtest/gtest.h"
#include "rpp_control/motion_controller2d_impl.hpp"
#include "rpp_control/motion_controller3d_impl.hpp"
#include "rpp_cpp/context_builder.hpp"


class TestMotionController : public ::testing::Test {

public:
    const double EPSILON = 1e-5;

    static std::string test_data_dir;
    static bool initialization_successful;

protected:
    static void SetUpTestSuite() {
        test_data_dir = "./tests/data";
        initialization_successful = true;
    }

    static void TearDownTestSuite() {
        // std::filesystem::remove_all(tmp_dir);
    }

};

std::string TestMotionController::test_data_dir = "";
bool TestMotionController::initialization_successful = false;

TEST_F(TestMotionController, TestInstance2DAndSetReferences) {

    auto context = rpp::ComponentContextBuilder(rpp::RPP_CLOCK_MOCK)
        .build_script_from_description_path(
            test_data_dir + "/test_description/motion_controller2d.json",
            std::nullopt,
            test_data_dir + "/test_ws_parts");

    auto clock = std::dynamic_pointer_cast<rpp::RppClockMock>(context.get_clock());

    clock->set_time(100.0);

    auto controller_2d = std::make_unique<rpp_control::MotionController2DImpl>(context);

    EXPECT_TRUE(controller_2d != nullptr);

    rpp_control::DOF dofs;
    controller_2d->get_active_dofs(dofs);
    EXPECT_EQ(dofs, rpp_control::DOF_X | rpp_control::DOF_Y | rpp_control::DOF_N);

    auto state = rpp_control::MotionController2DImpl::ControllerIO::State{};

    EXPECT_FALSE(state.has_any_pose_ext);
    EXPECT_FALSE(state.has_any_twist_ext);
    EXPECT_FALSE(state.has_any_wrench_ext);
    controller_2d->set_feedback({1.0, 2.0, 3.0}, {0.1, 0.2, 0.3});
    controller_2d->get_live_state(state);
    EXPECT_TRUE(state.has_feedback);

    controller_2d->set_current_pose_ref({4.0, 5.0, 6.0}, {true, false, true});
    controller_2d->set_current_twist_ref({0.4, 0.5, 0.6}, {false, true, true});
    controller_2d->set_current_wrench_ref({0.7, 0.8, 0.9}, {true, true, false});

    // It should be false before latch_is_called
    EXPECT_FALSE(state.has_pose_ext[0]);
    clock->set_time(100.1);
    auto now = clock->now_seconds();
    controller_2d->latch_references(now);

    controller_2d->get_live_state(state);

    EXPECT_TRUE(state.has_pose_ext[0]);
    EXPECT_FALSE(state.has_pose_ext[1]);
    EXPECT_TRUE(state.has_pose_ext[2]);
    EXPECT_FALSE(state.has_twist_ext[0]);
    EXPECT_TRUE(state.has_twist_ext[1]);
    EXPECT_TRUE(state.has_twist_ext[2]);
    EXPECT_TRUE(state.has_wrench_ext[0]);
    EXPECT_TRUE(state.has_wrench_ext[1]);
    EXPECT_FALSE(state.has_wrench_ext[2]);

    EXPECT_NEAR(state.pose_ref[0], 4.0, EPSILON);
    EXPECT_NEAR(state.pose_ref[1], 0.0, EPSILON);
    EXPECT_NEAR(state.pose_ref[2], 6.0, EPSILON);
    EXPECT_NEAR(state.twist_ref[0], 0.0, EPSILON);
    EXPECT_NEAR(state.twist_ref[1], 0.5, EPSILON);
    EXPECT_NEAR(state.twist_ref[2], 0.6, EPSILON);
    EXPECT_NEAR(state.wrench_ref[0], 0.7, EPSILON);
    EXPECT_NEAR(state.wrench_ref[1], 0.8, EPSILON);
    EXPECT_NEAR(state.wrench_ref[2], 0.0, EPSILON);

    using SignalStatus = rpp_control::SignalStatus;

    controller_2d->set_wrench_selection(
        {SignalStatus::SIGNAL_EXT,
         SignalStatus::SIGNAL_INT,
         SignalStatus::SIGNAL_DISABLED});
    controller_2d->set_twist_selection(
        {SignalStatus::SIGNAL_INT,
         SignalStatus::SIGNAL_EXT,
         SignalStatus::SIGNAL_DISABLED});
    controller_2d->get_live_state(state);

    EXPECT_EQ(state.twist_selection[0], SignalStatus::SIGNAL_INT);
    EXPECT_EQ(state.twist_selection[1], SignalStatus::SIGNAL_EXT);
    EXPECT_EQ(state.twist_selection[2], SignalStatus::SIGNAL_DISABLED);
    EXPECT_EQ(state.wrench_selection[0], SignalStatus::SIGNAL_EXT);
    EXPECT_EQ(state.wrench_selection[1], SignalStatus::SIGNAL_INT);
    EXPECT_EQ(state.wrench_selection[2], SignalStatus::SIGNAL_DISABLED);
}

TEST_F(TestMotionController, TestInstance2DAndStep) {
    auto context = rpp::ComponentContextBuilder(rpp::RPP_CLOCK_MOCK)
        .build_script_from_description_path(
            test_data_dir + "/test_description/motion_controller2d.json",
            std::nullopt,
            test_data_dir + "/test_ws_parts");

    auto clock = std::dynamic_pointer_cast<rpp::RppClockMock>(context.get_clock());
    clock->set_time(100.0);

    auto controller_2d = std::make_unique<rpp_control::MotionController2DImpl>(context);

    EXPECT_TRUE(controller_2d != nullptr);

    bool step_result = controller_2d->step(0.01);
    EXPECT_FALSE(step_result); // Not initialized yet.

    controller_2d->initialize();

    step_result = controller_2d->step(0.01);
    EXPECT_FALSE(step_result); // No feedback received.

    clock->elapse(0.01);

    controller_2d->set_feedback({1.0, 2.0, 3.0}, {0.1, 0.2, 0.3});
    EXPECT_FALSE(controller_2d->step(0.01)); // No external references received.

    controller_2d->set_current_pose_ref({4.0, 5.0, 6.0}, {true, false, true});
    EXPECT_FALSE(controller_2d->step(-0.01));  // Wrong dt value.

    controller_2d->set_twist_selection(
        {rpp_control::SignalStatus::SIGNAL_INT,
         rpp_control::SignalStatus::SIGNAL_INT,
         rpp_control::SignalStatus::SIGNAL_INT});

    controller_2d->set_wrench_selection(
        {rpp_control::SignalStatus::SIGNAL_INT,
         rpp_control::SignalStatus::SIGNAL_INT,
         rpp_control::SignalStatus::SIGNAL_INT});

    EXPECT_TRUE(controller_2d->step(0.01)); // Should succeed now.

    auto state = rpp_control::MotionController2DImpl::ControllerIO::State{};
    controller_2d->get_live_state(state);

    EXPECT_TRUE(state.has_feedback);
    EXPECT_TRUE(state.has_any_pose_ext);
    EXPECT_FALSE(state.has_any_twist_ext);
    EXPECT_FALSE(state.has_any_wrench_ext);

    EXPECT_NEAR(state.pose_ref[0], 4.0, EPSILON);
    EXPECT_NEAR(state.pose_ref[1], 0.0, EPSILON);
    EXPECT_NEAR(state.pose_ref[2], 6.0, EPSILON);
    EXPECT_NEAR(state.twist_ref[0], 0.0, EPSILON);
    EXPECT_NEAR(state.twist_ref[1], 0.0, EPSILON);
    EXPECT_NEAR(state.twist_ref[2], 0.0, EPSILON);
    EXPECT_TRUE(state.wrench_ref[0] != 0.0);
    EXPECT_TRUE(state.wrench_ref[1] != 0.0);
    EXPECT_TRUE(state.wrench_ref[2] != 0.0);

    EXPECT_NEAR(state.pose[0], 1.0, EPSILON);
    EXPECT_NEAR(state.pose[1], 2.0, EPSILON);
    EXPECT_NEAR(state.pose[2], 3.0, EPSILON);
    EXPECT_NEAR(state.twist[0], 0.1, EPSILON);
    EXPECT_NEAR(state.twist[1], 0.2, EPSILON);
    EXPECT_NEAR(state.twist[2], 0.3, EPSILON);
    EXPECT_TRUE(state.wrench[0] != 0.0);
    EXPECT_TRUE(state.wrench[1] != 0.0);
    EXPECT_TRUE(state.wrench[2] != 0.0);

    EXPECT_TRUE(state.commands.size() == 2);
    EXPECT_TRUE(state.commands[0] != 0.0);
    EXPECT_TRUE(state.commands[1] != 0.0);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);

    // setenv("TEST_DATA_DIR", TEST_DATA_DIR, 1);

    // Pokrećemo GoogleTest najnormalnije
    return RUN_ALL_TESTS();
}
