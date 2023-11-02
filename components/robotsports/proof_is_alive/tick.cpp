// this file was produced by MRA-codegen.py from template_tick.cpp
// with the intent of letting user add the implementation here

// generated component header:
#include "RobotsportsProofIsAlive.hpp"

// dependent libraries:
#include "geometry.hpp"

using namespace MRA;

// custom includes, if any
// #define DEBUG

// -----------------------------------------------------------------------------
int RobotsportsProofIsAlive::RobotsportsProofIsAlive::tick
(
    google::protobuf::Timestamp timestamp,   // absolute timestamp
    InputType  const           &input,       // input data, type generated from Input.proto
    ParamsType const           &params,      // configuration parameters, type generated from Params.proto
    StateType                  &state,       // state data, type generated from State.proto
    OutputType                 &output,      // output data, type generated from Output.proto
    LocalType                  &local        // local/diagnostics data, type generated from Local.proto
)
{
	int error_value = 0;
	MRA_LOG_TICK();

    auto const ws = input.worldstate();
	double rotation_angle_rad = MRA::Geometry::deg_to_rad(params.angle_in_degrees());

	if (state.phase() == StateType::TO_BE_STARTED)
	{
        *state.mutable_timestamp_start_phase() = timestamp;
	}

	double max_time_per_phase = params.max_time_per_phase();
    google::protobuf::Duration duration_phase = google::protobuf::util::TimeUtil::NanosecondsToDuration((int64_t)(1e9 * max_time_per_phase));
	if (timestamp - state.timestamp_start_phase() > duration_phase)
	{
		MRA_LOG_CRITICAL("TIMEOUT: FAILED due to too much time between phases (max: %4.2f seconds)", max_time_per_phase);
        output.set_actionresult(MRA::Datatypes::FAILED);
	}
	else if (!ws.robot().active())
    {
		// fail when robot is inactive
		output.set_actionresult(MRA::Datatypes::FAILED);
    }
	else
	{

	    output.set_actionresult(MRA::Datatypes::RUNNING);

	    auto robot_position = ws.robot().position();
		if (state.phase() == StateType::TO_BE_STARTED)
		{
            *state.mutable_timestamp_start_phase() = timestamp;
		    state.mutable_requested_position()->set_x(robot_position.x());
		    state.mutable_requested_position()->set_y(robot_position.y());
		    state.mutable_requested_position()->set_rz(robot_position.rz() + rotation_angle_rad);
			state.set_phase(StateType::TURN_TO_LEFT);
		}
		else if (state.phase() == StateType::TURN_TO_LEFT)
		{
			if (MRA::Geometry::min_angle(state.requested_position().rz(), robot_position.rz()) < MRA::Geometry::deg_to_rad(params.angle_tolerance_deg()))
			{
				// Robot is turned to the left
                *state.mutable_timestamp_start_phase() = timestamp;
		    	state.mutable_requested_position()->set_x(robot_position.x());
		    	state.mutable_requested_position()->set_y(robot_position.y());
		    	state.mutable_requested_position()->set_rz(robot_position.rz() - 2 * rotation_angle_rad);
				state.set_phase(StateType::TURN_TO_RIGHT);
			}
		}
		else if (state.phase() == StateType::TURN_TO_RIGHT)
		{
			if (MRA::Geometry::min_angle(state.requested_position().rz(), robot_position.rz()) < MRA::Geometry::deg_to_rad(params.angle_tolerance_deg()))
			{
                *state.mutable_timestamp_start_phase() = timestamp;
		    	state.mutable_requested_position()->set_x(robot_position.x());
		    	state.mutable_requested_position()->set_y(robot_position.y());
		    	state.mutable_requested_position()->set_rz(robot_position.rz() + rotation_angle_rad);
				state.set_phase(StateType::TURN_TO_MIDDLE);
			}
		}
		else if (state.phase() == StateType::TURN_TO_MIDDLE)
		{
			if (MRA::Geometry::min_angle(state.requested_position().rz(), robot_position.rz()) < MRA::Geometry::deg_to_rad(params.angle_tolerance_deg()))
			{
                *state.mutable_timestamp_start_phase() = timestamp;
		    	state.mutable_requested_position()->set_x(robot_position.x());
		    	state.mutable_requested_position()->set_y(robot_position.y());
		    	state.mutable_requested_position()->set_rz(robot_position.rz() - rotation_angle_rad);
				state.set_phase(StateType::FINISHED);
	        	output.set_actionresult(MRA::Datatypes::PASSED);
			}
		}
	    output.mutable_target()->mutable_position()->set_x(state.requested_position().x());
	    output.mutable_target()->mutable_position()->set_y(state.requested_position().y());
	    output.mutable_target()->mutable_position()->set_rz(state.requested_position().rz());
	}

	return error_value;
}

