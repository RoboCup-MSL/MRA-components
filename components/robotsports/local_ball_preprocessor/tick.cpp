// this file was produced by MRA-codegen.py from template_tick.cpp
// with the intent of letting user add the implementation here

// generated component header:
#include "RobotsportsLocalBallPreprocessor.hpp"

using namespace MRA;

// custom includes, if any
// ...

/* define ball types */
typedef enum source_type_e {
        eOMNIVISION = 1,
        eFRONT_CAMERA = 2,
} source_type_e;


int RobotsportsLocalBallPreprocessor::RobotsportsLocalBallPreprocessor::tick
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

    // user implementation goes here

    bool isFrontCameraBallAvailable = input.frontcamera_balls_size() > 0;
    const bool suppressOmni = params.suppress_omni();
    const bool includeOmni = !suppressOmni || (!isFrontCameraBallAvailable);
    output.mutable_candidates()->Clear();

    // first sensor is omni_vision
    // check for new features from omni camera
    if (includeOmni) {
        // Take omnivision features into account
        for (auto idx = 0; idx < input.omnivision_balls_size(); idx++) {
            if (input.omnivision_balls(idx).confidence() > params.ball_min_confidence()) {
                // enough confidence for using the feature
                output.mutable_candidates()->Add()->CopyFrom(input.omnivision_balls(idx));
            }
        }
    }
    // check for new features from front camera process
    if (isFrontCameraBallAvailable) {
        // Take front camera candidates into account
        for (auto idx = 0; idx < input.frontcamera_balls_size(); idx++) {
            if (input.frontcamera_balls(idx).confidence() > params.ball_min_confidence()) {
                // enough confidence for using the candidate
                output.mutable_candidates()->Add()->CopyFrom(input.frontcamera_balls(idx));
            }
        }
    }

    return error_value;
}

