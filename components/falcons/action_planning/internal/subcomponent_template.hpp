// generated component header:
#include "FalconsActionPlanning.hpp"


using namespace MRA::FalconsActionPlanning;

template <typename SubcomponentType, typename OutputFunc>
int handleAction(
    const google::protobuf::Timestamp &timestamp,
    const InputType &input,
    const ParamsType &params,
    StateType &state,
    OutputType &output,
    LocalType &local,
    OutputFunc outputFunc,
    const std::string &actionName // String specifying the action name, e.g., "getball"
)
{
    // get descriptor and reflection for input, params, and state types
    const google::protobuf::Descriptor* inputDesc = input.action().GetDescriptor();
    const google::protobuf::Descriptor* paramsDesc = params.action().GetDescriptor();
    const google::protobuf::Descriptor* stateDesc = state.action().GetDescriptor();
    const google::protobuf::Descriptor* localDesc = local.action().GetDescriptor();
    const google::protobuf::Reflection* inputRef = input.action().GetReflection();
    const google::protobuf::Reflection* paramsRef = params.action().GetReflection();
    const google::protobuf::Reflection* stateRef = state.action().GetReflection();
    const google::protobuf::Reflection* localRef = local.action().GetReflection();

    // find field descriptors for the action
    const google::protobuf::FieldDescriptor* inputField = inputDesc->FindFieldByName(actionName);
    const google::protobuf::FieldDescriptor* paramsField = paramsDesc->FindFieldByName(actionName);
    const google::protobuf::FieldDescriptor* stateField = stateDesc->FindFieldByName(actionName);
    const google::protobuf::FieldDescriptor* localField = localDesc->FindFieldByName(actionName);

    // checks
    if (!inputField)
    {
        throw std::runtime_error("invalid action name or incomplete protobuf input interface definition: " + actionName);
    }
    if (!paramsField)
    {
        throw std::runtime_error("invalid action name or incomplete protobuf params interface definition: " + actionName);
    }
    if (!stateField)
    {
        throw std::runtime_error("invalid action name or incomplete protobuf state interface definition: " + actionName);
    }
    if (!localField)
    {
        throw std::runtime_error("invalid action name or incomplete protobuf local interface definition: " + actionName);
    }

    // setup subcomponent data, access the action field dynamically
    typename SubcomponentType::InputType subcomponent_input = dynamic_cast<const typename SubcomponentType::InputType&>(inputRef->GetMessage(input.action(), inputField));
    subcomponent_input.mutable_worldstate()->CopyFrom(input.worldstate());
    typename SubcomponentType::ParamsType subcomponent_params = dynamic_cast<const typename SubcomponentType::ParamsType&>(paramsRef->GetMessage(params.action(), paramsField));
    typename SubcomponentType::StateType subcomponent_state = dynamic_cast<const typename SubcomponentType::StateType&>(stateRef->GetMessage(state.action(), stateField));
    typename SubcomponentType::OutputType subcomponent_output;
    typename SubcomponentType::LocalType subcomponent_local;

    // call component
    int error_value = SubcomponentType().tick(
        timestamp,
        subcomponent_input,
        subcomponent_params,
        subcomponent_state,
        subcomponent_output,
        subcomponent_local
    );

    if (error_value == 0)
    {
        // general action data handling
        output.set_actionresult(subcomponent_output.actionresult());
        stateRef->MutableMessage(state.mutable_action(), stateField)->CopyFrom(subcomponent_state);
        localRef->MutableMessage(local.mutable_action(), localField)->CopyFrom(subcomponent_local);

        // specific output mapping
        outputFunc(subcomponent_output, output.mutable_setpoints());
    }

    return error_value;
}
