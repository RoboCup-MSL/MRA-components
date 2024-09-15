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
    DiagnosticsType &diagnostics,
    OutputFunc outputFunc,
    const std::string &actionName // String specifying the action name, e.g., "getball"
)
{
    // get descriptor and reflection for input, params, and state types
    const google::protobuf::Descriptor* inputDesc = input.action().GetDescriptor();
    const google::protobuf::Descriptor* paramsDesc = params.action().GetDescriptor();
    const google::protobuf::Descriptor* stateDesc = state.action().GetDescriptor();
    const google::protobuf::Descriptor* diagnosticsDesc = diagnostics.action().GetDescriptor();
    const google::protobuf::Reflection* inputRef = input.action().GetReflection();
    const google::protobuf::Reflection* paramsRef = params.action().GetReflection();
    const google::protobuf::Reflection* stateRef = state.action().GetReflection();
    const google::protobuf::Reflection* diagnosticsRef = diagnostics.action().GetReflection();

    // find field descriptors for the action
    const google::protobuf::FieldDescriptor* inputField = inputDesc->FindFieldByName(actionName);
    const google::protobuf::FieldDescriptor* paramsField = paramsDesc->FindFieldByName(actionName);
    const google::protobuf::FieldDescriptor* stateField = stateDesc->FindFieldByName(actionName);
    const google::protobuf::FieldDescriptor* diagnosticsField = diagnosticsDesc->FindFieldByName(actionName);

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
    if (!diagnosticsField)
    {
        throw std::runtime_error("invalid action name or incomplete protobuf diagnostics interface definition: " + actionName);
    }
    // setup subcomponent data, access the action field dynamically
    typename SubcomponentType::InputType subcomponent_input;

    // copy fields from input to subcomponent_input
    subcomponent_input.mutable_worldstate()->CopyFrom(input.worldstate());

    // use reflection to copy all action fields and check that they are of the correct type
    // this is done to have the action_planning interface self-contained
    // alternatively, most of below could be replaced with this, when the datatype is nested (protobuf imports)
    //typename SubcomponentType::InputType subcomponent_input = dynamic_cast<const typename SubcomponentType::InputType&>(inputRef->GetMessage(input.action(), inputField));
    const google::protobuf::Message& inputActionMessage = inputRef->GetMessage(input.action(), inputField);
    const google::protobuf::Descriptor* subcomponentInputDesc = subcomponent_input.GetDescriptor();
    const google::protobuf::Reflection* subcomponentInputRef = subcomponent_input.GetReflection();

    for (int i = 0; i < inputActionMessage.GetDescriptor()->field_count(); ++i) {
        const google::protobuf::FieldDescriptor* field = inputActionMessage.GetDescriptor()->field(i);
        const google::protobuf::FieldDescriptor* subcomponentField = subcomponentInputDesc->FindFieldByName(actionName + "." + field->name());
        if (subcomponentField) {
            if (field->type() == subcomponentField->type()) {
                switch (field->cpp_type()) {
                    case google::protobuf::FieldDescriptor::CPPTYPE_MESSAGE:
                        subcomponentInputRef->MutableMessage(&subcomponent_input, subcomponentField)->CopyFrom(inputRef->GetMessage(inputActionMessage, field));
                        break;
                    case google::protobuf::FieldDescriptor::CPPTYPE_BOOL:
                        subcomponentInputRef->SetBool(&subcomponent_input, subcomponentField, inputRef->GetBool(inputActionMessage, field));
                        break;
                    case google::protobuf::FieldDescriptor::CPPTYPE_INT32:
                        subcomponentInputRef->SetInt32(&subcomponent_input, subcomponentField, inputRef->GetInt32(inputActionMessage, field));
                        break;
                    case google::protobuf::FieldDescriptor::CPPTYPE_INT64:
                        subcomponentInputRef->SetInt64(&subcomponent_input, subcomponentField, inputRef->GetInt64(inputActionMessage, field));
                        break;
                    case google::protobuf::FieldDescriptor::CPPTYPE_UINT32:
                        subcomponentInputRef->SetUInt32(&subcomponent_input, subcomponentField, inputRef->GetUInt32(inputActionMessage, field));
                        break;
                    case google::protobuf::FieldDescriptor::CPPTYPE_UINT64:
                        subcomponentInputRef->SetUInt64(&subcomponent_input, subcomponentField, inputRef->GetUInt64(inputActionMessage, field));
                        break;
                    case google::protobuf::FieldDescriptor::CPPTYPE_FLOAT:
                        subcomponentInputRef->SetFloat(&subcomponent_input, subcomponentField, inputRef->GetFloat(inputActionMessage, field));
                        break;
                    case google::protobuf::FieldDescriptor::CPPTYPE_DOUBLE:
                        subcomponentInputRef->SetDouble(&subcomponent_input, subcomponentField, inputRef->GetDouble(inputActionMessage, field));
                        break;
                    case google::protobuf::FieldDescriptor::CPPTYPE_STRING:
                        subcomponentInputRef->SetString(&subcomponent_input, subcomponentField, inputRef->GetString(inputActionMessage, field));
                        break;
                    default:
                        throw std::runtime_error("unsupported input field type: " + field->name());
                }
            } else {
                MRA_LOG_DEBUG("field type mismatch for input field: %s", field->name().c_str());
            }
        }
    }

    typename SubcomponentType::ParamsType subcomponent_params = dynamic_cast<const typename SubcomponentType::ParamsType&>(paramsRef->GetMessage(params.action(), paramsField));
    typename SubcomponentType::StateType subcomponent_state = dynamic_cast<const typename SubcomponentType::StateType&>(stateRef->GetMessage(state.action(), stateField));
    typename SubcomponentType::OutputType subcomponent_output;
    typename SubcomponentType::DiagnosticsType subcomponent_diagnostics;

    // call component
    int error_value = SubcomponentType().tick(
        timestamp,
        subcomponent_input,
        subcomponent_params,
        subcomponent_state,
        subcomponent_output,
        subcomponent_diagnostics
    );

    if (error_value == 0)
    {
        // general action data handling
        output.set_actionresult(subcomponent_output.actionresult());
        stateRef->MutableMessage(state.mutable_action(), stateField)->CopyFrom(subcomponent_state);
        diagnosticsRef->MutableMessage(diagnostics.mutable_action(), diagnosticsField)->CopyFrom(subcomponent_diagnostics);

        // specific output mapping
        outputFunc(subcomponent_output, output.mutable_setpoints());
    }

    return error_value;
}
