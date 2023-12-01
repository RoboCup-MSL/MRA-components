#ifndef _MRA_BASE_TEST_FACTORY_HPP
#define _MRA_BASE_TEST_FACTORY_HPP

#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include <google/protobuf/message.h>
#include <cmath>
using namespace ::testing;

#include "json_convert.hpp"


namespace MRA::TestFactory
{

// Compare protobuf objects with tolerance
bool areProtosEqualWithTolerance(const google::protobuf::Message& actual, const google::protobuf::Message& expected, double tolerance)
{
    if (actual.GetDescriptor() != expected.GetDescriptor()) {
        return false;
    }

    for (int i = 0; i < actual.GetDescriptor()->field_count(); ++i) {
        const auto* fieldDescriptor = actual.GetDescriptor()->field(i);
        const auto fieldType = fieldDescriptor->type();

        // Handle repeated fields
        if (fieldDescriptor->is_repeated()) {
            const int actualSize = actual.GetReflection()->FieldSize(actual, fieldDescriptor);
            const int expectedSize = expected.GetReflection()->FieldSize(expected, fieldDescriptor);

            if (actualSize != expectedSize) {
                return false;
            }

            for (int j = 0; j < actualSize; ++j) {
                const auto& actualElement = actual.GetReflection()->GetRepeatedMessage(actual, fieldDescriptor, j);
                const auto& expectedElement = expected.GetReflection()->GetRepeatedMessage(expected, fieldDescriptor, j);

                if (!areProtosEqualWithTolerance(actualElement, expectedElement, tolerance)) {
                    return false;
                }
            }
        }
        // Handle numerical fields
        else if (fieldType == google::protobuf::FieldDescriptor::Type::TYPE_DOUBLE ||
                 fieldType == google::protobuf::FieldDescriptor::Type::TYPE_FLOAT) {
            const double actualValue = actual.GetReflection()->GetDouble(actual, fieldDescriptor);
            const double expectedValue = expected.GetReflection()->GetDouble(expected, fieldDescriptor);

            if (std::abs(actualValue - expectedValue) > tolerance) {
                return false;
            }
        }
        // Add more field types as needed
        // For nested messages, recursively compare
        else if (fieldType == google::protobuf::FieldDescriptor::Type::TYPE_MESSAGE) {
            const auto& actualSubMessage = actual.GetReflection()->GetMessage(actual, fieldDescriptor);
            const auto& expectedSubMessage = expected.GetReflection()->GetMessage(expected, fieldDescriptor);

            if (!areProtosEqualWithTolerance(actualSubMessage, expectedSubMessage, tolerance)) {
                return false;
            }
        }
    }

    return true;
}

template <typename Tc>
typename Tc::OutputType run_testvector(std::string tv_filename, double tolerance = 0.0)
{
    // Arrange types
    auto m = Tc();
    auto input = typename Tc::InputType();
    auto params = typename Tc::ParamsType();
    auto state = typename Tc::StateType();
    auto local = typename Tc::LocalType();
    auto expected_output = typename Tc::OutputType();
    auto actual_output = typename Tc::OutputType();

    // Act - load testvector
    std::string js = read_file_as_string(tv_filename);
    nlohmann::json j = nlohmann::json::parse(js);
    convert_json_to_proto(j, "Input", input);
    if (j.contains("Params"))
    {
        convert_json_to_proto(j, "Params", params);
    }
    else
    {
        params = m.defaultParams();
    }
    convert_json_to_proto(j, "State", state);
    convert_json_to_proto(j, "Output", expected_output);

    // Act - tick
    int error_value = m.tick(input, params, state, actual_output, local);

    // Assert
    EXPECT_EQ(error_value, 0);
    if (tolerance == 0.0)
    {
        // string comparison is sensitive to numerical noise and float/double datatype choices
        EXPECT_EQ(convert_proto_to_json_str(actual_output), convert_proto_to_json_str(expected_output));
    }
    else
    {
        EXPECT_TRUE(areProtosEqualWithTolerance(actual_output, expected_output, tolerance));
    }

    // Return (for further checks) in test instance
    return actual_output;
}

}; // namespace MRA::TestFactory

#endif // _MRA_BASE_TEST_FACTORY_HPP
