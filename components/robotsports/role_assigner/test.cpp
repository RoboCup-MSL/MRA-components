// this file was produced by MRA-codegen.py from template_test.cpp
// with the intent of allowing user to add custom tests


// Include testframework
#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "test_factory.hpp"
using namespace ::testing;
#include "test/xmlRoleAssigner.hpp"
#include "test/roleassigner_regression_checker.hpp"
#include <string>
#include <filesystem>

// System under test:
#include "RobotsportsRoleAssigner.hpp"
using namespace MRA;

// Basic tick shall run OK and return error_value 0.
TEST(RobotsportsRoleAssignerTest, basicTick)
{
    // Arrange
    auto m = RobotsportsRoleAssigner::RobotsportsRoleAssigner();

    // Act
    int error_value = m.tick();

    // Assert
    EXPECT_EQ(error_value, 0);
}

// Basic tick shall run OK and return error_value 0.
TEST(RobotsportsRoleAssignerTest, xmlTest)
{
    std::filesystem::path component_source_directory = std::filesystem::current_path();
    std::filesystem::path MRA_base_directory = component_source_directory.parent_path().parent_path().parent_path();
    std::string path = "./testdata/xml-inputfiles";
    std::string output_base_directory = MRA_base_directory.string() + "/build/components/robotsports/role_assigner/test/";
    std::filesystem::create_directory(output_base_directory);
    for (const auto& file_entry : std::filesystem::directory_iterator(path)) {
        auto input_filename = file_entry.path();
        std::cerr << "\nxmltest with input-file: " << input_filename << std::endl;
        role_assigner_with_xml_input(input_filename, output_base_directory);
    }
    std::string regression_folder = MRA_base_directory.string() + "/components/robotsports/role_assigner/testdata/regression";
    // auto nr_failures = validate_regression(regression_folder, output_base_directory + "output_team/");
    validate_regression(regression_folder, output_base_directory + "output_team/");
    // EXPECT_EQ(nr_failures, 0); TODO
}


int main(int argc, char **argv)
{
    InitGoogleTest(&argc, argv);
    int r = RUN_ALL_TESTS();
    return r;
}

