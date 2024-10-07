// this file was produced by MRA-codegen.py from template_test.cpp
// with the intent of allowing user to add custom tests


// Include testframework
#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "test_factory.hpp"
using namespace ::testing;
#include "test/xmlRoleAssigner.hpp"
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
    std::string path = "./components/robotsports/role_assigner/testdata/xml-inputfiles";
    for (const auto& file_entry : std::filesystem::directory_iterator(path)) {
        //        auto input_filename = "./components/robotsports/role_assigner/testdata/xml-inputfiles/normal_defend_4.xml";
        auto input_filename = file_entry.path();
        std::cerr << "\nxmltest with input-file: " << input_filename << std::endl << std::flush;
        role_assigner_with_xml_input(input_filename);
    }
}


int main(int argc, char **argv)
{
    InitGoogleTest(&argc, argv);
    int r = RUN_ALL_TESTS();
    return r;
}

