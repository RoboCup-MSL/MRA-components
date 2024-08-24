load("@rules_python//python:defs.bzl", "py_binary")

py_library(
    name = "joystick_lib",
    srcs = [
        "joystick_controller.py",
        "joystick_core.py",
        "terminal.py",
        "joystick.py",
    ],
    data = [
        "joystick.json",
    ],
)

py_binary(
    name = "joystick",
    srcs = [
        "joystick.py",
    ],
    main = "joystick.py",
    deps = [
        ":joystick_lib",
    ],
)

py_binary(
    name = "joystick_falcons",
    srcs = [
        "joystick_falcons.py",
        "falcons_interface.py",
        "mra_interface.py",
    ],
    main = "joystick_falcons.py",
    deps = [
        ":joystick_lib",
        "//components/falcons/action_planning/interface:interface_py",
    ],
    data = [
        "//components/falcons/joystick/pybind:action_planning_pybind.so",
    ],
)
