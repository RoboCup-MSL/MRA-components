load("@rules_python//python:defs.bzl", "py_binary")

py_binary(
    name = "joystick",
    srcs = [
        "joystick.py",
        "joystick_controller.py",
        "joystick_pygame.py",
    ],
    data = [
        "joystick.json",
    ],
    main = "joystick.py",
)
