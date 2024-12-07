py_library(
    name = "action",
    srcs = [
        "action.py",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//components/falcons/action_planning/interface:interface_py",
        "//libraries/logging:logdata",
        "//libraries/plotting",
    ],
)

py_binary(
    name = "plot",
    srcs = [
        "plot.py",
    ],
    python_version = "PY3",
    visibility = ["//visibility:public"],
    deps = [
        ":action",
    ],
)

py_binary(
    name = "print",
    srcs = [
        "print.py",
    ],
    python_version = "PY3",
    visibility = ["//visibility:public"],
    deps = [
        ":action",
    ],
)
