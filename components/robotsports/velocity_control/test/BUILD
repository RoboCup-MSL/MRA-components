py_test(
    name = "python_protobuf",
    size = "small",
    srcs = [
        "demo_protobuf.py",
    ],
    main = "demo_protobuf.py",
    deps = [
        "//components/robotsports/velocity_control/interface:interface_py_proto",
    ],
    data = [
        "//components/robotsports/velocity_control/interface", # needed for DefaultParams.json
    ],
)

