load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")

# Define the xsd compiler action.
def _xsd_compile_impl(ctx):
    xsd_file = ctx.file.xsd
    output_dir = ctx.actions.declare_directory(ctx.label.name)
    hxx_suffix = ctx.attr.hxx_suffix
    cxx_suffix = ctx.attr.cxx_suffix
    namespace = ctx.attr.namespace

    args = [
        "xsdcxx",
        "--generate-serialization", # add serialization
        "--hxx-suffix", hxx_suffix,
        "--cxx-suffix", cxx_suffix,
        "--namespace", namespace,
        "--output-dir", output_dir.path,
        "--std c++11",
        xsd_file.path,
    ]

    ctx.actions.run(
        outputs = [output_dir],
        executable = ctx.executable._xsdcxx,
        arguments = args,
        mnemonic = "XsdCompile",
    )

    # Collect generated files for use in cc_library.
    generated_hdrs = []
    generated_srcs = []

    for f in output_dir.files.to_list():
        if f.path.endswith(hxx_suffix):
            generated_hdrs.append(f)
        elif f.path.endswith(cxx_suffix):
            generated_srcs.append(f)

    return [
        DefaultInfo(
            files = depset(output_dir.files),
        ),
        OutputGroupInfo(
            hdrs = depset(generated_hdrs),
            srcs = depset(generated_srcs),
            output_dir = output_dir
        ),
    ]

xsd_compile = rule(
    implementation = _xsd_compile_impl,
    attrs = {
        "xsd": attr.label(allow_single_file = True, mandatory = True),
        "hxx_suffix": attr.string(default = ".hxx"),
        "cxx_suffix": attr.string(default = ".cxx"),
        "namespace": attr.string(mandatory = True),
        "_xsdcxx": attr.label(
            default = Label("/usr/bin/xsdcxx"), # Path to your xsdcxx binary
            executable = True,
            cfg = "host",
        ),
    },
    outputs = {"out": "."},
)


# Example usage:
xsd_compile(
    name = "roleassigner_tester_schema_generated",
    xsd = "../testdata/StrategyTester.xsd",
    namespace = "http://www.robotsports.nl=robotsports",
    hxx_suffix="_generated.h", # Example custom suffix
    cxx_suffix="_generated.cpp", # Example custom suffix
)

cc_library(
    name = "roleassigner_tester_schema_lib",
    srcs = [":roleassigner_tester_schema_generated"],
    hdrs = [":roleassigner_tester_schema_generated"],
    copts = ["-std=c++11"], # Important for C++11 support
    deps = [], # Add any dependencies of the generated code here.
    includes = [native.package_name()],
)

cc_binary(
    name = "xml_test",
    srcs = [
        "xml_tester++.cpp",
        "xmlRoleAssigner.cpp",
    ],
    includes = [
        ".",
        "//components/robotsports/role_assigner"
    ],
    deps = [
        "//components/robotsports/role_assigner:implementation",
        ":roleassigner_tester_schema_lib"
    ],
    copts = ["-std=c++17"],
)

# Dummy xsdcxx target (replace with actual build rule)
sh_binary(
    name = "xsdcxx",
    srcs = ["empty.sh"], # or build xsdcxx here
)

# empty dummy file
genrule(
    name = "empty_file",
    outs = ["empty.sh"],
    cmd = "touch $@",
)

