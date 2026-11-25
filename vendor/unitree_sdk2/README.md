# G1 deployment SDK subset

This copy contains Unitree SDK2 headers and prebuilt libraries needed by the G1
controllers in this project, plus the original G1 C++ examples. Non-G1 examples
and unused robot clients have been removed. The BSD license is in `LICENSE`;
third-party notices are retained under `licenses/`.

Some Go2 DDS wrappers, Go2 JSON definitions and the B2 motion switcher remain
because the original G1 controller and G1 official clients include them.
Common IDL messages and x86_64/aarch64 libraries are retained. These are SDK
compatibility dependencies, not policies or simulation models for other robots.

The deployment build disables SDK examples. Build the project using the root
`build_cpp.sh`; the complete upstream SDK is available in the original source
project recorded in `CPP_BASELINE.json`.
