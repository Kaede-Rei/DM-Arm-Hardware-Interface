# dm_damiao_adapter

`dm_damiao_adapter` contains the Damiao-specific motor bus implementation.

This package converts `impedance_controller` joint-side command/state types to the
Damiao SDK types exposed by `dm_hw`. Keeping it outside `impedance_controller` lets
the control and dynamics library remain usable in simulation, Python bindings,
and other non-Damiao runtimes.
