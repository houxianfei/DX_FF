# FF GUI Interaction Requirements

This document records the product interaction rules for the FF GUI and is part of the project baseline.

## Connection flow

- The application starts on a dedicated Welcome / Connection Center screen.
- Welcome is responsible only for selecting the connection mode, selecting serial ports, loading the configuration, and opening the FF connection.
- A successful connection transitions to the Main UI. A failed connection remains on Welcome and shows the error.
- Disconnecting from Main UI returns to Welcome.

## Connection settings

- `Mode` is a drop-down with `Simulation` and `Real Hardware`.
- `Enabled Channels` is a drop-down for 1, 2, or 3 active channels. Only enabled channels have serial-port rows and physical serial connections.
- Each of CH1, CH2 and CH3 has an independent serial-port drop-down populated by scanning the COM ports currently enumerated by Windows. `Scan Serial Ports` refreshes the list.
- Real Hardware requires three distinct, detected COM ports before connection can start. Simulation retains the saved port selections but does not open them.
- Mode and serial-port selections are persisted in `%PROGRAMDATA%\RSIC\FF\ff_gui.ini`.
- Settings are restored on the next launch and are applied before connecting.

## Main UI separation

- Main UI contains normal operations, status monitoring, channel selection, target editing, planning and emergency stop.
- Rod positions are displayed and edited as `optical_position`. The first-release conversion is `optical_position = P0 + P1 * u`, where `u` is the encoder position and `P0`/`P1` are per-Rod JSON configuration values. Legacy `stroke_bias` and `stroke_slope` fields remain readable as P0/P1.
- `rod_layers.csv` uses one row for each `(channel_id, rod_id, layer)` membership. `layer` is an ordinary integer, never a bitmask. A Rod that belongs to two layers has two rows, with layer-specific `role`, `left_neighbor`, and `right_neighbor` values.
- Roles are fixed: `0` middle (both neighbors present), `1` left end (right neighbor only), `2` right end (left neighbor only), and `3` isolated (no neighbors). Neighbor links must be reciprocal inside the same layer.
- Every neighboring pair requires a `pair_clearance_rules.csv` record with the same `layer`. Current state, safe phase, and target phase are checked for every layer in which a Rod appears.

For example, if `CH1/Rod13` belongs to layers 1 and 4, it has two records. In layer 1 it can use `left_neighbor=0`, while in layer 4 it can use `left_neighbor=12`. If layer 4 starts at `Rod2`, its `role` is `1` (left end), not `0`.
- Configuration and parameter editing are separate from the Welcome connection screen and must not be mixed into connection controls.
- Channels are switchable through tabs; the UI must not force all channel controls into one combined panel.

## Language

- All user-visible GUI text is English. Fixed identifiers such as `CH1`, `Rod`, `COM` and error codes remain unchanged.
