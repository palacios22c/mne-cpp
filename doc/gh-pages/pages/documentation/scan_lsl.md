---
title: LSL
parent: MNE Scan
grand_parent: Documentation
nav_order: 12
---
# Lab Streaming Layer (LSL)

This plugin adds support for LSL data streams to MNE Scan. MNE-CPP includes a self-contained LSL library under `src/libraries/lsl` that implements the core LSL protocol (stream discovery via UDP multicast, data transport via TCP), eliminating the need for any external dependency.

For more information about the original LSL project please see:

* [LSL on Github](https://github.com/sccn/labstreaminglayer){:target="_blank" rel="noopener"}

## Building the LSL Plugin

The LSL plugin is an optional component controlled by the CMake option `WITH_LSL`. To enable it, configure with:

```
cmake -B build -S src -DWITH_LSL=ON
cmake --build build
```

No external submodule or library is required — the built-in `mne_lsl` library provides all necessary LSL functionality.

## LSL Plugin Setup

* Make sure the `WITH_LSL` CMake option is enabled when configuring the build.
* Build MNE Scan.
* Start MNE Scan — the LSL adapter plugin will be available in the sensor plugins list.
