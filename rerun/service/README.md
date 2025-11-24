# Rerun Service - `service` folder documentation

This folder contains scripts, binaries, and configuration files for connectivity management, file transfer, and automated checks in the Forerunner environment.

## Main files

### 1. `install_service.sh`
Script for installing and configuring the MinIO client (`mc`).
- Checks that the environment variables `MINIO_ACCESS_KEY` and `MINIO_SECRET_KEY` are set.
- Downloads and installs the `mc` binary.
- Configures the MinIO alias for the specified server.

### 2. `comprobation.cpp` and `comprobation.hpp`
C++ implementation for connectivity and registry process checking.
- `ConnectivityProbe` checks internet access (default to 8.8.8.8:53) and whether the `registry` process is running.
- Can run in "watch" mode for continuous monitoring.
- Outputs in key-value format: `internet=true/false registry=true/false`.

### 3. `comprobation_check.sh`
Script to check connectivity conditions and transfer files.
- Checks internet connectivity and the presence of the `registry` process using a probe binary.
- If conditions are met, transfers files from a local directory to the MinIO server.
- Deletes transferred files and cleans up empty directories.
- Cancels transfers if conditions change during the process.

### 4. `rerun-comprobation.service`
Systemd unit file.
- Runs the connectivity probe binary (`rerun_comprobation`) as a one-shot service.
- Allows setting the binary location via the `COMPROBATION_BIN` environment variable.

### 5. `rerun-comprobation.timer`
Systemd timer file.
- Schedules periodic execution of the probe service.
- Runs at system boot and then every 60 seconds.

## Typical usage flow
1. Install and configure the MinIO client with `install_service.sh`.
2. The probe binary (`rerun_comprobation`) checks connectivity and registry process status.
3. `comprobation_check.sh` uses the probe to decide whether to transfer files to MinIO.
4. Systemd services and timers (`.service` and `.timer`) automate periodic checks.
