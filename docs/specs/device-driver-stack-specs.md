# Device Driver Stack — EARS Specs

Prefix: **DDS**

## Driver Interface Contract

- [x] **DDS-API-001**: The system shall allow drivers to register with the context via a `REGISTER_LINKTIME` constructor attribute, requiring no central registration file.
- [x] **DDS-API-002**: The system shall provide each registered driver with a `SurviveContext*` during initialization and record a poll function and close function per driver.
- [x] **DDS-API-003**: The system shall call each driver's poll function on every iteration of the main event loop.
- [x] **DDS-API-004**: When a driver produces a light pulse, the system shall deliver it as a `LightcapElement` (sensor_id, timecode, length) via the `lightcap` hook.
- [x] **DDS-API-005**: When a driver produces an IMU reading, the system shall deliver accelerometer, gyroscope, and magnetometer data via the `raw_imu` hook.
- [x] **DDS-API-006**: When a driver produces a device configuration blob, the system shall deliver the JSON string via the `config` hook, triggering `SurviveObject` creation.

## Vive USB Driver

- [x] **DDS-BE-010**: The system shall support both libusb (Linux/macOS) and HIDAPI (Windows/fallback) USB backends, selected at compile time with a unified transfer abstraction.
- [x] **DDS-BE-011**: When a Vive-family USB device is detected, the system shall execute the initialization sequence: MAGICS → CONFIG → VERSION → IMU_SCALES → ATTACH_INTERFACES.
- [x] **DDS-BE-012**: When downloading device configuration, the system shall read the JSON blob in 256-byte feature report chunks and gzip-decompress the result.
- [x] **DDS-BE-013**: When decompressing device configuration, the system shall add 2 bytes to the advertised config length to account for the known off-by-two quirk in Vive firmware.
- [x] **DDS-BE-014**: The system shall support Lighthouse Gen1 (raw0), Gen2 (raw1), and reserved (raw2) lightcap modes, selected via USB magic packet sequence during initialization.
- [x] **DDS-BE-015**: When three consecutive USB transfer timeouts occur on a non-HMD device, the system shall treat the device as disconnected.
- [x] **DDS-BE-016**: The system shall demultiplex wireless watchman packets (report 0x23/0x24), delivering IMU and lightcap events per-device from a single RF dongle endpoint.
- [x] **DDS-BE-017**: Where libusb is the active backend, the system shall register for hotplug events and initialize newly-connected Vive devices without requiring restart.

## GATT Lighthouse Control

- [x] **DDS-BE-020**: When GATT support is enabled, the system shall scan for Bluetooth LE devices with name prefix "LHB" and connect to each discovered lighthouse.
- [x] **DDS-BE-021**: When a sleeping lighthouse is detected via GATT, the system shall wake it by writing to the POWER characteristic before tracking begins.
- [x] **DDS-BE-022**: When two lighthouses are assigned the same channel, the system shall detect the conflict and reassign one to an unused slot.
- [x] **DDS-BE-023**: Where the `gatt-sleep-at-exit` config flag is set, the system shall put all connected lighthouses to sleep when the session closes.

## Playback Driver

- [x] **DDS-BE-030**: When a playback file path is configured, the system shall replay the recording by re-injecting all recorded events through the same hooks a live driver would use.
- [x] **DDS-BE-031**: The system shall replay events at a configurable time scale (`playback-factor`, default 1.0×) by sleeping between events proportional to their timestamps.
- [x] **DDS-BE-032**: The system shall support replaying a subsection of a recording via `playback-start-time` and `playback-end-time` config keys.
- [x] **DDS-BE-033**: When a playback recording contains both raw light and raw IMU data, the system shall suppress raw light events and use only raw IMU to avoid double-processing.
- [x] **DDS-BE-034**: The system shall parse both 11-element (legacy, no magnetometer) and 12-element IMU lines in playback recordings for backward compatibility.

## UDP Driver

- [x] **DDS-BE-040**: When UDP driver is enabled, the system shall join multicast group 224.0.2.122 on port 2333 and receive pose and config data from external tracking systems.
- [x] **DDS-BE-041**: When a UDP config packet (command word 1) is received, the system shall fire the `config` hook to create a tracked object.
- [x] **DDS-BE-042**: When a UDP pose packet (command word 2) is received, the system shall fire the `pose` hook with the received position and quaternion.

## Global Scene Solver

- [x] **DDS-BE-050**: When sufficient sensor activation coverage is accumulated across all tracked objects (per the `gss-desired-coverage` threshold), the system shall trigger a global bundle adjustment solve.
- [x] **DDS-BE-051**: While a tracked object is in motion (detected via IMU), the system shall not accumulate measurements for global scene solving.
- [x] **DDS-BE-052**: Where the `gss-auto-floor-height` config flag is set, the system shall set the floor Z coordinate from the minimum observed tracked object height.
- [x] **DDS-BE-053**: Where the `disable-calibrate` config flag is set, the system shall skip all global scene solver activity.
- [ ] **DDS-BE-054**: If the global scene solver produces a result with error above a configurable threshold, the system shall reject the result and retain the previous calibration.
