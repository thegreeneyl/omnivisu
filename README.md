# omnivisu

An openFrameworks application for live IDS U3-30C0CP camera capture with Haar-based eye detection, pupil blob analysis, and a normalized per-eye FBO. Uses the [`ofxIdsPeak`](https://github.com/thegreeneyl/ofxIdsPeak) addon.

- **openFrameworks:** 0.12.1 (Linux x86_64, gcc 6 release build) — https://openframeworks.cc/download/
- **Platform:** Linux 64-bit (tested on the `of_v0.12.1_linux64_gcc6_release` distribution)
- **Hardware:** IDS U3-30C0CP (or any GenICam camera supported by IDS peak)

> This repository only contains the **app** (`src/`, build files, `bin/data/`). openFrameworks itself is **not** vendored — you install OF separately and clone this repo into its `apps/myApps/` folder.

## Install

1. Download and install openFrameworks **0.12.1** for Linux 64-bit (gcc 6 release) from <https://openframeworks.cc/download/>. Unpack so you have an `of_v0.12.1_linux64_gcc6_release/` directory.
2. From OF's `scripts/linux/<distro>/`, run the dependency installers (`install_dependencies.sh`, `install_codecs.sh`).
3. Install the [IDS peak SDK](https://en.ids-imaging.com/download-peak.html) (Linux package, includes `libids_peak` and `libids_peak_ipl`).
4. Clone the `ofxIdsPeak` addon into OF's `addons/` folder:

   ```bash
   cd of_v0.12.1_linux64_gcc6_release/addons
   git clone https://github.com/thegreeneyl/ofxIdsPeak.git
   ```

5. Clone this app into OF's `apps/myApps/` folder:

   ```bash
   cd ../apps/myApps
   git clone https://github.com/thegreeneyl/omnivisu.git
   ```

You should now have:

```
of_v0.12.1_linux64_gcc6_release/
├── addons/
│   └── ofxIdsPeak/
└── apps/
    └── myApps/
        └── omnivisu/   <- this repo
```

The build expects this layout because `Makefile` sets `OF_ROOT=../../..` by default.

## Build and run

From this directory on the Linux machine:

```bash
./run.sh
```

`run.sh` exports `GENICAM_GENTL64_PATH` (default `/usr/local/lib/x86_64-linux-gnu/ids-peak/cti`), builds `Release`, and runs the app.

If you start the binary without `run.sh` (for example `make RunRelease`), `ofxIdsPeak` still tries to set `GENICAM_GENTL64_PATH` automatically on Linux when it is missing, by scanning common IDS peak CTI directories. You can always set it yourself:

```bash
export GENICAM_GENTL64_PATH=/path/to/your/ids-peak/cti
```

Manual build:

```bash
export GENICAM_GENTL64_PATH=/usr/local/lib/x86_64-linux-gnu/ids-peak/cti
make Release -j$(nproc)
make RunRelease
```

## Controls

- `g` — toggle the optional IDS camera parameter panel (`ofxGui`)
- Console logs app FPS and camera frame/error counters about once per second

## Eye streams

Each `EyeCameraStream` owns one IDS camera, runs Haar eye detection + pupil blob analysis (OpenCV), smooths results with a One Euro filter, and renders a normalized **672×504** FBO (eye centered, leveled, fixed scale).

Press `g` to tune detection parameters. Cascade file: `bin/data/haarcascade_eye.xml` (shipped in this repo).

**Follow-up:** `ofxIdsPeak::Grabber::setup()` currently opens the first available device only. To run two eyes simultaneously, extend the grabber with a device index or serial so each `EyeCameraStream` can bind to a specific camera.

## Project layout

- `src/` — application source (`main.cpp`, `ofApp`, `EyeCameraStream`, `OneEuroFilter`)
- `addons.make` — `ofxIdsPeak`, `ofxGui`, `ofxOpenCv`
- `Makefile`, `config.make` — standard OF project makefiles (expect `OF_ROOT=../../..`)
- `run.sh` — Linux launcher (sets GenTL path, builds, runs)
- `bin/data/` — runtime assets (Haar cascade XML)
- `.vscode/` — VSCode build/debug tasks

## Dependencies

- openFrameworks 0.12.1 addons (bundled with OF): `ofxGui`, `ofxOpenCv`
- External addon: [`ofxIdsPeak`](https://github.com/thegreeneyl/ofxIdsPeak)
- IDS peak genericSDK (C++): `libids_peak`, `libids_peak_ipl`
- OpenCV (used via `ofxOpenCv`)

If header discovery fails, locate the IDS peak headers on the Linux machine with:

```bash
find /usr/include /usr/local/include /opt -path '*/peak/peak.hpp' 2>/dev/null
```

then set `IDS_PEAK_ROOT` in `config.make` (see the commented block at the bottom).

## License

MIT — see [LICENSE](LICENSE).
