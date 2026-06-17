# omnivisu configuration

All runtime configuration lives in JSON files under `bin/data/`:

| File | Purpose |
| --- | --- |
| `config.json` | Application-wide settings: initial display mode, streaming target, mask layout. |
| `eye_left.json` | Settings for the **left** eye stream (`streams[0]`). Top-level key: `"left"`. |
| `eye_right.json` | Settings for the **right** eye stream (`streams[1]`). Top-level key: `"right"`. |

Files are read once at startup. Most per-eye values can also be edited live in the
GUI (press `g`); press `s` to save the GUI state back to the eye JSON, `r` to reload
from disk. The `fbo` and `camera` blocks are **startup-only** (not shown in the GUI)
because they only take effect when the stream is created.

## Keyboard controls

| Key | Action |
| --- | --- |
| `g` | Toggle the per-eye parameter GUI panels. |
| `m` | Toggle between mask overlay and side-by-side view. |
| `f` | Toggle the on-screen FPS readout. |
| `s` | Save all eye parameters back to their JSON files. |
| `r` | Reload all eye parameters from their JSON files. |

---

## `config.json`

```json
{
    "general":   { "display_mode": "mask" },
    "streaming": { "enabled": false, "target_ip": "127.0.0.1", "target_port": 12345 },
    "mask": {
        "mask_image": "260408-TGE-OMNI-FrantalView.png",
        "image_size": { "w": 4000, "h": 4000 },
        "anchor":     { "x": 2000, "y": 1500 },
        "left_eye":   { "x": 1030, "y": 1073, "w": 414, "h": 280 },
        "right_eye":  { "x": 2551, "y": 1071, "w": 414, "h": 280 }
    }
}
```

### `general`

| Key | Type | Default | Options / notes |
| --- | --- | --- | --- |
| `display_mode` | string | `"mask"` | `"mask"` = full-screen mask overlay with eye openings; `"side_by_side"` = both eye FBOs side by side. Sets the initial state of the `m` toggle. Any value other than `side_by_side` is treated as `mask`. |

### `streaming`

Reserved for upcoming eye-FBO network streaming. Parsed and stored today, but no
network code consumes it yet.

| Key | Type | Default | Notes |
| --- | --- | --- | --- |
| `enabled` | bool | `false` | Master on/off for streaming. |
| `target_ip` | string | `"127.0.0.1"` | Destination host. |
| `target_port` | int | `12345` | Destination port. |

### `mask`

Defines the full-screen mask PNG and the eye openings, all in image-pixel space.
The image is "cover"-fitted to the window (no black bars) and the `anchor` is mapped
to the window center.

| Key | Type | Default | Notes |
| --- | --- | --- | --- |
| `mask_image` | string | `"260408-TGE-OMNI-FrantalView.png"` | Path (relative to `bin/data/`) of the mask PNG with transparent eye holes. |
| `image_size.w` / `image_size.h` | int | actual PNG size | Fallback dimensions; the loaded PNG's real size wins. |
| `anchor.x` / `anchor.y` | float | image center | Image-space point mapped to the window center. |
| `left_eye` / `right_eye` | rect | see above | `{ x, y, w, h }` opening in image pixels. `left_eye` is filled by `streams[0]` (left), `right_eye` by `streams[1]` (right). |

> Note: per-eye `"stream"` indices were removed. Left/right are now positional —
> `eye_left.json` always drives the `left_eye` opening, `eye_right.json` the `right_eye`.

---

## `eye_left.json` / `eye_right.json`

Both files share the same structure; only the top-level key differs (`"left"` vs
`"right"`). Each contains six blocks.

```json
{
    "left": {
        "ofxIdsPeak": { ... },
        "tracking":   { ... },
        "grading":    { ... },
        "view":       { ... },
        "fbo":        { ... },
        "camera":     { ... }
    }
}
```

### `camera` (startup-only)

Binds this eye to a specific physical camera. On every launch the console logs each
detected device, e.g.:

```
ofxIdsPeak: device[0] serial='4108724653' model='U3-30CxCP-C' openable=yes
ofxIdsPeak: device[1] serial='4108724649' model='U3-30CxCP-C' openable=yes
```

Copy the serial (or model) you want into `value`.

| Key | Type | Default | Options / notes |
| --- | --- | --- | --- |
| `select_by` | string | `"serial"` | `"serial"` — match `value` exactly against a camera serial. `"model"` — match `value` as a substring of the model name (only useful if the two cameras are different models). `"index"` — `value` is the 0-based index among openable devices (`"0"`, `"1"`, ...). `"any"` — first openable device (ignores `value`). |
| `value` | string | `""` | The serial/model string, or the index as a string. Must be non-empty for `serial`/`model`, otherwise no device matches and the stream fails to start. |

> If both cameras are the same model, use `serial` (the only unique identifier).
> Which serial is "left" vs "right" is your physical choice — swap the two `value`s if
> the eyes appear reversed.

### `fbo` (startup-only)

Per-eye render-target (FBO) dimensions. Applied at allocation time.

| Key | Type | Default | Range |
| --- | --- | --- | --- |
| `width` | int | `672` | 1 – 8192 |
| `height` | int | `504` | 1 – 8192 |

### `view`

Controls how the camera image is drawn into the FBO.

| Key | Type | Default | Range | Notes |
| --- | --- | --- | --- | --- |
| `scale` | float | `2.0` | 0.1 – 10.0 | With `fit_to_fill: true`, a multiplier on the aspect-fit baseline (whole frame fits the FBO). With `fit_to_fill: false`, the exact camera-pixel → FBO-pixel factor (`1.0` = native resolution, 1:1). |
| `fit_to_fill` | bool | `true` | — | `true`: shrink the whole frame to fit the FBO, then apply `scale`. `false`: `scale` is the literal camera-to-FBO pixel factor. |
| `follow_eye` | bool | `false` | — | Recenter the FBO on the detected eye center. |
| `follow_smoothing` | float | `0.5` | 0.0 – 1.0 | Eye-follow low-pass; `0` = no smoothing, `1` ≈ 0.5 s time constant. |

### `tracking`

Haar eye detection and presence/smoothing behavior (runs on a worker thread).

| Key | Type | Default | Range | Notes |
| --- | --- | --- | --- | --- |
| `enable_eye_tracking` | bool | `true` | — | Master on/off for detection. |
| `show_debug_overlay` | bool | `true` | — | Draw the detection box, center cross, and worker stats. |
| `tracking_downscale` | int | `8` | 1 – 16 | Downsamples the worker's detection copy by this factor (display unaffected). Higher = faster, coarser. |
| `haar_min_width` | int | `400` | 8 – 1000 | Minimum eye box width, in **source** pixels. |
| `haar_min_height` | int | `250` | 8 – 1000 | Minimum eye box height, in **source** pixels. |
| `haar_scale` | float | `1.01` | 1.001 – 1.5 | Cascade scale step. Closer to 1 = more thorough, slower. |
| `haar_neighbors` | int | `5` | 0 – 20 | Cascade min-neighbors; higher = fewer false positives. |
| `present_on_frames` | int | `4` | 1 – 30 | Consecutive hits required to flip to PRESENT. |
| `present_off_frames` | int | `10` | 1 – 60 | Consecutive misses required to flip to LOST. |
| `euro_min_cutoff` | float | `0.5` | 0.01 – 5.0 | One Euro filter min cutoff (lower = smoother, more lag). |
| `euro_beta` | float | `0.007` | 0.0 – 0.1 | One Euro filter speed coefficient (higher = less lag on fast motion). |

### `grading`

Per-eye color grading applied in a shader when drawing into the FBO.

| Key | Type | Default | Range | Notes |
| --- | --- | --- | --- | --- |
| `enable_grading` | bool | `true` | — | Enable the grading shader (off = direct draw). |
| `exposure__stops_` | float | `0.0` | -2.0 – 2.0 | Exposure in stops (multiplies by `2^stops`). |
| `brightness` | float | `0.0` | -0.5 – 0.5 | Additive brightness offset. |
| `contrast` | float | `1.6` | 0.0 – 2.0 | Contrast around mid-gray. |
| `gamma` | float | `1.0` | 0.3 – 3.0 | Display gamma. |
| `saturation` | float | `1.0` | 0.0 – 2.0 | `0` = grayscale, `1` = unchanged, `>1` = boosted. |

### `ofxIdsPeak`

Camera hardware parameters passed to the IDS peak SDK. JSON keys are the escaped
parameter names (spaces/parentheses become underscores).

| Key | Type | Default | Range | Notes |
| --- | --- | --- | --- | --- |
| `exposure_auto` | bool | `true` | — | Auto-exposure on/off. |
| `exposure__us_` | float | `10000` | 50 – 100000 | Manual exposure time, microseconds. |
| `gain_auto` | bool | `true` | — | Auto-gain on/off. |
| `gain` | float | `1.0` | 1.0 – 16.0 | Manual analog/digital gain. |
| `gamma` | float | `1.0` | 0.3 – 3.0 | Camera gamma. |
| `wb_auto` | bool | `true` | — | Auto white balance on/off. |
| `wb_red` | float | `1.0` | 0.0 – 8.0 | Manual WB red gain. |
| `wb_green` | float | `1.0` | 0.0 – 8.0 | Manual WB green gain. |
| `wb_blue` | float | `1.0` | 0.0 – 8.0 | Manual WB blue gain. |
| `fps_cap` | float | `50.0` | 1.0 – 120.0 | Frame-rate cap. |
| `black_level` | float | `0.0` | -32.0 – 255.0 | Sensor black level offset. |
| `reverse_x` | bool | `false` | — | Horizontal sensor flip. |
| `reverse_y` | bool | `false` | — | Vertical sensor flip. |
| `saturation` | float | `1.0` | 0.0 – 2.0 | Camera saturation (if supported). |
| `sharpness` | float | `0.0` | 0.0 – 4.0 | Camera sharpness (if supported). |
| `exposure_auto_upper__us_` | float | `100000` | 50 – 1000000 | Auto-exposure upper bound, microseconds. |
| `exposure_auto_lower__us_` | float | `50` | 10 – 100000 | Auto-exposure lower bound, microseconds. |
| `gain_auto_upper` | float | `16.0` | 1.0 – 64.0 | Auto-gain upper bound. |
| `gain_auto_lower` | float | `1.0` | 1.0 – 32.0 | Auto-gain lower bound. |
| `processing_mode` | int | `0` | 0 – 1 | `0` = ProcessAll (every frame), `1` = LatestOnly (drop backlog, lowest latency). |
| `output_format` | int | `0` | 0 – 2 | `0` = RGBA, `1` = RGB, `2` = Mono. The app requests RGB at startup. |

> Not every camera exposes every node; unsupported parameters are trimmed from the
> GUI per device and ignored on load.
