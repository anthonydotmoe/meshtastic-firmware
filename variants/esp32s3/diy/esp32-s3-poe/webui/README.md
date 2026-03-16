Place WebUI assets for `esp32-s3-poe` here.

Supported inputs:
- `build.tar`
- `static/` extracted from `build.tar`

Example:
- `variants/esp32s3/diy/esp32-s3-poe/webui/build.tar`
- `variants/esp32s3/diy/esp32-s3-poe/webui/static/index.html.gz`

When you run `pio run -e esp32-s3-poe -t buildfs`, the variant script stages:
- the normal project `data/` contents
- plus this variant-local WebUI payload

The staged filesystem is written to `.pio/build/esp32-s3-poe/webui-data/`, so the shared top-level `data/` directory stays untouched.

Notes:
- `build.tar` contents are staged under `/static` for the firmware web handler.
- The bundled `mklittlefs` tool rejects basenames longer than 32 characters. The variant script automatically skips the few overlong `devices/*.svg.gz` files from the upstream WebUI tarball so image creation can complete.
- To flash firmware plus LittleFS/WebUI:
  - `pio run -e esp32-s3-poe -t upload`
  - `pio run -e esp32-s3-poe -t uploadfs`
