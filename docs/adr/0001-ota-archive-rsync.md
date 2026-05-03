# ADR 0001: Archive-based OTA updates with rsyncable compression

- **Status:** Accepted
- **Date:** 2026-05-03

## Context

The rover deploy path ships OCI images and ESP32 firmware to a Raspberry Pi over Wi-Fi that can be slow or unstable. We want OTA updates to be resumable, efficient across releases, and simple to activate and roll back on-device.

The current runtime flow already deploys release bundles into stable paths:

- `~/grassclippr/image-cache/` for image archives
- `~/grassclippr/releases/<release-id>/` for manifests and start scripts
- `current` / `previous` symlinks for activation and rollback

## Decision

We will keep the OTA runtime update flow archive-based:

1. Build release-tagged Podman images locally.
2. Save each image as its own archive.
3. Compress archives with `zstd --rsyncable`.
4. Transfer them with `rsync --partial --partial-dir=.rsync-partial --delete`.
5. Load archives on the Pi with `podman load`.
6. Activate the new release by updating the `current` symlink and running `remote-start.sh`.

We explicitly do **not** use `rsync --append-verify` for runtime archives, because deploys are usually transferring a new version of an archive rather than resuming the exact same file.

We also keep the existing split-image layout (`core`, `serial_mux_proxy`, optional agent) instead of switching to a blob-directory transport such as `skopeo copy ... dir:`.

## Consequences

### Positive

- OTA transfers remain resumable on weak links.
- Rsync can delta against stable archive paths between releases.
- The deploy and rollback model stays compatible with the existing Pi seed and release layout.
- Operational complexity stays low: the Pi only needs `rsync`, `zstd`, `podman`, and the existing release scripts.

### Negative

- `podman load` still reloads whole archives on the Pi.
- Transfer efficiency depends on image layering and archive churn; large early-layer changes still cost more than a blob-aware layout would.

## Follow-up

Keep frequently changing application content in later Docker layers so version-to-version archive deltas stay smaller.
