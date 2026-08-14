#!/usr/bin/env bash
# Personal shortcut: one hard-coded local recording, replayed through the
# generic rawlog wrapper. Nothing dataset-specific lives here.
exec mola-lo-gui-rawlog \
  /mnt/storage/ual-ouster-backpack/2023-11-23_citeIV_small_loop/2023-11-23_citeIV_small_loop.rawlog \
  "$@"
