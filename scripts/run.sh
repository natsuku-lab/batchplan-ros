#!/usr/bin/env bash
# Runs a command in the built workspace. pixi activates an environment before a task's build
# dependency runs, so a fresh workspace's overlay is sourced here rather than at activation.
. "$PIXI_PROJECT_ROOT/install/setup.bash"
exec "$@"
