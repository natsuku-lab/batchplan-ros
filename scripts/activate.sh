# Puts cargo-ament-build on PATH for colcon, and overlays the environment's workspace once it is
# built.
export PATH="$PIXI_PROJECT_ROOT/.pixi/cargo/bin:$PATH"
if [ -f "$PIXI_PROJECT_ROOT/install/$PIXI_ENVIRONMENT_NAME/setup.sh" ]; then
  . "$PIXI_PROJECT_ROOT/install/$PIXI_ENVIRONMENT_NAME/setup.sh"
fi
