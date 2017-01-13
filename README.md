orrt-star-ros

Install the following dependencies in Ubuntu 16.04

`sudo apt install libboost-all-dev libglfw3-dev mesa-common-dev freeglut3-dev`

This uses submodules, so you will need to clone them. There are two ways to do this.

If you already clone run.
```
git submodule init
git submodule update
```

If you haven't clone yet use the `--recursive` option when you clone
