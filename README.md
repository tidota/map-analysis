# Map Analysis

This repository contains code to evaluate octree maps generated in [the simulation](https://github.com/tidota/mav-tunnel-nav).

# Build

```
mkdir build
cd build
cmake ..
make
```

# Usage

In the directory where the executable exists, copy the .bt files.

Edit the yaml file. The first entry must be the ground truth map.
A map can be composed of multiple .bt files. List the associated .bt files under the map name.

Then, run the executable and it will show the results.

