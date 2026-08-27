# Third-party notices

This repository contains **benchmark scripts, input decks, slim results, and documentation** produced during an internship evaluation of spacecraft simulation tools. It does **not** redistribute the simulation tools themselves.

Original benchmark code and documentation in this repository are MIT-licensed (see `LICENSE`), except where a copied file retains an upstream header.

## Tools you must install yourself

| Tool | Typical source | Notes |
|------|----------------|-------|
| GMAT | NASA / OpenGMAT | Mission scripts (`.script`) are included; the GMAT binary is not. |
| Basilisk | AVS Lab, CU Boulder (ISC) | Custom scenarios keep the upstream ISC header where they were derived from examples. |
| NASA 42 | NASA GSFC | Custom `Inp_*` / `SC_*` / `Orb_*` decks only. Overlay them on a local 42 install. |
| Orekit | CS Group (Apache 2.0) | Python runners included; Orekit data files are not. |
| FreeFlyer | a.i. solutions (commercial) | Mission plans / FreeForm snippets only. Do not publish the FreeFlyer install. |
| BSK-RL | AVS Lab | `rso_inspection.py` is the training entry point used here. |
| Space ROS | NASA | Canadarm2 demo was **run**, not forked. Screenshots and notes only. |
| ROS 2 / Acados | Open Robotics / Acados | PID vs MPC comparison scripts assume a local `bsk-ros2-bridge` workspace. |
| MuJoCo | Google DeepMind | Used through Basilisk's MuJoCo module. |

## What was intentionally left out

- Full GMAT / Basilisk / NASA 42 / Isaac Sim / Space ROS source trees
- Vizard `.bin` recordings and other multi-gigabyte state histories
- SPICE kernels (`de440s.bsp`, etc.)
- ROS 2 bag files
- Python virtualenvs and `node_modules`
