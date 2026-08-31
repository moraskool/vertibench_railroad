# Installing VertiBench (full build, WSL Ubuntu 24.04)

A working, step-by-step recipe for installing [VertiBench](https://github.com/RobotiXX/verti_bench)
with **full functionality** (all vehicles, all controllers, and the `sensor` module that VertiBench's
run path imports). Written for **WSL2 running Ubuntu 24.04** with an **NVIDIA RTX 3050 (6 GB)**, but the
gotchas apply to any recent Ubuntu.

The upstream guide assumes Ubuntu 22.04 + ROS2 Humble. This document is the 24.04 version: it stays on
24.04, substitutes the Jazzy ROS message packages, and works around every version mismatch we hit.

> Substitute for your machine: `<WINUSER>` = your Windows username (ours was `codebender`),
> `<GPU_ARCH>` = your GPU's CUDA architecture (RTX 3050 / Ampere = `86`; look yours up if different).

---

## 0. Why this build (the two decisions)

- **Full functionality needs the source build**, not the prebuilt conda package (which is HMMWV-only).
- **VertiBench's run path imports `pychrono.sensor`** (GPS/IMU/camera in `rl/off_road_VertiBench.py`), so
  the `sensor` module is required even for a headless PID run. That means **OptiX + CUDA are mandatory**.
  (If you ever only need physics and are willing to patch VertiBench, you can skip sensors, but the harness
  is sensor-coupled, so building the module is the robust path.)

Everything below installs into a dedicated conda env named `chrono9` (Python 3.9). We never touch the
system Python. If a project virtualenv (e.g. a `uv` `.venv`) auto-activates and shadows `python`, use the
env's interpreter by full path: `~/miniconda3/envs/chrono9/bin/python` (aliased to `vpy` at the end).

---

## 1. System packages + CUDA Toolkit 11.8

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y build-essential gcc-11 g++-11 git git-lfs swig libopenmpi-dev openmpi-bin libirrlicht-dev
sudo snap install cmake --classic
# ROS2 message packages VertiBench actually pulls (Jazzy equivalents on 24.04):
sudo apt install -y ros-jazzy-grid-map-msgs ros-jazzy-geometry-msgs
```

**Why `gcc-11`:** CUDA 11.8's `nvcc` does not support gcc 13 (the 24.04 default). We point the CUDA build
at gcc-11 later.

Install CUDA **Toolkit** 11.8 (inside WSL you install only the toolkit, never the driver, the driver lives
on Windows):

```bash
cd ~
wget https://developer.download.nvidia.com/compute/cuda/repos/wsl-ubuntu/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt update
sudo apt install -y cuda-toolkit-11-8
```

> **Gotcha (24.04):** `cuda-toolkit-11-8` pulls `nsight-systems`, which depends on `libtinfo5`, dropped in
> 24.04. Install the old lib from 22.04 first, then re-run the toolkit install:
> ```bash
> wget http://archive.ubuntu.com/ubuntu/pool/universe/n/ncurses/libtinfo5_6.3-2ubuntu0.1_amd64.deb
> sudo dpkg -i libtinfo5_6.3-2ubuntu0.1_amd64.deb
> sudo apt install -y cuda-toolkit-11-8
> ```

Put CUDA 11.8 on the path and verify:

```bash
echo 'export PATH=/usr/local/cuda-11.8/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda-11.8/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc
nvcc --version    # must report release 11.8
nvidia-smi        # driver visible; "CUDA Version" here is the driver max, not the toolkit
```

---

## 2. Miniconda + the `chrono9` environment

```bash
cd ~
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash Miniconda3-latest-Linux-x86_64.sh -b -p ~/miniconda3
~/miniconda3/bin/conda init bash
source ~/.bashrc
```

> **Gotcha:** newer conda gates the default channels behind a Terms-of-Service accept. If `conda create`
> errors with `CondaToSNonInteractiveError`, run:
> ```bash
> conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main
> conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r
> ```

Create the env and **add pip explicitly** (a fresh env may not include it):

```bash
conda create -n chrono9 python=3.9 -y
conda install -n chrono9 -c conda-forge pip -y
PY=~/miniconda3/envs/chrono9/bin/python     # use this full path if a venv shadows `python`
$PY -m pip --version                         # confirm pip lives in chrono9
```

Install the Python deps. **Order matters** and **use these exact pins**:

```bash
# torch FIRST (so stable-baselines3 can't drag in a mismatched build), pinned to cu118:
$PY -m pip install torch==2.7.1 torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
$PY -m pip install gymnasium "stable-baselines3[extra]"
$PY -m pip install pyyaml scipy evdev
# numpy pinned LAST so nothing bumps it (Chrono requires numpy 1.x):
$PY -m pip uninstall -y numpy
conda install -n chrono9 -c conda-forge "numpy=1.24.0" mkl=2020 -y

$PY -c "import numpy, torch; print(numpy.__version__, torch.__version__, torch.cuda.is_available())"
# want: 1.24.0  2.7.1+cu118  True
```

> **Why the pins:** `numpy 1.24.0` is a hard Chrono requirement (its bindings are built against numpy 1.x).
> Modern torch needs numpy 2.x, so we use an older cu118 torch that is happy with numpy 1.24. On Python 3.9,
> numpy 1.24 installs from a wheel; on 3.13 it fails to build.

---

## 3. OptiX SDK 7.7 (for the sensor module)

Manual download, behind an NVIDIA developer login (no command downloads it):

1. Go to <https://developer.nvidia.com/designworks/optix/downloads/legacy>, sign in (free account).
2. Download **OptiX SDK 7.7.0, Linux 64-bit** → `NVIDIA-OptiX-SDK-7.7.0-linux64-x86_64.sh`
   (lands in `C:\Users\<WINUSER>\Downloads`).

Extract it in WSL (self-extracting script, no root):

```bash
mkdir -p ~/Documents/optix
bash /mnt/c/Users/<WINUSER>/Downloads/NVIDIA-OptiX-SDK-7.7.0-linux64-x86_64.sh \
  --skip-license --prefix=$HOME/Documents/optix
ls ~/Documents/optix/include/optix.h    # must succeed
```

---

## 4. Give WSL enough memory (before building Chrono)

Chrono's FEA and CUDA files are RAM-hungry; the default ~4 GB WSL allocation gets OOM-killed. On **Windows**,
create `C:\Users\<WINUSER>\.wslconfig`:

```
[wsl2]
memory=6GB
swap=20GB
```

Then in **Windows PowerShell**: `wsl --shutdown`, reopen Ubuntu, and check `free -h` (want ~6 GB mem, 20 GB
swap). The big swap is what lets the heaviest compiles finish.

---

## 5. Build Project Chrono 9.0.1 from source

Clone the VertiBench-compatible fork and build its bundled dependencies (Eigen, GL, URDF):

```bash
git clone -b 901 https://github.com/madhan001/chrono.git ~/Documents/chrono
cd ~/Documents/chrono && git submodule update --init --recursive
cd ~/Documents/chrono/contrib/build-scripts/linux
chmod +x buildEigen.sh buildGL.sh buildURDF.sh
./buildEigen.sh && ./buildGL.sh && ./buildURDF.sh
```

The Multicore module needs the header-only **Blaze** library (the Bitbucket release tarball 404s, clone the
repo instead):

```bash
git clone https://bitbucket.org/blaze-lib/blaze.git ~/Documents/blaze
ls ~/Documents/blaze/blaze/system/Version.h    # confirm
```

Configure. **Note the option names are `ENABLE_MODULE_*` (no `CH_` prefix)** for this fork:

```bash
NUMPY_INC=$(~/miniconda3/envs/chrono9/bin/python -c "import numpy; print(numpy.get_include())")

mkdir -p ~/Documents/chrono_build && cd ~/Documents/chrono_build
cmake -G "Unix Makefiles" \
  -DCMAKE_BUILD_TYPE=Release \
  -DENABLE_MODULE_VEHICLE=ON \
  -DENABLE_MODULE_IRRLICHT=ON \
  -DENABLE_MODULE_PYTHON=ON \
  -DENABLE_MODULE_PARSERS=ON \
  -DENABLE_MODULE_MULTICORE=ON \
  -DENABLE_MODULE_SENSOR=ON \
  -DENABLE_MODULE_GPU=ON \
  -DBLAZE_INSTALL_DIR=$HOME/Documents/blaze \
  -DOptiX_INSTALL_DIR=$HOME/Documents/optix \
  -DCMAKE_CUDA_ARCHITECTURES=<GPU_ARCH> \
  -DCMAKE_CUDA_HOST_COMPILER=/usr/bin/gcc-11 \
  -DNUMPY_INCLUDE_DIR=$NUMPY_INC \
  -DPython3_EXECUTABLE=$HOME/miniconda3/envs/chrono9/bin/python \
  ~/Documents/chrono 2>&1 | tee ~/chrono_cmake.log
```

Verify before building:

```bash
grep -iE "sensor module|optix|cuda toolkit|sensor module will not be built|python SENSOR" ~/chrono_cmake.log
```

> **Gotcha:** if you see `The PyChrono sensor module will not be built!`, `NUMPY_INCLUDE_DIR` wasn't set,
> re-run cmake with `-DNUMPY_INCLUDE_DIR=$NUMPY_INC`. You want to see "add python SENSOR module" with the
> numpy include directory printed, and no "will not be built" warning.

Build (keep `-j2`, these translation units are memory-heavy; drop to `-j1` if anything gets `Killed`):

```bash
make -j2 2>&1 | tee ~/chrono_make.log
```

---

## 6. Environment variables + verify pychrono

Persist the runtime paths so `import pychrono` works from any shell:

```bash
cat >> ~/.bashrc <<'EOF'

# Chrono / VertiBench
export LD_LIBRARY_PATH=$HOME/Documents/chrono/libraries/urdf/lib:$HOME/Documents/chrono_build/bin:$LD_LIBRARY_PATH
export PYTHONPATH=$HOME/Documents/chrono_build/bin:$HOME/Documents:$PYTHONPATH
export CHRONO_DATA_DIR=$HOME/Documents/verti_bench/envs/data/
alias vpy="$HOME/miniconda3/envs/chrono9/bin/python"
EOF
source ~/.bashrc

vpy -c "import pychrono, pychrono.vehicle, pychrono.irrlicht, pychrono.parsers, pychrono.sensor; print('all modules OK')"
```

> **Gotcha (OpenMP clash):** if an import segfaults, conda's `libgomp` is shadowing the system one. Prefix
> with `LD_PRELOAD=/usr/lib/gcc/x86_64-linux-gnu/13/libgomp.so.1` and retry.

---

## 7. Clone VertiBench + pull assets

The repo + LFS assets are large; a flaky network drops a full clone. Clone shallow **without** LFS blobs
first, then pull the assets separately (resumable):

```bash
git config --global http.postBuffer 524288000
git config --global core.compression 0
GIT_LFS_SKIP_SMUDGE=1 git clone --depth 1 https://github.com/RobotiXX/verti_bench.git ~/Documents/verti_bench
cd ~/Documents/verti_bench
git lfs install
git lfs pull        # re-run if it drops; it resumes
du -sh ~/Documents/verti_bench   # real maps make this multiple GB (not a few MB of LFS pointers)
```

> **Gotcha:** if even the shallow clone dies with a GnuTLS decryption error, it's usually a WSL2 MTU issue:
> `sudo ip link set dev eth0 mtu 1400`, then retry.

---

## 8. Run the demo

Headless first (no display, pure physics), one HMMWV under a PID controller on world 1:

```bash
cd ~/Documents/verti_bench
vpy setup.py vehicle=hmmwv system=pid speed=4.0 world_id=1 scale_factor=1.0 \
  max_time=30 num_experiments=1 render=false use_gui=false
```

If it runs and prints a result (success/fail, time, distance), the install is validated.

---

## Troubleshooting quick reference

| Symptom | Cause | Fix |
|---|---|---|
| `cuda-toolkit-11-8` unmet dep `libtinfo5` | 24.04 dropped libtinfo5 | install libtinfo5 from 22.04 (§1) |
| `CondaToSNonInteractiveError` | conda channel ToS | `conda tos accept ...` (§2) |
| `externally-managed-environment` | using system pip, not the env's | use `$PY -m pip` / activate chrono9 |
| `No module named pip` in env | env created without pip | `conda install -n chrono9 pip` |
| torch stays `+cu128`, numpy stays `2.x` | sb3 pre-installed torch; wrong env | install torch first, pin numpy last, use `$PY` |
| `CH_ENABLE_MODULE_* not used` | wrong option prefix | use `ENABLE_MODULE_*` (no `CH_`) |
| `Cannot find blaze/system/Version.h` | Blaze missing | clone Blaze, set `BLAZE_INSTALL_DIR` |
| `Killed signal ... cc1plus` | OOM | raise WSL memory/swap (§4), use `-j2`/`-j1` |
| `PyChrono sensor module will not be built` | numpy include unset | `-DNUMPY_INCLUDE_DIR=...` (§5) |
| `No module named pychrono.sensor` | sensor module not built | build with `ENABLE_MODULE_SENSOR/GPU=ON` + OptiX |
| clone `GnuTLS decryption` / early EOF | large transfer / WSL MTU | shallow+skip-smudge clone; lower MTU |
