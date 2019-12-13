### Install Dependencies

My develop environment : a Windows Linux Subsystem with Ubuntu 18.04

So there will be more dependencies to install than in GUI Linux

```
sudo apt-get update
sudo apt-get install gcc  ## Will be needed in pip install pybullet
sudo apt-get install g++ ## Will be needed in pip install pybullet
sudo apt-get install cmake libopenmpi-dev python3-dev zlib1g-dev ## Baselines

sudo apt-get install libglew-dev ## PyBullet
```

Set up conda environment 

```
conda env create -f conda_env/environment.yml
## If failed (dependency errors pop out and stopped), rm the env and reinstall
conda remove -n bicycle --all
## I thought update should work but it always got stucked when I migrate my env
## conda env update -n bicycle -f conda_env/environment.yml
```

### Install the RL-bicycle module(`balance-bot`)

```
conda activate bicycle
cd balance-bot
pip install -e .
```

### WSL Setting for GUI window

You will need an app to run graphical app on WSL.

See https://virtualizationreview.com/articles/2017/02/08/graphical-programs-on-windows-subsystem-on-linux.aspx

```
## Linux modify bash/simply type
export DISPLAY=:0
## Windows VcXsrv Setting: 
# Mine set the display number as 0 and UNCHECK the "Native opengl"
# But default setting should works with `export LIBGL_ALWAYS_INDIRECT=1`
```

