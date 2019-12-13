### Install Dependencies

My develop environment : a Windows Linux Subsystem with Ubuntu 18.04

```
sudo apt-get update
sudo apt-get install gcc
sudo apt-get install cmake libopenmpi-dev python3-dev zlib1g-dev ## Baselines
```

Set up conda environment 

```
conda env create -f conda_env/environment.yml
```

### Install the RL-bicycle module(`balance-bot`)

```
cd balance-bot
pip install -e .
```

