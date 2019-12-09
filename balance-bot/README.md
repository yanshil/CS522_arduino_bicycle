# README

## Reproduce Experiment

```
conda create --name <env> --file conda_env_bicycleRL.txt

pip install baselines gym pybullet
cd balance_bot
pip install -e .
```

Note: I got some problem when installing `baselines` (Fail to build wheels for mujoco). Manually fixed by installing from its GitHub repo.

### Goal

1. Maintain Balance for long time
2. Get target position


Starting point (1, 0)  facing (-1, +1)

Reach a specific location e.g. (-5, 4)

### Control Signal

* Steering of the Bar
* Velocity of Back Wheel

### Action Space


### Observation Space



### Reference: 

1. https://backyardrobotics.eu/2017/11/27/build-a-balancing-bot-with-openai-gym-pt-i-setting-up/
2. https://backyardrobotics.eu/2017/11/29/build-a-balancing-bot-with-openai-gym-pt-ii-the-robot-and-environment/
3. https://github.com/yconst/balance-bot