import gym
from balance_bot.envs.balancebot_env import BalancebotEnv

from baselines import deepq

def main():

  env = BalancebotEnv(render=True)
  act = deepq.load("./output/train2.pkl")
  print(act)
  while True:
    obs, done = env.reset(), False
    print("===================================")
    print("obs")
    print(obs)
    episode_rew = 0
    while not done:
      env.render()
      obs, rew, done, _ = env.step(act(obs[None])[0])
      episode_rew += rew
    print("Episode reward", episode_rew)


if __name__ == '__main__':
  main()