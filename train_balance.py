import gym
from baselines import deepq
import balance_bot

# callback: (locals, globals) -> None
#     function called at every steps with state of the algorithm.
#     If callback returns true training stops.
def callback(lcl, glb):
    # stop training if reward exceeds 1e04 (currently is the iteration num before falling down)
    is_solved = lcl['t'] > 100 and sum(lcl['episode_rewards'][-101:-1]) / 100 >= 1e05
    return is_solved

def main():
    env = gym.make("balancebot-v0")
    ## Create learning agenet
    model = deepq.models.mlp([16, 12])
    act = deepq.learn(
        env,
        q_func=model,
        lr=1e-3,
        max_timesteps=100000,
        buffer_size=200000,
        exploration_fraction=0.1,
        exploration_final_eps=0.02,
        print_freq=10,
        callback=callback
    )
    print("Saving model to train.pkl")
    act.save("./output/train4.pkl")

if __name__ == '__main__':
    main()
