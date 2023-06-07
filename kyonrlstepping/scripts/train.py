import datetime

import hydra
from omegaconf import DictConfig

from hydra.utils import to_absolute_path

@hydra.main(config_name="config", config_path="../cfg")
def launch_rlg_hydra(cfg: DictConfig):

    time_str = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

if __name__ == "__main__":

    launch_rlg_hydra()
