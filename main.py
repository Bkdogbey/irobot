from __future__ import annotations

from pathlib import Path

import yaml

from utils import skip_run

# The configuration file
config_path = 'configs/config.yml'
config = yaml.load(Path.open(str(config_path)), Loader=yaml.SafeLoader)

with skip_run('skip', 'Data') as check, check():
    pass
