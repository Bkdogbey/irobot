from __future__ import annotations

from core.base import CrazyflieComponent
from core.config import CrazyflieConfig
from ros_sugar import Launcher

# Init your components
my_component = CrazyflieComponent(component_name='test', config=CrazyflieConfig())

# Create your launcher
launcher = Launcher()

# Add your package components
launcher.add_pkg(
    components=[my_component],
    activate_all_components_on_start=True,
)


# Bring up the system
launcher.bringup()
