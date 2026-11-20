"""
Fusion Reducers add-in: gearbox generators for stepper motors.

Adds three commands to SOLID > CREATE:
  - Cycloidal Drive  (1, 2 or 3 discs)
  - Belt Drive       (GT2 / HTD timing pulleys, tensioning motor plate)
  - Capstan Drive    (cable drive with a sector output for robot arm joints)
"""

import adsk.core
import traceback

from . import fusion_ui
from . import cycloid_command
from . import belt_command
from . import capstan_command

PANEL_ID = 'SolidCreatePanel'
COMMANDS = [cycloid_command.COMMAND, belt_command.COMMAND, capstan_command.COMMAND]


def run(context):
    ui = adsk.core.Application.get().userInterface
    try:
        panel = ui.allToolbarPanels.itemById(PANEL_ID)
        for cmd in COMMANDS:
            cmd.register(ui, panel)
    except Exception:
        ui.messageBox('Fusion Reducers failed to start:\n' + traceback.format_exc())


def stop(context):
    ui = adsk.core.Application.get().userInterface
    try:
        panel = ui.allToolbarPanels.itemById(PANEL_ID)
        for cmd in COMMANDS:
            cmd.unregister(ui, panel)
        fusion_ui.clear_handlers()
    except Exception:
        ui.messageBox('Fusion Reducers failed to stop:\n' + traceback.format_exc())
