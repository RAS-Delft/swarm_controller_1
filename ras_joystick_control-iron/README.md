# Joystick Control User Interface
Starts up a Graphical User Interface that allows streaming of vessel commands in a user-friendly manner for human operation. 
Currently hardcoded to work for TitoNeri actuation structure

| Inputs | Outputs |
|--------|----------|
| Logitech Extreme 3D PRO (usb)| ros2 Jointstate reference actuation on /<vessel_id>/reference/actuation_prio |


## Use
Clone this repository in ros2_ws/src. Build ros2 workspace. Source workspace. Start with:
```shell
ros2 run joystick_control_ras joystickgui
```

Plug in a joystick on usb and tick the "joystick" option if you want to use that. Spinner is in case of multiple joysticks per computer, but default with one device is 0. 

<p align="center" width="100%">
    <img width="33%" src="https://github.com/RAS-Delft/ras_joystick_control/assets/5917472/5363056e-cbde-4ccf-a5f5-42048acc1554">
</p>

