#!/usr/bin/env python3
# Copyright (C) 2022, Miklos Maroti
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

from typing import Any, Dict, List, Union

from components import MOTORS, PROPELLERS, BATTERIES


class Component():
    CONNECTORS = {}
    COMPONENT_TYPES = []
    COMPONENT_KIND = ""

    def __init__(self, name: str, type: str, parameters: Dict[str, Any]):
        assert type in self.COMPONENT_TYPES
        self.name = name
        self.type = type
        self.connections = {conn: None
                            for conn, dest in self.CONNECTORS.items()
                            if dest}
        self.parameters = parameters

    def __str__(self) -> str:
        return "{}.{}".format(
            self.COMPONENT_KIND,
            self.name)

    def get(self, param: str) -> Any:
        return self.parameters[param]

    def validate(self) -> bool:
        valid = True
        for conn, dest in self.connections.items():
            if dest is None:
                print(self, conn, "not connected")
                valid = False
        return valid


class Fuselage(Component):
    CONNECTORS = {
        "Top_Connector": [],
        "Bottom_Connector": [],
        "Left_Connector": [],
        "Right_Connector": [],
        "Rear_Connector": [],
        "Seat_1_Connector": [],
        "Seat_2_Connector": [],
    }

    COMPONENT_TYPES = [
        "FUSE_SPHERE_CYL_CONE",
        "NACA_Fuse",
    ]

    COMPONENT_KIND = "Fuselage"

    def __init__(self, name: str, type: str):
        super(Fuselage, self).__init__(
            name=name,
            type=type,
            parameters={
                "SPHERE_DIAMETER": 1520.0,
                "LENGTH": 2000.0,
                "MIDDLE_LENGTH": 750.0,
                "TAIL_DIAMETER": 200.0,
                "SEAT_1_LR": -200.0,
                "SEAT_1_FB": 1000.0,
                "SEAT_2_LR": 200.0,
                "SEAT_2_FB": 1000.0,
                "FLOOR_HEIGHT": 150.0,
                "PORT_THICKNESS": 100.0,
                "TOP_PORT_DISP": 0.0,
                "BOTTOM_PORT_DISP": 0.0,
                "LEFT_PORT_DISP": 0.0,
                "RIGHT_PORT_DISP": 0.0,
            })


class Passenger(Component):
    CONNECTORS = {
        "Connector": [
            "Fuselage.Seat_1_Connector",
            "Fuselage.Seat_2_Connector",
        ]
    }

    COMPONENT_TYPES = [
        "Passenger",
    ]

    COMPONENT_KIND = "Passenger"

    def __init__(self, name: str, type: str):
        super(Passenger, self).__init__(
            name=name,
            type=type,
            parameters={
            })


class Motor(Component):
    CONNECTORS = {
        "Prop_Connector": [],
        "Base_Connector": [
            "Fuselage.Top_Connector",
            "Fuselage.Bottom_Connector",
            "Fuselage.Left_Connector",
            "Fuselage.Right_Connector",
            "Fuselage.Rear_Connector",
            "Beam.TOP_CONNECTOR_IN",
            "Beam.BOTTOM_CONNECTOR_OUT",
        ],
        "MotorPower": [
            "BatteryController.MotorPower",
        ],
    }

    COMPONENT_TYPES = list(MOTORS.keys())

    COMPONENT_KIND = "Motor"

    def __init__(self, name: str, type: str,
                 control_channel: int):
        super(Motor, self).__init__(
            name=name,
            type=type,
            parameters={
                "CONTROL_CHANNEL": control_channel,
            })


class Propeller(Component):
    CONNECTORS = {
        "MOTOR_CONNECTOR_CS_IN": [
            "Motor.Prop_Connector",
        ],
    }

    COMPONENT_TYPES = list(PROPELLERS.keys())

    COMPONENT_KIND = "Propeller"

    def __init__(self, name: str, type: str,
                 prop_type: int,
                 direction: int):
        super(Propeller, self).__init__(
            name=name,
            type=type,
            parameters={
                "Prop_type": prop_type,
                "Direction": direction,
            })


class BatteryController(Component):
    CONNECTORS = {
        "BatteryPower": [],
        "MotorPower": [],
    }

    COMPONENT_TYPES = ["BatteryController"]

    COMPONENT_KIND = "BatteryController"

    def __init__(self, name: str,
                 voltage: float):
        super(BatteryController, self).__init__(
            name=name,
            type="BatteryController",
            parameters={
                # TODO: check if this works
                "Input_Voltage": voltage,
                "Output_Voltage": voltage,
            })


class Battery(Component):
    CONNECTORS = {
        "Battery_Connector_1_Out": [
            "Wing.Battery_Connector_1",
            "Wing.Battery_Connector_2",
        ],
        "Battery_Connector_2_Out": [
            "Wing.Battery_Connector_1",
            "Wing.Battery_Connector_2",
        ],
        "PowerBus": [
            "BatteryController.BatteryPower",
        ],
    }

    COMPONENT_TYPES = list(BATTERIES.keys())

    COMPONENT_KIND = "Battery"

    def __init__(self, name: str, type: str,
                 voltage_request: float,
                 mount_side: int,
                 chord: float,
                 profile: str,
                 span: float):
        assert mount_side in [1, 2]
        super(Battery, self).__init__(
            name=name,
            type=type,
            parameters={
                "VOLTAGE_REQUEST": voltage_request,
                "MOUNT_SIDE": mount_side,
                "VOLUME_PERCENT": 100.0,
                "CHORD_1": chord,
                "CHORD_2": chord,
                "SPAN": span,
                "THICKNESS": int(profile[2:4]),
                "TAPER_OFFSET": 0.0,
            })

    def validate(self) -> bool:
        valid = True
        if self.connections["Battery_Connector_1_Out"] is None and \
                self.connections["Battery_Connector_2_Out"] is None:
            print(self, "Battery_Connector_1/2_Out", "not connected")
            valid = False

        if self.connections["PowerBus"] is None:
            print(self, "PowerBus", "not connected")
            valid = False

        if self.get("MOUNT_SIDE") == 1:
            wing = self.connections["Battery_Connector_1_Out"][0]
        else:
            wing = self.connections["Battery_Connector_2_Out"][0]

        for param in ["CHORD_1", "CHORD_2", "SPAN", "THICKNESS"]:
            if self.get(param) != wing.get(param):
                print(self, param, "mismatch with", wing)

        return valid


class WingMount(Component):
    CONNECTORS = {
        "Bottom_connector": [
            "Fuselage.Top_Connector",
            "Fuselage.Bottom_Connector",
            "Fuselage.Left_Connector",
            "Fuselage.Right_Connector",
            "Fuselage.Rear_Connector",
            "Beam.TOP_CONNECTOR_IN",
            "Beam.BOTTOM_CONNECTOR_OUT",
        ],
        "End_Connection_1": [],
        "End_Connection_2": [],
    }

    COMPONENT_TYPES = [
        "naca_connector",
    ]

    COMPONENT_KIND = "NACA_Port_Connector"

    def __init__(self, name: str,
                 chord: float,
                 profile: str):
        assert isinstance(profile, str) and len(profile) == 4
        super(WingMount, self).__init__(
            name=name,
            type="naca_connector",
            parameters={
                "CHORD": chord,
                "THICKNESS": int(profile[2:4]),
                "BOTTOM_CONNECTION_DISP": 0.0,
                "PORT_THICKNESS": 100.0,
            })


class Wing(Component):
    CONNECTORS = {
        "Connector_1": [
            "NACA_Port_Connector.End_Connection_1",
            "NACA_Port_Connector.End_Connection_2",  # upside down
            "Fuselage.Left_Connector",
            "Fuselage.Right_Connector",  # upside down
        ],
        "Connector_2": [
            "NACA_Port_Connector.End_Connection_1",  # upside down
            "NACA_Port_Connector.End_Connection_2",
            "Fuselage.Left_Connector",  # upside down
            "Fuselage.Right_Connector",
        ],
        "Battery_Connector_1": [],
        "Battery_Connector_2": [],
    }

    COMPONENT_TYPES = ["naca_wing"]

    COMPONENT_KIND = "Wing"

    def __init__(self, name: str,
                 chord: float,
                 profile: str,
                 span: float,
                 load: float):
        assert isinstance(profile, str) and len(profile) == 4
        super(Wing, self).__init__(
            name=name,
            type="naca_wing",
            parameters={
                "CHORD_1": chord,
                "CHORD_2": chord,
                "NACA_Profile": profile,
                "SPAN": span,
                "LOAD": load,
                "THICKNESS": int(profile[2:4]),
                "TAPER_OFFSET": 0.0,
                "AILERON_BIAS": 0.5,
                "FLAP_BIAS": 0.5,
            })

    def validate(self) -> bool:
        valid = True
        if self.connections["Connector_1"] is None and \
                self.connections["Connector_2"] is None:
            print(self, "Connector_1/2", "not connected")
            valid = False

        dest = self.connections["Connector_1"]
        if dest is not None and isinstance(dest[0], WingMount):
            if self.get("CHORD_1") != dest[0].get("CHORD"):
                print(self, "CHORD mismatch with", dest[0])
                valid = False
            if self.get("THICKNESS") != dest[0].get("THICKNESS"):
                print(self, "THICKNESS mismatch with", dest[0])
                valid = False

        dest = self.connections["Connector_2"]
        if dest is not None and isinstance(dest[0], WingMount):
            if self.get("CHORD_2") != dest[0].get("CHORD"):
                print(self, "CHORD mismatch with", dest[0])
                valid = False
            if self.get("THICKNESS") != dest[0].get("THICKNESS"):
                print(self, "THICKNESS mismatch with", dest[0])
                valid = False

        return valid


class Beam(Component):
    CONNECTORS = {
        "END_CONNECTOR_1": [
            "Fuselage.Left_Connector",
            "Fuselage.Right_Connector",
            "Fuselage.Top_Connector",
            "Fuselage.Bottom_Connector",
            "Fuselage.Rear_Connector",
            "Beam.TOP_CONNECTOR_IN",
            "Beam.BOTTOM_CONNECTOR_OUT",
        ],
        "END_CONNECTOR_2": [
            "Fuselage.Left_Connector",
            "Fuselage.Right_Connector",
            "Fuselage.Top_Connector",
            "Fuselage.Bottom_Connector",
            "Fuselage.Rear_Connector",
            "Beam.TOP_CONNECTOR_IN",
            "Beam.BOTTOM_CONNECTOR_OUT",
        ],
        "TOP_CONNECTOR_IN": [],
        "BOTTOM_CONNECTOR_OUT": [],
    }

    COMPONENT_TYPES = [
        "Beam",
    ]

    COMPONENT_KIND = "Beam"

    def __init__(self, name: str):
        super(Beam, self).__init__(
            name=name,
            type="Beam",
            parameters={
            })


class Orient(Component):
    CONNECTORS = {}

    COMPONENT_TYPES = ["Orient"]

    COMPONENT_KIND = "Orient"

    def __init__(self, name: str,
                 z_angle: float):
        super(Orient, self).__init__(
            name=name,
            type="Orient",
            parameters={
                "Z_ANGLE": z_angle,
            })


class Design():
    def __init__(self, name: str):
        self.name = name
        self.components = dict()

    def get(self, name: str) -> 'Component':
        return self.components[name]

    def add(self, component: 'Component'):
        assert component.name not in self.components
        self.components[component.name] = component
        return component

    def conn(self,
             comp1: Union[str, 'Component'], conn1: str,
             comp2: Union[str, 'Component'], conn2: str):
        if isinstance(comp1, str):
            comp1 = self.get(comp1)
        assert comp1.name in self.components

        if isinstance(comp2, str):
            comp2 = self.get(comp2)
        assert comp2.name in self.components

        assert comp1.connections[conn1] is None
        assert comp2.COMPONENT_KIND + "." + conn2 in comp1.CONNECTORS[conn1]
        comp1.connections[conn1] = (comp2, conn2)

    def validate(self) -> bool:
        valid = True
        for comp in self.components.values():
            valid = comp.validate() and valid
        if valid:
            print("All valid")
        return valid


design = Design("Nothing")

design.add(Fuselage("fuselage", "FUSE_SPHERE_CYL_CONE"))
design.add(BatteryController("power", voltage=5000))

design.add(Passenger("Joe", "Passenger"))
design.conn("Joe", "Connector", "fuselage", "Seat_1_Connector")

design.add(Passenger("Jane", "Passenger"))
design.conn("Jane", "Connector", "fuselage", "Seat_2_Connector")

design.add(Motor("motor1", "Emrax268LV", control_channel=1))
design.conn("motor1", "Base_Connector", "fuselage", "Rear_Connector")
design.conn("motor1", "MotorPower", "power", "MotorPower")

design.add(Propeller("prop1", "41x6_3_3400_51_530", prop_type=1, direction=1))
design.conn("prop1", "MOTOR_CONNECTOR_CS_IN", "motor1", "Prop_Connector")

design.add(WingMount("mount1", chord=3000, profile="0012"))
design.conn("mount1", "Bottom_connector", "fuselage", "Top_Connector")

design.add(Wing("left_wing", chord=3000, profile="0012", span=5000, load=1e11))
design.conn("left_wing", "Connector_1", "mount1", "End_Connection_1")

design.add(Battery("left_battery", "Tattu 25Ah Li", voltage_request=1000,
                   mount_side=1, chord=3000, profile="0012", span=5000))
design.conn("left_battery", "Battery_Connector_1_Out", "left_wing", "Battery_Connector_1")
design.conn("left_battery", "PowerBus", "power", "BatteryPower")

design.add(Wing("right_wing", chord=3000, profile="0012", span=5000, load=1e11))
design.conn("right_wing", "Connector_2", "mount1", "End_Connection_2")

design.add(Battery("right_battery", "Tattu 25Ah Li", voltage_request=1000,
                   mount_side=2, chord=3000, profile="0012", span=5000))
design.conn("right_battery", "Battery_Connector_2_Out", "right_wing", "Battery_Connector_2")
design.conn("right_battery", "PowerBus", "power", "BatteryPower")

design.add(Orient("orient", z_angle=0))

design.validate()
