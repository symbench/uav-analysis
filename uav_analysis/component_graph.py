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
        "TOP_CONNECTOR": [],
        "BOTTOM_CONNECTOR": [],
        "LEFT_CONNECTOR": [],
        "RIGHT_CONNECTOR": [],
        "REAR_CONNECTOR": [],
        "Seat_1_Connector": [],
        "Seat_2_Connector": [],
        "ORIENT": [],
    }

    COMPONENT_TYPES = [
        "FUSE_SPHERE_CYL_CONE",
        "NACA_Fuse",
    ]

    COMPONENT_KIND = "Fuselage"

    def __init__(self, name: str, type: str,
                 sphere_diameter: float,
                 length: float,
                 middle_length: float,
                 floor_height: float,
                 tail_diameter: float,
                 seat_1_lr: float,
                 seat_1_fb: float,
                 seat_2_lr: float,
                 seat_2_fb: float,
                 port_thickness: float):
        super(Fuselage, self).__init__(
            name=name,
            type=type,
            parameters={
                "SPHERE_DIAMETER": sphere_diameter,
                "LENGTH": length,
                "MIDDLE_LENGTH": middle_length,
                "TAIL_DIAMETER": tail_diameter,
                "SEAT_1_LR": seat_1_lr,
                "SEAT_1_FB": seat_1_fb,
                "SEAT_2_LR": seat_2_lr,
                "SEAT_2_FB": seat_2_fb,
                "FLOOR_HEIGHT": floor_height,
                "PORT_THICKNESS": port_thickness,
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
            "Fuselage.TOP_CONNECTOR",
            "Fuselage.BOTTOM_CONNECTOR",
            "Fuselage.LEFT_CONNECTOR",
            "Fuselage.RIGHT_CONNECTOR",
            "Fuselage.REAR_CONNECTOR",
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

    def __init__(self, name: str):
        super(BatteryController, self).__init__(
            name=name,
            type="BatteryController",
            parameters={
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
        "BOTTOM_CONNECTOR": [
            "Fuselage.TOP_CONNECTOR",
            "Fuselage.BOTTOM_CONNECTOR",
            "Fuselage.LEFT_CONNECTOR",
            "Fuselage.RIGHT_CONNECTOR",
            "Fuselage.REAR_CONNECTOR",
            "Beam.TOP_CONNECTOR_IN",
            "Beam.BOTTOM_CONNECTOR_OUT",
        ],
        "End_Connector_1": [],
        "End_Connector_2": [],
    }

    COMPONENT_TYPES = [
        "naca_connector",
    ]

    COMPONENT_KIND = "NACA_Port_Connector"

    def __init__(self, name: str,
                 chord: float,
                 profile: str,
                 port_thickness: float):
        assert isinstance(profile, str) and len(profile) == 4
        super(WingMount, self).__init__(
            name=name,
            type="naca_connector",
            parameters={
                "CHORD": chord,
                "THICKNESS": int(profile[2:4]),
                "PORT_THICKNESS": port_thickness,
                "BOTTOM_CONNECTION_DISP": 0.0,
            })


class Wing(Component):
    CONNECTORS = {
        "Connector_1": [
            "NACA_Port_Connector.End_Connector_1",
            "NACA_Port_Connector.End_Connector_2",  # upside down
            "Fuselage.LEFT_CONNECTOR",
            "Fuselage.RIGHT_CONNECTOR",  # upside down
        ],
        "Connector_2": [
            "NACA_Port_Connector.End_Connector_1",  # upside down
            "NACA_Port_Connector.End_Connector_2",
            "Fuselage.LEFT_CONNECTOR",  # upside down
            "Fuselage.RIGHT_CONNECTOR",
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
            "Fuselage.LEFT_CONNECTOR",
            "Fuselage.RIGHT_CONNECTOR",
            "Fuselage.TOP_CONNECTOR",
            "Fuselage.BOTTOM_CONNECTOR",
            "Fuselage.REAR_CONNECTOR",
            "Beam.TOP_CONNECTOR_IN",
            "Beam.BOTTOM_CONNECTOR_OUT",
        ],
        "END_CONNECTOR_2": [
            "Fuselage.LEFT_CONNECTOR",
            "Fuselage.RIGHT_CONNECTOR",
            "Fuselage.TOP_CONNECTOR",
            "Fuselage.BOTTOM_CONNECTOR",
            "Fuselage.REAR_CONNECTOR",
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
    CONNECTORS = {
        "ORIENTCONN": [
            "Fuselage.ORIENT",
        ]
    }

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


# TROWEL

design = Design("Trowel")

design.add(Fuselage("fuselage", "FUSE_SPHERE_CYL_CONE",
                    sphere_diameter=1520,
                    length=2000,
                    middle_length=300,
                    floor_height=150,
                    tail_diameter=200,
                    seat_1_lr=210,
                    seat_1_fb=790,
                    seat_2_lr=-210,
                    seat_2_fb=790,
                    port_thickness=150))

design.add(Orient("Orient", z_angle=0))
design.conn("Orient", "ORIENTCONN", "fuselage", "ORIENT")

design.add(BatteryController("777V_Bat_System"))

design.add(Passenger("Passenger1", "Passenger"))
design.conn("Passenger1", "Connector", "fuselage", "Seat_1_Connector")

design.add(Passenger("Passenger2", "Passenger"))
design.conn("Passenger2", "Connector", "fuselage", "Seat_1_Connector")

design.add(WingMount("top_naca_connector",
                     chord=2000,
                     profile="4512",
                     port_thickness=150))
design.conn("top_naca_connector", "BOTTOM_CONNECTOR",
            "fuselage", "TOP_CONNECTOR")

design.add(Wing("left_top_wing",
                chord=2000,
                profile="4512",
                span=10000,
                load=7000))
design.conn("left_top_wing", "Connector_2",
            "top_naca_connector", "End_Connector_2")

design.add(Wing("right_top_wing",
                chord=2000,
                profile="4512",
                span=10000,
                load=7000))
design.conn("right_top_wing", "Connector_1",
            "top_naca_connector", "End_Connector_1")

design.add(WingMount("bot_naca_connector",
                     chord=1000,
                     profile="0025",
                     port_thickness=150))
design.conn("bot_naca_connector", "BOTTOM_CONNECTOR",
            "fuselage", "BOTTOM_CONNECTOR")

design.add(Wing("right_bot_wing",
                chord=1000,
                profile="0025",
                span=4500,
                load=2000))
design.conn("right_bot_wing", "Connector_1",
            "bot_naca_connector", "End_Connector_2")

design.add(Wing("left_bot_wing",
                chord=1000,
                profile="0025",
                span=4500,
                load=2000))
design.conn("left_bot_wing", "Connector_2",
            "bot_naca_connector", "End_Connector_1")

design.add(Battery("left_battery", "Tattu25AhLi",
                   volume_percent=100,
                   voltage_request=777,
                   mount_side=2,
                   chord=1000,
                   profile="0025",
                   span=4500))
design.conn("left_battery", "Battery_Connector_2_out",
            "left_bot_wing", "Battery_Connector_2")

design.validate()
