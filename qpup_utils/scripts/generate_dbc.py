#!/usr/bin/env python3
import subprocess

import cantools

from cantools.database.can import Signal, Message, Database, c_source
from pathlib import Path
from typing import List, Union

FILE_NAME_STEM = 'qpup_can_generated'
PACKAGE_ROOT = (Path(__file__) / '..' / '..').resolve(strict=True)
CONFIG_PATH = (PACKAGE_ROOT / 'config').resolve(strict=True)
INCLUDE_PATH = (PACKAGE_ROOT / 'include').resolve(strict=True)
SOURCE_PATH = (PACKAGE_ROOT / 'src').resolve(strict=True)


def get_can_id(axis_id: int, cmd_id: int) -> int:
    return axis_id << 5 | cmd_id


def generate_odrive_dbc_messages(axis_id: Union[int, None] = None) -> List[Message]:
    if axis_id is None:
        odrive_msg_prefix = 'Odrive'
        # force frame id's to be based off odrive node 0 (ie. unused node id)
        axis_id = 0
    else:
        odrive_msg_prefix = f'Axis_{axis_id}'

    # 0x00 - NMT Message (Reserved)

    # 0x001 - Heartbeat
    heartbeatMsg = Message(
        frame_id=get_can_id(axis_id=axis_id, cmd_id=0x001),
        name=f'{odrive_msg_prefix}_Heartbeat',
        length=8,
        signals=[
            Signal(name='Axis_Error', start=0, length=32),
            Signal(name='Axis_State', start=32, length=8),
            Signal(name='Motor_Flags', start=40, length=8),
            Signal(name='Encoder_Flags', start=48, length=8),
            Signal(name='Controller_Flags', start=56, length=8)]
    )

    # 0x002 - E-Stop Message
    estopMsg = Message(
        frame_id=get_can_id(axis_id=axis_id, cmd_id=0x002),
        name=f'{odrive_msg_prefix}_Estop',
        length=0,
        signals=[]
    )

    # 0x003 - Motor Error
    motorErrorMsg = Message(
        frame_id=get_can_id(axis_id=axis_id, cmd_id=0x003),
        name=f'{odrive_msg_prefix}_Get_Motor_Error',
        length=8,
        signals=[Signal(name='Motor_Error', start=0, length=32)]
    )

    # 0x004 - Encoder Error
    encoderErrorMsg = Message(
        frame_id=get_can_id(axis_id=axis_id, cmd_id=0x004),
        name=f'{odrive_msg_prefix}_Get_Encoder_Error',
        length=8,
        signals=[Signal(name='Encoder_Error', start=0, length=32)]
    )

    # 0x005 - Sensorless Error
    sensorlessErrorMsg = Message(
        frame_id=get_can_id(axis_id=axis_id, cmd_id=0x005),
        name=f'{odrive_msg_prefix}_Get_Sensorless_Error',
        length=8,
        signals=[Signal(name='Sensorless_Error', start=0, length=32)]
    )

    # 0x006 - Axis Node ID
    axisNodeMsg = Message(
        frame_id=get_can_id(axis_id=axis_id, cmd_id=0x006),
        name=f'{odrive_msg_prefix}_Set_Axis_Node_ID',
        length=8,
        signals=[Signal(name='Axis_Node_ID', start=0, length=32)]
    )

    # 0x007 - Requested State
    setAxisState = Message(
        frame_id=get_can_id(axis_id=axis_id, cmd_id=0x007),
        name=f'{odrive_msg_prefix}_Set_Axis_State',
        length=8,
        signals=[Signal(name='Axis_Requested_State', start=0, length=32)]
    )

    # 0x008 - Startup Config (Reserved)

    # 0x009 - Encoder Estimates
    encoderEstimates = Message(
        frame_id=get_can_id(axis_id=axis_id, cmd_id=0x009),
        name=f'{odrive_msg_prefix}_Get_Encoder_Estimates',
        length=8,
        signals=[
            Signal(name='Pos_Estimate', start=0, length=32, is_float=True),
            Signal(name='Vel_Estimate', start=32, length=32, is_float=True),
        ]
    )

    # 0x00A - Get Encoder Count
    encoderCountMsg = Message(
        frame_id=get_can_id(axis_id=axis_id, cmd_id=0x00A),
        name=f'{odrive_msg_prefix}_Get_Encoder_Count',
        length=8,
        signals=[
            Signal(name='Shadow_Count', start=0, length=32),
            Signal(name='Count_in_CPR', start=32, length=32),
        ]
    )

    # 0x00B - Set Controller Modes
    setControllerModeMsg = Message(
        frame_id=get_can_id(axis_id=axis_id, cmd_id=0x00B),
        name=f'{odrive_msg_prefix}_Set_Controller_Mode',
        length=8,
        signals=[
            Signal(name='Control_Mode', start=0, length=32),
            Signal(name='Input_Mode', start=32, length=32),
        ]
    )

    # 0x00C - Set Input Pos
    setInputPosMsg = Message(
        frame_id=get_can_id(axis_id=axis_id, cmd_id=0x00C),
        name=f'{odrive_msg_prefix}_Set_Input_Pos',
        length=8,
        signals=[
            Signal(name='Input_Pos', start=0, length=32, is_float=True),
            Signal(name='Vel_FF', start=32, length=16, is_signed=True, scale=0.001),
            Signal(name='Torque_FF',
                   start=48,
                   length=16,
                   is_signed=True,
                   scale=0.001
                   )
        ]
    )

    # 0x00D - Set Input Vel
    setInputVelMsg = Message(
        frame_id=get_can_id(axis_id=axis_id, cmd_id=0x00D),
        name=f'{odrive_msg_prefix}_Set_Input_Vel',
        length=8,
        signals=[
            Signal(name='Input_Vel', start=0, length=32, is_float=True),
            Signal(name='Input_Torque_FF', start=32, length=32, is_float=True)
        ]
    )

    # 0x00E - Set Input Torque
    setInputTqMsg = Message(
        frame_id=get_can_id(axis_id=axis_id, cmd_id=0x00E),
        name=f'{odrive_msg_prefix}_Set_Input_Torque',
        length=8,
        signals=[Signal(name='Input_Torque', start=0, length=32, is_float=True)]
    )

    # 0x00F - Set Velocity Limit
    setVelLimMsg = Message(
        frame_id=get_can_id(axis_id=axis_id, cmd_id=0x00F),
        name=f'{odrive_msg_prefix}_Set_Limits',
        length=8,
        signals=[
            Signal(name='Velocity_Limit', start=0, length=32, is_float=True),
            Signal(name='Current_Limit', start=32, length=32, is_float=True),
        ]
    )

    # 0x010 - Start Anticogging
    startAnticoggingMsg = Message(
        frame_id=get_can_id(axis_id=axis_id, cmd_id=0x010),
        name=f'{odrive_msg_prefix}_Start_Anticogging',
        length=0,
        signals=[]
    )

    # 0x011 - Set Traj Vel Limit
    setTrajVelMsg = Message(
        frame_id=get_can_id(axis_id=axis_id, cmd_id=0x011),
        name=f'{odrive_msg_prefix}_Set_Traj_Vel_Limit',
        length=8,
        signals=[Signal(name='Traj_Vel_Limit', start=0, length=32, is_float=True)]
    )

    # 0x012 - Set Traj Accel Limits
    setTrajAccelMsg = Message(
        frame_id=get_can_id(axis_id=axis_id, cmd_id=0x012),
        name=f'{odrive_msg_prefix}_Set_Traj_Accel_Limits',
        length=8,
        signals=[
            Signal(name='Traj_Accel_Limit', start=0, length=32, is_float=True),
            Signal(name='Traj_Decel_Limit', start=32, length=32, is_float=True)
        ]
    )

    # 0x013 - Set Traj Inertia
    trajInertiaMsg = Message(
        frame_id=get_can_id(axis_id=axis_id, cmd_id=0x013),
        name=f'{odrive_msg_prefix}_Set_Traj_Inertia',
        length=8,
        signals=[Signal(name='Traj_Inertia', start=0, length=32, is_float=True)]
    )

    # 0x014 - Get Iq
    getIqMsg = Message(frame_id=get_can_id(axis_id=axis_id, cmd_id=0x014),
                       name=f'{odrive_msg_prefix}_Get_Iq',
                       length=8,
                       signals=[
                           Signal(name='Iq_Setpoint', start=0, length=32, is_float=True),
                           Signal(name='Iq_Measured', start=32, length=32, is_float=True)
                       ]
                       )

    # 0x015 - Get Sensorless Estimates
    getSensorlessEstMsg = Message(
        frame_id=get_can_id(axis_id=axis_id, cmd_id=0x015),
        name=f'{odrive_msg_prefix}_Get_Sensorless_Estimates',
        length=8,
        signals=[
            Signal(name='Sensorless_Pos_Estimate', start=0, length=32, is_float=True),
            Signal(name='Sensorless_Vel_Estimate', start=32, length=32, is_float=True)
        ]
    )

    # 0x016 - Reboot ODrive
    rebootMsg = Message(
        frame_id=get_can_id(axis_id=axis_id, cmd_id=0x016),
        name=f'{odrive_msg_prefix}_Reboot',
        length=0,
        signals=[]
    )

    # 0x017 - Get vbus Voltage
    getVbusVMsg = Message(
        frame_id=get_can_id(axis_id=axis_id, cmd_id=0x017),
        name=f'{odrive_msg_prefix}_Get_Vbus_Voltage',
        length=8,
        signals=[Signal(name='Vbus_Voltage', start=0, length=32, is_float=True)])

    # 0x018 - Clear Errors
    clearErrorsMsg = Message(
        frame_id=get_can_id(axis_id=axis_id, cmd_id=0x018),
        name=f'{odrive_msg_prefix}_Clear_Errors',
        length=0,
        signals=[])

    # 0x019 - Set Linear Count
    setLinearCountMsg = Message(
        frame_id=get_can_id(axis_id=axis_id, cmd_id=0x019),
        name=f'{odrive_msg_prefix}_Set_Linear_Count',
        length=8,
        signals=[
            Signal(name='Position', start=0, length=32, is_signed=True)
        ]
    )

    # 0x01A - Set Pos gain
    setPosGainMsg = Message(
        frame_id=get_can_id(axis_id=axis_id, cmd_id=0x01A),
        name=f'{odrive_msg_prefix}_Set_Pos_Gain',
        length=8,
        signals=[Signal(name='Pos_Gain', start=0, length=32, is_float=True)])

    # 0x01B - Set Vel Gains
    setVelGainsMsg = Message(
        frame_id=get_can_id(axis_id=axis_id, cmd_id=0x01B),
        name=f'{odrive_msg_prefix}_Set_Vel_gains',
        length=8,
        signals=[
            Signal(name='Vel_Gain', start=0, length=32, is_float=True),
            Signal(name='Vel_Integrator_Gain', start=32, length=32, is_float=True)
        ]
    )

    return [
        heartbeatMsg,
        estopMsg,
        motorErrorMsg,
        encoderErrorMsg,
        sensorlessErrorMsg,
        axisNodeMsg,
        setAxisState,
        encoderEstimates,
        encoderCountMsg,
        setControllerModeMsg,
        setInputPosMsg,
        setInputVelMsg,
        setInputTqMsg,
        setVelLimMsg,
        startAnticoggingMsg,
        setTrajVelMsg,
        setTrajAccelMsg,
        trajInertiaMsg,
        getIqMsg,
        getSensorlessEstMsg,
        rebootMsg,
        getVbusVMsg,
        clearErrorsMsg,
        setLinearCountMsg,
        setPosGainMsg,
        setVelGainsMsg
    ]


def generate_dbc() -> None:
    odrive_nodes = range(1, 13)

    # Add Dummy Odrive @ node 0 for friendlier message names to use as generic
    odrive_messages = generate_odrive_dbc_messages()
    for node in odrive_nodes:
        odrive_messages.extend(generate_odrive_dbc_messages(axis_id=node))

    db = Database(
        messages=odrive_messages
    )

    cantools.database.dump_file(db, CONFIG_PATH / (FILE_NAME_STEM + '.dbc'))


def generate_dbc_dump() -> None:
    dump_path = CONFIG_PATH / (FILE_NAME_STEM + '_dump.txt')

    completed_process = subprocess.run(
        [
            'cantools',
            'dump',
            f'{CONFIG_PATH / (FILE_NAME_STEM + ".dbc")}',
            '--with-comments'
        ],
        capture_output=True,
        text=True,
    )
    with open(dump_path, 'wt') as outfile:
        outfile.write(completed_process.stdout.strip())

    print(f'Successfully generated {dump_path}.')


def generate_c_code() -> None:
    can_database = cantools.database.load_file(
        filename=CONFIG_PATH / (FILE_NAME_STEM + '.dbc'),
    )

    filename_h = 'qpup_generated/' + FILE_NAME_STEM + '.h'
    filename_c = FILE_NAME_STEM + '.c'

    header, source, _, _ = c_source.generate(
        database=can_database,
        database_name='qpup',
        header_name=filename_h,
        source_name=filename_c,
        fuzzer_source_name=None,
        floating_point_numbers=False,
        bit_fields=False,
    )

    header_path = INCLUDE_PATH / filename_h
    source_path = SOURCE_PATH / filename_c

    with open(header_path, 'wt') as fout:
        fout.write(header)

    with open(source_path, 'wt') as fout:
        fout.write(source)

    print(f'Successfully generated {header_path} and {source_path}.')


if __name__ == '__main__':
    generate_dbc()
    generate_dbc_dump()
    generate_c_code()
