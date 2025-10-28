import pymodbus.client as ModbusClient
from pymodbus import FramerType, ModbusException
import os


def read_registers_to_float(client, address: int) -> float:
    """
    Read a 32-bit floating-point value from Modbus input registers.

    Args:
        client (ModbusTcpClient): The Modbus TCP client instance.
        address (int): The startubg Modbus address to read from.

    Returns:
        float: The converted floating-point value from the input registers.
    """

    # Read input registers
    try:
        read_register = client.read_input_registers(
            address=address,
            count=2,
            no_response_expected=False
        )
    except ModbusException as exc:
        print(exc)
        return None

    # Convert to float
    value = client.convert_from_registers(
        read_register.registers,
        data_type=client.DATATYPE.FLOAT32
    )
    return value


def save_TM_robot_flange_pose(tm_model: str) -> None:
    """
    Read the TM robot flange pose by Modbus protocol, and save as .dat file.
    Args:
        TM_model (str): TM5_900 or TM5x_700
    """

    # TM5-900 ip configuration
    TM5_900_IP = "192.168.50.38"
    TM5_900_PORT = 502
    # TM5x-700 ip configuration
    TM5x_700_IP = "192.168.50.49"
    TM5x_700_PORT = 502

    # TM robot Modbus address
    ADDR_X = 7037
    ADDR_Y = 7039
    ADDR_Z = 7041
    ADDR_RX = 7043
    ADDR_RY = 7045
    ADDR_RZ = 7047
    JOINT1 = 7013
    JOINT2 = 7015
    JOINT3 = 7017
    JOINT4 = 7019
    JOINT5 = 7021
    JOINT6 = 7023

    # File configuration for saving
    FILE_DIR = "./pose_dat/"
    FILE_NAME = "flange_pose"
    EXTENSION = ".dat"

    # Choose the TM robot model
    if tm_model == "TM5_900":
        tm_robot_ip = TM5_900_IP
        tm_robot_port = TM5_900_PORT
    elif tm_model == "TM5x_700":
        tm_robot_ip = TM5x_700_IP
        tm_robot_port = TM5x_700_PORT
    else:
        print("Wrong TM model !!")
        return

    # Use Modbus TCP to connect the TM robot.
    modbus_client = ModbusClient.ModbusTcpClient(
        host=tm_robot_ip,
        port=tm_robot_port,
        framer=FramerType.SOCKET
    )
    try:
        modbus_client.connect()
    except ModbusException as exc:
        print(exc)
        exit()

    # Read the TM robot's flange pose by Modbus registers
    x = read_registers_to_float(modbus_client, ADDR_X) / 1000
    y = read_registers_to_float(modbus_client, ADDR_Y) / 1000
    z = read_registers_to_float(modbus_client, ADDR_Z) / 1000
    rx = read_registers_to_float(modbus_client, ADDR_RX)
    ry = read_registers_to_float(modbus_client, ADDR_RY)
    rz = read_registers_to_float(modbus_client, ADDR_RZ)

    modbus_client.close()

    # Create the content of .dat
    content = f"""\
#
# 3D POSE PARAMETERS: rotation and translation
#

# Used representation type:
f 2

# Rotation angles [deg] or Rodriguez-vector:
r {rx} {ry} {rz}

# Translational vector (x y z [m]):
t {x} {y} {z}
"""
    # Save the content as a .dat file
    file_number = 0
    file_path = f"{FILE_DIR}{FILE_NAME}{file_number:02d}{EXTENSION}"
    while os.path.exists(file_path):
        file_number += 1
        file_path = f"{FILE_DIR}{FILE_NAME}{file_number:02d}{EXTENSION}"
    with open(file_path, "w") as file:
        file.write(content)

    print(f"Saved: {file_path}")

if __name__ == "__main__":
    save_TM_robot_flange_pose("TM5x_700")