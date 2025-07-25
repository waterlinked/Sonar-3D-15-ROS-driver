#!/usr/bin/env python3
"""
Simple API example script to interact with the Sonar 3D-15 HTTP API.

REQUIREMENTS:
    - Python 3
    - requests (pip install requests)
    - Access to the Sonar 3D-15 IP address
    - Sonar 3D-15 with version 1.4.0 or newer

FUNCTIONALITY:
    - Get system information (version, status)
    - Get and set acoustics settings
    - Get and set speed of sound

USAGE:
    1. Run with required IP address: python endpoints.py --ip <args.ip>
    2. Optional parameters:
         --speed <SPEED>: Set speed of sound in m/s
         --acoustics enable/disable: Enable or disable acoustics
    3. Observe the output in the terminal.
"""

import requests


integration_api_payload = {
    "mode": "disabled",
    "unicast_destination_ip": "",
    "unicast_destination_port": 0
}

def get_about(ip: str) -> str:
    """
    Get system version, order it in a readable format and return it.
    """
    resp_about = requests.get(f"http://{ip}/api/v1/integration/about")
    about = resp_about.json()
    msg = ""
    for key, value in about.items():
        msg = msg + f"  {key}: {value}\n"

    return msg

def get_status(ip: str) -> str:
    """
    Get the status of the system, order it in a readable format and return it.
    """
    resp_status = requests.get(f"http://{ip}/api/v1/integration/status")
    status = resp_status.json()
    msg = ""
    for key, value in status.items():
        if type(value) is dict:
            msg = msg + f"  {key}:\n"
            for sub_key, sub_value in value.items():
                msg = msg + f"    {sub_key}: {sub_value}\n"
        else:
            msg = msg + f"{key}: {value}\n"

    return msg

def get_speed(ip: str) -> int:
    """
    Get the speed of sound used in the sonar as an int.
    Returns the error if request fails.
    """
    resp_speed = requests.get(f"http://{ip}/api/v1/integration/acoustics/speed_of_sound")
    try:
        speed_of_sound = resp_speed.json()
    except Exception as e:
        speed_of_sound = e

    return speed_of_sound

def get_acoustics(ip: str) -> bool:
    """
    Get the status of the acoustics (Enabled/Disabled).
    Returns the boolean status of the acoustics or the error if the request fails.
    """
    resp_acoustics = requests.get(f"http://{ip}/api/v1/integration/acoustics/enabled")
    try:
        acoustics = resp_acoustics.json()
    except Exception as e:
        acoustics = e

    return acoustics


def set_speed(ip: str, value: int) -> requests.models.Response:
    """
    Set the desired speed of sound to use in the sonar. 
    Returns the HTTP response of the posting.
    """
    post = requests.post(f"http://{ip}/api/v1/integration/acoustics/speed_of_sound", json=value)

    return post


def set_acoustics(ip: str, value: bool) -> requests.models.Response:
    """
    Enable or disable the acoustics of the sonar.
    Returns the HTTP response of the posting.
    """
    post = requests.post(f"http://{ip}/api/v1/integration/acoustics/enabled", json=value)

    return post

def enable_multicast(ip: str) -> requests.models.Response:
    """
    Enable the udp multicast of the sonar.
    Returns the HTTP response of the posting.
    """
    integration_api_payload["mode"] = "multicast"
    post = requests.post(f"http://{ip}/api/v1/integration/udp", json=integration_api_payload)

    return post

def disable_multicast(ip: str):
    """
    Disable the udp multicast of the sonar.
    Returns the HTTP response of the posting.
    """
    integration_api_payload["mode"] = "disable"
    post = requests.post(f"http://{ip}/api/v1/integration/udp", json=integration_api_payload)

    return post

def describe_response(endpoint: str, response: requests.models.Response) -> None:
    """
    Prints a more detailed response for the endpoint.
    """
    if response.ok:
        print(f"{endpoint} responded good")
    else:
        print(f"{endpoint} responded badly: {response.reason}")




if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(
        description="Interfacing the Sonar 3D-15 integration API.")
    parser.add_argument(
        "--ip",
        type=str,
        required=True,
        help="IP address of the sonar."
    )
    parser.add_argument(
        "--speed",
        type=int,
        default=None,
        help="Speed of sound in m/s",
    )
    parser.add_argument(
        "--acoustics",
        type=str,
        choices=["enable", "disable"],
        help="Enable or disable acoustics",
        default=None,
    )
    # Parse arguments
    args = parser.parse_args()

    # Use API to print details about the sonar
    about = get_about(args.ip)
    print(f"About:\n{about}")

    status = get_status(args.ip)
    print(f"Status:\n{status}")


    speed = get_speed(args.ip)
    print(f"Speed of Sound:\t   {speed}")

    acoustics = get_acoustics(args.ip)
    print(f"Acoustics enabled: {acoustics}\n")
    
    # Change acoustics and/or the speed of sound used by the sonar if specified by the user
    if args.acoustics is not None:
        acoustics_response = set_acoustics(args.ip, args.acoustics)
        describe_response("Acoustics:\t  ", acoustics_response)

    if args.speed is not None:
        speed_response = set_speed(args.ip, args.speed)
        describe_response("Speed of sound:\t  ", speed_response)