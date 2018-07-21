#!/usr/bin/env python
# coding=utf-8
"""
    Translates Netatmo Weather Station data to ROS.
    The time between updates is one minute.
    Value -1 in CO2, noise, pressure or battery_percent means no value.
    This node expect the configuration path to be specified via private parameter configpath. If the path a file called
     config.ini, it will be used. If this file is not found, the configuration values will be read from the following
     files: username, password, client_id, client_secret and device_id.
    Author: José Jaime Ariza (jariza@ieee.org).
    Python version: 2.7
"""

import ConfigParser
import os.path
import requests
import rospy
import sys
from netatmo2ros.msg import WeatherdataArray, Weatherdata


def get_access_token(username, password, client_id, client_secret):
    """
    Get the Netatmo access token
    :param username: Netamo username
    :param password: Netamo password
    :param client_id: Netamo Client ID
    :param client_secret: Netamo Client Secret
    :return: access token
    """
    # Query data
    payload = {'grant_type': 'password',
               'username': username,
               'password': password,
               'client_id': client_id,
               'client_secret': client_secret,
               'scope': 'read_station'}

    # Do the query
    response = requests.post("https://api.netatmo.com/oauth2/token", data=payload)
    response.raise_for_status()
    if response.json()["scope"][0] != 'read_station':
        raise ValueError("Unexpected scope: " + ' '.join(response.json()["scope"]))

    # Return access token
    return response.json()["access_token"]


def get_station_values(access_token, device_id):
    """
    Get values from a station and it's modules
    :param access_token: Netatmo access token
    :param device_id: Netatmo device ID
    :return: station weather data
    """

    # Query data
    params = {
        'access_token': access_token,
        'device_id': device_id
    }

    # Do the query
    response = requests.post("https://api.netatmo.com/api/getstationsdata", params=params)
    response.raise_for_status()
    device_data = response.json()["body"]["devices"][0]
    if device_data["_id"] != device_id:
        raise Exception("Unexpected device: " + device_data["_id"])

    # Populate indoor module data
    output = WeatherdataArray()
    wd = Weatherdata()
    wd.name = str(device_data["module_name"])
    wd.battery_percent = -1
    wd.co2 = int(device_data["dashboard_data"]["CO2"])
    wd.humidity = int(device_data["dashboard_data"]["Humidity"])
    wd.noise = int(device_data["dashboard_data"]["Noise"])
    wd.pressure = float(device_data["dashboard_data"]["Pressure"])
    wd.temperature = float(device_data["dashboard_data"]["Temperature"])
    wd.link_status = int(device_data["wifi_status"])
    wd.last_seen = -1
    wd.time_utc = long(device_data["dashboard_data"]["time_utc"])
    output.wd.append(wd)

    # Populate outdoor modules data
    for module_data in device_data["modules"]:
        wd = Weatherdata()
        wd.name = str(module_data["module_name"])
        wd.battery_percent = int(module_data["battery_percent"])
        wd.co2 = -1
        wd.humidity = int(module_data["dashboard_data"]["Humidity"])
        wd.noise = -1
        wd.pressure = -1
        wd.temperature = float(module_data["dashboard_data"]["Temperature"])
        wd.link_status = int(module_data["rf_status"])
        wd.last_seen = long(module_data["last_seen"])
        wd.time_utc = -1
        output.wd.append(wd)

    # Return data
    return output


def publisher_loop(access_token, device_id):
    """
    Loop for publishing data in ROS
    :param access_token: Netatmo access token
    :param device_id: Netatmo Weather Station device ID
    """

    while not rospy.is_shutdown():
        station_data = get_station_values(access_token, device_id)
        pub.publish(station_data)
        rospy.sleep(10)


if __name__ == '__main__':
    # ROS node initialization
    pub = rospy.Publisher("weather", WeatherdataArray, queue_size=1)
    rospy.init_node("netatmo2ros", anonymous=False)

    # Read configuration
    config_path = rospy.get_param("~configpath")

    if os.path.isfile(os.path.join(config_path, "config.ini")):
        # File.ini expected
        config = ConfigParser.ConfigParser()
        config.read(os.path.join(config_path, "config.ini"))
        uname = config.get("user", "username")
        passw = config.get("user", "password")
        clientid = config.get("user", "client_id")
        clientsecret = config.get("user", "client_secret")
        deviceid = config.get("device", "device_id")
    elif os.path.isdir(config_path):
        # Directory with multiple files expected
        with open(os.path.join(config_path, "username"), "r") as f:
            uname = f.readline().rstrip()
        with open(os.path.join(config_path, "password"), "r") as f:
            passw = f.readline().rstrip()
        with open(os.path.join(config_path, "client_id"), "r") as f:
            clientid = f.readline().rstrip()
        with open(os.path.join(config_path, "client_secret"), "r") as f:
            clientsecret = f.readline().rstrip()
        with open(os.path.join(config_path, "device_id"), "r") as f:
            deviceid = f.readline().rstrip()
    else:
        sys.exit("Couldn't find configuration in path {0}.".format(config_path))

    # Get token and start loop
    wd_access_token = get_access_token(uname, passw, clientid, clientsecret)
    publisher_loop(wd_access_token, deviceid)
