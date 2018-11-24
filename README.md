# AWS Iot Bridge Example

## Overview
The **`aws_iot_mqtt_bridge`** package contains configuration and launch files to integrate a robot running ROS with AWS IoT by using the **`mqtt_bridge`** package. The mqtt_bridge acts a bridge between ROS systems and servers running the MQTT protocol. For more information please visit the [mqtt_bridge] repository.

**AWS IoT Core Summary**: AWS IoT provides secure, bi-directional communication between Internet-connected devices such as sensors, actuators, embedded micro-controllers, or smart appliances and the AWS Cloud. This enables you to collect telemetry data from multiple devices, and store and analyze the data. You can also create applications that enable your users to control these devices from their phones or tablets.

**Features in Active Development**:
- Use the built-in X.509 certificate as the unique device identity to authenticate AWS requests using the ROS AWS cloud service integrations, for more information, please visit [authorizating direct AWS calls](https://docs.aws.amazon.com/iot/latest/developerguide/authorizing-direct-aws.html)

**Keywords**: ROS IoT Core, mqtt, ROS bridge, mqtt_bridge

### License
The source code is released under an [Apache 2.0].

**Author**: AWS RoboMaker<br/>
**Affiliation**: [Amazon Web Services (AWS)]<br/>
**Maintainer**: AWS RoboMaker, ros-contributions@amazon.com

### Supported ROS Distributions
- Kinetic
- Lunar
- Melodic

## Installation

### IoT Device Certificate
You will need to create an AWS Account and configure the credentials to be able to communicate with AWS services. You may find [AWS Configuration and Credential Files] helpful. In addition to AWS credentials to communicate with AWS IoT, you will need to generate an X.509 certificate in order to identify your robot to AWS IoT Core. For more information, please visit [managing device certificates](https://docs.aws.amazon.com/iot/latest/developerguide/managing-device-certs.html).

### Building from Source
Create a ROS workspace and a source directory

    mkdir -p ~/ros-workspace/src

To build from source, clone the latest version from master branch and compile the package.

- Clone the package into the source directory

        cd ~/ros-workspace/src
        git clone https://github.com/aws-robotics/aws-iot-bridge-example.git

- Install dependencies

        cd ~/ros-workspace && sudo apt-get update
        rosdep install --from-paths src --ignore-src -r -y

- Build the packages

        cd ~/ros-workspace && colcon build

- Configure ROS library Path

        source ~/ros-workspace/install/setup.bash


## Launch Files
An example launch file called `aws_iot_bridge.launch` is provided, however the launch file cannot work without proper configuration. Since AWS IoT uses unique endpoints and certificates for each account and device, you must first discover your AWS IoT endpoint and generate a certificate before you can connect.

## Usage

### Run the node
- **With** launch file using parameters in .yaml format (need to be configured first)
  - ROS: `roslaunch aws_iot_mqtt_bridge aws_iot_bridge.launch bridge_params:=config/example_aws_iot_params.yaml`

- **Without** launch file using default values
  - ROS: `rosrun mqtt_bridge mqtt_bridge_node.py`

### Verify topics are publishing
- Go to your AWS account
- Find AWS IoT Core
- On the upper right corner, change region to `Oregon` if you launched the node using the launch file, or change to `N. Virginia` if you launched the node without using the launch file
- Select `Test` from the menu on the left side of the screen
- Type the name of one of the configured topics and click `Subscribe to topic`
- You should see messages begin appearing in the bottom half of the screen

## Configuration File and Parameters
An example configuration file called `example_aws_iot_params.yaml` is provided. When the parameters are absent in the ROS parameter server, the mqtt_bridge will use defaults, please refer to the [mqtt_bridge] package.

| Parameter Name | Description | Type | Allowed Values |
| -------------- | ----------- | ---- | -------------- |
| tls/ca_certs | Path to the Root CA obtained from the [AWS site](https://docs.aws.amazon.com/iot/latest/developerguide/managing-device-certs.html) | *std::string* | Path to valid root CA file |
| tls/certfile | Path to the device certificate | *std::string* | Path to valid certificate file in PEM format |
| tls/keyfile | Path to the device private key | *std::string* | Path to valid private key in PEM format |
| tls/tls_version | Protocol version to use when connecting to AWS TLS endpoint (5 is TLS1.2) | *int* | A valid python [TLS protocol](https://docs.python.org/2/library/ssl.html) |
| tls/tls_insecure | Indicator if certificates validation should be disabled, defaults to false  | *bool* | *bool* |
| connection/host | Host name of the AWS endpoint to connect | *std::string* | A valid DNS name or IP address |
| connection/port | Port to use when connecting to AWS IoT core (default is 8883) | *int* | A valid port number (1-65535) |
| connection/keepalive | TCP connection keep-alive | *int* | Number of seconds greater than zero |
| client/protocol | MQTT protocol to use when connecting (default is MQTT311 or 4) | *int* | Valid protocol number |

## Node

### mqtt_bridge
Bridges topics between ROS systems and an MQTT server

#### Subscribed Topics
None by default - mqtt_bridge must be configured to bridge topics to IoT

#### Published Topics
None by default - mqtt_bridge must be configured to bridge topics to IoT

#### Services
None


## Bugs & Feature Requests
Please contact the team directly if you would like to request a feature.

Please report bugs in [Issue Tracker].


[Amazon Web Services (AWS)]: https://aws.amazon.com/
[Apache 2.0]: https://aws.amazon.com/apache-2-0/
[mqtt_bridge]: https://github.com/groove-x/mqtt_bridge
[Issue Tracker]: https://github.com/aws-robotics/aws-iot-bridge-example/issues
[ROS]: http://www.ros.org
