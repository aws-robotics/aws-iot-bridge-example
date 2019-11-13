
# Connecting to AWS IoT Core
In order to connect a system running ROS to the AWS IoT Core service, you must bridge some topics using
the mqtt_bridge - https://github.com/groove-x/mqtt_bridge

Some amount of setup is needed in order to be able to use the mqtt_bridge to connect to AWS IoT Core.
Specifically, you will need a certificate, private key, root CA certificate, and the related configuration, however you will also need the device registered with the AWS IoT Core service.

This package contains configuration intended to make this process easier. There is no reason
you cannot configure everything manually using the command line or console, using the data here as an example.

# Quickstart
1) Configure your AWS credentials:
```
aws configure
```

2) Create the IoT device configuration

3) Define your topics:
```
--> my_package_params.yaml
bridge:
  # ping pong
  - factory: mqtt_bridge.bridge:RosToMqttBridge
    msg_type: std_msgs.msg:Bool
    topic_from: /ping
    topic_to: ping
  - factory: mqtt_bridge.bridge:MqttToRosBridge
    msg_type: std_msgs.msg:Bool
    topic_from: ping
    topic_to: /pong
```

4) Define your launch file:
```
--> my_package.launch
<launch>
    <include file="$(find aws_iot_mqtt_bridge)/launch/aws_iot_bridge.launch">
        <arg name="bridge_params" value="$(find my_package)/config/my_package_params.yaml" />
    </include>
</launch>
```

5) Run your launch file:
```
roslaunch my_package my_package.launch
```

# Setting up mqtt_bridge
The [mqtt_bridge](https://github.com/groove-x/mqtt_bridge) package depends on a few pip and system packages which are not in the ROS Kinetic distro. In order to install mqtt_bridge, you must either follow the installation instructions in the repository, or you can also just add the (mqtt-bridge-deps.yaml)[rosdistro/mqtt-bridge-deps.yaml] to your sources list.

```
sudo bash -c "echo \"yaml https://raw.githubusercontent.com/aws-robotics/aws-iot-bridge-example/release-v1/aws_iot_mqtt_bridge/rosdep/mqtt-bridge.yaml\" > /etc/ros/rosdep/sources.list.d/30-mqtt-bridge.list"
rosdep update
rosdep install --from-paths aws_iot_mqtt_bridge --ignore-src -y
```

# Configuring AWS
In order to connect to AWS IoT, you will need an AWS account and an access key and secret key in order
to sign API requests. This key is only needed during setup, however you will need to make sure you run
the create-device.py script on a machine that has already been configured with your AWS credentials.

Setup your AWS account: https://aws.amazon.com/getting-started/

Configure the CLI for authentication: https://docs.aws.amazon.com/cli/latest/userguide/cli-chap-getting-started.html

# Creating your certificates
AWS IoT devices can use X.509 certificates for device authentication. For more information, please refer
to https://docs.aws.amazon.com/iot/latest/developerguide/managing-device-certs.html

Several different IoT resources need to be created and related to one another before you can begin
communicating from your robot. These elements are:
- Certificate, Public key, Private key
- Policy attached to the certificate
- IoT Thing
- Thing principal attached to the certificate
- An IoT endpoint

Once all of these parts have been created, the mqtt_bridge needs to be configured to use the certificate
and endpoint when connecting to the AWS IoT service.

# Creating your configuration
The configuration needed to securely connect to AWS IoT are the Root CA certificate, a device certificate, including both the public and private key, and a yaml file containing the correctly parameters for the mqtt_bridge node.
```
Params file: ./config/aws_iot_params.yaml
  Root file: ./config/certs/AmazonRootCA1.pem
  Cert file: ./config/certs/device.cert.pem
 Public key: ./config/certs/device.public.pem
Private key: ./config/certs/device.private.pem
```

### Parameters File
The parameters are defined under the **Configuration** section of the [mqtt_bridge](https://github.com/groove-x/mqtt_bridge) package. Here is an example configuration:

```
client:
  protocol: 4
connection:
  host: abcexample-ats.iot.us-west-2.amazonaws.com
  port: 8883
  keepalive: 60
tls:
  tls_version: 5  # ssl.PROTOCOL_TLSv1_2
  tls_insecure: false
private_path: device/0xdeadbeef
```

### Device Certificates
Additionally, the bridge will also need the *tls/ca_certs*, *tls/certfile*, *tls/keyfile* parameters. The [aws_iot_bridge.launch](launch/aws_iot_bridge.launch) file loads assumes these files exist under the **config/certs** directory and sets the parameters.

```
<rosparam param="mqtt/tls/ca_certs" subst_value="true">$(find aws_iot_mqtt_bridge)/config/certs/AmazonRootCA1.pem</rosparam>
<rosparam param="mqtt/tls/certfile" subst_value="true">$(find aws_iot_mqtt_bridge)/config/certs/device.cert.pem</rosparam>
<rosparam param="mqtt/tls/keyfile" subst_value="true">$(find aws_iot_mqtt_bridge)/config/certs/device.private.pem</rosparam>
```

While this is convenient to use ```$(find)``` for the certificates, it is recommended that you do not include the keys with your software distribution and place them in a separate, secure location on the device. 

# Running the bridge
The default launch scripts in the aws_iot_mqtt_bridge package expect the certificate files
to be living under the **config/certs** directory. The aws_bridge.launch configuration loads
the parameters from the generated aws_iot_params.yaml file, as well as taking in a **bridge_params**
argument, which should set all of the topics which should be bridged.

The default configuration layout:
```
aws_iot_mqtt_bridge
    \-- config
        - aws_iot_params.yaml
        \-- certs
            - AmazonRootCA1.pem
            - device.cert.pem
            - device.private.pem
            - device.public.pem
```

In order to actually bridge some topics, you will need to include a bridge configuration. The
**aws_bridge.launch** expects a **bridge_param** argument, pointing to the parameter file
containing the topic configuration. An example of a launch file to run the bridge:
```
<launch>
    <include file="$(find aws_iot_mqtt_bridge)/launch/aws_iot_bridge.launch">
        <arg name="bridge_params" value="$(find my_package)/config/my_package_params.yaml" />
    </include>
</launch>
```

An example configuration topic configuration (borrowed from mqtt_bridge):
```
bridge:
  # ping pong
  - factory: mqtt_bridge.bridge:RosToMqttBridge
    msg_type: std_msgs.msg:Bool
    topic_from: /ping
    topic_to: ping
  - factory: mqtt_bridge.bridge:MqttToRosBridge
    msg_type: std_msgs.msg:Bool
    topic_from: ping
    topic_to: /pong
```
