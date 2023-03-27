# TCP-IP communication scale driver

ROS driver for UC1 PEMU scale 

## TCP-IP address

### Parameters
	<!-- Desired scale IP -->
	<arg name="server_ip" default="10.60.0.102" />
	<!-- Desired port -->
	<arg name="server_port" default = "1433" />
           

### Fixed msgs
        readWeight_msg = b"q%\r\n"
        tareMachine_msg = b"q\"\r\n"

## Services

        ~read_weight (getWeight)
        ~tare_machine (std_srvs/Trigger)
        ~send_string (robotnik_msg/SetString)
        ~read_string (std_srvs/Trigger)
        ~open_connection (std_srvs/Trigger)
        ~close_connection (std_srvs/Trigger)
