# Tools

This folder contains tools which are useful for running examples from the IASDK under certain set ups

## UDP Proxy

This tool is used to forward UDP traffic between two sets of IP addresses and ports. It is useful when running the iasdk under WSL and the UDP traffic needs to be sent between your local pc and your WSL instance.

Usage example is as follows:
sudppipe.exe -b <machine_ip_address> <wsl_ip_address> <destination_port> <source_port>

Ensure that source and destination ports are NOT the same and that the radar is configured to send point cloud messages to the machine ip address as listed above