# biogap-ros2
ROS2 wrapper for BioGAP. It consists in two ROS nodes:

- the _streamer_ is the actual wrapper for BioGAP: it reads the EMG data packets from a serial port and publishes them under the `emg` topic;
- the _logger_ subscribes to the `emg` topic and dumps the EMG data packets it receives to a UNIX FIFO.

## Usage
The project relies on Docker:

- first, build the image with `docker build --rm -t biogap-ros2:humble .`;
- then, launch the _streamer_ with `docker run --rm -it --device=<SERIAL_PORT> biogap-ros2:humble streamer --ros-args -p serial_port:="<SERIAL_PORT>" -p baud_rate:=BAUD_RATE -p gain:=GAIN` (by default, the Baud rate is 256000 and the gain is 6);
- to log the data to the UNIX FIFO, launch the _logger_ with `docker run --rm -it -v ./biogap-ros2:/root/shared biogap-ros2:humble logger --ros-args -p fifo_path:=/root/shared/<FIFO_NAME>`.

One can then read the EMG data stream from the UNIX FIFO: for instance, to stream the data to the [BioGUI](https://github.com/pulp-bio/biogui), the [`interface_biogap_ros2.py`](https://github.com/pulp-bio/emg-ros2/blob/main/biogap-ros2/interface_biogap_ros2.py) interface file is provided.

## Author
- [Mattia Orlandi](https://www.unibo.it/sitoweb/mattia.orlandi/en)
- [Pierangelo Maria Rapa](https://www.unibo.it/sitoweb/pierangelomaria.rapa/en)

## License
All files are released under the Apache-2.0 license (see [`LICENSE`](https://github.com/pulp-bio/emg-ros2/blob/main/LICENSE)).