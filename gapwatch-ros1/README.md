# gapwatch-ros1
ROS1 wrapper for GAPWatch. It consists in two ROS nodes:

- the _streamer_ is the actual wrapper for GAPWatch: it reads the EMG data packets from a TCP client socket and publishes them under the `emg` topic;
- the _logger_ subscribes to the `emg` topic and re-transmits the EMG data packets it receives via TCP.

## Usage
The project relies on Docker:

- first, build the image with `docker build --rm -t gapwatch-ros1:noetic .`;
- then, launch the _streamer_ with `docker run --rm -it --net=host gapwatch-ros1:noetic gapwatch_streamer streamer.py _socket_port:=<SOCKET_PORT>` (by default, the socket port is 3333);
- to log the data via TCP, launch the _logger_ with `docker run --rm -it --net=host gapwatch-ros1:noetic gapwatch_logger logger.py _server_addr:=<SERVER_ADDR> _server_port:=>SERVER_PORT>` (by default, the server address is 172.17.0.1 and the server port is 3334).

One can then read the EMG data stream from the _logger_'s TCP: for instance, to stream the data to the [BioGUI](https://github.com/pulp-bio/biogui), the [`interface_gapwatch_ros1.py`](https://github.com/pulp-bio/emg-ros/blob/main/gapwatch-ros1/interface_gapwatch_ros1.py) interface file is provided.

## Author
- [Mattia Orlandi](https://www.unibo.it/sitoweb/mattia.orlandi/en)
- [Pierangelo Maria Rapa](https://www.unibo.it/sitoweb/pierangelomaria.rapa/en)

## License
All files are released under the Apache-2.0 license (see [`LICENSE`](https://github.com/pulp-bio/emg-ros/blob/main/LICENSE)).
