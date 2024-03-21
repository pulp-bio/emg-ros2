# biogap-ros2
ROS2 wrapper for BioGAP. It consists in two ROS nodes:

- the _streamer_ is the actual wrapper for BioGAP: it reads the EMG data packets from a serial port (default: `/dev/ttyACM0`) and publishes them under the `emg` topic;
- the _dumper_ subscribes to the `emg` topic and dumps the EMG data packets it receives to a UNIX FIFO in `./biogap-ros2/fifo`.

## Usage
The project relies on Docker:

- first, build the image with `docker build --rm -t biogap-ros2:humble .`;
- then, launch both the publisher container and the receiver container with `docker-compose up`;
- to stop the containers, press `Ctrl+C` and run `docker-compose down`.

One can then read the EMG data stream from the UNIX FIFO.

## Author
- [Mattia Orlandi](https://www.unibo.it/sitoweb/mattia.orlandi/en)
- [Pierangelo Maria Rapa](https://www.unibo.it/sitoweb/pierangelomaria.rapa/en)

## License
All files are released under the Apache-2.0 license (see [`LICENSE`](https://github.com/pulp-bio/biogap-ros2/blob/main/LICENSE)).
