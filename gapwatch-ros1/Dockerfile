# Copyright 2024 Mattia Orlandi, Pierangelo Maria Rapa

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

# https://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

FROM ros:noetic
SHELL ["/bin/bash", "-c"]

# Update system
RUN apt update && apt upgrade -y

# Copy code and set working directory
COPY . /root
WORKDIR /root/gapwatch-ros1/ros1_ws

# By default, run roscore
ENTRYPOINT ["/root/gapwatch-ros1/entrypoint.sh"]
