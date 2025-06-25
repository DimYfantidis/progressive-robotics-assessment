# Progressive Robotics Technical Assessment

* Development for this project took place in a Ubuntu 22.04 Linux machine. 
* Docker is required for this project. By simply installing [Docker Desktop](https://www.docker.com/products/docker-desktop/), all of the necessary docker components should be installed.


## Build Instructions

1. Clone the project locally:
    ```
    git clone https://github.com/DimYfantidis/progressive-robotics-assessment
    ```
2. Start the Docker daemon. This could be achieved by simply opening the Docker Desktop app.

3. Open a terminal and navigate to the project's root directory.

4. Build the docker image and start the [noVNC](https://wiki.ros.org/docker/Tutorials/GUI) image using:
    ```
    docker compose up -d dimitris-container novnc
    ```
    When finished, the image is built and the container will start running in the background while blocking the terminal. `noVNC` is necessary for accessing graphical elements while working within containers.


## Execution Instructions

1. If the container is not already running: By typing the following command in a shell within the root directory, the container should start by using the already built image:
    ```
    docker compose up --no-build -d dimitris-container novnc
    ```
    Make sure that the docker daemon is running in the background.

2. In a new terminal, launch a BASH shell inside the running container using:
    ```
    docker exec -it dimitris-container /bin/bash
    ```
    This can be repeated in as many terminals as needed, allowing multiple ROS 2 components to operate concurrently.

3. Top stop the containers' execution, type `exit` in each terminal accessed within the aforementioned command (guest terminals) and then type `docker compose down` in the root directory of the project (host machine).

## Build the ROS2 Workspace 

Within the BASH shell inside the running container, navigate into the workspace and build it by typing:
```
cd ros2_ws && colcon build --symlink-install
```

## Run the implemented ROS2 nodes

* You will first need to source the workspace by typing:
    ```
    source install/setup.bash
    ```

* To execute the service client node, type:
    ```
    ros2 run linear_algebra_common client
    ```

    Once it starts the client will start publishing to the "client" topic (can be specified as different through the `topic` node parameter). 
    While the server is unavailable (e.g. it hasn't started yet), the client will attempt to connect with it every one second, logging every attempt
    in the terminal.  

    The node will notify the user when communication with the service has been established and when the response is received along with its result. 
    After that the node's publisher will keep publishing messages forever until interrutped.  

    The client fetches the matrix and vector data from yaml files found under the `config` directory, found within the `linear_algebra_resources` package. 
    By default the client reads the `client_data.yaml` file but can be instructed to read from another YAML file under the `config` directory by using the 
    node's `yaml_filename` parameter. In the case of any ill-formatted input within the YAML file, the user will be informed and the request will not be sent.  

* To execute the service server node, type:
    ```
    ros2 run linear_algebra_common service
    ```

    Once it starts, the server will notify the user when it's ready (listening for requests), when it has received a request and when it has processed it. 
    In the case of receiving wrong input from the client (matrix and vector dimension mismatch), the server will log an error, mark the processing of 
    the request as unsuccessful and return from the service callback prematurely.

* There is no specific sequence in which the client and server nodes must be executed or stopped.

