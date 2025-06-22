# Progressive Robotics Technical Assessment

* Development for this project took place in a Ubuntu 22.04 Linux machine. 
* Docker is required for this project. By simply installing [Docker Desktop](https://www.docker.com/products/docker-desktop/), all of the necessary docker components should be installed.

## Build Instructions
2. Clone the project locally:
    ```
    git clone https://github.com/DimYfantidis/progressive-robotics-assessment
    ```
3. Start the Docker daemon. This could be achieved by simply opening the Docker Desktop app.
4. Open a terminal and navigate to the project's root directory.
5. Build the docker image using:
    ```
    docker compose up
    ```
    When finished, the image is built and the container will start running in the background while blocking the terminal.

## Execution Instructions
1. If the container is not already running, `docker compose up` within a terminal should launch the project using the already built image.
2. In a new terminal, launch a BASH shell inside the running container using:
    ```
    docker exec -it dimitris-container /bin/bash
    ```
    This can be repeated in as many terminals as needed, allowing multiple ROS 2 components to operate concurrently.
