# BlackBird-gym

### Description

Blackbird is an open source, low-cost bipedal robot capable of high resolution force control. It is a research and education platform designed for college and post-grad students interested in studying the advanced field of robotics. Using a novel custom actuator design and supporting software, the system is capable of matching the control requirements of modern research robots for a fraction of the cost by replacing expensive hardware with complex electrical-control solutions. We have implemented multiple algorithms that allow the platform walk, run, spin, and jump based on modern research methods. Most of the design is 3D printed, including the actuators, which allows it to be easily manufactured by students and enthusiasts.

### Installation

Install the blackBird Env with `python3 -m pip install -e gym-blackBird`.
If errors arise, try: `python3 -m pip install --user -e gym-blackBird`


You can create an instance of the environment with `gym.make('gym_blackBird:blackBird-v0')`

Test the environment using : ```python3 BlackBird-gym/EnvTest.py ```

![demo](demo.gif)

More details about the hardware implementation can be found at [here](https://hackaday.io/project/160882-blackbird-bipedal-robot)
