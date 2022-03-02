<!-- Installation

        py -3 -m pip install robotpy robotpy[ctre, navx] numpy -->

<!-- Start simularor

        py -3 robot.py sim -->
<!-- Install to RoboRIO

        py -3 -m robotpy_installer download-python
        py -3 -m robotpy_installer install-python

Install modules to RoboRIO
  

        py -3 -m robotpy_installer download robotpy robotpy[ctre, navx] numpy
        py -3 -m robotpy_installer install robotpy robotpy[ctre, navx] numpy

Testing code

Coverage

        py -3 -m pip install coverage
        py -3 -m coverage run --source=util robot.py sim
        py -3 -m coverage report -m


Deployment

        py -3 robot.py deploy -->


# frc2022 3985 RobotPy code

### Connect a controller and try out the robot on the simulator.

Commands:
- Installation
    - `py -3 -m venv robot-venv`
    - in git bash: `source ./robot-venv/Scripts/activate`
    - in cmd: `.\robot-venv\Scripts\activate.bat`
    - `pip install robotpy robotpy[ctre] robotpy[navx] numpy`
- Start simularor
    - `py -3 robot.py sim`
- Install to RoboRIO
    - `py -3 -m robotpy_installer download-python`
    - `py -3 -m robotpy_installer install-python`
- Install modules to RoboRIO
    <!-- - `py -3 -m robotpy_installer download robotpy`
    - `py -3 -m robotpy_installer install robotpy`
    - `py -3 -m robotpy_installer download robotpy[ctre, navx]`
    - `py -3 -m robotpy_installer install robotpy[ctre, navx]` -->
    - `py -3 -m robotpy_installer download robotpy robotpy[ctre, navx] numpy`
    - `py -3 -m robotpy_installer install robotpy robotpy[ctre, navx] numpy`
- Testing code
    - Coverage
        - `py -3 -m pip install coverage`
        - `py -3 -m coverage run --source=util robot.py sim`
        - `py -3 -m coverage report -m`
- Deployment
    - `py -3 robot.py deploy`
