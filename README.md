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