# MobileRobot

## About The Project

[![Project Screen Shot][project-screenshot]]()

This repository contains a series of code for mobile robots based on Arduino (compatible) boards [Arduino](https://www.arduino.cc/). 

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Built With

* [![Arduino][arduino-shield]][arduino-url]

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- GETTING STARTED -->
## Getting Started

To set up the project locally, you need to install (if not already the case) some dependencies.
To get a local copy up and running follow these steps.

### Prerequisites

* Arduino

Launch Arduino IDE

* mBot library

1. Download the mBot library here: https://codeload.github.com/Makeblock-official/Makeblock-Libraries/zip/master
2. From the Arduino IDE: "Sketch-> Include Library-> Add .ZIP Library-> select the downloaded file-> Open”
  
* Adafruit motor shield v2 library

1. From the Arduino IDE: "Tools-> Manage library"
2. Type adafruit motor to locate the library
3. Click Install

More information available [here](https://learn.adafruit.com/adafruit-motor-shield-v2-for-arduino/overview).
  
### Installation

1. Clone the repo
   ```sh
   git clone https://github.com/GuillaumeGibert/MobileRobot.git
   ```
2. Open Arduino IDE
3. Open one of the provided code
4. Compile and upload it to your Arduino board

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- USAGE EXAMPLES -->
## Usage

### Motion (mBot)

1. Open Arduino IDE
2. Open the code `motion.ino`
3. Compile and upload it to your Arduino board


### Sensing distance (mBot)

1. Open Arduino IDE
2. Open the code `ultrasonicSensor.ino`
3. Compile and upload it to your Arduino board


### Roomba behavior (mBot)

1. Open Arduino IDE
2. Open the code `roomba.ino`
3. Compile and upload it to your Arduino board


### Inertial Measurement Unit (mBot)

1. Open Arduino IDE
2. Open the code `gyro.ino`
3. Compile and upload it to your Arduino board


### Square path (mBot)

1. Open Arduino IDE
2. Open the code `control.ino`
3. Compile and upload it to your Arduino board


### Line follower (mBot)

1. Open Arduino IDE
2. Open the code `lineFollower.ino`
3. Compile and upload it to your Arduino board


### Mecanum (Arduino + Adafruit Motor Shield v2)

1. Open Arduino IDE
2. Open the code `holonomic_control.ino`
3. Compile and upload it to your Arduino board


<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- LICENSE -->
## License

Distributed under the GPL License. See `LICENSE.txt` for more information.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- CONTACT -->
## Contact

Guillaume Gibert

Project Link: [https://github.com/GuillaumeGibert/MobileRobot](https://github.com/GuillaumeGibert/MobileRobot)

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[arduino-shield]: https://img.shields.io/badge/Arduino_IDE-00979D?style=for-the-badge&logo=arduino&logoColor=white
[arduino-url]: https://www.arduino.cc/
[python-shield]: https://img.shields.io/badge/Python-3776AB?style=for-the-badge&logo=python&logoColor=white
[python-url]: https://www.python.org/
[opencv-shield]: https://img.shields.io/badge/OpenCV-27338e?style=for-the-badge&logo=OpenCV&logoColor=white
[opencv-url]: https://opencv.org/

[project-screenshot]: images/screenshot.png

[contributors-shield]: https://img.shields.io/github/contributors/GuillaumeGibert/MobileRobot.svg?style=for-the-badge
[contributors-url]: https://github.com/GuillaumeGibert/MobileRobot/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/GuillaumeGibert/MobileRobot.svg?style=for-the-badge
[forks-url]: https://github.com/GuillaumeGibert/MobileRobot/network/members
[stars-shield]: https://img.shields.io/github/stars/GuillaumeGibert/MobileRobot.svg?style=for-the-badge
[stars-url]: https://github.com/GuillaumeGibert/MobileRobot/stargazers
[issues-shield]: https://img.shields.io/github/issues/GuillaumeGibert/MobileRobot.svg?style=for-the-badge
[issues-url]: https://github.com/GuillaumeGibert/MobileRobot/issues
[license-shield]: https://img.shields.io/github/license/GuillaumeGibert/MobileRobot.svg?style=for-the-badge
[license-url]: https://github.com/GuillaumeGibert/MobileRobot/blob/master/LICENSE.txt
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://linkedin.com/in/guillaume-gibert-06502ba4
