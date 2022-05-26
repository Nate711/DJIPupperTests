# Stanford TC-Pupper

## Installation
* Clone this repository ``git clone https://github.com/Nate711/DJIPupperTests.git``
* Go into the repo directory ``cd DJIPupperTests``
* Run ```git submodule update --init``` to install required libraries
* Install PlatformIO IDE (addon for VSCode)
* Use the "Upload" feature of PlatformIO to upload the firmware to the Teensy
* The last two steps are covered by this video: https://knowledge.autodesk.com/community/screencast/cbf5a477-08e8-4b54-aa1b-aeffc3e5aa3d 
* If it's working, the Teensy should blink for 5s on bootup and then start printing debug info over Serial. The debug information is encoded using msgpack so it will look like gibberish unless you decode it.

## Usage

## Notes
* Use the no-imu branch if you have an older Pupper without an IMU.
