# simple_unav_joypad
A simple "virtual joypad" in Qt to control motors connected to a uNav board

This demo is useful to understand how to use "orblibcpp" [https://github.com/officinerobotiche/orblibcpp] library to communicate with uNav and to send velocity commands.

## Usage
```
$ mk dir simple_unav_joypad
$ cd simple_unav_joypad
$ git clone https://github.com/officinerobotiche/orblibcpp
```
* Open the "pro" file with QtCreator and modify "ORBLIBCPP_PATH" with the absolute path to "orblibcpp"
* Compile and run
