# simple_unav_console
A simple "virtual joypad" in Qt to control motors connected to a uNav board

This demo is useful to understand how to use "orblibcpp" [https://github.com/officinerobotiche/orblibcpp] library to communicate with uNav and to send velocity commands.

## Usage
```
$ git clone https://github.com/officinerobotiche/simple_unav_console.git
```
* Open the "pro" file with QtCreator and modify "ORBLIBCPP_PATH" with the absolute path to "orblibcpp" (e.g. ```include(../../orblibcpp/orblibcpp/orblibcpp.pri)```)
* Compile and run
