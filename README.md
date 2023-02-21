# temperature-logger-arduino

Arduino based temperature logger

## Description

Temperature logger using Arduino Uno.

### Components:
- Temperature Sensor DS18B20
- LCD: 16x2 Alphanumeric LCD
- SD card
- RTC DS 1307

## Features
Logs temperature data into SD card at the interval specified by `LOGGER_INTERVAL_DATA_LOG` macro defined.

## Prototype Image
![Prototype Image](https://github.com/nepaldigitalsystems/temperature-logger-arduino/blob/main/res/snapshot.png)

## Usage

To use this project, follow these steps:

1. Clone the repository to your local machine.
2. Connect the components as shown in the prototype image.
3. Open the project file in the PlatformIo IDE.
4. Upload the code to the Arduino Uno board.
5. Monitor the output on the LCD or retrieve the data from the SD card.

## Contributing

To contribute to this project, follow these steps:

1. Fork this repository.
2. Create a branch: `git checkout -b <branch_name>`
3. Make your changes and commit them: `git commit -m '<commit_message>'`
4. Push to the original branch: `git push origin <project_name>/<location>`
5. Create a pull request.

Please see the [Contributing Guidelines](CONTRIBUTING.md) for more information.

## License

This project is licensed under the [MIT License](LICENSE).```
