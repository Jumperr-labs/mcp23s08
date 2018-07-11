# Jumper Virtual Lab Peripheral Model - MCP23S08
This repo contains a model for the [MCP23S08](https://datasheet.octopart.com/MCP23S08-E/SO-Microchip-datasheet-15865.pdf) GPIO extender for [Jumper Virtual Lab](https://vlab.jumper.io).

For more information, visit [the docs](https://docs.jumper.io).

### Prerequisites
- GCC and Make: `apt install build-essential`
- [Jumper Virtual Lab](https://docs.jumper.io)

## Usage
- Follow the following steps in order to build the peripheral model:

  ```bash
  git clone https://github.com/Jumperr-labs/mcp23s08.git
  cd mcp23s08
  make
  ```

- If the peripheral model was build successfully, the result will be ready under "_build/MCP23S08.so".
Copy this file to you working diretory, same one as the "board.json" file.
- Add the following component in your "board.json" file. Make sure to change the pin numbers to fit your configuration.

```json
{
  "name": "MCP23S08",
  "id": 1,
  "type": "Peripheral",
  "file": "_build/MCP23S08.so",
  "config": {
    "pins": {
      "cs": 9,
      "sck": 5,
      "si": 7,
      "so": 6,
      "INT": 1,
      "RESET": 30,
      "A0": 31,
      "A1": 30,
      "M0": 0,
      "M1": 12,
      "M2": 13,
      "M3": 14,
      "M4": 15,
      "M5": 16,
      "M6": 17,
      "M7": 18
    }
  }
}
```

## License
Licensed under the Apache License, Version 2.0. See the LICENSE file for more information
