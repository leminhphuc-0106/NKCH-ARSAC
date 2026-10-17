---
name: check-input-data
description: "Use when: checking whether a program, sketch, script, or library receives external input such as serial data, command strings, user input, sensor values, or file-based data. Helps trace initialization, read calls, parsing, and the data-ready flow to determine if input is actually accepted."
---

# Check Whether Code Receives Input Data

## Goal

Determine whether the code accepts incoming data and, if so, where it reads, parses, and uses it.

## Workflow

1. Locate the input entry points.
   - Look for `Serial.begin(...)`, `Serial.available()`, `Serial.read()`, `read()`, `scanf()`, `cin >>`, `input()`, `parseInt()`, or `recv*` functions.
   - Check whether the code is set up to accept data from a port, terminal, file, socket, or sensor.

2. Verify that the program initializes the input channel.
   - Confirm there is a setup call such as `Serial.begin(115200);` or the equivalent for another interface.
   - If no initialization exists, the program is unlikely to receive real data.

3. Trace the data-reading loop.
   - Inspect the main loop or a helper method that repeatedly checks for incoming bytes or values.
   - Watch for patterns like:
     - `while (Serial.available())`
     - `char c = Serial.read();`
     - `if (*newData)`
     - `sscanf(...)` or `atoi(...)`

4. Confirm the parsing and state-change path.
   - Check whether the code converts the bytes into variables, buffers, or commands.
   - A valid signal is a flag such as `newData = true`, a queue push, or a parsed command being processed.

5. Decide whether input is actually being received.
   - If the code reads bytes and stores or parses them, it receives input data.
   - If it only prints or calculates without any read path, it does not receive live input.

## Decision Points

- If `Serial.available()` and `Serial.read()` are present, the code is receiving serial input.
- If data is parsed into a buffer or command and then queued, the code is actively processing input.
- If there is no read call and no input initialization, the code is not receiving external data.
- If data is read but not used, the code still receives input but may ignore it.

## Completion Checks

The code is confirmed to receive input if all of these are true:

- The input channel is initialized.
- A read function or listener is triggered.
- Incoming bytes or values are captured.
- The result is stored, parsed, or used in logic.

## Example from This Project

This workspace contains a real serial-command flow:

- [mega2560/src/mega2560.cpp](../../mega2560/src/mega2560.cpp): `Serial.begin(115200);` in `setup()` and `indata.recvByte(line, buffer, newData);` inside `loop()`.
- [mega2560/lib/MControl/MControl.cpp](../../mega2560/lib/MControl/MControl.cpp): `while (Serial.available()) { char c = Serial.read(); ... }` shows that incoming serial data is read.
- The code then checks `if (*newData)` and enqueues the received buffer, proving the input is processed.

Conclusion for this project: yes, this code does receive input data through the serial interface.

## Suggested Prompt Patterns

- "Does this code receive input data?"
- "Where does this program read external values?"
- "Trace the input path from the serial port to parsing and processing."
- "Show whether this sketch accepts user or serial commands."
