# Violin-pd host applications

This directory contains the xmit server and receiver applications running on host PCs.

The receiver must be run on the PC connected to the receiver nucleo board with a USB cable.
(This cable is connected to the USB connector located next to the Ethernet connector on the Nucleo board.)

The transmitter must be run an a PC with an Internet connection, with port 3443 enabled in the firewall.
The transmitter's IP address is compiled into the firmware running on the xmit nucleo board.
Thus you need to re-build the transmitter firmware after you have selected a PC as the xmit server.

## Credits

The VIOLIN project has been co-financed by the European Union and Greek national funds through the Operational Program Competitiveness, Entrepreneurship and Innovation, under the call RESEARCH-CREATE-INNOVATE (project code: T1EDK-02419).

The [libserialport library](https://sigrok.org/wiki/Libserialport) is used for USB/Serial communication.

The Reed-Solomon encoder/decoder uses the [libcorrect library](https://github.com/quiet/libcorrect) Copyright (c) 2016, Brian Armstrong

The COBS decoder is from the corresponding [Wikipedia article](https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing)


## Transmit server

The transmit server continuously transmits a file.
(There is no reverse VLC channel to acknowledge the reception, so the transmission is continuous).
You must kill it to stop it!

The MCU on the Nucleo board is acting as a network bridge between Ethernet and the VLC link.
This design choice keeps the firmware simple.
It also means that minimal storage is required on the MCU board.
Therefore the server has to do much of the encoding, split into packets (we call them **flits** in this project), etc.

A flit is essentially a VLC frame.
It contains up to 1024 bytes of data to which parity and some headers are added.

The server executable expects a single argument: the filename of the file to be transmitted.
The file is first split into flits, and encoded for error correction using Reed-Solomon.
To avoid doing this encoding all the time, it is done once and stored encoded in a temporary file.

The nucleo xmit board sends a request message though its Ethernet connection to the host to port 3443, when it has just started the transmission of the previous flit over the VLC link.
The server responds with the next flit of the file.
Once the whole file has been transmitted, the process re-starts from the beginning of the file.
When network errors are encountered, the transmission starts aftesh.
An automatically created log file records when disconnections occur, etc.

## Receiver application

 The VLC receiver application instructs the attached nucleo board to start the reception of VLC frames, reads the USB/Serial port for incoming data and stores it into a file. 
 When the file is complete, the nucleo board is instructed to stop receiving and the app exits.

The executable takes 3 optional command-line arguments:
- `-v`: verbosity level, higher number means more information is shown.
- `-n`: The filename, in the current directory, to store the incoming data. "received" is used if no filename is specified.
- `-f`: A number for the nucleo timer's digital filter. This filters out narrow glitches. The default is 0.

Do not add spaces between the option and the argument. E.g. verbose 3 is -v3.

The application starts two threads.
The `usbTask` communicates with the nucleo board and stores received flits into a queue.
The main thread reads from the queue, performs Reed-Solomon decoding and stores the data into the file system.
We decided to split the application so that a potentially slow Reed-Solomon decoding and file access does not delay the reception of incoming data from the board.

## Build

To build run
```bash
make xmit
make recv
```

The executables will be found under directory `build`.

