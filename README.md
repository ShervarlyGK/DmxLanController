C++Style branch
DMX input/output controller based on Arduino-MEGA board. 
Accepts control sequences via the Ethernet port and provides:
1) the issuance of appropriate commands to DMX devices;
2) the transit of commands to DMX devices from the DMX input to the output with the possibility of changing channel numbers.

The modified DMX 512 library is used in the project:
/***************************************************************************
*
* Title          : Arduino DMX512 library. 4 input/output universes.
* Version        : v 0.3 beta
* Last updated   : 07.07.2012
* Target         : Arduino mega 2560, Arduino mega 1280, Arduino nano (1 universe)  
* Author         : Toni Merino - merino.toni at gmail.com
* Web            : www.deskontrol.net/blog
*
* Based on ATmega8515 Dmx library written by Hendrik Hoelscher, www.hoelscher-hi.de
***************************************************************************
