---
source paper: https://autonaut.itk.ntnu.no/lib/exe/fetch.php?media=dune-help.pdf
---


# IMC: what is it?
Message Oriented Protocol - not a communication protocol, a messaging protocol
-  One XML document defines all messages
-  Generators for documentation, C++ and Java code
-  Serialization/deserialization to/from:
	– JSON
	– XML
	– Binary
- Serialized messages are used for logging and communication
- Binary serialization format can be translated to human-readable format (LLF)
![[Pasted image 20240903113410.png]]



# Control Architecture

See slides p.34
![[Pasted image 20240903113855.png]]


# DUNE core - Class database


- **Hardware** - Serial Port, GPIO, I2C, UCTK
- **Coordinates** - Transformation between referentials, WGS84, UTM
- **Database **- Connect, Run Statement, etc
- **IMC** - To deal with IMC messages (parser, serialization, json)
- **Math** - You cand find almost every math functions, matrix operations, derivative, etc
- **Network** - TCP, UDP, TDMA, etc
- **Parsers** - NMEAReader/Write, PlanConfigurations, etc
- **Time** - Delay, Delta, Counter, etc
- **Utils** - String, XML, NMEA parser, ByteCopy (big/little endian), etc


## *Source Code* 
navigation can be read from the presentation under [LSTS](LSTS), slide 30-34