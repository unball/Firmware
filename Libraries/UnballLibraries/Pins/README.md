**Pins connections for Unball robot components** 

- Radio - Teensy
  - gnd
  - vcc
  - mosi     -> pin 11
  - miso     -> pin 12
  - sck 	      -> pin 13
  - csn 	      -> pin 10
  - ce          -> pin 9
- Radio - Pro micro
  - gnd
  - vcc
  - mosi	-> pin 16
  - miso	-> pin 14
  - sck		-> pin 15
  - csn		-> pin 1
  - ce		-> pin 0

- Driver - Teensy/Bateria/Motor
  - gnd		-> gnd (Teensy)
  - vcc		-> vcc (Teensy)
  - vm 		-> positivo (Bateria)
  - gnd 	-> negativo (Bateria)
  - A01		-> 1 (Motor)
  - A02 	-> 2 (Motor)
  - pwmA	-> 3 (Teensy)
  - aIn2 	-> 4 (Teensy)
  - aIn1 	-> 5 (Teensy)
  - pwmB 	-> 21 (Teensy)
  - bIn2 	-> 22 (Teensy)
  - bIn1 	-> 23 (Teensy)
  - stby 	-> 6 (Teensy)

- Motor - Teensy
  - Vdd enc	(4)	-> vcc (Ambos)
  - gnd enc	(3)	-> gnd (Ambos)
  - EncA (6)	        -> 2 (MotorA)
  - EncB (6)	        -> 20 (MotorB)



