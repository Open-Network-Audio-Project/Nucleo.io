# Nucleo.io
AES67 to TDM/I2S converter based on the STM32 Nucleo F767ZI development board.

TODO:

* get basic TCP/IP example up and running: basic DHCP Autodiscovery vs. Static IP

* get PTP daemon running on STM32F7. Port F4 code

* final system architecture draft, create TODO list from standards

* get basic publisher to multicast stream (test sine wave) which can be picked up 
  by a virtual soundcard.

* create dummy publisher/subscriber 8I8O system. Create corresponding ANEMAN profile

* get SAI output working for simple I2S interface

* [SAI to ADAT is ultimate goal - possibly external with FPGA?]

# Reference Software Tools:

* ANEMAN https://www.aneman.net/ : This is similar to Dante Controller, but works
  with a plugin format that means anyone can create a device profile/protocol that 
  can interoperate with an AES67 network.

* Merging VAD https://www.merging.com/products/aes67_vad_standard : OSX only, but
  allows for up to 64 channels of IO with the free version. Plays nicely with ANEMAN.

* Ravenna RVSC https://www.ravenna-network.com/resources/ : a windows-compatible 
  AES67 virtual soundcard which can be used for testing up to 16 channels of IO.

* Open-Source AES67 Daemon https://github.com/bondagit/aes67-linux-daemon : recently
  released, looks good, might free any long-term RAVENNA dependencies. Not really a
  production release yet.
