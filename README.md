LibreVisa
=========

My Fork of the [samangh/librevisa-orig:master](https://github.com/samangh/librevisa-orig) librevisa for adding some basic serial support so that "ASRL1::INSTR" etc works


samangh/librevisa-orig README
-----------------------------

My Fork of LibreVisa, extended for use with PyVisa.

I am trying to control a USBTMC Device using PyVisa and LibreVisa on a Debian Linux System.
For use with PyVisa I had to implement the viParseRsrc() function in LibreVisa. 

My Python Script: 
```
  #!/usr/bin/env python3
  import visa
  rm = visa.ResourceManager('/home/charly/dev/librevisa.git/src/.libs/libvisa.so')
  rm.list_resources()
  inst = rm.open_resource('USB5::0x03eb::0x2423::123456');
  idn = inst.query('*IDN?')
  print(idn)
```

Links: 
* LibreVisa samangh fork: https://github.com/samangh/librevisa-orig
* LibreVisa: http://www.librevisa.org
* LibreVISA Original Code: http://www.librevisa.org/git/librevisa.git
* PyVisa Code: https://github.com/hgrecco/pyvisa
* PyVisa Docu: https://pyvisa.readthedocs.org/en/1.6/
* IVI Specifications: http://www.ivifoundation.org/Downloads/Specifications.htm
* The Visa Library Specification: http://www.ivifoundation.org/docs/vpp43.doc
