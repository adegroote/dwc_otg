It is an attempt to write missing driver for the Rasberry Pi for NetBSD,
including:
   - the usb controller host dwc_otg
   - the ethernet part of smsc 95xx (the hub is handled by uhub(4)

Both drivers originate from the FreeBSD versions. At the moment, dwc_otg
compiles and attaches succesfully and is able to perform some control
transfers

The code has been merged in NetBSD trunk. Future developpement is now done
directly in the NetBSD repository. So, no more developpement is expected in 
this repository.
