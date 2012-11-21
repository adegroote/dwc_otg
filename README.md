It is an attempt to write missing driver for the Rasberry Pi for NetBSD,
including:
   - the usb controller host dwc_otg
   - the ethernet card smsc 95xx

We start from the FreeBSD one. At the moment, the dwc_otg compiles and attaches
succesfully but is not really capable. 
