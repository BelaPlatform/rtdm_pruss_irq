
#!/bin/bash
#Once the kernel module is loaded, and a file `open`ed the device,
#the below will trigger a hardware interrupt
devmem2 0x4a320200 w 0x100000
