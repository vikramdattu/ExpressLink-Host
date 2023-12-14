# ExpressLink Host example running on ESP32

### Pin configuration
 - Please set the UART and event pins for the communication via `idf.py menuconfig`:

```bash
menuconfig > ExpressLink Pin Config
```

 - You will also need to modify following line to set the IOT Core endpoint:
```c
const char *AWS_IOT_CORE_ENDPOINT = "<hash>-ats.iot.<region>.amazonaws.com";
```

### License
 - ExpressLink interface library is using sources by STMicroelectronics's `I-CUBE-ExpressLink` [project](https://github.com/stm32-hotspot/I-CUBE-ExpressLink).
 - Espressif specific code is covered under `Apache-2` license. Other licenses are as per mentioned in the files.
