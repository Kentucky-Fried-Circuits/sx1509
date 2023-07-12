# V1.0.0 
## Major Changes 
removed I2C driver intstall from sx1509::begin()

## Bugfixes
changed master/slave language to client/server per the Modbus consortium where it didn't affect the interfaces
FIXME(?) remove depencency on UncleRus' i2cdev and esp_idf_lib_helpers
* FIXME sx1509.hpp:52:39: warning: comparison is always false due to limited range of 
