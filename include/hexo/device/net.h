#if !defined(DEVICE_H) || defined(DEVICE_NET_H_)
#error This file can not be included directly
#else

#define DEVICE_NET_H_

#include "../types.h"
#include "../error.h"




/** Net device class read() function tempate. */
#define DEVNET_READ(n)	ssize_t  (n) (struct device_s *dev, uint8_t *data, size_t size)

/** Net device class read() methode shortcut */

#define dev_net_read(dev, ...) (dev)->drv->f.net.f_read(dev, __VA_ARGS__ )
/**
   Net device class read() function type.  Read bytes data from the
   device. Should not block if unable to read more bytes.

   @param dev pointer to device descriptor
   @param data pointer to data buffer
   @param size max data read bytes count
   @return data bytes count read from the device or negative error code
*/
typedef DEVNET_READ(devnet_read_t);




/** Net device class write() function tempate. */
#define DEVNET_WRITE(n)	ssize_t  (n) (struct device_s *dev, const uint8_t *data, size_t size)

/**
    Net device class write() function type.  Write bytes data to the
    device. Return number of bytes written. Should not block if unable
    to write more bytes.

    @param dev pointer to device descriptor
    @param data pointer to read only data buffer
    @param size data bytes count
    @return data bytes count written to the device or negative error code
*/
typedef DEVNET_WRITE(devnet_write_t);

/** Net device class write() methode shortcut */
#define dev_net_write(dev, ...) (dev)->drv->f.net.f_write(dev, __VA_ARGS__ )




/** Net device class methodes */
struct dev_class_net_s
{
  devnet_read_t		*f_read;
  devnet_write_t	*f_write;
};


#endif

