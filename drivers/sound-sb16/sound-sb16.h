
#ifndef DRIVER_SOUND_SB16_H_
#define DRIVER_SOUND_SB16_H_

#include <hexo/device/sound.h>
#include <hexo/device.h>

#define SOUND_SB16_ADDR	0

DEV_IRQ(sound_sb16_irq);
DEV_INIT(sound_sb16_init);
DEV_CLEANUP(sound_sb16_cleanup);
DEVSOUND_READ(sound_sb16_read);
DEVSOUND_WRITE(sound_sb16_write);
DEVSOUND_MODE(sound_sb16_mode);

#endif




