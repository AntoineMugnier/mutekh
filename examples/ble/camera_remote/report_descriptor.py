from hrdc.usage import *
from hrdc.descriptor import *

bit_in = lambda x: Value(Value.Input, x, 1,
                         logicalMin = 0, logicalMax = 1)

remote = TopLevel(
    Collection(Collection.Application, consumer.ConsumerControl,
               Report(1,
                      bit_in(consumer.VolumeIncrement),
                      bit_in(consumer.VolumeDecrement),
                      bit_in(consumer.ScanPreviousTrack),
                      bit_in(consumer.ScanNextTrack),
                      bit_in(consumer.PlayPause),
                      bit_in(consumer.Stop),
                      bit_in(consumer.Power),
                      bit_in(consumer.Sleep),
                      ),
               Report(2,
                      Value(Value.Input, usage = device.BatteryStrength,
                            size = 8,
                            logicalMin = 0,
                            logicalMax = 100)
                      )
               )
    )

if __name__ == "__main__":
    compile_main(remote)
    
