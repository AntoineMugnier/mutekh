from hrdc.usage import *
from hrdc.descriptor import *

bit_in = lambda x: Value(Value.Input, x, 1,
                         logicalMin = 0, logicalMax = 1)

keyboard = TopLevel(
    Collection(Collection.Application, desktop.Keyboard,
               Report(1,
                      bit_in(keyboard.LeftControl),
                      bit_in(keyboard.LeftShift),
                      bit_in(keyboard.LeftAlt),
                      bit_in(keyboard.LeftGui),
                      bit_in(keyboard.RightControl),
                      bit_in(keyboard.RightShift),
                      bit_in(keyboard.RightAlt),
                      bit_in(keyboard.RightGui),

                      Value(Value.Input, usage = None, size = 8,
                            namedArray = UsageRange(keyboard.NoEvent,
                                                    0x700ff),
                            logicalMin = 0, count = 1)
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
    compile_main(keyboard)
    
