#ifndef ARM_IDENTIFICATION_H_
#define ARM_IDENTIFICATION_H_

#include <hexo/types.h>

struct arm_identification_s
{
    uint32_t devid;
    uint32_t pid0, pid1;
    uint32_t cid;
};

void arm_identification_read(struct arm_identification_s *id, uintptr_t base_addr);
void arm_identification_dump(const struct arm_identification_s *id);

#endif
