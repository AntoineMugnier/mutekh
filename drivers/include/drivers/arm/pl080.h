#ifndef DRIVER_ARM_PL080_H_
#define DRIVER_ARM_PL080_H_

/**
   Determines whether the peripheral is providing the flow control (i.e. transfer size).
   See you SoC's documentation and ARM's PL080 DDI0196G chapter 3.8.
 */
#define ARM_PL080_FLOW 0x10
#define ARM_PL080_LINK(per, is_flow) ((per) | ((is_flow) ? ARM_PL080_FLOW : 0))
#define ARM_PL080_LINK_IS_FLOW(link) (!!((link) & ARM_PL080_FLOW))
#define ARM_PL080_LINK_PERIPHERAL(link) ((link) & 0xf)

/**
 * @this is the source IRQ index when mapping the IP IRQ lines.
 *
 * There are two options for integration of PL080. Either with
 * separate error and completion interrupts, or with mixed interrupt.
 *
 * @see CONFIG_DRIVER_ARM_PL080_IRQ_SEPARATE
 */
enum arm_pl080_irq_source_id_e
{
#if defined(CONFIG_DRIVER_ARM_PL080_IRQ_SEPARATE)
    ARM_PL080_IRQ_DMACINTERR,
    ARM_PL080_IRQ_DMACINTTC,
#else
    ARM_PL080_IRQ_DMACINTR,
#endif
    ARM_PL080_IRQ_COUNT,
};

#endif
