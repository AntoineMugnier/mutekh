.section .vectors

.extern vect1
.extern vect2
.extern vect3
.extern vect4
.extern vect5
.extern vect6
.extern vect7
.extern vect8
.extern vect9
.extern vect10
.extern vect11
.extern vect12
.extern vect13
.extern vect14
.extern vect15
.extern vect16
.extern vect17
.extern vect18
.extern vect19
.extern vect20
.extern vect21
.extern vect22
.extern vect23
.extern vect24
.extern vect25
.extern vect26
.extern vect27
.extern vect28
.extern vect29
.extern vect30
.extern vect31
.extern vect32
.extern vect33
.extern vect34
.extern vect35
.extern vect36
.extern vect37
.extern vect38
.extern vect39
.extern vect40
.extern vect41
.extern vect42
.extern vect43
.extern vect44
.extern vect45
.extern vect46
.extern vect47
.extern vect48
.extern vect49
.extern vect50
.extern vect51
.extern vect52
.extern vect53
.extern vect54
.extern vect55
.extern vect56
.extern vect57
.extern vect58
.extern vect59
.extern vect60
.extern cpu_boot

uvector63:	bsr    vect63  ; /* Reserved */
uvector62:	bsr    vect62  ; /* Reserved */
uvector61:	bsr    vect61  ; /* Reserved */
uvector60:	bsr    vect60  ; /* Reserved */
uvector59:	bsr    vect59  ; /* Reserved */
uvector58:	bsr    vect58  ; /* Reserved */
uvector57:	bsr    vect57  ; /* Reserved */
uvector56:	bsr    vect56  ; /* Reserved */
uvector55:	bsr    vect55  ; /* Reserved */
uvector54:	bsr    vect54  ; /* Reserved */
uvector53:	bsr    vect53  ; /* Reserved */
uvector52:	bsr    vect52  ; /* Reserved */
uvector51:	bsr    vect51  ; /* Reserved */
uvector50:	bsr    vect50  ; /* Reserved */
uvector49:	bsr    vect49  ; /* Reserved */
uvector48:	bsr    vect48  ; /* Reserved */
uvector47:	bsr    vect47  ; /* EMAC Excessive Collision */
uvector46:	bsr    vect46  ; /* EMAC Late Collision */
uvector45:	bsr    vect45  ; /* EMAC Babbling Receive Error */
uvector44:	bsr    vect44  ; /* EMAC B Overrun */
uvector43:	bsr    vect43  ; /* EMAC A Overrun */
uvector42:	bsr    vect42  ; /* EMAC Receive Error */
uvector41:	bsr    vect41  ; /* EMAC MII */
uvector40:	bsr    vect40  ; /* EMAC Received */
uvector39:	bsr    vect39  ; /* EMAC Transmitted */
uvector38:	bsr    vect38  ; /* EMAC B Full */
uvector37:	bsr    vect37  ; /* EMAC A Full */
uvector36:	bsr    vect36  ; /* EPHY */
uvector35:	bsr    vect35  ; /* Flash */
uvector34:	bsr    vect34  ; /* Reserved */
uvector33:	bsr    vect33  ; /* Reserved */
uvector32:	bsr    vect32  ; /* Reserved */
uvector31:	bsr    vect31  ; /* I2C */
uvector30:	bsr    vect30  ; /* Reserved */
uvector29:	bsr    vect29  ; /* Self-Clock */
uvector28:	bsr    vect28  ; /* PLL Lock */
uvector27:	bsr    vect27  ; /* Reserved */
uvector26:	bsr    vect26  ; /* Port G */
uvector25:	bsr    vect25  ; /* Port H */
uvector24:	bsr    vect24  ; /* Port J */
uvector23:	bsr    vect23  ; /* Reserved */
uvector22:	bsr    vect22  ; /* ATD */
uvector21:	bsr    vect21  ; /* SCI1 */
uvector20:	bsr    vect20  ; /* SCI0 */
uvector19:	bsr    vect19  ; /* SPI */
uvector18:	bsr    vect18  ; /* Pulse Accumulator Edge */
uvector17:	bsr    vect17  ; /* Pulse Accumulator Overflow */
uvector16:	bsr    vect16  ; /* Timer Overflow */
uvector15:	bsr    vect15  ; /* Timer Channel 7 */
uvector14:	bsr    vect14  ; /* Timer Channel 6 */
uvector13:	bsr    vect13  ; /* Timer Channel 5 */
uvector12:	bsr    vect12  ; /* Timer Channel 4 */
uvector11:	bsr    vect11  ; /* Reserved */
uvector10:	bsr    vect10  ; /* Reserved */
uvector09:	bsr    vect9  ; /* Reserved */
uvector08:	bsr    vect8  ; /* Reserved */
uvector07:	bsr    vect7  ; /* RTIRQ */
uvector06:	bsr    vect6  ; /* IRQ */
uvector05:	bsr    vect5  ; /* XIRQ */
uvector04:	bsr    vect4  ; /* SWI */
uvector03:	bsr    vect3  ; /* Unimplemented opcode */
uvector02:	bsr    vect2  ; /* COP Reset */
uvector01:	bsr    vect1  ; /* Clock Monitor Reset */
uvector00:      bsr    cpu_boot ; /* Reset vector */

.end
