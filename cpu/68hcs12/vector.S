.section .vectors

.extern	excp
.extern cpu_boot

uvector63:	bsr    excp    ; /* vector 63 */
uvector62:	bsr    excp    ; /* vector 62 */
uvector61:	bsr    excp    ; /* vector 61 */
uvector60:	bsr    excp    ; /* vector 60 */
uvector59:	bsr    excp    ; /* vector 59 */
uvector58:	bsr    excp    ; /* vector 58 */
uvector57:	bsr    excp    ; /* vector 57 */
uvector56:	bsr    excp    ; /* vector 56 */
uvector55:	bsr    excp    ; /* vector 55 */
uvector54:	bsr    excp    ; /* vector 54 */
uvector53:	bsr    excp    ; /* vector 53 */
uvector52:	bsr    excp    ; /* vector 52 */
uvector51:	bsr    excp    ; /* vector 51 */
uvector50:	bsr    excp    ; /* vector 50 */
uvector49:	bsr    excp    ; /* vector 49 */
uvector48:	bsr    excp    ; /* vector 48 */
uvector47:	bsr    excp    ; /* vector 47 */
uvector46:	bsr    excp    ; /* vector 46 */
uvector45:	bsr    excp    ; /* vector 45 */
uvector44:	bsr    excp    ; /* vector 44 */
uvector43:	bsr    excp    ; /* vector 43 */
uvector42:	bsr    excp    ; /* vector 42 */
uvector41:	bsr    excp    ; /* vector 41 */
uvector40:	bsr    excp    ; /* vector 40 */
uvector39:	bsr    excp    ; /* vector 39 */
uvector38:	bsr    excp    ; /* vector 38 */
uvector37:	bsr    excp    ; /* vector 37 */
uvector36:	bsr    excp    ; /* vector 36 */
uvector35:	bsr    excp    ; /* vector 35 */
uvector34:	bsr    excp    ; /* vector 34 */
uvector33:	bsr    excp    ; /* vector 33 */
uvector32:	bsr    excp    ; /* vector 32 */
uvector31:	bsr    excp    ; /* vector 31 */
uvector30:	bsr    excp    ; /* vector 30 */
uvector29:	bsr    excp    ; /* vector 29 */
uvector28:	bsr    excp    ; /* vector 28 */
uvector27:	bsr    excp    ; /* vector 27 */
uvector26:	bsr    excp    ; /* vector 26 */
uvector25:	bsr    excp    ; /* vector 25 */
uvector24:	bsr    excp    ; /* vector 24 */
uvector23:	bsr    excp    ; /* vector 23 */
uvector22:	bsr    excp    ; /* vector 22 */
uvector21:	bsr    excp    ; /* vector 21 */
uvector20:	bsr    excp    ; SCI0, used for monitor (place holder only)
uvector19:	bsr    excp    ; /* vector 19 */
uvector18:	bsr    excp    ; /* vector 18 */
uvector17:	bsr    excp    ; /* vector 17 */
uvector16:	bsr    excp    ; /* vector 16 */
uvector15:	bsr    excp    ; /* vector 15 */
uvector14:	bsr    excp    ; /* vector 14 */
uvector13:	bsr    excp    ; /* vector 13 */
uvector12:	bsr    excp    ; /* vector 12 */
uvector11:	bsr    excp    ; /* vector 11 */
uvector10:	bsr    excp    ; /* vector 10 */
uvector09:	bsr    excp    ; /* vector 09 */
uvector08:	bsr    excp    ; /* vector 08 */
uvector07:	bsr    excp    ; /* vector 07 */
uvector06:	bsr    excp    ; /* vector 06 */
uvector05:	bsr    excp    ; /* vector 05 */
uvector04:	bsr    excp    ; SWI, used for breakpoints (place holder only)
uvector03:	bsr    excp    ; /* vector 03 */
uvector02:	bsr    excp    ; /* vector 02 */
uvector01:	bsr    excp    ; /* vector 01 */
uvector00:      bsr    cpu_boot ; /* Reset vector */

.end
