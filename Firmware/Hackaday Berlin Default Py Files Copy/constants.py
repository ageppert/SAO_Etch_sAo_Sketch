
WCH_DM_CPBR     = const(0x007C)
WCH_DM_CFGR     = const(0x007D)
WCH_DM_SHDWCFGR = const(0x007E)
WCH_DM_PART     = const(0x007F) # not in doc but appears to be part info
SECRET          = const(0x5AA50000)
OUTSTA          = const(SECRET | (1 << 10)) ## OUTSTA: 0: The debug slave has output function.
DM_CTRL         = const(0x0010)  ## in debug mode
DM_STATUS       = const(0x0011)  # debug mode, read/write

## Some debug-mode constants
DMDATA0        = const(0x04)
DMDATA1        = const(0x05)
DMCONTROL      = const(0x10)
DMSTATUS       = const(0x11)
DMHARTINFO     = const(0x12)
DMABSTRACTCS   = const(0x16)
DMCOMMAND      = const(0x17)
DMABSTRACTAUTO = const(0x18)

FLASH_STATR_WRPRTERR = const(0x10)
CR_PAGE_PG           = const(0x00010000)
CR_BUF_LOAD          = const(0x00040000)
FLASH_CTLR           = const(0x40022010)
FLASH_CTLR_MER       = const(0x0004)   #  /* Mass Erase */)
CR_STRT_Set          = const(0x00000040)
CR_PAGE_ER           = const(0x00020000)
CR_BUF_RST           = const(0x00080000)

DMPROGBUF0 = const(0x20)
DMPROGBUF1 = const(0x21)
DMPROGBUF2 = const(0x22)
DMPROGBUF3 = const(0x23)
DMPROGBUF4 = const(0x24)
DMPROGBUF5 = const(0x25)
DMPROGBUF6 = const(0x26)
DMPROGBUF7 = const(0x27)

## Some unavoidable bit-twiddling
SIDE_PINDIR = const(29)
PIO0_BASE = const(0x50200000)
PIO1_BASE = const(0x50300000)
SM0_EXECCTRL = const(0x0cc)  # p 375
SM1_EXECCTRL = const(0xe4)
SM2_EXECCTRL = const(0xfc)
SM3_EXECCTRL = const(0x114)
SM4_EXECCTRL = const(0x0cc)  # p 375
SM5_EXECCTRL = const(0xe4)
SM6_EXECCTRL = const(0xfc)
SM7_EXECCTRL = const(0x114)

