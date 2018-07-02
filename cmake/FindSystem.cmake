set(SYSTEM_FOUND 1)

IF(WIN32)
    set(SYSTEM_LIBRARIES Iphlpapi Ws2_32 Vfw32 Winmm Comctl32)
    #set(SYSTEM_LIBRARIES pthread Iphlpapi Ws2_32)
ELSE(WIN32)
    set(SYSTEM_LIBRARIES pthread dl)
ENDIF(WIN32)
