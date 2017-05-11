# !/usr/bin/env python

import rospy
import serial
import sys
import string
import operator
import math
import threading
import struct
import binascii
import zlib
import crcmod

# a = 0x104c11db7
# print binascii.a2b_hex(a)

crc32_func = crcmod.mkCrcFun(0x104C11DB7,xorOut=0x00FFFFFF)


b = '''BESTPOSA,COM1,0,78.0,FINESTEERING,1427,325298.000,00000000,6145,2748;SOL_COMPUTED,SINGLE,51.11678928753,-114.03886216575,1064.3470,-16.2708,WGS84,2.3434,1.3043,4.7300,"",0.000,0.000,7,7,0,0,0,06,0,03'''

a = '''BESTPOSA,COM2,0,77.5,FINESTEERING,1285,160578.000,00000020,5941,1164;SOL_COMPUTED,SINGLE,51.11640941570,-114.03830951024,1062.6963,-16.2712,WGS84,1.6890,1.2564,2.7826,\"\",0.000,0.000,10,10,0,0,0,0,0,0'''

crc = crc32_func(b)

def mycrc32(szString):
    m_pdwCrc32Table = [0 for x in range(0,256)]
    dwPolynomial = 0xEDB88320
    dwCrc = 0
    for i in range(0, 255):
        dwCrc = i
        for j in [8,7,6,5,4,3,2,1]:
            if dwCrc & 1:
                dwCrc = (dwCrc >> 1) ^ dwPolynomial
            else:
                dwCrc >>= 1
        m_pdwCrc32Table[i] = dwCrc
    dwCrc32 = 0xFFFFFFFFL
    for i in szString:
        b = ord(i)
        dwCrc32 = ((dwCrc32) >> 8) ^ m_pdwCrc32Table[(b) ^ ((dwCrc32) & 0x000000FF)]
    dwCrc32 = dwCrc32 ^ 0xFFFFFFFFL
    return dwCrc32

crc2 = mycrc32(b)


print 'crc32 = 0x%08x, 0x%08x' % (crc, crc2)

