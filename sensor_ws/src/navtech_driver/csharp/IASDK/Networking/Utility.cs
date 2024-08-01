/*
Copyright 2024 Navtech Radar Limited
This file is part of IASDK which is released under The MIT License (MIT).
See file LICENSE.txt in project root or go to https://opensource.org/licenses/MIT
for full license details.

Disclaimer:
Navtech Radar is furnishing this item "as is". Navtech Radar does not provide 
any warranty of the item whatsoever, whether express, implied, or statutory,
including, but not limited to, any warranty of merchantability or fitness
for a particular purpose or any warranty that the contents of the item will
be error-free.
In no respect shall Navtech Radar incur any liability for any damages, including,
but limited to, direct, indirect, special, or consequential damages arising
out of, resulting from, or any way connected to the use of the item, whether
or not based upon warranty, contract, tort, or otherwise; whether or not
injury was sustained by persons or property or otherwise; and whether or not
loss was sustained from, or arose out of, the results of, the item, or any
services that may be provided by Navtech Radar.
*/
namespace Navtech.IASDK.Networking
{
    /// <summary>
    /// Network utility class
    /// </summary>
    public static class Utility
    {
        /// <summary>
        /// Swaps bytes in an Int32
        /// </summary>
        /// <param name="value">Int32 to manipulate</param>
        /// <returns>Corrected Int32 value</returns>
        public static int SwapInt32(int value)
        {
            return ((SwapUInt16((ushort)((value & 0x000ffff))) << 0x10)) | (SwapUInt16((ushort)((value >> 0x10) & 0xffff)));
        }

        /// <summary>
        /// Swaps bytes in an UInt32
        /// </summary>
        /// <param name="value">UInt32 to manipulate</param>
        /// <returns>Corrected UInt32 value</returns>
        public static uint SwapUInt32(uint value)
        {
            return (uint)((SwapUInt16((ushort)((value & 0x000ffff))) << 0x10)) | (SwapUInt16((ushort)((value >> 0x10) & 0xffff)));
        }

        /// <summary>
        /// Swaps bytes in an UInt16
        /// </summary>
        /// <param name="value">UInt16 to manipulate</param>
        /// <returns>Corrected Uint16 value</returns>
        public static ushort SwapUInt16(ushort value)
        {
            return (ushort)(((value & 0xff) << 8) | ((value >> 8) & 0xff));
        }
    }
}
