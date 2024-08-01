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

using System.Runtime.InteropServices;

namespace Navtech.IASDK.Extensions
{
    /// <summary>
    /// Extensions for byte[]
    /// </summary>
    public static class RawStructExtensions
    {
        /// <summary>
        /// Marshalls a byte array into the supplied type
        /// </summary>
        /// <typeparam name="T">The type to construct from the bytes</typeparam>
        /// <param name="data">The byte data to use to create the type</param>
        /// <returns>An instance of type T</returns>
        public static T MarshalToObject<T>(this byte[] data) where T : struct
        {
            var rawsize = Marshal.SizeOf(typeof(T));
            if (rawsize > data.Length)
                return new T();

            var buffer = Marshal.AllocHGlobal(rawsize);
            Marshal.Copy(data, 0, buffer, rawsize);
            var retobj = Marshal.PtrToStructure(buffer, typeof(T));
            Marshal.FreeHGlobal(buffer);
            return (T)retobj;
        }
    }
}
