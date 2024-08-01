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
    /// Represents a client radar connection within the TCP Server
    /// This is specialised to handle radar specific connections
    /// with clients
    /// </summary>
    public class RadarTcpClientConnection : TcpClientConnection<TcpNetworkDataMessage>
    {
        /// <summary>
        /// Overridable method that extracts Colossus network data messages
        /// from the TCP stream
        /// </summary>
        /// <returns>Returns a <see cref="TcpNetworkDataMessage"/></returns>
        protected override TcpNetworkDataMessage ExtractFromStream()
        {
            var headerData = new byte[TcpNetworkDataMessage.HeaderLength];
            headerData.Initialize();
            var headerBytesRead = 0;
            while (headerBytesRead != TcpNetworkDataMessage.HeaderLength)
            {
                headerBytesRead += ClientStream.Read(headerData, headerBytesRead, TcpNetworkDataMessage.HeaderLength - headerBytesRead);
            }
            var message = new TcpNetworkDataMessage(headerData);
            if (!message.Valid) return null;

            var bytesRead = 0;
            while (bytesRead < message.PayloadSize)
            {
                bytesRead += ClientStream.Read(message.Payload, bytesRead, (int)message.PayloadSize - bytesRead);
            }

            return message;
        }
    }
}
