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

using Navtech.IASDK.Events;
using System;
using System.Net.Sockets;

namespace Navtech.IASDK.Interfaces
{
    /// <summary>
    /// Represents a client connection within the TCP Server
    /// </summary>
    /// <typeparam name="T">Type of message to be handled</typeparam>
    public interface ITcpClientConnection<T> : IDisposable
    {
        /// <summary>
        /// Informs subscribers that a message of type T has been received
        /// </summary>
        event EventHandler<GenericEventArgs<T>> OnData;
        /// <summary>
        /// The IP address of the connected client
        /// </summary>
        string ClientIpAddress { get; }
        /// <summary>
        /// A property that exposes the TcpClient
        /// </summary>
        TcpClient TcpClient { get; set; }
        /// <summary>
        /// Disconnects the server from this specific client
        /// </summary>
        void Disconnect();
        /// <summary>
        /// Sends a compatible raw data message to the client
        /// <see cref="IRawData"/>
        /// </summary>
        /// <param name="rawData">Compatiable raw data message</param>
        void SendData(IRawData rawData);
    }
}
