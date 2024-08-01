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
using Navtech.IASDK.Interfaces;
using System;
using System.Net;
using System.Net.Sockets;
using System.Threading;

namespace Navtech.IASDK.Networking
{
    /// <summary>
    /// Represents a client radar connection within the TCP Server
    /// </summary>
    /// <typeparam name="T"></typeparam>
    public abstract class TcpClientConnection<T> : ITcpClientConnection<T> where T : IRawData
    {
        /// <summary>
        /// Informs subscribers that a message of type T has been received
        /// </summary>
        public event EventHandler<GenericEventArgs<T>> OnData;
        private TcpClient _client;
        private Thread _readThread;
        private bool _disposed;
        private IPEndPoint _clientEndPoint;

        /// <summary>
        /// A flag to indicate if the system is still reading data from the client
        /// </summary>
        protected bool Reading;
        /// <summary>
        /// The network stream used to connect to the client
        /// </summary>
        protected NetworkStream ClientStream;
        /// <summary>
        /// The IP address of the connected client
        /// </summary>
        public string ClientIpAddress { get; private set; }
        /// <summary>
        /// A property that exposes the TcpClient
        /// </summary>
        public TcpClient TcpClient
        {
            get { return _client; }
            set
            {
                if (value == null) return;

                _client = value;
                ClientStream = _client.GetStream();

                _clientEndPoint = (IPEndPoint)_client.Client.RemoteEndPoint;
                ClientIpAddress = _clientEndPoint.Address.ToString();

                _readThread = new Thread(ReadData) { IsBackground = true };
                Reading = true;
                _readThread.Start();
            }
        }

        /// <summary>
        /// Sends a compatible raw data message to the client
        /// <see cref="IRawData"/>
        /// </summary>
        /// <param name="rawData">Compatiable raw data message</param>
        public void SendData(IRawData rawData)
        {
            if (ClientStream == null) return;
            try
            {
                ClientStream.Write(rawData.Data, 0, rawData.Data.Length);
            }
            catch
            {
                Disconnect();
            }
        }

        /// <summary>
        /// Implmentation of the IDisposable interface
        /// </summary>
        public void Dispose()
        {
            Dispose(true);
            GC.SuppressFinalize(this);
        }

        /// <summary>
        /// Disconnects the server from this specific client
        /// </summary>
        public void Disconnect()
        {
            Reading = false;

            try
            {
                ClientStream?.Close();
                _client?.Close();
                _readThread.Join(1000);
            }
            finally
            {
                ClientStream = null;
                _client = null;
            }
        }

        private void RaiseOnData(T message)
        {
            if (message == null) return;
            OnData?.Invoke(this, new GenericEventArgs<T> { Payload = message });
        }

        private void ReadData()
        {
            if (ClientStream == null) return;

            while (Reading)
            {
                try
                {
                    RaiseOnData(ExtractFromStream());
                }
                catch (Exception)
                {
                    Disconnect();
                    break;
                }
            }

            Reading = false;
        }

        /// <summary>
        /// Abstract method to handling the incoming message protocol
        /// from the client
        /// </summary>
        /// <returns>Returns messages of type T</returns>
        protected abstract T ExtractFromStream();

        /// <summary>
        /// Implmentation of the IDisposable interface
        /// </summary>
        /// <param name="disposing">Flag to indicate if we are disposing</param>
        protected virtual void Dispose(bool disposing)
        {
            if (_disposed)
                return;

            if (disposing)
            {
                if (Reading)
                    Disconnect();

                if (_readThread != null && _readThread.ThreadState == ThreadState.Running)
                    _readThread.Abort();

                ClientStream?.Dispose();

                if (_client != null && _client.Connected)
                    _client.Close();

                ClientStream = null;
                _client = null;
            }

            _disposed = true;
        }
    }
}
