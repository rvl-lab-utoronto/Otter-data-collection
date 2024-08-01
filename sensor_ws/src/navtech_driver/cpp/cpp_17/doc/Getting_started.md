Programming with the SDK
========================
This document provides a basic tutorial for programming a client to receive and send Colossus messages to a radar.  This tutorial uses the TCP client, but the principles apply to the UDP client.
Working through this tutorial will give you a basic working client. We recommend you work through this example first, then look at the example projects provided by the SDK, which give more detailed examples.


## Initialising the SDK
The SDK must be set up and configured, prior to using any radar clients.  The `SDK::initialise()` function enables and sets up these features.  The counterpart function, `SDK::shutdown()` should be called before exiting `main()`.
```C++
#include "sdk.h"

using namespace Navtech;

int main()
{
    SDK::initialise();

    // more...

    SDK::shutdown();
}

```

## Constructing the `Client`
Connection to a radar is handled through a `Client` object.  A `Client` provides two basic interfaces:

* Setting up callbacks for incoming Colossus messages
* Sending Colossus messages (to the radar)


The `Client` must be constructed with the IP address and port number of the server:
```C++
#include "sdk.h"
#include "Colossus_client.h"
#include "IP_address.h"

using namespace Navtech;
using namespace Navtech::Networking;

int main()
{
    SDK::initialise();

    Endpoint radar_address { "198.168.0.1"_ipv4, 6317 };
    Colossus_protocol::Client radar_client { radar_address };

    // more...

    SDK::shutdown();
}
```

In the above code we are using the user-defined literal `_ipv4` to construct an `IP_address` object from a string literal.


## Creating a callback
Incoming messages are dispatched to an appropriate callback, which must be provided by the client.  The callback can be any C++ _Callable type_ - that is, a function, a member function or a lambda expression - that satisfies the function signature:
```C++
void (*callback)(Navtech::Networking::Colossus_protocol::Client&, Navtech::Networking::Colossus_protocol::Message&);
```

Note, the callback has two parameters, the (incoming) message, and a reference to the (calling) `Client`.  This reference allows the callback to access the API of the `Client` without resorting to static (global) `Client` objects.  An example of how this may be exploited is shown below. 

In this simple example, we'll use a free function to process the incoming configuration.
```C++
#include "sdk.h"
#include "Colossus_client.h"
#include "IP_address.h"
#include "Colossus_protocol.h"

using namespace Navtech::Networking;

// Define a callback to process incoming configuration messages
//
void process_config(Colossus_protocol::Client& radar_client, Colossus_protocol::Message& msg)
{
    // See later...
}


int main()
{
    SDK::initialise();

    Endpoint radar_address { "198.168.0.1"_ipv4, 6317 };
    Colossus_protocol::Client radar_client { radar_address };

    // more...

    SDK::shutdown();
}
```

We must register the handler with the `Client` and associate it with a particular message type.
```C++
#include "sdk.h"
#include "Colossus_client.h"
#include "IP_address.h"
#include "Colossus_protocol.h"

using namespace Navtech::Networking;


void process_config(Colossus_protocol::Client& radar_client, Colossus_protocol::Message& msg)
{
    // See later...
}


int main()
{
    SDK::initialise();

    Endpoint radar_address { "198.168.0.1"_ipv4, 6317 };
    Colossus_protocol::Client radar_client { radar_address };
    
    // Register the callback
    //
    radar_client.set_handler(Colossus_protocol::Type::configuration, process_config);

    // more...

    SDK::shutdown();
}
```

Next, we must set the `Client` running, so that it can connect to the radar and begin processing messages.  The `Client` runs in its own thread.  Callbacks are executed in the context of the `Client` (more specifically, in the context of the `Client`'s `Dispatcher` thread). It is your responsibility to protect against race conditions if your callback functions interact with other threads-of-control.

For this example, we will simply let the `Client` run for a period of time, before stopping.
```C++
#include "sdk.h"
#include "Colossus_client.h"
#include "IP_address.h"
#include "Colossus_protocol.h"

using namespace Navtech::Networking;
using namespace Navtech::Time;
using namespace Navtech::Time::Monotonic;

void process_config(Colossus_protocol::Client& radar_client, Colossus_protocol::Message& msg)
{
    // See later...
}


int main()
{
    SDK::initialise();

    Endpoint radar_address { "198.168.0.1"_ipv4, 6317 };
    Colossus_protocol::Client radar_client { radar_address };
    
    radar_client.set_handler(Colossus_protocol::Type::configuration, process_config);

    // Start the Client and let it run for a period of time.
    // Note the use of a user-defined literal for the sleep duration.
    //
    radar_client.start();
    sleep_for(30_sec);
    radar_client.stop();

    SDK::shutdown();
}
```

## Handling incoming messages
The `Colossus_protocol::Message` type encapsulates a buffer with a basic interface for accessing data. For more details on the `Message` type, please read `/network/protocol/README.md`, which explains the concept behind the Colossus messaging classes.

To access the radar data in a message, the simplest way (and our recommended way) is to perform a memory overlay onto the message data. The `Message` class provides an interface for this.  Once the overlay has been done, the specific message's API can be accessed to read from the message buffer.  The API has been constructed to hide any endianness issues, scaling or conversion that may be required to get from 'raw' message bytes to usable radar information.
```C++
#include "sdk.h"
#include "Colossus_client.h"
#include "IP_address.h"
#include "Colossus_protocol.h"
#include "Log.h"

using namespace Navtech::Networking;
using namespace Navtech::Time;
using namespace Navtech::Time::Monotonic;

void process_config(Colossus_protocol::Client& radar_client, Colossus_protocol::Message& msg)
{
    using Navtech::Utility::stdout_log;
    using Navtech::Utility::endl;

    // Interpret the message data as a Configuration message
    //
    Colossus_protocol::Configuration* config = msg.view_as<Colossus_protocol::Configuration>();

    // Access the data via the pointer
    //
    stdout_log << "Azimuth samples [" << config->azimuth_samples() << "]" << endl;
    stdout_log << "Bin size        [" << config->bin_size()<< "]" << endl;
    stdout_log << "Range in bins   [" << config->range_in_bins()<< "]" << endl;
    stdout_log << "Encoder size    [" << config->encoder_size()<< "]" << endl;
    stdout_log << "Rotation rate   [" << config->rotation_speed()<< "]" << endl;
    stdout_log << "Range gain      [" << config->range_gain()<< "]" << endl;
    stdout_log << "Range offset    [" << config->range_offset()<< "]" << endl;

    // more...
}
```

In the example above we are using the logging utility, `stdout_log`.  This utility extends C++ console output, whilst maintaining a similar interface (all the IO manipulators available to `std::ostream` are also available to `stdout_log`, for example).

## Handling protocol buffer from a message
If the incoming message contains a protocol buffer, this data can be extracted into a protobuf object and then accessed through the normal protobuf API.
```C++
#include "sdk.h"
#include "Colossus_client.h"
#include "IP_address.h"
#include "Colossus_protocol.h"
#include "Log.h"
#include "configurationdata.pb.h"
#include "Protobuf_helpers.h"

// As previously...

void process_config(Colossus_protocol::Client& radar_client, Colossus_protocol::Message& msg)
{
    using Colossus::Protobuf::ConfigurationData;
    using Navtech::Utility::stdout_log;
    using Navtech::Utility::endl;
    using Navtech::Protobuf::from_vector_into;

    Colossus_protocol::Configuration* config = msg.view_as<Colossus_protocol::Configuration>();

    stdout_log << "Azimuth samples [" << config->azimuth_samples() << "]" << endl;
    stdout_log << "Bin size        [" << config->bin_size()<< "]" << endl;
    stdout_log << "Range in bins   [" << config->range_in_bins()<< "]" << endl;
    stdout_log << "Encoder size    [" << config->encoder_size()<< "]" << endl;
    stdout_log << "Rotation rate   [" << config->rotation_speed()<< "]" << endl;
    stdout_log << "Range gain      [" << config->range_gain()<< "]" << endl;
    stdout_log << "Range offset    [" << config->range_offset()<< "]" << endl;

    // Overlay onto the message memory, then extract the raw message payload
    // data into std::optional protobuf object using the helper function
    // Protobuf::from_vector_into.  If the conversion fails, the function
    // will return std::nullopt
    //
    std::optional<ConfigurationData> protobuf = from_vector_into<ConfigurationData>(config->to_vector());

    if (protobuf.has_value()) {
        stdout_log << "Radar ID   [" << protobuf->model().id()   << "]" << endl;
        stdout_log << "Radar name [" << protobuf->model().name() << "]" << endl;
        // etc...
    }
}
```

## Sending a message
The full details of sending Colossus messages is beyond the scope of this 'getting started' overview.  For full details of how to construct and send Colossus messages, please read `/network/protocol/README.md`.

In the case of a radar client, typically only simple messages are sent to the radar, to enable/disable features such as FFT data, or radar health.

Remember - if you enable data transmission from the radar you *must* have a handler for it, otherwise you will receive an error each time a message arrives!
```C++
void process_FFT(Colossus_protocol::Client& radar_client, Colossus_protocol::Message& msg)
{
    // Process the incoming FFT data...
}


int main()
{
    SDK::initialise();

    Endpoint radar_address { "198.168.0.1"_ipv4, 6317 };
    Colossus_protocol::Client radar_client { radar_address };
    
    radar_client.set_handler(Colossus_protocol::Type::configuration, process_config);
    radar_client.set_handler(Colossus_protocol::Type::fft_data, process_FFT);

    radar_client.start();
    sleep_for(5_sec);

    // Construct a message to start FFT data and send it to the radar
    //
    Colossus_protocol::Message msg { };
    msg.type(Colossus_protocol::Type::start_fft_data);

    radar_client.send(std::move(msg));

    sleep_for(30_sec);
    radar_client.stop();

    SDK::shutdown();
}
```

## Sending a message within a callback
Commonly, you will want to send a message from within a callback. Since the callback has the `Client` as an argument, you can directly send a message.

Note, in the case of simple messages (with no header or payload), the `send()` method can construct the `Colossus_protocol::Message` object implicitly as part of the call.
```C++
void process_config(Colossus_protocol::Client& radar_client, Colossus_protocol::Message& msg)
{
    using Colossus::Protobuf::ConfigurationData;
    using Navtech::Utility::stdout_log;
    using Navtech::Utility::endl;
    using Navtech::Protobuf::from_vector_into;

    Colossus_protocol::Configuration* config = msg.view_as<Colossus_protocol::Configuration>();

    stdout_log << "Azimuth samples [" << config->azimuth_samples() << "]" << endl;
    stdout_log << "Bin size        [" << config->bin_size()<< "]" << endl;
    stdout_log << "Range in bins   [" << config->range_in_bins()<< "]" << endl;
    stdout_log << "Encoder size    [" << config->encoder_size()<< "]" << endl;
    stdout_log << "Rotation rate   [" << config->rotation_speed()<< "]" << endl;
    stdout_log << "Range gain      [" << config->range_gain()<< "]" << endl;
    stdout_log << "Range offset    [" << config->range_offset()<< "]" << endl;

    
   auto protobuf = from_vector_into<ConfigurationData>(config->to_vector());

    if (protobuf.has_value()) {
        stdout_log << "Radar ID   [" << protobuf->model().id()   << "]" << endl;
        stdout_log << "Radar name [" << protobuf->model().name() << "]" << endl;
    }

    // Send a simple message to the radar
    //
    radar_client.send(Colossus_protocol::Type::start_fft_data);
}
```

