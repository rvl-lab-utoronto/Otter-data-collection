// ---------------------------------------------------------------------------------------------------------------------
// Copyright 2024 Navtech Radar Limited
// This file is part of IASDK which is released under The MIT License (MIT).
// See file LICENSE.txt in project root or go to https://opensource.org/licenses/MIT
// for full license details.
//
// Disclaimer:
// Navtech Radar is furnishing this item "as is". Navtech Radar does not provide 
// any warranty of the item whatsoever, whether express, implied, or statutory,
// including, but not limited to, any warranty of merchantability or fitness
// for a particular purpose or any warranty that the contents of the item will
// be error-free.
// In no respect shall Navtech Radar incur any liability for any damages, including,
// but limited to, direct, indirect, special, or consequential damages arising
// out of, resulting from, or any way connected to the use of the item, whether
// or not based upon warranty, contract, tort, or otherwise; whether or not
// injury was sustained by persons or property or otherwise; and whether or not
// loss was sustained from, or arose out of, the results of, the item, or any
// services that may be provided by Navtech Radar.
// ---------------------------------------------------------------------------------------------------------------------
#ifndef MESSAGE_WRITER_H
#define MESSAGE_WRITER_H

#include <fstream>
#include <string>
#include <string_view>
#include <functional>


namespace Navtech::Utility {

    // The Message_writer template allows any (Colossus) message to be written
    // to a file.
    // Inherit from this class to implement your own message-specific output
    // formatting.
    //
    template <typename Client_Ty, typename Message_Ty> 
    class Message_writer : public Utility::Active {
    public:
        Message_writer() = default;
        Message_writer(std::string_view output_file);
        Message_writer(std::string_view output_file, std::ios_base::openmode mode);
        virtual ~Message_writer();

        void filename(std::string_view output_filepath);
        std::string filename() const;

        // Use this function to change the open-mode of the file stream;
        // for example, if you are concatenating outputs from different
        // messages into the same file.
        //
        void open_mode(std::ios_base::openmode m);

        void write(const Message_Ty& msg);

        // A Message_writer object can be used as a handler for the TCP
        // or UDP client.  Instead of writing a function to process the
        // message, invoke callback(), which will pass a function-object.
        // By default, the function-object will invoke write() with the
        // incoming message, but you can override this, as required (see
        // below).
        // The call operator has been implemented, so the Message_writer
        // object can be passed as if it is a function.
        //
        std::function<void(Client_Ty&, Message_Ty&)> callback();
        std::function<void(Client_Ty&, Message_Ty&)> operator()();

    protected:
        std::string             output_filepath   { };
        std::ofstream           output_file       { };
        std::ios_base::openmode mode              { std::ios_base::out };

        // Override these functions to implement your own functionality.
        //
        // header() and footer() allow output at the beginning of the file
        // and end, respectively.
        // on_write() should be overridden in your derived class to write
        // the message to the output file in whatever format you require.
        // on_callback() may be overridden if you want to do something other
        // than just write to the output file. Remember, you must explicitly
        // call write() if you actually want output!
        //
        virtual void header()                        { /* Do nothing by default */ }
        virtual void footer()                        { /* Do nothing by default */ }
        virtual void on_write(const Message_Ty&)     { /* Do nothing by default */ }
        virtual void on_callback(Client_Ty&, Message_Ty&);

    private:
        void on_start() override;
        void on_stop()  override;
    };


    template <typename Client_Ty, typename Message_Ty>
    Message_writer<Client_Ty, Message_Ty>::Message_writer(std::string_view output_file) :
        output_filepath { output_file }
    {
    }


    template <typename Client_Ty, typename Message_Ty>
    Message_writer<Client_Ty, Message_Ty>::Message_writer(std::string_view output_file, std::ios_base::openmode m) :
        output_filepath { output_file },
        mode            { m }
    {
    }


    template <typename Client_Ty, typename Message_Ty>
    Message_writer<Client_Ty, Message_Ty>::~Message_writer()
    {
        stop();
    }

    template <typename Client_Ty, typename Message_Ty>
    void Message_writer<Client_Ty, Message_Ty>::filename(std::string_view output_file)
    {
        output_filepath = output_file;
    }


    template <typename Client_Ty, typename Message_Ty>
    std::string Message_writer<Client_Ty, Message_Ty>::filename() const
    {
        return output_filepath;
    }


    template <typename Client_Ty, typename Message_Ty>
    void Message_writer<Client_Ty, Message_Ty>::open_mode(std::ios_base::openmode m)
    {
        mode = m;
    }


    template <typename Client_Ty, typename Message_Ty>
    void Message_writer<Client_Ty, Message_Ty>::on_start()
    {
        output_file.open(output_filepath, mode);
        header();
    }


    template <typename Client_Ty, typename Message_Ty>
    void Message_writer<Client_Ty, Message_Ty>::on_stop()
    {
        footer();
        output_file.close();
    }


    template <typename Client_Ty, typename Message_Ty>
    void Message_writer<Client_Ty, Message_Ty>::write(const Message_Ty& msg) 
    { 
        async_call(&Message_writer::on_write, this, msg); 
    }


    template <typename Client_Ty, typename Message_Ty>
    std::function<void(Client_Ty&, Message_Ty&)> Message_writer<Client_Ty, Message_Ty>::callback()
    {
        return std::bind(
            &Message_writer<Client_Ty, Message_Ty>::on_callback, 
            this,
            std::placeholders::_1,
            std::placeholders::_2 
        );
    }


    template <typename Client_Ty, typename Message_Ty>
    std::function<void(Client_Ty&, Message_Ty&)> Message_writer<Client_Ty, Message_Ty>::operator()()
    {
        return callback();
    }


    template <typename Client_Ty, typename Message_Ty>
    void Message_writer<Client_Ty, Message_Ty>::on_callback(Client_Ty&, Message_Ty& msg)
    {
        write(msg);
    }


} // namespace Navtech::Utility



#endif // MESSAGE_WRITER_H