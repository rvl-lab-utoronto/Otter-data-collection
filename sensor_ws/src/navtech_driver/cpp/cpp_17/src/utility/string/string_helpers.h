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
#ifndef STRING_HELPERS_H
#define STRING_HELPERS_H

#include <algorithm> 
#include <functional> 
#include <cctype>
#include <locale>
#include <string>
#include <vector>
#include <sstream>
#include <type_traits>
#include <iomanip>

namespace Navtech::Utility {

    inline std::string to_memory_string(std::uint64_t num_bytes)
    {
        // Convert the value to a convenient number
        // of bytes, kilobytes or megabytes.
        //
        std::stringstream stream { };

        stream << std::fixed;
        stream << std::setprecision(2);

        if (num_bytes > 1'000'000'000'000) {
            float value { num_bytes / 1'000'000'000'000.0f };
            stream << value << " TB";
        }
        else if (num_bytes > 1'000'000'000) {
            float value { num_bytes / 1'000'000'000.0f };
            stream << value << " GB";
        }
        else if (num_bytes > 1'000'000) {
            float value { num_bytes / 1'000'000.0f };
            stream << value << " MB";
        }
        else if (num_bytes > 1000) {
            float value { num_bytes / 1'000.0f };
            stream << value << " kB";
        }
        else {
            stream << num_bytes << " bytes";
        }

        return stream.str();
    }


    inline std::string to_hex_string(const std::vector<uint8_t>& v) 
	{
		std::ostringstream ss { };

		ss << std::hex << std::setfill('0') << std::uppercase;
	
		for(const auto i : v) {
			ss << std::setw(2) << static_cast<unsigned>(i);
		}

		return ss.str();
	}


	template <typename T, typename std::enable_if<std::is_integral<T>::value, bool>::type = true>
	inline std::string to_hex_string(const T& v) 
	{
		std::ostringstream ss { };

		ss << "0x";
		ss << std::hex << std::setfill('0') << std::uppercase;
		ss << std::setw(2) << static_cast<unsigned>(v);
		return ss.str();
	}


    // trim from start
    //
    static inline std::string& ltrim(std::string& s) 
    {
        s.erase(
            s.begin(), 
            // std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace)))
            std::find_if(s.begin(), s.end(), [](int chr) { return !std::isspace(chr); })
        );

        return s;
    }


    static inline std::string ltrim(std::string&& s) 
    {
        s.erase(
            s.begin(), 
            // std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace)))
            std::find_if(s.begin(), s.end(), [](int chr) { return !std::isspace(chr); })
        );

        return s;
    }

    // trim from end
    //
    static inline std::string& rtrim(std::string& s) 
    {
        s.erase(
            std::find_if(s.rbegin(), 
            s.rend(),
            // std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end()
            [](int chr) { return !std::isspace(chr); }).base(), 
            s.end()
        );

        return s;
    }


    static inline std::string rtrim(std::string&& s) 
    {
        s.erase(
            std::find_if(s.rbegin(), 
            s.rend(),
            // std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end()
            [](int chr) { return !std::isspace(chr); }).base(),
            s.end()
        );

        return s;
    }


    // trim from both ends
    //
    static inline std::string& trim(std::string& s) 
    {
        return ltrim(rtrim(s));
    }
    

    static inline std::string trim(std::string&& s) 
    {
        return ltrim(rtrim(s));
    }


    static inline std::vector<std::string>& split(const std::string& s, char delim, std::vector<std::string>& elems) 
    {
        std::stringstream ss { s };
        std::string item;

        while (std::getline(ss, item, delim)) {
            elems.push_back(item);
        }

        return elems;
    }


    static inline std::vector<std::string> split(const std::string& s, char delim) 
    {
        std::vector<std::string> elems { };

        split(s, delim, elems);
        return elems;
    }


    static inline void replace(std::string& s, const std::string& search, const std::string& replace) 
    {
        for (size_t pos = 0; ; pos += replace.length()) {
            pos = s.find(search, pos);

            if (pos == std::string::npos) break;

            s.erase(pos, search.length());
            s.insert(pos, replace);
        }
    }


} // namespace Navtech::Utility

#endif // STRING_HELPERS_H