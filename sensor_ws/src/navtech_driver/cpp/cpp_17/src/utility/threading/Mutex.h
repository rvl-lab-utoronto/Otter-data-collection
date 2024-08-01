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
#ifndef MUTEX_H
#define MUTEX_H

#include <mutex>

namespace Navtech {

    class Condition;

    class Mutex {
    public:
        Mutex();
        virtual ~Mutex();

        Mutex(Mutex&&)                  = delete;
        Mutex& operator=(Mutex&&)       = delete;
        Mutex(const Mutex&)             = delete;
        Mutex& operator= (const Mutex&) = delete;

        void lock();
        void unlock();
        bool try_lock();
        
    private:
        friend class Condition;
        std::mutex   mutex;

    };


    // Scoped_lock provides a simple class to manage the lifetime of a mutex.
    // It is a basic implementation of the 'Scope-locked idiom'.
    // This class should be used within local scope and once
    // out of scope it will autoamtically release the mutex lock
    // enabling us to support protected code with multiple exit paths.
    //
    class Scoped_lock {
    public:
        Scoped_lock(Mutex& mtx) : mutex { &mtx } 
        {
            mutex->lock();
        }
        
        ~Scoped_lock() 
        {
            mutex->unlock();
        }

        // Required for CRITICAL_SECTION macro
        //
        operator bool()
        {
            return true;
        }

    private:
        Mutex* mutex;
    };

} // namespace Navtech

// The Scoped_lock will keep its associated Mutex locked until
// the Scoped_lock is destroyed.  This may keep the Mutex locked
// longer than is desired.
// This convenience macro allows us to introduce a new block
// to constrain the scope of the Scoped_lock.
// The macro allows a block ( { ... } ) to be placed around code
// that must be in a critical section.  For example:
//
// CRITICAL_SECTION(my_mutex) 
// {
//     This code is performed with
//     the my_mutex locked...
//     
// }   // unlock the mutex
//
// The END_CRITICAL_SECTION macro doesn't do anything, but may be
// useful for documentation if the critical section block is large
//
#define CRITICAL_SECTION(mutex) if (Navtech::Scoped_lock _lock_ = mutex)
#define END_CRITICAL_SECTION(mutex)

#endif // MUTEX_H
