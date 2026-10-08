#ifndef STEPPER_IDLE_FLUSHER_H
#define STEPPER_IDLE_FLUSHER_H

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>

// Shows the state an analyzer is in while it waits for more data.
//
// Results here are spans that only end when the next change arrives, and the Analyzer SDK never
// says a capture has ended: reading past the last sample just blocks. Without help, the final
// span of every capture (and the current one during a pause in a live capture) would never appear.
//
// The worker holds Lock() while it decodes and emits results, and releases it around SDK calls
// that may block. If the worker waits for data longer than the current delay, this thread takes
// the lock and runs `flush` once, which emits the open span up to the data read so far.
//
// Logic finalizes an analyzer's data table within milliseconds of the worker waiting past the end of
// a capture, so a flush there usually reaches only the bubbles, not the data table or exports; the
// SDK offers no way to learn where a capture ends in time to do better. During a live capture the
// worker also waits for data, so each flush doubles the delay (up to `max_delay`) until the worker
// reports Activity(), which keeps a long hold from being cut into many pieces.
class IdleFlusher
{
  public:
    IdleFlusher() = default;
    ~IdleFlusher()
    {
        Stop();
    }

    IdleFlusher( const IdleFlusher& ) = delete;
    IdleFlusher& operator=( const IdleFlusher& ) = delete;

    void Start( std::function<void()> flush, std::chrono::milliseconds delay, std::chrono::milliseconds max_delay )
    {
        Stop();
        mFlush = std::move( flush );
        mBaseDelay = delay;
        mMaxDelay = max_delay;
        mDelay = delay;
        mStopping = false;
        mWaiting = false;
        mFlushed = true;
        mThread = std::thread( [ this ]() { Run(); } );
    }

    void Stop()
    {
        {
            std::lock_guard<std::mutex> guard( mStateMutex );
            mStopping = true;
        }
        mWake.notify_all();
        if( mThread.joinable() )
            mThread.join();
    }

    // Guards the worker's decoding state and result output.
    std::mutex& Lock()
    {
        return mWorkMutex;
    }

    // The worker is about to wait for data (it must not hold Lock()).
    void Waiting()
    {
        std::lock_guard<std::mutex> guard( mStateMutex );
        mWaiting = true;
        mWaitingSince = std::chrono::steady_clock::now();
    }

    // The worker got more data.
    void Progressed()
    {
        std::lock_guard<std::mutex> guard( mStateMutex );
        mWaiting = false;
        mFlushed = false;
    }

    // Something changed (a new result began): go back to the short delay.
    void Activity()
    {
        std::lock_guard<std::mutex> guard( mStateMutex );
        mDelay = mBaseDelay;
    }

  private:
    void Run()
    {
        std::unique_lock<std::mutex> state( mStateMutex );
        while( !mStopping )
        {
            mWake.wait_for( state, std::min( std::chrono::milliseconds( 50 ), std::max( std::chrono::milliseconds( 1 ), mDelay / 4 ) ) );
            if( mStopping || !mWaiting || mFlushed || std::chrono::steady_clock::now() - mWaitingSince < mDelay )
                continue;

            mFlushed = true;
            mDelay = std::min( mDelay * 2, mMaxDelay );
            state.unlock();
            {
                std::lock_guard<std::mutex> work( mWorkMutex );
                mFlush();
            }
            state.lock();
        }
    }

    std::function<void()> mFlush;
    std::chrono::milliseconds mBaseDelay{ 10 };
    std::chrono::milliseconds mMaxDelay{ 1000 };
    std::chrono::milliseconds mDelay{ 10 };
    std::thread mThread;

    std::mutex mWorkMutex;
    std::mutex mStateMutex;
    std::condition_variable mWake;
    bool mStopping = false;
    bool mWaiting = false;
    bool mFlushed = true;
    std::chrono::steady_clock::time_point mWaitingSince;
};

#endif // STEPPER_IDLE_FLUSHER_H
