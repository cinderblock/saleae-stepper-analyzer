#ifndef STEPPER_PIPELINE_H
#define STEPPER_PIPELINE_H

#include "EdgeQueue.h"
#include "PwmEstimator.h"
#include "StepperDecoder.h"

#include <cstdint>
#include <vector>

namespace stepper
{
    enum class PwmMode
    {
        Auto,   // detect the PWM period from the data
        Manual, // use the configured PWM frequency
        None,   // logic-level drive: no averaging beyond the minimum window
    };

    struct PipelineConfig
    {
        double sample_rate = 0; // Hz
        PwmMode pwm_mode = PwmMode::Auto;
        double pwm_frequency = 0;         // Hz, used with PwmMode::Manual
        int periods_per_window = 2;       // PWM periods averaged per window
        double minimum_window = 5e-6;     // seconds; the window when no PWM is known
        double minimum_energized = 20e-6; // seconds; shorter energized blips (switching skew) count as off
        double zero_settle = 1e-3;        // seconds of drive before the start-at-zero reference is taken
        double max_pwm_period = 250e-6;   // seconds; slower repetition is treated as stepping, not PWM
        double lookahead = 2e-3;          // seconds of data read ahead of the decoding point
        double estimator_history = 3e-3;  // seconds of past edges the PWM estimator remembers
        DecoderConfig decoder;
    };

    // Windows the four drive channels, averages each terminal's duty over every window and feeds
    // the decoder.
    //
    // Data arrives as edges plus a "filled" mark that says every edge before it has been pushed.
    // Decoding trails the filled mark by the lookahead so the PWM estimator has already seen the
    // edges around each window. Idle stretches (no edges on any channel) are covered by a single
    // long window, so long holds and de-energized periods cost almost nothing.
    class StepperPipeline
    {
      public:
        explicit StepperPipeline( const PipelineConfig& config );

        void Start( uint64_t start_sample, const bool initial_states[ TERMINAL_COUNT ] );

        // Record that `channel` changes state at `sample`. Edges must be pushed in order per channel.
        void PushEdge( int channel, uint64_t sample );

        // Every edge before `sample` has been pushed, on all channels.
        void SetFilled( uint64_t sample );

        // How far data must be filled before the next window can be decoded.
        uint64_t NeededFill();

        // Decode every window the filled data allows. Completed segments are appended to `out`.
        void Process( std::vector<Segment>& out );

        // End of data: decode up to the filled mark without waiting for lookahead, then flush.
        void Finish( std::vector<Segment>& out );

        // Decode up to the filled mark without waiting for lookahead and close the open segment
        // there, then carry on: shows the current state while waiting for more data.
        void Checkpoint( std::vector<Segment>& out );

        const StepperDecoder& Decoder() const
        {
            return mDecoder;
        }

        uint64_t Cursor() const
        {
            return mCursor;
        }

        // The window length in samples that will be used next (for diagnostics).
        double CurrentWindow();

      private:
        bool DecodeOneWindow( bool finishing, std::vector<Segment>& out );

        PipelineConfig mConfig;
        EdgeQueue mQueues[ TERMINAL_COUNT ];
        bool mStates[ TERMINAL_COUNT ] = {};
        PwmEstimator mEstimator;
        StepperDecoder mDecoder;

        uint64_t mCursor = 0;
        uint64_t mFilled = 0;
        double mFraction = 0;     // fractional samples carried between windows
        double mStickyPeriod = 0; // last detected PWM period, in samples
        uint64_t mLookahead = 0;
    };
}

#endif // STEPPER_PIPELINE_H
