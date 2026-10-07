#include "StepperWaveform.h"

#include <algorithm>
#include <cmath>
#include <iterator>

namespace stepper
{
    namespace
    {
        const double kPi = 3.14159265358979323846;

        enum class Kind
        {
            Off,
            Pwm,      // sinusoidal microstepping, center-aligned PWM on all four terminals
            HalfStep, // logic-level half stepping, no PWM
        };

        struct Phase
        {
            double start; // seconds into the program
            double duration;
            Kind kind;
            double from; // full steps
            double to;
            double current; // fraction of full scale, PWM phases only
        };

        // The demo program. Moves use a raised-cosine profile; holds are moves with from == to.
        const Phase kProgram[] = {
            { 0.000, 0.002, Kind::Off, 0, 0, 0 },     { 0.002, 0.003, Kind::Pwm, 0, 0, 0.4 },     { 0.005, 0.012, Kind::Pwm, 0, 12, 1.0 },
            { 0.017, 0.003, Kind::Pwm, 12, 12, 0.4 }, { 0.020, 0.006, Kind::HalfStep, 12, 6, 0 }, { 0.026, 0.002, Kind::Off, 6, 6, 0 },
            { 0.028, 0.003, Kind::Pwm, 6, 6, 0.5 },   { 0.031, 0.010, Kind::Pwm, 6, -2, 0.7 },    { 0.041, 0.004, Kind::Pwm, -2, -2, 0.4 },
            { 0.045, 0.003, Kind::Pwm, -2, 0, 0.7 },
        };
        const double kProgramDuration = 0.048;

        const Phase& PhaseAt( double seconds )
        {
            double t = std::fmod( seconds, kProgramDuration );
            if( t < 0 )
                t += kProgramDuration;
            for( auto it = std::rbegin( kProgram ); it != std::rend( kProgram ); ++it )
                if( t >= it->start )
                    return *it;
            return kProgram[ 0 ];
        }

        double PhaseProgress( const Phase& phase, double seconds )
        {
            double t = std::fmod( seconds, kProgramDuration );
            if( t < 0 )
                t += kProgramDuration;
            return std::min( 1.0, std::max( 0.0, ( t - phase.start ) / phase.duration ) );
        }

        // Coil drive (-1..1) for a half-step index: index 0 is coil A positive, 45 degrees per index.
        void HalfStepDrive( long index, int& a, int& b )
        {
            static const int kA[ 8 ] = { 1, 1, 0, -1, -1, -1, 0, 1 };
            static const int kB[ 8 ] = { 0, 1, 1, 1, 0, -1, -1, -1 };
            const long i = ( ( index % 8 ) + 8 ) % 8;
            a = kA[ i ];
            b = kB[ i ];
        }
    }

    StepperWaveform::StepperWaveform( double sample_rate, double pwm_frequency )
        : mSampleRate( sample_rate ), mPeriod( 1.0 / pwm_frequency )
    {
    }

    double StepperWaveform::ProgramDuration()
    {
        return kProgramDuration;
    }

    bool StepperWaveform::EnergizedAt( double seconds )
    {
        return PhaseAt( seconds ).kind != Kind::Off;
    }

    double StepperWaveform::PositionAt( double seconds )
    {
        const Phase& phase = PhaseAt( seconds );
        const double u = PhaseProgress( phase, seconds );

        if( phase.kind == Kind::HalfStep )
            return std::round( ( phase.from + ( phase.to - phase.from ) * u ) * 2.0 ) / 2.0;

        return phase.from + ( phase.to - phase.from ) * ( 1.0 - std::cos( kPi * u ) ) / 2.0;
    }

    uint64_t StepperWaveform::GenerateNext( std::vector<Transition>& out )
    {
        const double t0 = double( mPeriodIndex ) * mPeriod;
        const double center = t0 + mPeriod / 2.0;
        const uint64_t s0 = uint64_t( std::llround( t0 * mSampleRate ) );
        const uint64_t s1 = uint64_t( std::llround( ( t0 + mPeriod ) * mSampleRate ) );
        ++mPeriodIndex;

        const Phase& phase = PhaseAt( center );

        // Duty of each terminal for this period. Logic-level drive is a duty of exactly 0 or 1.
        double duty[ TERMINAL_COUNT ] = {};
        if( phase.kind == Kind::Pwm )
        {
            const double angle = PositionAt( center ) * kPi / 2.0;
            const double a = phase.current * std::cos( angle );
            const double b = phase.current * std::sin( angle );
            duty[ A_POS ] = 0.5 + 0.5 * a;
            duty[ A_NEG ] = 0.5 - 0.5 * a;
            duty[ B_POS ] = 0.5 + 0.5 * b;
            duty[ B_NEG ] = 0.5 - 0.5 * b;
        }
        else if( phase.kind == Kind::HalfStep )
        {
            int a, b;
            HalfStepDrive( std::lround( PositionAt( center ) * 2.0 ), a, b );
            duty[ A_POS ] = a > 0;
            duty[ A_NEG ] = a < 0;
            duty[ B_POS ] = b > 0;
            duty[ B_NEG ] = b < 0;
        }

        const size_t first = out.size();
        auto set_level = [ & ]( int channel, uint64_t sample, bool level )
        {
            if( mLevel[ channel ] != level )
            {
                out.push_back( Transition{ sample, channel } );
                mLevel[ channel ] = level;
            }
        };

        for( int channel = 0; channel < TERMINAL_COUNT; ++channel )
        {
            const double d = duty[ channel ];
            const uint64_t rise = uint64_t( std::llround( ( center - d * mPeriod / 2.0 ) * mSampleRate ) );
            const uint64_t fall = uint64_t( std::llround( ( center + d * mPeriod / 2.0 ) * mSampleRate ) );

            if( rise >= fall )
            {
                set_level( channel, s0, false );
            }
            else if( rise <= s0 || fall >= s1 )
            {
                set_level( channel, s0, true );
            }
            else
            {
                set_level( channel, s0, false );
                set_level( channel, rise, true );
                set_level( channel, fall, false );
            }
        }

        std::stable_sort( out.begin() + first, out.end(), []( const Transition& x, const Transition& y ) { return x.sample < y.sample; } );
        return s1;
    }
}
