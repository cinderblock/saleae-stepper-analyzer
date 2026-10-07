#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include "EdgeQueue.h"
#include "PwmEstimator.h"
#include "StepperDecoder.h"
#include "StepperPipeline.h"
#include "StepperWaveform.h"

#include <cmath>
#include <vector>

using namespace stepper;

namespace
{
    const double kPi = 3.14159265358979323846;

    // Duties for a coil drive vector at `degrees` electrical with magnitude `drive`.
    void DutiesFor( double degrees, double drive, double duty[ TERMINAL_COUNT ] )
    {
        const double a = drive * std::cos( degrees * kPi / 180.0 );
        const double b = drive * std::sin( degrees * kPi / 180.0 );
        duty[ A_POS ] = 0.5 + 0.5 * a;
        duty[ A_NEG ] = 0.5 - 0.5 * a;
        duty[ B_POS ] = 0.5 + 0.5 * b;
        duty[ B_NEG ] = 0.5 - 0.5 * b;
    }

    const Segment* SegmentAt( const std::vector<Segment>& segments, uint64_t sample )
    {
        for( const Segment& segment : segments )
            if( segment.start <= sample && sample < segment.end )
                return &segment;
        return nullptr;
    }
}

TEST_CASE( "EdgeQueue integrates high time across window boundaries" )
{
    EdgeQueue queue;
    queue.Reset( 100, false );
    queue.Push( 110 ); // high from 110
    queue.Push( 130 ); // low from 130
    queue.Push( 140 ); // high from 140

    CHECK( queue.ConsumeHighSamples( 120 ) == 10 );
    CHECK( queue.ConsumeHighSamples( 135 ) == 10 );
    CHECK( queue.NextEdge() == 140 );
    CHECK( queue.ConsumeHighSamples( 200 ) == 60 );
    CHECK( queue.State() );
    CHECK( queue.ConsumeHighSamples( 210 ) == 10 );
}

TEST_CASE( "PwmEstimator finds a steady PWM period and ignores step-rate edges" )
{
    PwmEstimator::Config config;
    config.max_period = 1000;
    config.horizon = 100000;

    SUBCASE( "PWM" )
    {
        PwmEstimator estimator( config, 2 );
        for( int k = 0; k < 50; ++k )
        {
            // Center-aligned jitter of a couple of samples around a 366.4 sample period.
            estimator.AddRisingEdge( 0, uint64_t( 366.4 * k + ( k % 3 ) ) );
            estimator.AddRisingEdge( 1, uint64_t( 366.4 * k + 100 ) );
        }
        CHECK( estimator.PeriodAt( 366 * 50 ) == doctest::Approx( 366.4 ).epsilon( 0.01 ) );
    }

    SUBCASE( "slow stepping is not PWM" )
    {
        PwmEstimator estimator( config, 1 );
        for( int k = 0; k < 50; ++k )
            estimator.AddRisingEdge( 0, uint64_t( 5000 * k ) );
        CHECK( estimator.PeriodAt( 5000 * 50 ) == 0 );
    }

    SUBCASE( "old intervals expire" )
    {
        PwmEstimator estimator( config, 1 );
        for( int k = 0; k < 50; ++k )
            estimator.AddRisingEdge( 0, uint64_t( 400 * k ) );
        CHECK( estimator.PeriodAt( 400 * 50 ) == doctest::Approx( 400 ) );
        CHECK( estimator.PeriodAt( 400 * 50 + 200000 ) == 0 );
    }
}

TEST_CASE( "StepperDecoder counts full steps in both directions" )
{
    DecoderConfig config;
    config.resolution = 0.25;
    StepperDecoder decoder( config );
    std::vector<Segment> out;
    double duty[ TERMINAL_COUNT ];

    // Full-step drive: 45, 135, 225, 315, 405 degrees, then back down two steps.
    const double angles[] = { 45, 135, 225, 315, 405, 315, 225 };
    uint64_t t = 0;
    for( double angle : angles )
    {
        DutiesFor( angle, std::sqrt( 2.0 ), duty );
        decoder.AddWindow( t, t + 100, duty, out );
        t += 100;
    }
    decoder.Flush( out );

    REQUIRE( out.size() == 7 );
    const double expected[] = { 0, 1, 2, 3, 4, 3, 2 };
    for( size_t i = 0; i < out.size(); ++i )
    {
        CHECK( out[ i ].type == SegmentType::Position );
        CHECK( out[ i ].position == doctest::Approx( expected[ i ] ) );
    }
    CHECK( out[ 0 ].direction == 1 );
    CHECK( out[ 4 ].direction == -1 );
    CHECK( out[ 0 ].angle_degrees == doctest::Approx( 45 ) );
    CHECK( out[ 0 ].drive == doctest::Approx( std::sqrt( 2.0 ) ) );
}

TEST_CASE( "StepperDecoder reports absolute electrical position when not starting at zero" )
{
    DecoderConfig config;
    config.start_at_zero = false;
    StepperDecoder decoder( config );
    std::vector<Segment> out;
    double duty[ TERMINAL_COUNT ];

    DutiesFor( -112.5, 0.4, duty );
    decoder.AddWindow( 0, 100, duty, out );
    decoder.Flush( out );

    REQUIRE( out.size() == 1 );
    CHECK( out[ 0 ].position == doctest::Approx( -1.25 ) );
}

TEST_CASE( "StepperDecoder inverts direction" )
{
    DecoderConfig config;
    config.invert = true;
    config.resolution = 1;
    StepperDecoder decoder( config );
    std::vector<Segment> out;
    double duty[ TERMINAL_COUNT ];

    DutiesFor( 45, 1, duty );
    decoder.AddWindow( 0, 100, duty, out );
    DutiesFor( 135, 1, duty );
    decoder.AddWindow( 100, 200, duty, out );
    decoder.Flush( out );

    REQUIRE( out.size() == 2 );
    CHECK( out[ 1 ].position == doctest::Approx( -1 ) );
    CHECK( out[ 0 ].direction == -1 );
}

TEST_CASE( "StepperDecoder handles de-energized gaps and ambiguous jumps" )
{
    DecoderConfig config;
    config.resolution = 0.5;
    StepperDecoder decoder( config );
    std::vector<Segment> out;
    double duty[ TERMINAL_COUNT ];

    SUBCASE( "re-energizing resumes from the nearest electrical cycle" )
    {
        // Step up to +3 full steps, switch off, come back one step further on.
        const double angles[] = { 0, 90, 180, 270 };
        uint64_t t = 0;
        for( double angle : angles )
        {
            DutiesFor( angle, 1, duty );
            decoder.AddWindow( t, t + 100, duty, out );
            t += 100;
        }
        const double off[ TERMINAL_COUNT ] = { 0, 0, 0, 0 };
        decoder.AddWindow( t, t + 1000, off, out );
        t += 1000;
        DutiesFor( 360, 1, duty );
        decoder.AddWindow( t, t + 100, duty, out );
        decoder.Flush( out );

        REQUIRE( out.size() == 6 );
        CHECK( out[ 4 ].type == SegmentType::Off );
        CHECK( out[ 4 ].start == 400 );
        CHECK( out[ 4 ].end == 1400 );
        CHECK( out[ 5 ].type == SegmentType::Position );
        CHECK( out[ 5 ].position == doctest::Approx( 4 ) );
    }

    SUBCASE( "a 180 degree jump is flagged" )
    {
        DutiesFor( 0, 1, duty );
        decoder.AddWindow( 0, 100, duty, out );
        DutiesFor( 180, 1, duty );
        decoder.AddWindow( 100, 200, duty, out );
        DutiesFor( 180, 1, duty );
        decoder.AddWindow( 200, 300, duty, out );
        decoder.Flush( out );

        REQUIRE( out.size() == 3 );
        CHECK( out[ 1 ].type == SegmentType::Ambiguous );
        CHECK( out[ 1 ].start == 100 );
        CHECK( out[ 1 ].end == 200 );
        CHECK( out[ 2 ].type == SegmentType::Position );
    }
}

TEST_CASE( "StepperDecoder ignores energized blips shorter than the minimum" )
{
    DecoderConfig config;
    config.min_energized_samples = 50;
    StepperDecoder decoder( config );
    std::vector<Segment> out;
    double duty[ TERMINAL_COUNT ];
    const double off[ TERMINAL_COUNT ] = { 0, 0, 0, 0 };

    decoder.AddWindow( 0, 100, off, out );
    DutiesFor( 45, 1, duty );
    decoder.AddWindow( 100, 110, duty, out ); // 10-sample switching blip
    decoder.AddWindow( 110, 300, off, out );
    DutiesFor( -112.5, 0.4, duty );
    decoder.AddWindow( 300, 340, duty, out );
    decoder.AddWindow( 340, 400, duty, out );
    decoder.Flush( out );

    REQUIRE( out.size() == 2 );
    CHECK( out[ 0 ].type == SegmentType::Off );
    CHECK( out[ 0 ].end == 300 );
    CHECK( out[ 1 ].type == SegmentType::Position );
    CHECK( out[ 1 ].start == 300 );
    CHECK( out[ 1 ].position == doctest::Approx( 0 ) ); // the blip did not become the zero reference
}

TEST_CASE( "StepperDecoder takes the zero reference after the drive settles" )
{
    DecoderConfig config;
    config.zero_settle_samples = 300;
    StepperDecoder decoder( config );
    std::vector<Segment> out;
    double duty[ TERMINAL_COUNT ];

    // Power-up: one window of ramping current at a slightly different angle, then the hold.
    DutiesFor( -104.6, 0.14, duty );
    decoder.AddWindow( 0, 100, duty, out );
    DutiesFor( -112.3, 0.38, duty );
    for( uint64_t t = 100; t < 1000; t += 100 )
        decoder.AddWindow( t, t + 100, duty, out );
    decoder.Flush( out );

    REQUIRE( out.size() == 2 );
    CHECK( out[ 1 ].position == doctest::Approx( 0 ) );
    CHECK( out[ 1 ].start == 100 );
}

TEST_CASE( "Pipeline decodes the synthetic program to the commanded positions" )
{
    struct Case
    {
        double sample_rate;
        double pwm_frequency;
        PwmMode mode;
    };
    const Case cases[] = {
        { 4e6, 20000, PwmMode::Auto },  { 6.25e6, 17077, PwmMode::Auto }, { 1e6, 20000, PwmMode::Auto },
        { 50e6, 31250, PwmMode::Auto }, { 4e6, 20000, PwmMode::Manual },
    };

    for( const Case& c : cases )
    {
        CAPTURE( c.sample_rate );
        CAPTURE( c.pwm_frequency );

        PipelineConfig config;
        config.sample_rate = c.sample_rate;
        config.pwm_mode = c.mode;
        config.pwm_frequency = c.pwm_frequency;
        config.decoder.resolution = 1.0 / 16;

        StepperWaveform wave( c.sample_rate, c.pwm_frequency );
        StepperPipeline pipeline( config );
        bool initial[ TERMINAL_COUNT ];
        for( int i = 0; i < TERMINAL_COUNT; ++i )
            initial[ i ] = wave.InitialState( i );
        pipeline.Start( 0, initial );

        // Two loops of the program.
        const uint64_t total = uint64_t( 2 * StepperWaveform::ProgramDuration() * c.sample_rate );
        std::vector<Transition> transitions;
        std::vector<Segment> segments;
        uint64_t generated = 0;
        while( generated < total )
        {
            transitions.clear();
            generated = wave.GenerateNext( transitions );
            for( const Transition& transition : transitions )
                pipeline.PushEdge( transition.channel, transition.sample );
            pipeline.SetFilled( generated );
            pipeline.Process( segments );
        }
        pipeline.Finish( segments );

        // No ambiguous jumps anywhere.
        for( const Segment& segment : segments )
            CHECK( segment.type != SegmentType::Ambiguous );

        // Segments tile the decoded range without gaps.
        for( size_t i = 1; i < segments.size(); ++i )
            CHECK( segments[ i ].start == segments[ i - 1 ].end );

        // At the middle of every hold and off phase (and spot checks inside moves), the decoded
        // position matches the command. The first energized window defines position 0, which is
        // also where the program starts.
        struct Check
        {
            double time;
            double tolerance; // full steps
        };
        // Holds are exact. Inside moves the decoded position may trail by the window length plus
        // the hysteresis band.
        const Check checks[] = { { 0.001, 0 }, { 0.0035, 0 }, { 0.011, 0.35 }, { 0.0185, 0 }, { 0.023, 0 },
                                 { 0.027, 0 }, { 0.0295, 0 }, { 0.036, 0.35 }, { 0.043, 0 },  { 0.047, 0.35 } };
        for( int loop = 0; loop < 2; ++loop )
        {
            for( const Check& check : checks )
            {
                const double seconds = check.time + loop * StepperWaveform::ProgramDuration();
                CAPTURE( seconds );
                const Segment* segment = SegmentAt( segments, uint64_t( seconds * c.sample_rate ) );
                REQUIRE( segment != nullptr );
                if( !StepperWaveform::EnergizedAt( seconds ) )
                {
                    CHECK( segment->type == SegmentType::Off );
                    continue;
                }
                CHECK( segment->type == SegmentType::Position );
                CHECK( std::fabs( segment->position - StepperWaveform::PositionAt( seconds ) ) <= check.tolerance + 1e-9 );
            }
        }
    }
}
