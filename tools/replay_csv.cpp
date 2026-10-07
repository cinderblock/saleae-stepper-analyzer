// Replays a Logic 2 digital CSV export ("Time [s],<ch>,<ch>,...") through the stepper decoder and
// prints the resulting segments. Used to check the C++ decoder against real captures without
// loading it into Logic.
//
//   stepper_replay <digital.csv> --rate <Hz> [--columns A+,A-,B+,B-] [--pwm auto|none|<Hz>]
//                  [--resolution <denominator>] [--absolute] [--min-ms <ms>] [--group-ms <ms>]
//
// --columns picks the CSV data columns (0-based, after the time column) for A+, A-, B+, B-.
// --min-ms hides segments shorter than the given duration in the listing (the summary still
// counts everything). --group-ms groups quick successive positions into moves, as the
// analyzer's "Moves and holds" results do, with the given hold time.

#include "MoveGrouper.h"
#include "StepperPipeline.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace stepper;

namespace
{
    const char* TypeName( SegmentType type )
    {
        switch( type )
        {
        case SegmentType::Position:
            return "position";
        case SegmentType::Off:
            return "off";
        case SegmentType::Ambiguous:
            return "ambiguous";
        case SegmentType::Move:
            return "move";
        }
        return "?";
    }

    int Usage()
    {
        std::cerr << "usage: stepper_replay <digital.csv> --rate <Hz> [--columns 0,1,2,3] [--pwm auto|none|<Hz>]"
                     " [--resolution 16] [--absolute] [--min-ms 0] [--group-ms 20]\n";
        return 2;
    }
}

int main( int argc, char** argv )
{
    if( argc < 2 )
        return Usage();

    std::string path = argv[ 1 ];
    double rate = 0;
    int columns[ TERMINAL_COUNT ] = { 0, 1, 2, 3 };
    PipelineConfig config;
    double min_ms = 0;
    double group_ms = 0;

    for( int i = 2; i < argc; ++i )
    {
        std::string arg = argv[ i ];
        auto next = [ & ]() -> std::string { return i + 1 < argc ? argv[ ++i ] : ""; };
        if( arg == "--rate" )
            rate = std::atof( next().c_str() );
        else if( arg == "--columns" )
        {
            std::stringstream list( next() );
            std::string item;
            for( int c = 0; c < TERMINAL_COUNT && std::getline( list, item, ',' ); ++c )
                columns[ c ] = std::atoi( item.c_str() );
        }
        else if( arg == "--pwm" )
        {
            std::string mode = next();
            if( mode == "auto" )
                config.pwm_mode = PwmMode::Auto;
            else if( mode == "none" )
                config.pwm_mode = PwmMode::None;
            else
            {
                config.pwm_mode = PwmMode::Manual;
                config.pwm_frequency = std::atof( mode.c_str() );
            }
        }
        else if( arg == "--resolution" )
            config.decoder.resolution = 1.0 / std::atof( next().c_str() );
        else if( arg == "--absolute" )
            config.decoder.start_at_zero = false;
        else if( arg == "--min-ms" )
            min_ms = std::atof( next().c_str() );
        else if( arg == "--group-ms" )
            group_ms = std::atof( next().c_str() );
        else
            return Usage();
    }
    if( rate <= 0 )
        return Usage();
    config.sample_rate = rate;

    std::ifstream file( path );
    if( !file )
    {
        std::cerr << "cannot open " << path << "\n";
        return 1;
    }

    std::string line;
    std::getline( file, line ); // header

    StepperPipeline pipeline( config );
    std::vector<Segment> segments;
    bool started = false;
    double t0 = 0;
    bool state[ TERMINAL_COUNT ] = {};
    uint64_t last_sample = 0;
    std::vector<std::string> fields;

    while( std::getline( file, line ) )
    {
        fields.clear();
        std::stringstream row( line );
        std::string field;
        while( std::getline( row, field, ',' ) )
            fields.push_back( field );

        const double t = std::atof( fields[ 0 ].c_str() );
        bool values[ TERMINAL_COUNT ];
        for( int c = 0; c < TERMINAL_COUNT; ++c )
            values[ c ] = std::atoi( fields[ 1 + columns[ c ] ].c_str() ) != 0;

        if( !started )
        {
            t0 = t;
            pipeline.Start( 0, values );
            for( int c = 0; c < TERMINAL_COUNT; ++c )
                state[ c ] = values[ c ];
            started = true;
            continue;
        }

        const uint64_t sample = uint64_t( std::llround( ( t - t0 ) * rate ) );
        for( int c = 0; c < TERMINAL_COUNT; ++c )
        {
            if( values[ c ] != state[ c ] )
            {
                pipeline.PushEdge( c, sample );
                state[ c ] = values[ c ];
            }
        }
        last_sample = sample;
        pipeline.SetFilled( sample );
        pipeline.Process( segments );
    }

    pipeline.SetFilled( last_sample + 1 );
    pipeline.Finish( segments );

    if( group_ms > 0 )
    {
        MoveGrouper grouper( uint64_t( group_ms * 1e-3 * rate ) );
        std::vector<Segment> grouped;
        for( const Segment& segment : segments )
            grouper.Add( segment, grouped );
        grouper.Flush( grouped );
        segments.swap( grouped );
    }

    size_t counts[ 4 ] = {};
    std::printf( "start_s,end_s,type,position_steps,from_steps,mean_steps,angle_deg,drive,direction\n" );
    for( const Segment& segment : segments )
    {
        counts[ int( segment.type ) ]++;
        const double start = t0 + double( segment.start ) / rate;
        const double end = t0 + double( segment.end ) / rate;
        if( ( end - start ) * 1000.0 < min_ms && segment.type != SegmentType::Ambiguous )
            continue;
        std::printf( "%.7f,%.7f,%s,%.4f,%.4f,%.4f,%.1f,%.3f,%d\n", start, end, TypeName( segment.type ), segment.position,
                     segment.from_position, segment.mean_position, segment.angle_degrees, segment.drive, segment.direction );
    }

    double final_position = 0;
    for( auto it = segments.rbegin(); it != segments.rend(); ++it )
    {
        if( it->type == SegmentType::Position || it->type == SegmentType::Move )
        {
            final_position = it->position;
            break;
        }
    }

    std::fprintf( stderr, "segments: %zu position, %zu move, %zu off, %zu ambiguous; final position %.4f full steps\n",
                  counts[ int( SegmentType::Position ) ], counts[ int( SegmentType::Move ) ], counts[ int( SegmentType::Off ) ],
                  counts[ int( SegmentType::Ambiguous ) ], final_position );
    return 0;
}
