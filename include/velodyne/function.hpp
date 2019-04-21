#ifndef VELODYNE_FUNCTION_H_
#define VELODYNE_FUNCTION_H_

#include <iostream>
#ifdef HAVE_PCAP
#include <pcap.h>
#endif

namespace velodyne {
    static const int MAX_NUM_LASERS = 16;
    static const int LASER_PER_FIRING = 32;
    static const int FIRING_PER_PKT = 12;

    #pragma pack(push, 1)
    typedef struct LaserReturn
    {
        uint16_t distance;
        uint8_t intensity;
    } LaserReturn;
    #pragma pack(pop)

    struct FiringData
    {
        uint16_t blockIdentifier;
        uint16_t rotationalPosition;
        LaserReturn laserReturns[LASER_PER_FIRING];
    };

    struct DataPacket
    {
        FiringData firingData[FIRING_PER_PKT];
        uint32_t gpsTimestamp;
        uint8_t mode;
        uint8_t sensorType;
    };

            #ifdef HAVE_PCAP
    int getPcapTotalFrame(std::string filename) {
        int totalFrame;
        // Open PCAP File
        char error[PCAP_ERRBUF_SIZE];
        pcap_t* pcap = pcap_open_offline( filename.c_str(), error );
        if( !pcap ){
            throw std::runtime_error( error );
            return false;
        }

        // Convert PCAP_NETMASK_UNKNOWN to 0xffffffff
        struct bpf_program filter;
        std::ostringstream oss;
        if( pcap_compile( pcap, &filter, oss.str().c_str(), 0, 0xffffffff ) == -1 ){
            throw std::runtime_error( pcap_geterr( pcap ) );
            return false;
        }

        if( pcap_setfilter( pcap, &filter ) == -1 ){
            throw std::runtime_error( pcap_geterr( pcap ) );
            return false;
        }
        
        double last_azimuth = 0.0;
        
        while( true ){
            // Retrieve Header and Data from PCAP
            struct pcap_pkthdr* header;
            const unsigned char* data;
            const int ret = pcap_next_ex( pcap, &header, &data );
            if( ret <= 0 ){
                break;
            }

            // Check Packet Data Size
            // Data Blocks ( 100 bytes * 12 blocks ) + Time Stamp ( 4 bytes ) + Factory ( 2 bytes )
            if( ( header->len - 42 ) != 1206 ){
                continue;
            }

            // Convert to DataPacket Structure ( Cut Header 42 bytes )
            // Sensor Type 0x21 is HDL-32E, 0x22 is VLP-16
            const DataPacket* packet = reinterpret_cast<const DataPacket*>( data + 42 );
            assert( packet->sensorType == 0x21 || packet->sensorType == 0x22 );

            // Processing Packet
            for( int firing_index = 0; firing_index < FIRING_PER_PKT; firing_index++ ){
                // Retrieve Firing Data
                const FiringData firing_data = packet->firingData[firing_index];
                for( int laser_index = 0; laser_index < LASER_PER_FIRING; laser_index++ ){
                    // Retrieve Rotation Azimuth
                    double azimuth = static_cast<double>( firing_data.rotationalPosition );

                    // Complete Retrieve Capture One Rotation Data
                    if( last_azimuth > azimuth ){
                        totalFrame++;
                    }
                    // Update Last Rotation Azimuth
                    last_azimuth = azimuth;
                }
            }
        }

        return totalFrame;
    };
    #endif
}
#endif