// VelodyneCapture
//
// VelodyneCapture is the capture class to retrieve the laser data from Velodyne sensors using Boost.Asio and PCAP.
// VelodyneCapture will be able to retrieve lasers infomation about azimuth, vertical and distance that capture at one rotation.
// This class supports direct capture form Velodyne sensors, or capture from PCAP files.
// ( This class only supports VLP-16 and HDL-32E sensor, and not supports Dual Return mode. )
//
// If direct capture from sensors, VelodyneCapture are requires Boost.Asio and its dependent libraries ( Boost.System, Boost.Date_Time, Boost.Regex ).
// Please define HAVE_BOOST in preprocessor.
//
// If capture from PCAP files, VelodyneCapture are requires PCAP.
// Please define HAVE_PCAP in preprocessor.
//
// This source code is licensed under the MIT license. Please see the License in License.txt.
// Copyright (c) 2017 Tsukasa SUGIURA
// t.sugiura0204@gmail.com

#ifndef VELODYNE_CAPTURE_CLOUD_H
#define VELODYNE_CAPTURE_CLOUD_H

#include <string>
#include <sstream>
#include <thread>
#include <atomic>
#include <mutex>
#include <queue>
#include <vector>
#include <cassert>
#include <cstdint>
#include <chrono>
#include <iomanip>
#include <algorithm>
#include <functional>
#ifdef HAVE_BOOST
#include <boost/asio.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <pcl-1.8/pcl/point_cloud.h>
#include <eigen3/Eigen/src/Core/Matrix.h>
#endif
#ifdef HAVE_PCAP
#include <pcap.h>
#endif
#define EPSILON 0.001

namespace velodyne
{
    struct Laser
    {
        double azimuth;
        double vertical;
        unsigned short distance;
        unsigned char intensity;
        unsigned char id;
        long long time;

        const bool operator < ( const struct Laser& laser ){
            if( azimuth == laser.azimuth ){
                return id < laser.id;
            }
            else{
                return azimuth < laser.azimuth;
            }
        }
    };

    template<typename PointT>
    class VelodyneCapture
    {
        typedef pcl::PointCloud<PointT> PointCloudT;
        typedef boost::shared_ptr<pcl::PointCloud<PointT>> PointCloudPtrT;
        protected:
            #ifdef HAVE_BOOST
            boost::asio::io_service ioservice;
            boost::asio::ip::udp::socket* socket = nullptr;
            boost::asio::ip::address address;
            unsigned short port = 2368;
            #endif

            #ifdef HAVE_PCAP
            pcap_t* pcap = nullptr;
            std::string filename = "";
            #endif

            std::thread* thread = nullptr;
            std::atomic_bool run = { false };
            std::mutex mutex;
            std::queue<PointCloudPtrT> queue;
            Eigen::Matrix4f *transformMatrixPtr = nullptr;

            int MAX_NUM_LASERS;
            std::vector<double> lut;

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

        public:
            // Constructor
            VelodyneCapture()
            {
            };

            #ifdef HAVE_BOOST
            // Constructor ( direct capture from Sensor )
            VelodyneCapture( const boost::asio::ip::address& address, const unsigned short port = 2368 )
            {
                open( address, port );
            };
            #endif

            #ifdef HAVE_PCAP
            // Constructor ( capture from PCAP )
            VelodyneCapture( const std::string& filename )
            {
                open( filename );
            };
            #endif

            // Destructor
            ~VelodyneCapture()
            {
                close();
            };

            #ifdef HAVE_BOOST
            // Open Direct Capture from Sensor
            const bool open( const boost::asio::ip::address& address, const unsigned short port = 2368 )
            {
                // Check Running
                if( isRun() ){
                    close();
                }

                // Set IP-Address and Port
                this->address = ( !address.is_unspecified() ) ? address : boost::asio::ip::address::from_string( "255.255.255.255" );
                this->port = port;

                // Create Socket
                try{
                    socket = new boost::asio::ip::udp::socket( ioservice, boost::asio::ip::udp::endpoint( this->address, this->port ) );
                }
                catch( ... ){
                    delete socket;
                    socket = new boost::asio::ip::udp::socket( ioservice, boost::asio::ip::udp::endpoint( boost::asio::ip::address_v4::any(), this->port ) );
                }

                // Start IO-Service
                try{
                    ioservice.run();
                }
                catch( const std::exception& e ){
                    std::cerr << e.what() << std::endl;
                    return false;
                }

                // Start Capture Thread
                run = true;
                thread = new std::thread( std::bind( &VelodyneCapture::captureSensor, this ) );

                return true;
            };
            #endif

            #ifdef HAVE_PCAP
            // Open Capture from PCAP
            const bool open( const std::string& filename, const Eigen::Matrix4f &transformMatrix )
            {
                this->transformMatrixPtr = new Eigen::Matrix4f(transformMatrix);
                return open(filename);
            };
            const bool open( const std::string& filename )
            {
                // Check Running
                if( isRun() ){
                    close();
                }

                // Open PCAP File
                char error[PCAP_ERRBUF_SIZE];
                pcap_t* pcap = pcap_open_offline( filename.c_str(), error );
                if( !pcap ){
                    throw std::runtime_error( error );
                    std::cout << "AAAA" << std::endl;
                    return false;
                }

                // Convert PCAP_NETMASK_UNKNOWN to 0xffffffff
                struct bpf_program filter;
                std::ostringstream oss;
                if( pcap_compile( pcap, &filter, oss.str().c_str(), 0, 0xffffffff ) == -1 ){
                    throw std::runtime_error( pcap_geterr( pcap ) );
                    std::cout << "BBBB" << std::endl;
                    return false;
                }

                if( pcap_setfilter( pcap, &filter ) == -1 ){
                    throw std::runtime_error( pcap_geterr( pcap ) );
                    std::cout << "CCCC" << std::endl;
                    return false;
                }

                this->pcap = pcap;
                this->filename = filename;

                // Start Capture Thread
                run = true;
                thread = new std::thread( std::bind( &VelodyneCapture::capturePCAP, this ) );

                return true;
            };
            #endif

            // Check Open
            const bool isOpen()
            {
                std::lock_guard<std::mutex> lock( mutex );
                return (
                    #if defined( HAVE_BOOST ) || defined( HAVE_PCAP )
                    #ifdef HAVE_BOOST
                    ( socket && socket->is_open() )
                    #endif
                    #if defined( HAVE_BOOST ) && defined( HAVE_PCAP )
                    ||
                    #endif
                    #ifdef HAVE_PCAP
                    pcap != nullptr
                    #endif
                    #else
                    false
                    #endif
                );
            };

            // Check Run
            const bool isRun()
            {
                // Returns True when Thread is Running or Queue is Not Empty
                std::lock_guard<std::mutex> lock( mutex );
                return ( run || !queue.empty() );
            }

            // Close Capture
            void close()
            {
                std::lock_guard<std::mutex> lock( mutex );
                run = false;
                // Close Capturte Thread
                if( thread && thread->joinable() ){
                    thread->join();
                    thread->~thread();
                    delete thread;
                    thread = nullptr;
                }

                #ifdef HAVE_BOOST
                // Close Socket
                if( socket && socket->is_open() ){
                    socket->close();
                    delete socket;
                    socket = nullptr;
                }

                // Stop IO-Service
                if( ioservice.stopped() ){
                    ioservice.stop();
                    ioservice.reset();
                }
                #endif

                #ifdef HAVE_PCAP
                // Close PCAP
                if( pcap ){
                    pcap_close( pcap );
                    pcap = nullptr;
                    filename = "";
                }
                #endif

                // Clear Queue
                std::queue<PointCloudPtrT>().swap( queue );
            };

            // Retrieve Capture Data
            void retrieve( PointCloudPtrT& cloud )
            {
                // Pop One Rotation Data from Queue
                if( mutex.try_lock() ){
                    if( !queue.empty() ){
                        cloud = queue.front();
                        queue.pop();
                    }
                    mutex.unlock();
                }
            };

            // Retrieve Capture Data
            void retrieve_block( PointCloudPtrT& cloud )
            {
                // Pop One Rotation Data from Queue
                while(isRun())
                {
                    if(queue.size() > 0) break;
                }
                if( mutex.try_lock() ){
                    if( !queue.empty() ){
                        cloud = queue.front();
                        queue.pop();
                    }
                    mutex.unlock();
                }
            };

            // Operator Retrieve Capture Data with Sort
            void operator >> ( PointCloudPtrT& cloud )
            {
                // Retrieve Capture Data
                retrieve( cloud );
            };

            size_t getQueueSize()
            {
                std::lock_guard<std::mutex> lock( mutex );
                return queue.size();
            }

        private:
            #ifdef HAVE_BOOST
            // Capture Thread from Sensor
            void captureSensor()
            {
                struct timeval last_time = { 0 };
                double last_azimuth = 0.0;
                PointCloudPtrT cloud;
                unsigned char data[1500];
                boost::asio::ip::udp::endpoint sender;

                cloud.reset(new PointCloudT());
                while( socket->is_open() && ioservice.stopped() && run ){
                    // Receive Packet
                    boost::system::error_code error;
                    const size_t length = socket->receive_from( boost::asio::buffer( data, sizeof( data ) ), sender, 0, error );
                    if( error == boost::asio::error::eof ){
                        break;
                    }

                    // Check IP-Address and Port
                    if( sender.address() != address && sender.port() != port ){
                        continue;
                    }

                    // Check Packet Data Size
                    // Data Blocks ( 100 bytes * 12 blocks ) + Time Stamp ( 4 bytes ) + Factory ( 2 bytes )
                    if( length != 1206 ){
                        continue;
                    }

                    // Retrieve Unix Time ( microseconds )
                    const std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
                    const std::chrono::microseconds epoch = std::chrono::duration_cast<std::chrono::microseconds>( now.time_since_epoch() );
                    const long long unixtime = epoch.count();

                    // Convert to DataPacket Structure
                    // Sensor Type 0x21 is HDL-32E, 0x22 is VLP-16
                    const DataPacket* packet = reinterpret_cast<const DataPacket*>( data );
                    assert( packet->sensorType == 0x21 || packet->sensorType == 0x22 );

                    // Caluculate Interpolated Azimuth
                    double interpolated = 0.0;
                    if( packet->firingData[1].rotationalPosition < packet->firingData[0].rotationalPosition ){
                        interpolated = ( ( packet->firingData[1].rotationalPosition + 36000 ) - packet->firingData[0].rotationalPosition ) / 2.0;
                    }
                    else{
                        interpolated = ( packet->firingData[1].rotationalPosition - packet->firingData[0].rotationalPosition ) / 2.0;
                    }

                    // Processing Packet
                    for( int firing_index = 0; firing_index < FIRING_PER_PKT; firing_index++ ){
                        // Retrieve Firing Data
                        const FiringData firing_data = packet->firingData[firing_index];
                        for( int laser_index = 0; laser_index < LASER_PER_FIRING; laser_index++ ){
                            // Retrieve Rotation Azimuth
                            double azimuth = static_cast<double>( firing_data.rotationalPosition );

                            // Interpolate Rotation Azimuth
                            if( laser_index >= MAX_NUM_LASERS )
                            {
                                azimuth += interpolated;
                            }

                            // Reset Rotation Azimuth
                            if( azimuth >= 36000 )
                            {
                                azimuth -= 36000;
                            }

                            // Complete Retrieve Capture One Rotation Data
                            #ifndef PUSH_SINGLE_PACKETS
                            if( last_azimuth > azimuth ){
                                // Push One Rotation Data to Queue
                                cloud->width = static_cast<uint32_t>(cloud->points.size());
                                cloud->height = 1;
                                mutex.lock();
                                queue.push( cloud );
                                mutex.unlock();
                                cloud.reset(new PointCloudT());
                                #ifdef MAX_QUEUE_SIZE
                                while(!(queue.size() < MAX_QUEUE_SIZE)) {
                                    std::this_thread::sleep_for( std::chrono::milliseconds( 100 ) );
                                }
                                #endif
                            }
                            #endif
                            #ifdef NO_EMPTY_RETURNS
                            if( firing_data.laserReturns[laser_index % MAX_NUM_LASERS].distance < EPSILON ){
                              continue;
                            }
                            #endif
                            Laser laser;
                            laser.azimuth = azimuth / 100.0;
                            laser.vertical = lut[laser_index % MAX_NUM_LASERS];
                            #ifdef USE_MILLIMETERS
                            laser.distance = firing_data.laserReturns[laser_index % MAX_NUM_LASERS].distance * 2.0;
                            #else
                            laser.distance = firing_data.laserReturns[laser_index % MAX_NUM_LASERS].distance * 2.0 / 10;
                            #endif
                            laser.intensity = firing_data.laserReturns[laser_index % MAX_NUM_LASERS].intensity;
                            laser.id = static_cast<unsigned char>( laser_index % MAX_NUM_LASERS );
                            #ifdef HAVE_GPSTIME
                            laser.time = packet->gpsTimestamp;
                            #else
                            laser.time = unixtime;
                            #endif

                            PointT p;
                            p.x = static_cast<float>( ( laser.distance * std::cos( laser.vertical * M_PI / 180.0 ) ) * std::sin( laser.azimuth  * M_PI / 180.0 ) );
                            p.y = static_cast<float>( ( laser.distance * std::cos( laser.vertical * M_PI / 180.0 ) ) * std::cos( laser.azimuth  * M_PI / 180.0 ) );
                            p.z = static_cast<float>( ( laser.distance * std::sin( laser.vertical * M_PI / 180.0 ) ) );
                        
                            // Update Last Rotation Azimuth
                            last_azimuth = azimuth;
                            if( (p.x == 0.0f) && (p.y == 0.0f) && (p.z == 0.0f) ){
                                continue;
                            }

                            cloud->points.push_back( p );
                        }
                    }
                    #ifdef PUSH_SINGLE_PACKETS
                    // Push packet after processing
                    mutex.lock();
                    queue.push( std::move( cloud ) );
                    mutex.unlock();
                    cloud.reset(new PointCloudT());
                    #endif
                }
                run = false;
            };
            #endif

            #ifdef HAVE_PCAP
            // Capture Thread from PCAP
            void capturePCAP()
            {
                struct timeval last_time = { 0 };
                double last_azimuth = 0.0;
                PointCloudPtrT cloud;
                float matrix[4][4];
                if(transformMatrixPtr) {
                    for(int i = 0; i < 4; i++) {
                        for(int j = 0; j < 4; j++) {
                            matrix[i][j] = (*transformMatrixPtr)(i,j);
                        }
                    }
                }

                cloud.reset(new PointCloudT());
                while( run ){
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

                    // Retrieve Unix Time ( microseconds )
                    std::stringstream ss;
                    ss << header->ts.tv_sec << std::setw( 6 ) << std::left << std::setfill( '0' ) << header->ts.tv_usec;
                    const long long unixtime = std::stoll( ss.str() );

                    // Convert to DataPacket Structure ( Cut Header 42 bytes )
                    // Sensor Type 0x21 is HDL-32E, 0x22 is VLP-16
                    const DataPacket* packet = reinterpret_cast<const DataPacket*>( data + 42 );
                    assert( packet->sensorType == 0x21 || packet->sensorType == 0x22 );

                    // Wait This Thread Difference Time
                    if( last_time.tv_sec == 0 )
                    {
                        last_time = header->ts;
                    }

                    if( last_time.tv_usec > header->ts.tv_usec )
                    {
                        last_time.tv_usec -= 1000000;
                        last_time.tv_sec++;
                    }

                    last_time = header->ts;

                    // Caluculate Interpolated Azimuth
                    double interpolated = 0.0;
                    if( packet->firingData[1].rotationalPosition < packet->firingData[0].rotationalPosition ){
                        interpolated = ( ( packet->firingData[1].rotationalPosition + 36000 ) - packet->firingData[0].rotationalPosition ) / 2.0;
                    }
                    else{
                        interpolated = ( packet->firingData[1].rotationalPosition - packet->firingData[0].rotationalPosition ) / 2.0;
                    }

                    // Processing Packet
                    for( int firing_index = 0; firing_index < FIRING_PER_PKT; firing_index++ ){
                        // Retrieve Firing Data
                        const FiringData firing_data = packet->firingData[firing_index];
                        for( int laser_index = 0; laser_index < LASER_PER_FIRING; laser_index++ ){
                            // Retrieve Rotation Azimuth
                            double azimuth = static_cast<double>( firing_data.rotationalPosition );

                            // Interpolate Rotation Azimuth
                            if( laser_index >= MAX_NUM_LASERS )
                            {
                                azimuth += interpolated;
                            }

                            // Reset Rotation Azimuth
                            if( azimuth >= 36000 )
                            {
                                azimuth -= 36000;
                            }

                            // Complete Retrieve Capture One Rotation Data
                            #ifndef PUSH_SINGLE_PACKETS
                            if( last_azimuth > azimuth ){
                                // Push One Rotation Data to Queue
                                cloud->width = static_cast<uint32_t>(cloud->points.size());
                                cloud->height = 1;
                                mutex.lock();
                                queue.push( cloud );
                                mutex.unlock();
                                cloud.reset(new PointCloudT());
                                #ifdef MAX_QUEUE_SIZE
                                while(!(queue.size() < MAX_QUEUE_SIZE)) {
                                    std::this_thread::sleep_for( std::chrono::milliseconds( 100 ) );
                                }
                                #endif
                            }
                            #endif
                            #ifdef NO_EMPTY_RETURNS
                            if( firing_data.laserReturns[laser_index % MAX_NUM_LASERS].distance < EPSILON ){
                              continue;
                            }
                            #endif
                            Laser laser;
                            laser.azimuth = azimuth / 100.0;
                            laser.vertical = lut[laser_index % MAX_NUM_LASERS];
                            #ifdef USE_MILLIMETERS
                            laser.distance = firing_data.laserReturns[laser_index % MAX_NUM_LASERS].distance * 2.0;
                            #else
                            laser.distance = firing_data.laserReturns[laser_index % MAX_NUM_LASERS].distance * 2.0 / 10;
                            #endif
                            laser.intensity = firing_data.laserReturns[laser_index % MAX_NUM_LASERS].intensity;
                            laser.id = static_cast<unsigned char>( laser_index % MAX_NUM_LASERS );
                            #ifdef HAVE_GPSTIME
                            laser.time = packet->gpsTimestamp;
                            #else
                            laser.time = unixtime;
                            #endif

                            // Update Last Rotation Azimuth
                            last_azimuth = azimuth;
                            /*if( (p.x == 0.0f) && (p.y == 0.0f) && (p.z == 0.0f) ){
                                continue;
                            }*/

                            PointT p1,p2;
                            p1.x = static_cast<float>( ( laser.distance * std::cos( laser.vertical * M_PI / 180.0 ) ) * std::sin( laser.azimuth  * M_PI / 180.0 ) );
                            p1.y = static_cast<float>( ( laser.distance * std::cos( laser.vertical * M_PI / 180.0 ) ) * std::cos( laser.azimuth  * M_PI / 180.0 ) );
                            p1.z = static_cast<float>( ( laser.distance * std::sin( laser.vertical * M_PI / 180.0 ) ) );
                        
                            if( p1.x == 0.0f && p1.y == 0.0f && p1.z == 0.0f ) continue;

                            if(transformMatrixPtr) {
                                p2.x = static_cast<float>( matrix[0][0] * p1.x + matrix[0][1] * p1.y + matrix[0][2] * p1.z + matrix[0][3] );
                                p2.y = static_cast<float>( matrix[1][0] * p1.x + matrix[1][1] * p1.y + matrix[1][2] * p1.z + matrix[1][3] );
                                p2.z = static_cast<float>( matrix[2][0] * p1.x + matrix[2][1] * p1.y + matrix[2][2] * p1.z + matrix[2][3] );
                                p1 = p2;
                            }

                            cloud->points.push_back( p1 );
                        }
                    }
                    #ifdef PUSH_SINGLE_PACKETS
                    // Push packet after processing
                    mutex.lock();
                    queue.push( cloud );
                    mutex.unlock();
                    cloud.reset(new PointCloudT());
                    #endif
                }

                run = false;
            };
            #endif
    };

    template<typename PointT>
    class VLP16Capture : public VelodyneCapture<PointT>
    {
        private:
            static const int MAX_NUM_LASERS = 16;
            const std::vector<double> lut = { -15.0, 1.0, -13.0, 3.0, -11.0, 5.0, -9.0, 7.0, -7.0, 9.0, -5.0, 11.0, -3.0, 13.0, -1.0, 15.0 };

        public:
            VLP16Capture() : VelodyneCapture<PointT>()
            {
                initialize();
            };

            #ifdef HAVE_BOOST
            VLP16Capture( const boost::asio::ip::address& address, const unsigned short port = 2368 ) : VelodyneCapture<PointT>( address, port )
            {
                initialize();
            };
            #endif

            #ifdef HAVE_PCAP
            VLP16Capture( const std::string& filename ) : VelodyneCapture<PointT>( filename )
            {
                initialize();
            };
            #endif

            ~VLP16Capture()
            {
            };

        private:
            void initialize()
            {
                VelodyneCapture<PointT>::MAX_NUM_LASERS = MAX_NUM_LASERS;
                VelodyneCapture<PointT>::lut = lut;
            };
    };

    template<typename PointT>
    class HDL32ECapture : public VelodyneCapture<PointT>
    {
        private:
            static const int MAX_NUM_LASERS = 32;
            const std::vector<double> lut = { -30.67, -9.3299999, -29.33, -8.0, -28, -6.6700001, -26.67, -5.3299999, -25.33, -4.0, -24.0, -2.6700001, -22.67, -1.33, -21.33, 0.0, -20.0, 1.33, -18.67, 2.6700001, -17.33, 4.0, -16, 5.3299999, -14.67, 6.6700001, -13.33, 8.0, -12.0, 9.3299999, -10.67, 10.67 };

        public:
            HDL32ECapture() : VelodyneCapture<PointT>()
            {
                initialize();
            };

            #ifdef HAVE_BOOST
            HDL32ECapture( const boost::asio::ip::address& address, const unsigned short port = 2368 ) : VelodyneCapture<PointT>( address, port )
            {
                initialize();
            };
            #endif

            #ifdef HAVE_PCAP
            HDL32ECapture( const std::string& filename ) : VelodyneCapture<PointT>( filename )
            {
                initialize();
            };
            #endif

            ~HDL32ECapture()
            {
            };

        private:
            void initialize()
            {
                VelodyneCapture<PointT>::MAX_NUM_LASERS = MAX_NUM_LASERS;
                VelodyneCapture<PointT>::lut = lut;
            };
    };
}

#endif
