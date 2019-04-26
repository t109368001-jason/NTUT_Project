#define DEBUG_INFO

#define HAVE_BOOST
#define HAVE_PCAP
#define HAVE_FAST_PCAP
#ifndef MAX_QUEUE_SIZE
#define MAX_QUEUE_SIZE 100
#endif

#include <iostream>
#include <QApplication>
#include <args/args.hxx>
#include <microStopwatch.hpp>
#include <velodyne/gui.hpp>
#include <velodyne/function.hpp>

args::ArgumentParser parser("This is a test program.", "This goes after the options.");
args::HelpFlag help(parser, "help", "Display this help menu", {'h', "help"});

int main(int argc, char * argv[])
{
    myClass::MicroStopwatch tt1("initial velodyne gui");

    QApplication app (argc, argv);
    tt1.tic();
    velodyne::GUI v;
    tt1.toc();
    
    v.show();

    return app.exec();
/*
    try
    {
        parser.ParseCLI(argc, argv);

    }
    catch (args::Help)
    {
        std::cout << parser;
        return 0;
    }
    catch (args::ParseError e)
    {
        std::cout << e.what() << std::endl;
        parser.Help(std::cout);
        return 1;
    }
    catch (args::ValidationError e)
    {
        std::cout << e.what() << std::endl;
        parser.Help(std::cout);
        return 1;
    }
    */
}