#define HAVE_BOOST
#define HAVE_PCAP
#define HAVE_FAST_PCAP
//#define VELOFRAME_USE_MULTITHREAD

#define TIMEZONE (+8)

#include <iostream>
#include "../3rdparty/args/args.hxx"
#include "../3rdparty/VelodyneCapture/VelodyneCapture.h"
#include "../include/veloFrame.h"
#include "../include/microStopwatch.h"


const std::string red("\033[0;31m");
const std::string green("\033[1;32m");
const std::string yellow("\033[1;33m");
const std::string cyan("\033[0;36m");
const std::string magenta("\033[0;35m");
const std::string reset("\033[0m");

std::string filename;

args::ArgumentParser parser("This is a test program.", "This goes after the options.");
args::HelpFlag help(parser, "help", "Display this help menu", {'h', "help"});

args::Group requirementGroup(parser, "This group is all required:", args::Group::Validators::All);
    args::ValueFlag<std::string> inputPcap(requirementGroup, "inputPcap", "input pcap", {"pcap"});
    args::ValueFlag<std::string> inputBackground(requirementGroup, "inputBackground", "input background", {"back"});
args::Group dontCareGroup(parser, "This group is dont care:", args::Group::Validators::DontCare);
    args::ValueFlag<double> backgroundSeg(dontCareGroup, "backgroundSeg", "Enable background segmentation", {"bs", "background_segmentation"});
    args::ValueFlagList<double> noiseRemoval(dontCareGroup, "noiseRemoval", "Enable noise removal", {"nr", "noise_removal"});
    args::Flag output(dontCareGroup, "output", "output", {'o', "output"});
    args::Flag outputAll(dontCareGroup, "outputAll", "output all", {"oa", "outputAll"});

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing)
{
    if(event.keyDown())
    {
        std::cout << "Keyboard pressed: " << ((event.isAltPressed())? "Alt + " : "") << ((event.isCtrlPressed())? "Ctrl + " : "") << ((event.isShiftPressed())? "Shift + " : "") << event.getKeySym() << std::endl;
    }
}

int main(int argc, char * argv[])
{
    myClass::MicroStopwatch tt("main");
    VeloFrame::VeloFrames veloFrames;
    boost::shared_ptr<VeloFrame::VeloFrameViewer> veloFrameViewer;

    try
    {
        parser.ParseCLI(argc, argv);
        
        boost::filesystem::path pcapPath{args::get(inputPcap)};
        boost::filesystem::path backgroundPath{args::get(inputBackground)};

        veloFrames.setPcapFile(pcapPath);
        veloFrames.setBackgroundCloud(backgroundPath);
        if(backgroundSeg)
        {
            veloFrames.setBackgroundSegmentationResolution(args::get(backgroundSeg));
        }
        if(noiseRemoval)
        {
            veloFrames.setNoiseRemovalParameter(args::get(noiseRemoval));
        }
        veloFrames.setOffsetPoint(1000.0, 2000.0, 3000.0);

        std::cout << "Loading...";tt.tic();
        veloFrames.load(pcapPath.parent_path().string());
        std::cout << " >> Done: " << tt.toc_string() << " us\n";
        
        if(outputAll)
        {
            std::cout << "Saving...";tt.tic();
            veloFrames.save(pcapPath.parent_path().string());
            std::cout << " >> Done: " << tt.toc_string() << " us\n";
        }

        if(backgroundSeg)
        {
            std::cout << "Background segmentation...";tt.tic();
            veloFrames.backgroundSegmentation();
            std::cout << " >> Done: " << tt.toc_string() << " us\n";
        }

        if(outputAll)
        {
            std::cout << "Saving...";tt.tic();
            veloFrames.save(pcapPath.parent_path().string());
            std::cout << " >> Done: " << tt.toc_string() << " us\n";
        }

        if(noiseRemoval)
        {
            std::cout << "Noise removal...";tt.tic();
            veloFrames.noiseRemoval();
            std::cout << " >> Done: " << tt.toc_string() << " us\n";
        }

        if(output||outputAll)
        {
            std::cout << "Saving...";tt.tic();
            veloFrames.save(pcapPath.parent_path().string());
            std::cout << " >> Done: " << tt.toc_string() << " us\n";
        }

        std::cout << "Offset...";tt.tic();
        veloFrames.offset();
        std::cout << " >> Done: " << tt.toc_string() << " us\n";
        veloFrames.print();

        veloFrameViewer.reset(new VeloFrame::VeloFrameViewer);
        veloFrameViewer->registerKeyboardCallback(keyboardEventOccurred);
        veloFrameViewer->addFrames(veloFrames);
        veloFrameViewer->run();
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
    catch(VeloFrame::VeloFrameException &e)
    {
        std::cout << e.message() << std::endl;
        return 1;
    }

    return 0;
}
