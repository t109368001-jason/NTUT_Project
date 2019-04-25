#ifndef MYZIP_H_
#define MYZIP_H_

#include <iostream>
#include <boost/filesystem.hpp>
#include <zipios/zipfile.hpp>
#include <zipios/directorycollection.hpp>

namespace myZip
{
    class myZip : public zipios::ZipFile
    {
        public:
            myZip() : zipios::ZipFile() {};
            myZip(std::string filename) : zipios::ZipFile(filename) {};
            
            bool search(boost::filesystem::path filename)
            {
                return (this->getEntry(filename.string()) == NULL);
            }

            bool search(std::string filename)
            {
                return this->search(boost::filesystem::path{filename});
            }

            bool extra(boost::filesystem::path filename, boost::filesystem::path outputFilename)
            {
                boost::filesystem::path outputFilePath = outputFilename;
                zipios::ZipFile::stream_pointer_t zfsp;
                zfsp = this->getInputStream(filename.string());
                if(zfsp == NULL) return false;
                boost::filesystem::create_directories(outputFilePath.parent_path());
                std::ofstream ofs(outputFilePath.string());
                ofs << zfsp->rdbuf();
                ofs.close();
                return true;
            }

            bool extra(std::string filename, std::string outputFilename)
            {
                return this->extra(boost::filesystem::path{filename}, boost::filesystem::path{outputFilename});
            }

            std::stringstream getStringstream(boost::filesystem::path filename)
            {

                zipios::ZipFile::stream_pointer_t zfsp;
                zfsp = this->getInputStream(filename.string());

                std::stringstream ss;
                std::string lines;

                ss << zfsp->rdbuf();

                return ss;
            }

            std::string getString(boost::filesystem::path filename)
            {
                return this->getStringstream(filename).str();
            }

    };
}

#endif