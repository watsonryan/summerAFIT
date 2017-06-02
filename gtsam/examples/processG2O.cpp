/**
 * @file ProcessG2O.cpp
 * @brief Script to automate the processing of G2O pose-graphs 
 * @author Ryan

  * How to run ::  ./processG2O -i graph.g2o -o output.txt -k huber -w 1  
 */


// GTSAM 
#include <gtsam/slam/dataset.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>

// BOOST 
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/serialization/export.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

// STL
#include <fstream>


using namespace std;
using namespace gtsam;
using namespace boost;
namespace po = boost::program_options;

int main(const int argc, const char *argv[]) {

  double kerWidth;
  string g2oFile, outputFile, kernelType("none");
  const string green("\033[0;32m"), red("\033[0;31m");

  po::options_description desc("Available options");
  desc.add_options()
  ("help,h", "Print help message")
  ("input,i", po::value<string>(&g2oFile)->default_value(""), 
   "Input GNSS data file")
  ("kernel,k", po::value<string>(&kernelType)->default_value("none"),
  "define the robust cost function (e.g., Huber, Tukey, Cauchy ... ).")
  ("kerWidth,w", po::value<double>(&kerWidth)->default_value(1),
  "define the robust cost function (e.g., Huber, Tukey, Cauchy ... ).")
  ("output,o", po::value<string>(&outputFile)->default_value(""), 
   "Input INS data file")
  ;

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);
  po::notify(vm);

  if ( g2oFile.empty() ) { 
        cout << red << "\n\n GNSS data must be specified\n" 
             << "\n\n" << green << desc << endl;
        exit(1);
    }

  // reading file and creating factor graph
  NonlinearFactorGraph::shared_ptr graph;
  Values::shared_ptr initial;
  bool is3D = false;
  if(kernelType.compare("none") == 0){
    boost::tie(graph, initial) = readG2o(g2oFile,is3D);
  }
  if(kernelType.compare("huber") == 0){
    boost::tie(graph, initial) = readG2o(g2oFile,is3D, KernelFunctionTypeHUBER);
  }
  if(kernelType.compare("tukey") == 0){
    boost::tie(graph, initial) = readG2o(g2oFile,is3D, KernelFunctionTypeTUKEY);
  }

  NonlinearFactorGraph graphWithPrior = *graph;
  noiseModel::Diagonal::shared_ptr priorModel = //
      noiseModel::Diagonal::Variances(Vector3(1e-6, 1e-6, 1e-8));
  graphWithPrior.add(PriorFactor<Pose2>(0, Pose2(), priorModel));
  Values result = GaussNewtonOptimizer(graphWithPrior, *initial).optimize();  

  std::cout << "final_error= " <<graph->error(result)<< std::endl;

  return 0;
}
