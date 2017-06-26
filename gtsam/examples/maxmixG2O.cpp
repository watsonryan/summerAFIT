/**
 * @file maxmixG2O.cpp
 * @brief Script to automate the processing of G2O pose-graphs with max-mix
 * @author Ryan

  * How to run ::  ./maxmixG2O -i graph.g2o -o output.txt 
 */


// GTSAM 
#include <gtsam/slam/dataset.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/robustModels/BetweenFactorMaxMix.h>

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

  string g2oFile, outputFile, kernelType("none"), kerWidth("none");
  const string green("\033[0;32m"), red("\033[0;31m");

  po::options_description desc("Available options");
  desc.add_options()
  ("help,h", "Print help message")
  ("input,i", po::value<string>(&g2oFile)->default_value(""), 
   "Input GNSS data file")
  ("output,o", po::value<string>(&outputFile)->default_value(""), 
   "Input INS data file") ;

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
  NonlinearFactorGraph mixGraph;
  Values::shared_ptr initial;
  bool is3D = false;
  boost::tie(graph, initial) = readG2o(g2oFile,is3D);

  NonlinearFactorGraph graphWithPrior = *graph;
  noiseModel::Diagonal::shared_ptr priorModel = //
      noiseModel::Diagonal::Variances(Vector3(1, 1, 0.2));
  graphWithPrior.add(PriorFactor<Pose2>(0, Pose2(), priorModel));
  Values result = GaussNewtonOptimizer(graphWithPrior, *initial).optimize();  

  cout << graph->error(result) << endl;
  // Run Max Mixture over inital results
  // For now, run BMM offline to get cov. est. 
  Vector9 hyp, null;
  hyp << 16.89882198, -0.90303439, 0.08602105, 
        -0.90303439, 17.08403007, 0.06889149,
         0.08602105, 0.06889149, 10.95290421;
  null << 2087.43888892, -651.08477092, 198.82917436,
         -651.08477092, 1977.03231206, -92.72713996,
         198.82917436, -92.72713996, 1264.0179354;

  auto hypothesis = noiseModel::Gaussian::Covariance( 
    ( Matrix(3,3) << hyp ).finished() );
  auto null_model = noiseModel::Gaussian::Covariance( 
    ( Matrix(3,3) << null ).finished() );

  ifstream is(g2oFile.c_str());
  string tag;
  Key id1, id2;
  while ( !is.eof() ) {
    if (!(is >> tag)) { break; }
    if ((tag == "EDGE2") || (tag == "EDGE") || (tag == "EDGE_SE2")
        || (tag == "ODOMETRY")) {
      double x, y, yaw;
      is >> id1 >> id2 >> x >> y >> yaw;
      Pose2 l1Xl2(x, y, yaw);
      NonlinearFactor::shared_ptr factor(
          new BetweenFactorMaxMix<Pose2>(id1, id2, l1Xl2, hypothesis, null_model,
            hyp, null ));

      mixGraph.add(factor);
    }
  }
  Values resultMix = GaussNewtonOptimizer(mixGraph, *initial).optimize();  
  cout << mixGraph.error(resultMix) << endl;
  return 0;
}
