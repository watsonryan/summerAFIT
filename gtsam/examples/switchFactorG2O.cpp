/**
 * @file switchFactorG2O.cpp
 * @brief Script to automate the processing of G2O pose-graphs with switchConstraints
 * @author Ryan

  * How to run ::  ./maxmixG2O -i graph.g2o -o output.txt 
 */


// GTSAM 
#include <gtsam/slam/dataset.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/robustModels/BetweenFactorSwitchable.h>

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

#ifndef FOREACH_HPP
  #define FOREACH_HPP
  #include <boost/foreach.hpp>
  #define foreach BOOST_FOREACH
#endif

#include "boost/foreach.hpp"

#define foreach BOOST_FOREACH

double median(vector<double> medi) {
	int size = medi.size();
	double tmedian;
	if (size % 2 == 0) { // even
		tmedian = (medi[medi.size() / 2 - 1] + medi[medi.size() / 2]) / 2;
	}

	else //odd
		tmedian = medi[medi.size() / 2];
	return (tmedian);
}


int main(const int argc, const char *argv[]) {

  string g2oFile, outputFile, kernelType("none"), kerWidth("none"),truePose;
  const string green("\033[0;32m"), red("\033[0;31m");

  po::options_description desc("Available options");
  desc.add_options()
  ("help,h", "Print help message")
  ("input,i", po::value<string>(&g2oFile)->default_value(""), 
   "Input GNSS data file")
  ("trueGraph,t", po::value<string>(&truePose)->default_value(""), 
   "Input true pose graph for RMS comp.")
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
  NonlinearFactorGraph switchGraph;
  Values::shared_ptr initial;
  bool is3D = false;
  boost::tie(graph, initial) = readG2o(g2oFile,is3D);

  NonlinearFactorGraph graphWithPrior = *graph;
  noiseModel::Diagonal::shared_ptr priorModel = //
      noiseModel::Diagonal::Variances(Vector3(1, 1, 0.2));
  graphWithPrior.add(PriorFactor<Pose2>(0, Pose2(), priorModel));
  SharedNoiseModel switchPriorModel = noiseModel::Diagonal::Sigmas(
      (Vector(1) << 1 ).finished() ); 


  Values result = GaussNewtonOptimizer(graphWithPrior, *initial).optimize();  

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

      boost::shared_ptr<PriorFactor <SwitchVariableLinear> > switchPriorFactor(
    			new PriorFactor<SwitchVariableLinear>(Symbol('s',i),
    			SwitchVariableLinear(1.0),
          switchPriorModel));
      NonlinearFactor::shared_ptr factor(
          new BetweenFactorSwitchable<Pose2>(id1, id2, l1Xl2, hypothesis, null_model,
            hyp, null ));

      switchGraph.add(factor);
    }
  }
  Values resultMix = GaussNewtonOptimizer(switchGraph, *initial).optimize();  
//  switchGraph.printErrors(resultMix);


	vector<Pose2> finalPose, initPose;
	Values::ConstFiltered<Pose2> result_poses = resultMix.filter<Pose2>();
	foreach (const Values::ConstFiltered<Pose2>::KeyValuePair& key_value, result_poses) {
    Pose2 q = key_value.value;
    cout << q.x() << " " <<  q.y() << endl;
    finalPose.push_back(q);
		}

  if ( !truePose.empty() ) { boost::tie(graph, initial) = readG2o(truePose,is3D); }

	Values::ConstFiltered<Pose2> init_poses = initial->filter<Pose2>();
	foreach (const Values::ConstFiltered<Pose2>::KeyValuePair& key_value, init_poses) {
    Pose2 q = key_value.value;
    initPose.push_back(q);
  }

  vector<double> rss;
  for(unsigned int i = 0; i < initPose.size(); i++ ) {
    Point2 err = initPose[i].translation() - finalPose[i].translation();
    double e = sqrt( pow( err.x(),2) + pow(err.y(),2) ); 
    rss.push_back( e );
  }

  double med = median(rss);
  cout << med << endl;

  return 0;
}
