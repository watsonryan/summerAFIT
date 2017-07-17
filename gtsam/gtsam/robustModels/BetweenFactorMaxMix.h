/*
 *  @file   BetweenFactorMaxMix.h
 *  @author Ryan
 *  @brief  Header file for between factor with max mixtures
 *
 * This needs a re-write.
    * Should not explicitly write out mixture components.
    * Should not have to pass vec and noise model.
 */

#include <Eigen/Eigen>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/slam/BetweenFactor.h>

namespace gtsam {

  template<class VALUE>
  class BetweenFactorMaxMix : public NoiseModelFactor2<VALUE, VALUE>
  {
    public:
      BetweenFactorMaxMix() {};
      BetweenFactorMaxMix(Key key1, Key key2, const VALUE& measured,
        const SharedNoiseModel& model, const SharedNoiseModel& model2,
        const Vector& hyp, const Vector& null)
          : NoiseModelFactor2<VALUE, VALUE>(model, key1, key2),
          nullHypothesisModel(model2), hypVec(hyp), nullVec(null),
        betweenFactor(key1, key2, measured, model)  {   };

      Vector evaluateError(const VALUE& p1, const VALUE& p2,
          boost::optional<Matrix&> H1 = boost::none,
          boost::optional<Matrix&> H2 =  boost::none) const
        {

          // calculate error
          Vector error = betweenFactor.evaluateError(p1, p2, H1, H2);

          // which hypothesis is more likely
          auto g1 = noiseModel::Gaussian::Covariance(
            ( Matrix(3,3) << hypVec ).finished() );

          auto g2 = noiseModel::Gaussian::Covariance(
            ( Matrix(3,3) << nullVec ).finished() );

          double m1 = this->noiseModel_->distance(error);
          Matrix info1(g1->information());
          double nu1 = 1.0/sqrt(inverse(info1).determinant());
          double l1 = nu1 * exp(-0.5*m1);

          double m2 = nullHypothesisModel->distance(error);
          Matrix info2(g2->information());
          double nu2 = 1.0/sqrt(inverse(info2).determinant());
          double l2 = nu2 * exp(-0.5*m2);

          // Remove this weight later. This was calculated
          // externally by taking forbenius( hyp-null )
          Matrix diff = (Matrix(3,3) << hypVec - nullVec).finished();
          double weight = 1/diff.norm();
          if (l2>l1) {
            if (H1) *H1 = *H1 * weight;
            if (H2) *H2 = *H2 * weight;
            error *= sqrt(weight);
          }

          return error;
        };

    private:
      BetweenFactor<VALUE> betweenFactor;
      SharedNoiseModel nullHypothesisModel;
      Vector hypVec, nullVec;

  };
}
